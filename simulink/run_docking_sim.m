function results = run_docking_sim(seeds, mode)
%RUN_DOCKING_SIM Rebuilds the model, flies one or more approaches and scores the contact.
%   seeds : sensor-noise seeds (Monte Carlo), 1 by default. The first run is plotted and exported.
%   mode  : 0 true pose, 1 statistical model of the CNN error (default: P.sensor).
%   Examples: r = run_docking_sim();              % one approach, plotted
%             r = run_docking_sim(1:20, 1);       % Monte Carlo with the vision error model
%             r = run_docking_sim(1, 0);          % perfect sensor, as a reference
P = docking_params();
if nargin < 1
    seeds = 1;
end
if nargin < 2
    mode = P.sensor;
end
folder = fileparts(mfilename('fullpath'));
addpath(folder);
build_docking_model();
for k = 1:numel(seeds)
    sim_in = Simulink.SimulationInput('docking_model');
    sim_in = sim_in.setBlockParameter('docking_model/Noise', 'Seed', num2str(seeds(k)));
    sim_in = sim_in.setBlockParameter('docking_model/Sensor_mode', 'Value', num2str(mode));
    out = sim(sim_in);
    r = score(out, P);
    r.seed = seeds(k);
    results(k) = r; %#ok<AGROW>
    if k == 1
        plot_run(out, folder, mode);
        export_video_trajectory(out, folder, mode);
    end
end
names = {'true pose', 'statistical CNN error model', 'CNN in the loop'};
fprintf('Sensor: %s | %d approach(es)\n', names{mode + 1}, numel(results));
fprintf('Success: %d / %d\n', sum([results.success]), numel(results));
fprintf('Lateral error: mean %.2f cm, worst %.2f cm (tolerance %.0f cm)\n', ...
    100*mean([results.lateral_error_m]), 100*max([results.lateral_error_m]), 100*P.tol_lateral);
fprintf('Angle error: mean %.2f deg, worst %.2f deg (tolerance %.0f deg)\n', ...
    mean([results.angle_error_deg]), max([results.angle_error_deg]), P.tol_angle_deg);
fprintf('Closing speed: %.1f to %.1f cm/s (max %.0f cm/s)\n', ...
    100*min([results.closing_speed_m_s]), 100*max([results.closing_speed_m_s]), 100*P.tol_speed);
fprintf('Duration: %.0f to %.0f s | mean delta-v: %.3f m/s\n', min([results.t_contact]), ...
    max([results.t_contact]), mean([results.delta_v_m_s]));
end

function r = score(out, P)
[t, x] = read_logged_series(out.state, 13);
[~, u] = read_logged_series(out.command, 6);
xf = x(end, :)';
r.contact = xf(3) <= P.z_contact;
r.t_contact = t(end);
r.lateral_error_m = norm(xf(1:2));
r.angle_error_deg = angle_deg(xf(7:10));
r.closing_speed_m_s = -xf(6);
r.lateral_speed_m_s = norm(xf(4:5));
r.delta_v_m_s = sum(vecnorm(u(:, 1:3), 2, 2)) * P.Ts;   % command held over Ts
r.success = r.contact && r.lateral_error_m <= P.tol_lateral && r.angle_error_deg <= P.tol_angle_deg ...
    && r.closing_speed_m_s > 0 && r.closing_speed_m_s <= P.tol_speed;
end

function plot_run(out, folder, mode)
[t, x] = read_logged_series(out.state, 13);
[tc, u] = read_logged_series(out.command, 6);
[~, pose] = read_logged_series(out.measured_pose, 12);
measured = nan(size(pose, 1), 3);            % measured camera position, port frame
for i = 1:size(pose, 1)
    if all(isfinite(pose(i, :)))
        R = reshape(pose(i, 1:9), 3, 3)';
        measured(i, :) = (-R * pose(i, 10:12)')';
    end
end
angles = arrayfun(@(i) angle_deg(x(i, 7:10)'), 1:size(x, 1));
fig = figure('Visible', 'off', 'Position', [0 0 1100 750]);
subplot(2, 2, 1); hold on;
plot(measured(:, 3), 100*measured(:, 1), '.', 'Color', [0.6 0.75 1], 'MarkerSize', 4);
plot(x(:, 3), 100*x(:, 1), 'b', x(:, 3), 100*x(:, 2), 'r'); set(gca, 'XDir', 'reverse');
xlabel('Axial distance z (m)'); ylabel('Lateral offset (cm)'); legend('x measured', 'x true', 'y true'); grid on;
title('Approach corridor');
subplot(2, 2, 2); plot(x(:, 3), -100*x(:, 6)); set(gca, 'XDir', 'reverse');
xlabel('Axial distance z (m)'); ylabel('Closing speed (cm/s)'); grid on; title('Speed profile');
subplot(2, 2, 3); plot(t, angles); xlabel('Time (s)'); ylabel('Attitude error (deg)'); grid on; title('Alignment');
subplot(2, 2, 4); stairs(tc, 100*u(:, 1:3)); xlabel('Time (s)');
ylabel('Commanded acceleration (cm/s^2)'); legend('x cam', 'y cam', 'z cam'); grid on; title('Pilot commands');
names = {'truth', 'sensor_model', 'cnn'};
exportgraphics(fig, fullfile(folder, ['docking_' names{mode + 1} '.png']));
close(fig);
end

function a = angle_deg(q)
a = rad2deg(acos(min(1, max(-1, (trace(diag([1 -1 -1]) * quat_to_rotm(q)) - 1) / 2))));
end
