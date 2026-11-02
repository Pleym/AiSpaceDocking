function file = export_video_trajectory(out, folder, mode)
%EXPORT_VIDEO_TRAJECTORY Simulation state resampled at 30 fps for the video tools (video/*.py).
%   Columns: time, camera position (port frame), camera -> port quaternion (w, x, y, z), closing speed,
%   commanded acceleration (camera frame), last measured position (NaN without measurement) and the
%   index of the image seen by the CNN (mode 2 only, -1 otherwise).
P = docking_params();
FPS = 30;
[t, x] = read_logged_series(out.state, 13);
[t, keep] = unique(t, 'last');               % duplicated time stamps at discrete steps
x = x(keep, :);
[tc, u] = read_logged_series(out.command, 6);
[tp, pose] = read_logged_series(out.measured_pose, 12);
time = (0:1/FPS:t(end))';
state = interp1(t, x, time);                 % solver steps <= 0.1 s: linear is enough
state(:, 7:10) = state(:, 7:10) ./ vecnorm(state(:, 7:10), 2, 2);
command = interp1(tc, u, time, 'previous', 'extrap');
measured = nan(numel(tp), 3);
for i = 1:numel(tp)
    if all(isfinite(pose(i, :)))
        R = reshape(pose(i, 1:9), 3, 3)';
        measured(i, :) = (-R * pose(i, 10:12)')';
    end
end
measured = interp1(tp, measured, time, 'previous', 'extrap');
cnn_image = -ones(numel(time), 1);
if mode == 2
    valid = all(isfinite(measured), 2);
    cnn_image(valid) = floor(time(valid) / P.Ts + 1e-9);   % one image per control step
end
table_out = array2table([time, state(:, 1:3), state(:, 7:10), -state(:, 6), command(:, 1:3), measured, cnn_image], ...
    'VariableNames', {'time_s', 'x_m', 'y_m', 'z_m', 'qw', 'qx', 'qy', 'qz', 'closing_speed_m_s', ...
    'ax_m_s2', 'ay_m_s2', 'az_m_s2', 'meas_x_m', 'meas_y_m', 'meas_z_m', 'cnn_image'});
names = {'truth', 'sensor_model', 'cnn'};
file = fullfile(folder, ['video_trajectory_' names{mode + 1} '.csv']);
writetable(table_out, file);
fprintf('Video trajectory: %d frames (%.0f s) -> %s\n', numel(time), time(end), file);
end
