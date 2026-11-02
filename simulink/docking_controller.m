function u = docking_controller(pose)
%DOCKING_CONTROLLER Baseline pilot of the public beta: guidance + PD control from the MEASURED pose.
%   pose = [R_camera_from_port(:) ; t_camera_from_port], NaN when there is no measurement.
%   Output u = [acceleration ; angular acceleration], camera frame.
%   Velocities come from finite differences, which amplifies vision noise (try sensor mode 1 and
%   look at the thruster commands). The full pilot (Kalman filter with orbital model, proper dead
%   reckoning in the blind zone) is withheld until v1.0.
P = docking_params();
persistent p_prev R_prev v w started
if isempty(started)
    p_prev = zeros(3, 1); R_prev = diag([1 -1 -1]); v = zeros(3, 1); w = zeros(3, 1); started = false;
end
measured = all(isfinite(pose));
if ~started && ~measured
    u = zeros(6, 1);
    return
end
if measured
    R = reshape(pose(1:9), 3, 3)';           % port <- camera
    p = -R * pose(10:12);                    % camera position in the port frame
    if started
        v = (p - p_prev) / P.Ts;
        w = rotm_to_rotvec(R_prev' * R) / P.Ts;
    end
    started = true;
else
    p_prev = p_prev + v * P.Ts;              % no image (target too close): coast, thrusters off.
    u = zeros(6, 1);                         % The full pilot keeps steering with its Kalman estimate.
    return
end
p_prev = p; R_prev = R;
% Attitude: optical axis on -Z port
phi = rotm_to_rotvec(diag([1 -1 -1]) * R);
alpha = saturate(-P.kp_att*phi - P.kd_att*w, P.alpha_max);
% Guidance: move forward only inside the corridor and once aligned
gap = p(3) - P.z_contact;
corridor = P.corridor_base + P.corridor_slope * max(gap, 0);
v_setpoint = -max(P.v_contact, min(P.v_max, gap / P.tau_approach));
if norm(p(1:2)) > corridor || norm(phi) > deg2rad(P.max_misalignment_deg)
    v_setpoint = 0;
end
a_port = [-P.kp_lat*p(1:2) - P.kd_lat*v(1:2); P.kv_axial*(v_setpoint - v(3))];
u = [saturate(R' * a_port, P.a_max); alpha];
end

function phi = rotm_to_rotvec(R)
% Valid far from 180 deg, which is plenty for attitude errors of a few degrees
angle = acos(min(1, max(-1, (trace(R) - 1) / 2)));
if angle < 1e-6
    s = 1;
else
    s = sin(angle) / angle;
end
phi = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] * (0.5 / max(s, 1e-9));
end

function y = saturate(x, m)
y = min(max(x, -m), m);
end
