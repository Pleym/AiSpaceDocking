function P = docking_params()
%DOCKING_PARAMS All assumptions and tuning knobs of the docking simulation, in one place.
% Default values: replace them with your own vehicle / docking mechanism.
%
% Frames (same conventions as the synthetic dataset):
%   port   : origin at the docking ring, +Z pointing out of the port (towards the chaser).
%   camera : X right, Y down, Z forward, mounted on the chaser's docking axis.
%   pose   : X_camera = R_camera_from_port * X_port + t_camera_from_port  (what the vision outputs).
% Orbit : the port is fixed in the LVLH frame of a station on a circular orbit (Clohessy-Wiltshire).
%   +Z port = +V-bar (approach from the front), +Y port = radial, +X port = orbit normal.

n = 2*pi/(92.9*60);                % rad/s, ISS-like orbit (local variable: codegen forbids adding
P.n = n;                           % a struct field after the struct has been read)
% Chaser: ideal thrusters, acceleration commands saturated per axis (camera frame)
P.a_max = 0.02;                    % m/s^2 (e.g. 10 N on 500 kg)
P.alpha_max = 0.01;                % rad/s^2
P.inertia = diag([500 500 400]);   % kg.m^2 (gyroscopic term only)
P.Ts = 0.5;                        % s, control period (~ vision rate)
% Guidance: approach corridor along +Z port
P.z_contact = 0.10;                % m, front face of the ring
P.v_max = 0.10;                    % m/s, far approach speed
P.v_contact = 0.03;                % m/s, final approach speed
P.tau_approach = 40;               % s, speed setpoint = gap / tau, bounded by v_contact and v_max
P.corridor_base = 0.05;            % m, corridor half-width at contact
P.corridor_slope = 0.05;           % m of half-width per metre of gap
P.max_misalignment_deg = 3;        % no forward motion until the attitude is aligned
% PD gains (damping ~1)
P.kp_lat = 0.02;  P.kd_lat = 0.28;
P.kv_axial = 0.2;
P.kp_att = 0.05;  P.kd_att = 0.45;
% Success criteria at contact (DEFAULTS: use your mechanism's capture envelope)
P.tol_lateral = 0.05;              % m
P.tol_angle_deg = 2;               % deg
P.tol_speed = 0.05;                % m/s, max closing speed at contact
P.t_max = 900;                     % s
% Sensor (Sensor_mode block, can be overridden per run):
%   0 true pose | 1 statistical model of the vision error | 2 real CNN in the loop (not in this beta)
P.sensor = 1;
P.z_min_camera = 0.5;              % m: closer than this the target overflows the image -> no measurement
% Statistical sensor (mode 1): per-axis std, camera frame, sigma = a + c*d^2 (fitted on validation data).
% Attitude error dominates the lateral error in the port frame (~2 cm at 2.5 m, 7-11 cm at 9 m).
P.noise_pos_a = [0.25; 0.16; 0.30] / 100;        % m
P.noise_pos_c = [0.0020; 0.0033; 0.040] / 100;   % m per m^2
P.noise_rot_sigma = deg2rad([0.50; 0.42; 0.26]); % rad, per camera axis
P.noise_min_distance = 1.9;        % m
P.noise_seed = 1;
% Initial state x = [p (3) ; v (3) ; q camera->port wxyz (4) ; inertial omega, camera frame (3)]
p0 = [0.3; -0.2; 10];
q_aligned = [0; 1; 0; 0];          % camera on the axis: R = diag(1,-1,-1)
q0 = quat_mult(q_aligned, quat_from_rotvec(deg2rad([5; -3; 8])));
R0 = quat_to_rotm(q0);
P.x0 = [p0; zeros(3, 1); q0; R0' * [n; 0; 0]];   % at rest relative to the port
end

function q = quat_from_rotvec(phi)
angle = norm(phi);
q = [cos(angle/2); sin(angle/2) * phi / max(angle, eps)];
end

function q = quat_mult(a, b)
q = [a(1)*b(1) - a(2:4)'*b(2:4); a(1)*b(2:4) + b(1)*a(2:4) + cross(a(2:4), b(2:4))];
end
