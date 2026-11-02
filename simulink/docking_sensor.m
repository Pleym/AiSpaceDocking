function pose = docking_sensor(x, noise, mode)
%DOCKING_SENSOR Measured pose in the vision format: [R_camera_from_port(:) ; t_camera_from_port].
%   mode 0 : true pose.
%   mode 1 : true pose + statistical model of the CNN error vs distance (noise: 6 standard normal draws).
%   mode 2 : real CNN in the loop (cnn_sensor.m, withheld in the public beta).
%   Modes 1 and 2 : no measurement below P.z_min_camera (NaN), the pilot has to fly blind.
coder.extrinsic('cnn_sensor');
P = docking_params();
if mode > 0 && x(3) < P.z_min_camera
    pose = nan(12, 1);
    return
end
if mode == 2
    pose = zeros(12, 1);
    pose = cnn_sensor(x);                    % runs in MATLAB, outside code generation
    return
end
R = quat_to_rotm(x(7:10));                   % port <- camera
Rcp = R';
t = -Rcp * x(1:3);
if mode == 1
    d = max(norm(t), P.noise_min_distance);
    t = t + (P.noise_pos_a + P.noise_pos_c * d^2) .* noise(1:3);
    Rcp = rotvec_to_rotm(P.noise_rot_sigma .* noise(4:6)) * Rcp;   % attitude error, camera frame
end
pose = [Rcp(:); t];
end
