function dx = docking_dynamics(x, u)
%DOCKING_DYNAMICS State derivative of the chaser relative to the port (port frame).
%   x = [p ; v ; q camera->port (wxyz) ; inertial omega (camera frame)]
%   u = [acceleration (3) ; angular acceleration (3)], camera frame.
P = docking_params();
p = x(1:3); v = x(4:6); q = x(7:10); w = x(11:13);
R = quat_to_rotm(q);                         % port <- camera
a_cw = cw_acceleration(p, v, P.n);
w_rel = w - R' * [P.n; 0; 0];                % the port frame spins at n around the orbit normal (+X port)
qv = q(2:4);
dq = 0.5 * [-qv'*w_rel; q(1)*w_rel + cross(qv, w_rel)] + 0.5 * (1 - q'*q) * q;   % + norm restoring term
dw = u(4:6) - P.inertia \ cross(w, P.inertia*w);
dx = [v; R*u(1:3) + a_cw; dq; dw];
end
