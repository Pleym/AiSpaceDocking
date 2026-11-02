function a = cw_acceleration(p, v, n)
%CW_ACCELERATION Clohessy-Wiltshire acceleration in the port frame (port fixed in LVLH).
%   Hill: x radial (= +Y port), y along-track (= +Z port), z orbit normal (= +X port).
M = [0 1 0; 0 0 1; 1 0 0];                   % Hill <- port
r = M*p; w = M*v;
a = M' * [3*n^2*r(1) + 2*n*w(2); -2*n*w(1); -n^2*r(3)];
end
