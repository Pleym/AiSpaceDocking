function R = rotvec_to_rotm(phi)
%ROTVEC_TO_ROTM Rotation matrix of a rotation vector (Rodrigues' formula).
angle = norm(phi);
if angle < 1e-12
    R = eye(3);
    return
end
k = phi / angle;
K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
end
