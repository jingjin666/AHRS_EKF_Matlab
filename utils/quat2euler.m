%NED
function [roll, pitch, yaw] = quat2euler(q)
    rad2deg=180/pi;
    
%     ∂‘”¶R_ZYX_b2n DCMæÿ’Û
%     [ cosYaw*cosPitch, cosYaw*sinRoll*sinPitch - cosRoll*sinYaw, sinYaw*sinRoll + cosYaw*cosRoll*sinPitch]
%     [ cosPitch*sinYaw, cosYaw*cosRoll + sinYaw*sinRoll*sinPitch, cosRoll*sinYaw*sinPitch - cosYaw*sinRoll]
%     [       -sinPitch,                         cosPitch*sinRoll,                         cosRoll*cosPitch]
    R=[ 1 - 2 * (q(4) *q(4)+q(3) * q(3))          2 * (q(2) * q(3)-q(1) * q(4))         2 * (q(2) * q(4)+q(1) * q(3));
            2 * (q(2) * q(3)+q(1) * q(4))         1 - 2 * (q(4) *q(4)+q(2) * q(2))      2 * (q(3) * q(4)-q(1) * q(2));
            2 * (q(2) * q(4)-q(1) * q(3))         2 * (q(3) * q(4)+q(1) * q(2))         1 - 2 * (q(2) *q(2)+q(3) * q(3))];

    roll  = atan2(R(3,2),R(3,3))*rad2deg;
    pitch = -asin(R(3,1))*rad2deg;
    yaw   = atan2(R(2,1),R(1,1))*rad2deg;
end