%NED
function [roll, pitch, yaw] = quat2euler(q)      
    rad2deg=180/pi;
    roll  = atan2(2*(q(3)*q(4) + q(1)*q(2)), 1-2*(q(2)*q(2) + q(3)*q(3))) * rad2deg;
    pitch = -asin(2*(q(2)*q(4) - q(1)*q(3))) * rad2deg;
    yaw   = atan2(2*(q(2)*q(3) + q(1)*q(4)), 1-2*(q(3)*q(3) + q(4)*q(4))) * rad2deg;  
end