function [roll, pitch, yaw] = quat2euler(q)      
    rad2deg=180/pi;
    R_b2n=[ 1 - 2 * (q(4) *q(4) + q(3) * q(3)),  2 * (q(2) * q(3)-q(1) * q(4)),               2 * (q(2) * q(4) +q(1) * q(3)) ;
            2 * (q(2) * q(3) +q(1) * q(4)),      1 - 2 * (q(4) *q(4) + q(2) * q(2)),          2 * (q(3) * q(4)-q(1) * q(2));
            2 * (q(2) * q(4)-q(1) * q(3)),       2 * (q(3) * q(4)+q(1) * q(2)),               1 - 2 * (q(2) *q(2) + q(3) * q(3))];%cbn

    roll  = atan2(R_b2n(3,2),R_n2b(3,3))*rad2deg;
    pitch = -asin(R_b2n(3,1))*rad2deg;
    yaw   = atan2(R_b2n(2,1),R_n2b(1,1))*rad2deg;  
end