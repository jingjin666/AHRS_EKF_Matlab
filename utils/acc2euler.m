% NED
function [roll, pitch, yaw] = acc2euler(acce_data, mag_data)
    ax = acce_data(1)/norm(acce_data);
    ay = acce_data(2)/norm(acce_data);
    az = acce_data(3)/norm(acce_data);
    roll = atan2(-ay, -az);
    pitch = atan2(ax, sqrt(ay*ay + az*az));
    
    mx = mag_data(1)/norm(mag_data);
    my = mag_data(2)/norm(mag_data);
    mz = mag_data(3)/norm(mag_data);
    
	m_x = mx * cos(pitch) + my * sin(roll)*sin(pitch) + mz * cos(roll)*sin(pitch);
    m_y = my * cos(roll) - mz * sin(roll);
	yaw = atan2(-m_y, m_x);
    
%     if(yaw < 0)
%         yaw = yaw+360*pi/180;
%     end
end