% NED
function [yaw] = getYaw(mag_data, roll, pitch)
    mx = mag_data(1)/norm(mag_data);
    my = mag_data(2)/norm(mag_data);
    mz = mag_data(3)/norm(mag_data);
    
	m_x = mx * cos(pitch) + my * sin(roll)*sin(pitch) + mz * cos(roll)*sin(pitch);
    m_y = my * cos(roll) - mz * sin(roll);
	yaw = atan2(-m_y, m_x);
    
    if(yaw < 0)
        yaw = yaw+360*pi/180;
    end
end