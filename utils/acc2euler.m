% NED
function [roll, pitch, yaw] = acc2euler(acce_data, mag_data)
    roll = atan2(acce_data(2)/norm(acce_data), acce_data(3)/norm(acce_data));
	pitch = -asin(acce_data(1)/norm(acce_data));
    
	m_x = mag_data(1)*cos(roll)+mag_data(3)*sin(roll);
    m_y = mag_data(1)*sin(pitch)*sin(roll)+ mag_data(2)*cos(pitch)-mag_data(3)*cos(roll)*sin(pitch); 
	yaw = atan2(m_y, m_x);
end