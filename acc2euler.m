% NED
function [roll, pitch, yaw] = acc2euler(acce_data, mag_data)
    roll = atan2(acce_data(2)/norm(acce_data), acce_data(3)/norm(acce_data));
	pitch = asin(acce_data(1)/norm(acce_data));
	r1 = -mag_data(2)*cos(roll) + mag_data(3)*sin(roll);
	r2 = mag_data(1)*cos(pitch) + mag_data(2)*sin(pitch)*sin(roll) + mag_data(3)*sin(pitch)*cos(roll);
	yaw = atan2(r1, r2);
end

% %ENU
% function [pitch0,roll0,yaw0] = acc2euler(acce_data,mag_data)
%     pitch0 = asin(acce_data(2)/norm(acce_data,2));
%     roll0 = atan2(-acce_data(1),acce_data(3));
%     
%     m_x = mag_data(1)*cos(roll0)+mag_data(3)*sin(roll0);
%     m_y = mag_data(1)*sin(pitch0)*sin(roll0)+ mag_data(2)*cos(pitch0)-mag_data(3)*cos(roll0)*sin(pitch0); 
%     yaw0 = atan2(m_y,m_x);
% end