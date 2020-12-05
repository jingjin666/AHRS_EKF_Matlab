% NED
function [roll, pitch, yaw] = acc2euler(acce_data, mag_data)
    roll = atan2(acce_data(2)/norm(acce_data), acce_data(3)/norm(acce_data));
	pitch = -asin(acce_data(1)/norm(acce_data));
	r1 = -mag_data(2)*cos(roll) + mag_data(3)*sin(roll);
	r2 = mag_data(1)*cos(pitch) + mag_data(2)*sin(pitch)*sin(roll) + mag_data(3)*sin(pitch)*cos(roll);
	yaw = atan2(r1, r2);
end