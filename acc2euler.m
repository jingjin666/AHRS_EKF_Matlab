function [roll, pitch, yaw] = acc2euler(acc_x, acc_y, acc_z, mag_x, mag_y, mag_z)
	pitch = atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z));
	roll = -atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z));
	r1 = -mag_y*cos(roll) + mag_z*sin(roll);
	r2 = mag_x*cos(pitch) + mag_y*sin(pitch)*sin(roll) + mag_z*sin(pitch)*cos(roll);
	yaw = atan2(r1, r2);
end