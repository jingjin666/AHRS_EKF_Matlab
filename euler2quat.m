%NED
function [q1, q2, q3, q4] = euler2quat(roll, pitch, yaw)
	cYaw = cos(yaw / 2);
    sYaw = sin(yaw / 2);
	cPit = cos(pitch / 2);
    sPit = sin(pitch / 2);
	cRol = cos(roll / 2);
    sRol = sin(roll / 2);

	q1 = cRol*cPit*cYaw + sRol*sPit*sYaw;
	q2 = sRol*cPit*cYaw - cRol*sPit*sYaw;
	q3 = cRol*sPit*cYaw + sRol*cPit*sYaw;
	q4 = cRol*cPit*sYaw - sRol*sPit*cYaw;
end

% %ENU
% function [w,x,y,z]=euler2quat( pitch,roll,yaw)         
% w = cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
% x = cos(roll/2)*cos(yaw/2)*sin(pitch/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2); 
% y = cos(pitch/2)*cos(yaw/2)*sin(roll/2) + cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
% z = cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);
