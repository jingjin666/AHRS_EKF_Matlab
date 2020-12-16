%NED
function [w, x, y, z] = euler2quat(roll, pitch, yaw)
	cYaw = cos(yaw / 2);
    sYaw = sin(yaw / 2);
	cPit = cos(pitch / 2);
    sPit = sin(pitch / 2);
	cRol = cos(roll / 2);
    sRol = sin(roll / 2);

	w = cRol*cPit*cYaw + sRol*sPit*sYaw;
	x = sRol*cPit*cYaw - cRol*sPit*sYaw;
	y = cRol*sPit*cYaw + sRol*cPit*sYaw;
	z = cRol*cPit*sYaw - sRol*sPit*cYaw;
end
