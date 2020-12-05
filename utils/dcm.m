clc;
clear;
close all;

syms cosRoll sinRoll cosPitch sinPitch cosYaw sinYaw real


Rz_NED = [cosYaw,sinYaw,0;
        -sinYaw,cosYaw,0;
        0,0,1];
Ry_NED = [cosPitch,0,-sinPitch;
    0,1,0;
    sinPitch,0,cosPitch];

Rx_NED = [1,0,0;
        0,cosRoll,sinRoll;
        0,-sinRoll,cosRoll];
    
Rz_ENU = [cosYaw,sinYaw,0;
        -sinYaw,cosYaw,0;
        0,0,1];
Ry_ENU = [cosPitch,0,-sinPitch;
    0,1,0;
    sinPitch,0,cosPitch];

Rx_ENU = [1,0,0;
        0,cosRoll,sinRoll;
        0,-sinRoll,cosRoll];

%%ENU
R_ENU_ZXY = Ry_ENU * Rx_ENU * Rz_ENU;    
disp(R_ENU_ZXY);
    
%%NED
R_NED_ZYX_N2B = Rx_NED * Ry_NED * Rz_NED;
disp(R_NED_ZYX_N2B);


%%mag
C_nb = Rx_NED * Ry_NED * eye(3);
disp(C_nb);

syms Mx My Mz
Mag_n = [Mx;My;Mz];
Mag_b = C_nb * Mag_n;
disp(Mag_b);
