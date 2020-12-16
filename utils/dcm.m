%% Å·À­½ÇÓëÐý×ª¾ØÕó

clc;
clear;
close all;

syms cosRoll sinRoll cosPitch sinPitch cosYaw sinYaw real

%% ENU
Rz_enu = [cosYaw,-sinYaw,0;
        sinYaw,cosYaw,0;
        0,0,1];
Ry_enu = [cosRoll,0,sinRoll;
        0,1,0;
        -sinRoll,0,cosRoll];
Rx_enu = [1,0,0;
        0,cosPitch,-sinPitch;
        0,sinPitch,cosPitch];
R_ZXY_b2n = Rz_enu * Rx_enu * Ry_enu;
%disp(R_ZXY_b2n);


%% NED
Rz_ned = [cosYaw,-sinYaw,0;
        sinYaw,cosYaw,0;
        0,0,1];
Ry_ned = [cosPitch,0,sinPitch;
        0,1,0;
        -sinPitch,0,cosPitch];
Rx_ned = [1,0,0;
        0,cosRoll,-sinRoll;
        0,sinRoll,cosRoll];
R_ZYX_b2n = Rz_ned * Ry_ned * Rx_ned;
disp(R_ZYX_b2n);

syms Mx My Mz
Mag_n = [Mx;My;Mz];
%disp(Mag_b);
