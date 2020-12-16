% 基于EKF的姿态估计算法
% 导航坐标系:东北天坐标系(ENU)
% IMU:16375
% 载体坐标系:右前上(XYZ)
% 旋转方式:Z-X-Y(偏航-俯仰-横滚)右手定则

clc;
clear;
close all;

addpath('utils');
addpath('datafiles');

rad2deg = 180/pi;
deg2rad = pi/180;

%% 读取IMU原始数据
filename = 'session.txt';
sensor_data = load(filename);
gap=1;
data_num=round(size(sensor_data,1));
j=0;
for i=1:gap:data_num-gap
    j=j+1;
    DATA_CNTR(j,1) = i;
    X_GYRO(j,1) = mean(sensor_data(i:i+gap-1,1));
    Y_GYRO(j,1) = mean(sensor_data(i:i+gap-1,2));
    Z_GYRO(j,1) = mean(sensor_data(i:i+gap-1,3));
    X_ACCL(j,1) = mean(sensor_data(i:i+gap-1,4));
    Y_ACCL(j,1) = mean(sensor_data(i:i+gap-1,5));
    Z_ACCL(j,1) = mean(sensor_data(i:i+gap-1,6));
    X_MAG(j,1) = mean(sensor_data(i:i+gap-1,7));
    Y_MAG(j,1) = mean(sensor_data(i:i+gap-1,8));
    Z_MAG(j,1) = mean(sensor_data(i:i+gap-1,9));
end

%% 初始化数据
L = size(DATA_CNTR,1);
Time=zeros(L,1);
Time(1,1)=0.01;

acc = [X_ACCL, Y_ACCL, Z_ACCL];
gyro_bias = [0.01, 0.01, 0.01];
gyro = [X_GYRO - gyro_bias(1), Y_GYRO - gyro_bias(2), Z_GYRO - gyro_bias(3)];
mag = [X_MAG, Y_MAG, Z_MAG];

%角速度过程噪声,四元数噪声
w_process_noise = 0.0001 * ones(1, 4);
%角速度偏移过程噪声
w_bias_process_noise = 0.003 * ones(1,3);
%过程噪声矩阵
Q = diag([w_process_noise, w_bias_process_noise]);
%disp(Q);

%加速度计测量噪声
a_measure_noise = 0.03 * ones(1, 3);
%磁力计测量噪声
m_measure_noise = 1e-5 * ones(1, 3);
%测量噪声矩阵
R = diag(a_measure_noise);
%disp(R)

%状态矩阵
X = zeros(L, 7);
%初始化状态矩阵
[pitch_init, roll_init, yaw_init] = acc2euler_ENU(acc(1, :), mag(1, :));
%初始化角度
pitchEKF(1,1) = pitch_init*rad2deg;
rollEKF(1,1) = roll_init*rad2deg;
yawEKF(1,1) = yaw_init*rad2deg;
disp(pitchEKF);
disp(rollEKF);
disp(yawEKF);

%初始化四元数
[q1, q2, q3, q4] = euler2quat_ENU(pitch_init, roll_init, yaw_init);

q_init = [q1, q2, q3, q4];
[pitch_init1,roll_init1,yaw_init1] = quat2euler_ENU(q_init);
disp(pitch_init1);
disp(roll_init1);
disp(yaw_init1);

X(1,:) = [q1, q2, q3, q4, gyro_bias];

%协方差矩阵
Pk = zeros(7, 7, L);
Pk(:,:,1) = eye(7); 
%disp(P);

%观测矩阵
Z = zeros(L, 3);

%delta_t
T = 0.01;

att = zeros(L, 3);
acc_att = zeros(L, 3);

%% EKF滤波
for k=1:L
    Time(k+1,1)=Time(k)+T; 
    
    [pitch_, roll_, yaw_] = acc2euler_ENU(acc(k,:), mag(k,:));
    acc_att(k,:) = [pitch_*rad2deg, roll_*rad2deg, yaw_*rad2deg];
    
    %加速度计/磁力计数据归一化
    acc(k, :) = acc(k, :) / norm(acc(k, :), 2);
    mag(k, :) = mag(k, :) / norm(mag(k, :), 2);
    Z(k, :) = acc(k, :);
    
    %% 预测过程
    %真实角速度
    gyro_x_bias = gyro(k,1) - X(k,5);
    gyro_y_bias = gyro(k,2) - X(k,6);
    gyro_z_bias = gyro(k,3) - X(k,7);
    
    A_11=[1,-(T/2)*gyro_x_bias,-(T/2)*gyro_y_bias,-(T/2)*gyro_z_bias;
            (T/2)*gyro_x_bias,1,(T/2)*gyro_z_bias,-(T/2)*gyro_y_bias;
            (T/2)*gyro_y_bias,-(T/2)*gyro_z_bias,1,(T/2)*gyro_x_bias;
            (T/2)*gyro_z_bias,(T/2)*gyro_y_bias,-(T/2)*gyro_x_bias,1];
    
    A_12=[0,0,0;
          0,0,0;
          0,0,0;
          0,0,0]; 
    A_21=[0,0,0,0;
          0,0,0,0;
          0,0,0,0];
    A_22=[1,0,0;
          0,1,0;
          0,0,1];   
    A=[A_11,A_12;A_21,A_22];
    
    Ak = eye(7)+T/2*...
    [0    -(gyro_x_bias)  -(gyro_y_bias) -(gyro_z_bias)   X(k,2) X(k,3)  X(k,4);
    (gyro_x_bias) 0    (gyro_z_bias)  -(gyro_y_bias)   -X(k,1) X(k,4)  -X(k,3);
    (gyro_y_bias) -(gyro_z_bias)  0  (gyro_x_bias)      -X(k,4) -X(k,1) X(k,2);
    (gyro_z_bias) (gyro_y_bias)   -(gyro_x_bias)  0     X(k,3) -X(k,2) -X(k,1);
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0];
    
    X_ = (A*X(k,:)')';  
    X_(1:4) = X_(1:4)/norm(X_(1:4),2); 
    
    Pk_ = Ak * Pk(:,:,k) * Ak' + Q; 
    hk = [2*(X_(2)*X_(4)-X_(1)*X_(3)) 2*(X_(1)*X_(2)+X_(3)*X_(4)) X_(1)^2+X_(4)^2-X_(2)^2-X_(3)^2];
    
    Hk = 2*[-X_(3)  X_(4) -X_(1)  X_(2) 0 0 0;
               X_(2)  X_(1)  X_(4)  X_(3) 0 0 0;
               X_(1) -X_(2) -X_(3)  X_(4) 0 0 0;];

    Kk = Pk_ * Hk' * ((Hk * Pk_ * Hk' + R)^(-1));  
    X(k+1,:) = (X_' + Kk * (Z(k,:) - hk)')';      
    X(k+1,1:4) = X(k+1,1:4)/norm(X(k+1,1:4),2);  
    Pk(:,:,k+1) = (eye(7) - Kk*Hk) * Pk_;
    
    q=[X(k,1),X(k,2),X(k,3),X(k,4)];
    
    [pitchEKF(k,1),rollEKF(k,1),yawEKF(k,1)] = quat2euler_ENU(q);
end

%% 画图
figure;
acc_roll = acc_att(:,2);

rollEKF(3000) = 0;
acc_roll(3000) = 0;

plot(Time(16:2000),acc_roll(16:2000),Time(16:2000),rollEKF(16:2000));
legend('roll-acc','roll-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('roll','FontSize',20)
title('roll','FontSize',20);


figure;
acc_pitch = acc_att(:,1);

pitchEKF(3000) = 0;
acc_pitch(3000) = 0;

plot(Time(16:2000),acc_pitch(16:2000),Time(16:2000),pitchEKF(16:2000));
legend('pitch-acc','pitch-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('pitch','FontSize',20)
title('pitch','FontSize',20);

figure;
mag_yaw = acc_att(:,3);

yawEKF(3000) = 0;
mag_yaw(3000) = 0;

plot(Time(16:2000),mag_yaw(16:2000),Time(16:2000),yawEKF(16:2000));
legend('yaw-mag','yaw-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('yaw','FontSize',20)
title('yaw','FontSize',20);