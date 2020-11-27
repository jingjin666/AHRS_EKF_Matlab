% 基于EKF的姿态估计算法
% 导航坐标系:北东地坐标系(NED)
% IMU:16375
% 载体坐标系:前右下(XYZ)
% 旋转方式:Z-Y-X(偏航-俯仰-横滚)右手定则

clc;
clear;
close all;
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
Time(1,1)=0;

acc = [X_ACCL, Y_ACCL, Z_ACCL];
gyro_bias = [0.01; 0.01; 0.01];
gyro_bias = gyro_bias * deg2rad;
gyro = [X_GYRO - gyro_bias(1), Y_GYRO - gyro_bias(2), Z_GYRO - gyro_bias(3)];
mag = [X_MAG, Y_MAG, Z_MAG];

%角速度过程噪声,四元数噪声
w_process_noise = 1e-6 * ones(1, 4);

%角速度偏移过程噪声
w_bias_process_noise = 1e-8 * ones(1,3);

%过程噪声矩阵
Q = diag([w_process_noise, w_bias_process_noise]);
%disp(Q);

%加速度计测量噪声
a_measure_noise = 1e-1;
R = a_measure_noise * eye(3);
%disp(R)

%状态矩阵
state_vector = zeros(7, 1);
%初始化状态矩阵
[roll_init, pitch_init, yaw_init] = acc2euler(acc(1, 1), acc(1, 2), acc(1, 3), mag(1, 1), mag(1, 2), mag(1, 3));
%disp(roll_init);
%disp(pitch_init);
%disp(yaw_init);
q_init = zeros(4, 1);
[q_init(1), q_init(2), q_init(3), q_init(4)]= quatfromeuler(roll_init, pitch_init, yaw_init);
state_vector = [q_init; gyro_bias];

%协方差矩阵
p1 = 1 * ones(1, 4);
p2 = 0 * ones(1,3);
P = diag([p1, p2]);
%disp(P);

%delta_t
T = 0.01;

L = 2;
%% EKF滤波
for k=1:L-1
    %% 预测过程
    %真实角速度
    w_truth = [gyro(1), gyro(2), gyro(3)];
    
    %Ω矩阵4x4
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
    
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = gyro_bias;
    
    %状态预测
    new_state_vector = [q_new; w_bias_new];
    
    %line矩阵4x3
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
        
    %状态转移矩阵F
    Ak = [omega, line;
            zeros(3, 4), eye(3)];
    F = eye(7) + 0.5 * T * Ak;
    
    %协方差矩阵预测
    P_next = F * P * F' + Q;
    
    %% 更新过程
    %计算观测矩阵H
    

    
end




