% 基于EKF的姿态估计算法
% 导航坐标系:北东地坐标系(NED)
% IMU:16375
% 载体坐标系:前右下(XYZ)
% 旋转方式:Z-Y-X(偏航-俯仰-横滚)右手定则

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
data_num=round(size(sensor_data,1));
j=0;
for i=1:data_num
    j=j+1;
    DATA_CNTR(j,1) = i;
    X_GYRO(j,1) = mean(sensor_data(i,1));  %单位：rad
    Y_GYRO(j,1) = mean(sensor_data(i,2));
    Z_GYRO(j,1) = mean(sensor_data(i,3));
    X_ACCL(j,1) = mean(sensor_data(i,4));  %单位：g
    Y_ACCL(j,1) = -mean(sensor_data(i,5));
    Z_ACCL(j,1) = -mean(sensor_data(i,6));
    X_MAG(j,1) = mean(sensor_data(i,7));
    Y_MAG(j,1) = mean(sensor_data(i,8));
    Z_MAG(j,1) = mean(sensor_data(i,9));
    GAP_TIME(j,1) = mean(sensor_data(i,10));  %单位：s
end

%% 初始化数据
L = size(DATA_CNTR,1);
Time = zeros(L,1);
Time(1, 1) = 0.01;

acc = [X_ACCL, Y_ACCL, Z_ACCL];
%初始陀螺仪偏移
gyro_bias = [0.01; 0.01; 0.01];
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
%测量噪声矩阵
R = diag(a_measure_noise);
%disp(R)

%根据加速度计初始化欧拉角(弧度)和四元数
[roll_init, pitch_init, yaw_init] = acc2euler(acc(1, :), mag(1, :));
roll_init1 = roll_init * rad2deg;
pitch_init1 = pitch_init * rad2deg;
yaw_init1 = yaw_init * rad2deg;
disp("加速度计/磁力计初始化欧拉角");
disp(roll_init1);
disp(pitch_init1);
disp(yaw_init1);

% q_test = angle2quat(yaw_init, pitch_init, roll_init);
% disp(q_test);
% [yaw_test, pitch_test, roll_test] = quat2angle(q_test);

q_init = zeros(4, 1);
[q_init(1), q_init(2), q_init(3), q_init(4)]= euler2quat(roll_init, pitch_init, yaw_init);
%disp(q_init);
%打印初始欧拉角单位(度)
[roll_init, pitch_init, yaw_init] = quat2euler(q_init);
disp("四元数初始化欧拉角");
disp(roll_init);
disp(pitch_init);
disp(yaw_init);

%状态矩阵初始化
state_vector = [q_init; gyro_bias];

%协方差矩阵
p1 = 1 * ones(1, 4);
p2 = 0 * ones(1, 3);
P = diag([p1, p2]);
%disp(P);

%观测矩阵
Z = zeros(L, 3);

estimate_roll = zeros(L, 1);
acc_roll = zeros(L, 1);
estimate_pitch = zeros(L, 1);
acc_pitch = zeros(L, 1);
estimate_yaw = zeros(L, 1);
mag_yaw = zeros(L, 1);

%% EKF滤波
for k=1:L
    T = abs(GAP_TIME(k,1));
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end
    
     %% 根据加速度计和磁力计得到姿态角
    [acc_roll(k, 1), acc_pitch(k, 1), mag_yaw(k, 1)] = acc2euler(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    mag_yaw(k) = mag_yaw(k, 1) * rad2deg;
    
    %归一化观测数据  
    Z(k, :) = acc(k, :) / norm(acc(k, :));
    
    %% 预测过程
    %真实角速度
    gyro_x_bias_current = state_vector(5);
    gyro_y_bias_current = state_vector(6);
    gyro_z_bias_current = state_vector(7);
    w_truth = [gyro(k, 1)-gyro_x_bias_current, gyro(k, 2)-gyro_y_bias_current, gyro(k, 3)-gyro_z_bias_current];
    
    %Ω矩阵4x4
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
      
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    %状态预测方程
    %q_k = q_k_1 + q^ * deltaT
    %q^ = 0.5 * omega * q_k-1;
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = [gyro_x_bias_current; gyro_y_bias_current; gyro_z_bias_current];
    next_state_vector = [q_new; w_bias_new];

    %归一化
    next_state_vector(1:4) = next_state_vector(1:4) / norm(next_state_vector(1:4));
    
    %line矩阵4x3
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
        
    %状态转移矩阵F
    Ak = [omega, line;
          zeros(3, 7)];
    F = eye(7) + 0.5 * T * Ak;
    
    %协方差矩阵预测
    P_next = F * P * F' + Q;
    
    %% 更新过程
    %计算观测矩阵H
    %hk全部反向了NED下*-g
    hk = 2 * [next_state_vector(3), -next_state_vector(4), next_state_vector(1), -next_state_vector(2);
            -next_state_vector(2), -next_state_vector(1), -next_state_vector(4), -next_state_vector(3);
            -next_state_vector(1), next_state_vector(2), next_state_vector(3), -next_state_vector(4)];
    H = [hk, zeros(3)];
   
    %计算残差协方差
    SP = H * P_next * H' + R;
    %SP_inv = inv(SP);
    SP_inv = SP^(-1);
    
    %计算卡尔曼增益
    K = P_next * H'* SP_inv;
    
    %计算残差
    S = Z(k, :)' - H * next_state_vector;
    
    %更新状态矩阵
    state_vector = next_state_vector + K * S;
    %归一化
    state_vector(1:4) = state_vector(1:4) / norm(state_vector(1:4));
    
    %更新协方差矩阵
    P = (eye(7) - K * H) * P_next;
    
    %% 输出估计后的姿态
    estimate_q = [state_vector(1), state_vector(2), state_vector(3), state_vector(4)];
    [estimate_roll(k, 1), estimate_pitch(k, 1), estimate_yaw(k, 1)] = quat2euler(estimate_q);
end

%% 画图
figure;
plot(Time,acc_roll,Time,estimate_roll);
legend('roll-acc','roll-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('roll','FontSize',20)
title('roll','FontSize',20);

figure;
plot(Time,acc_pitch,Time,estimate_pitch);
legend('pitch-acc','pitch-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('pitch','FontSize',20)
title('pitch','FontSize',20);

figure;
plot(Time,mag_yaw,Time,estimate_yaw);
legend('yaw-mag','yaw-ekf','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('yaw','FontSize',20)
title('yaw','FontSize',20);




