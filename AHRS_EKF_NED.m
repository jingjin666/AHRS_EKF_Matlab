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
data_num=round(size(sensor_data,1));
j=0;
for i=1:data_num
    j=j+1;
    DATA_CNTR(j,1) = i;
    X_GYRO(j,1) = mean(sensor_data(i,1));  %单位：rad
    Y_GYRO(j,1) = mean(sensor_data(i,2));
    Z_GYRO(j,1) = mean(sensor_data(i,3));
    X_ACCL(j,1) = mean(sensor_data(i,4));  %单位：g
    Y_ACCL(j,1) = mean(sensor_data(i,5));
    Z_ACCL(j,1) = mean(sensor_data(i,6));
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
a_measure_noise = 0.03;
R = a_measure_noise * eye(3);
%disp(R)

%根据加速度计初始化欧拉角(弧度)和四元数
[roll_init, pitch_init, yaw_init] = acc2euler(acc(1, :), mag(1, :));
q_init = zeros(4, 1);
[q_init(1), q_init(2), q_init(3), q_init(4)]= euler2quat(roll_init, pitch_init, yaw_init);
%打印初始欧拉角单位(度)
[roll_init, pitch_init, yaw_init] = quat2euler(q_init);
disp("初始化欧拉角");
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

estimate_roll = zeros(L, 1);
acc_roll = zeros(L, 1);
estimate_pitch = zeros(L, 1);
acc_pitch = zeros(L, 1);
estimate_yaw = zeros(L, 1);
acc_yaw = zeros(L, 1);

%% EKF滤波
for k=1:L
    T = abs(GAP_TIME(k,1));
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end
    
    %% 预测过程
    %真实角速度
    w_truth = [gyro(k, 1), gyro(k, 2), gyro(k, 3)];
    
    %Ω矩阵4x4
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
    
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = gyro_bias;
    
    %状态预测
    next_state_vector = [q_new; w_bias_new];
    
    %归一化
    q_normalize = sqrt(next_state_vector(1)*next_state_vector(1) + next_state_vector(2)*next_state_vector(2) + next_state_vector(3)*next_state_vector(3) + next_state_vector(4)*next_state_vector(4));
    next_state_vector(1) =  next_state_vector(1) / q_normalize;
    next_state_vector(2) =  next_state_vector(2) / q_normalize;
    next_state_vector(3) =  next_state_vector(3) / q_normalize;
    next_state_vector(4) =  next_state_vector(4) / q_normalize;

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
    hk = 2 * [-next_state_vector(3), next_state_vector(4), -next_state_vector(1), next_state_vector(2);
            next_state_vector(2), next_state_vector(1), next_state_vector(4), next_state_vector(3);
            next_state_vector(1), -next_state_vector(2), -next_state_vector(3), next_state_vector(4)];
    H = [hk, zeros(3)];
   
    %计算残差协方差
    SP = H * P_next * H' + R;
    SP_inv = inv(SP);
    
    %计算卡尔曼增益
    K = P_next * H'* SP_inv;
    
    %计算残差
    %归一化  
    acc_normalize = sqrt(acc(k, 1)*acc(k, 1) + acc(k, 2)*acc(k, 2) + acc(k, 3)*acc(k, 3));
    acc_x = acc(k, 1) / acc_normalize;
    acc_y = acc(k, 2) / acc_normalize;
    acc_z = acc(k, 3) / acc_normalize;
    Z = [acc_x;acc_y;acc_z];
    S = Z - H * next_state_vector;
    
    %更新状态矩阵
    state_vector = next_state_vector + K * S;
    %归一化
    q_normalize = sqrt(state_vector(1)*state_vector(1) + state_vector(2)*state_vector(2) + state_vector(3)*state_vector(3) + state_vector(4)*state_vector(4));
    state_vector(1) =  state_vector(1) / q_normalize;
    state_vector(2) =  state_vector(2) / q_normalize;
    state_vector(3) =  state_vector(3) / q_normalize;
    state_vector(4) =  state_vector(4) / q_normalize;
    
    %更新协方差矩阵
    P = (eye(7) - K * H) * P_next;
    
    %% 输出估计后的姿态
    estimate_q = [state_vector(1), state_vector(2), state_vector(3), state_vector(4)];
    [estimate_roll(k, 1), estimate_pitch(k, 1), estimate_yaw(k, 1)] = quat2euler(estimate_q);
    
    [acc_roll(k, 1), acc_pitch(k, 1), acc_yaw(k, 1)] = acc2euler(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    acc_yaw(k) = acc_yaw(k, 1) * rad2deg;
end

%% 画图
figure;
plot(Time,estimate_roll,Time,acc_roll);
legend('roll-ekf','roll-acc','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('roll','FontSize',20)
title('roll','FontSize',20);

figure;
plot(Time,estimate_pitch,Time,acc_pitch);
legend('pitch-ekf','pitch-acc','FontSize',10);
xlabel('t / s','FontSize',20)
ylabel('pitch','FontSize',20)
title('pitch','FontSize',20);




