% ����EKF����̬�����㷨
% ��������ϵ:����������ϵ(NED)
% IMU:16375
% ��������ϵ:ǰ����(XYZ)
% ��ת��ʽ:Z-Y-X(ƫ��-����-���)���ֶ���

clc;
clear;
close all;

addpath('utils');
addpath('datafiles');

rad2deg = 180/pi;
deg2rad = pi/180;

%% ��ȡIMUԭʼ����
filename = 'session.txt';
sensor_data = load(filename);
data_num=round(size(sensor_data,1));
j=0;
for i=1:data_num
    j=j+1;
    DATA_CNTR(j,1) = i;
    X_GYRO(j,1) = mean(sensor_data(i,1));  %��λ��rad
    Y_GYRO(j,1) = mean(sensor_data(i,2));
    Z_GYRO(j,1) = mean(sensor_data(i,3));
    X_ACCL(j,1) = mean(sensor_data(i,4));  %��λ��g
    Y_ACCL(j,1) = -mean(sensor_data(i,5));
    Z_ACCL(j,1) = -mean(sensor_data(i,6));
    X_MAG(j,1) = mean(sensor_data(i,7));
    Y_MAG(j,1) = mean(sensor_data(i,8));
    Z_MAG(j,1) = mean(sensor_data(i,9));
    GAP_TIME(j,1) = mean(sensor_data(i,10));  %��λ��s
end

%% ��ʼ������
L = size(DATA_CNTR,1);
Time = zeros(L,1);
Time(1, 1) = 0.01;

acc = [X_ACCL, Y_ACCL, Z_ACCL];
%��ʼ������ƫ��
gyro_bias = [0.01; 0.01; 0.01];
gyro = [X_GYRO - gyro_bias(1), Y_GYRO - gyro_bias(2), Z_GYRO - gyro_bias(3)];
mag = [X_MAG, Y_MAG, Z_MAG];

%���ٶȹ�������,��Ԫ������
w_process_noise = 1e-6 * ones(1, 4);

%���ٶ�ƫ�ƹ�������
w_bias_process_noise = 1e-8 * ones(1,3);

%������������ 
Q = diag([w_process_noise, w_bias_process_noise]);
%disp(Q);

%���ٶȼƲ�������
a_measure_noise = 1e-1 * ones(1, 3);
m_measure_noise = 0.2 * ones(1, 3);
%������������
R = diag([a_measure_noise, m_measure_noise]);
%disp(R)

%���ݼ��ٶȼƳ�ʼ��ŷ����(����)����Ԫ��
[roll_init, pitch_init, yaw_init] = acc2euler(acc(1, :), mag(1, :));
roll_init1 = roll_init * rad2deg;
pitch_init1 = pitch_init * rad2deg;
yaw_init1 = yaw_init * rad2deg;
disp("���ٶȼ�/�����Ƴ�ʼ��ŷ����");
disp(roll_init1);
disp(pitch_init1);
disp(yaw_init1);

% q_test = angle2quat(yaw_init, pitch_init, roll_init);
% disp(q_test);
% [yaw_test, pitch_test, roll_test] = quat2angle(q_test);

q_init = zeros(4, 1);
[q_init(1), q_init(2), q_init(3), q_init(4)]= euler2quat(roll_init, pitch_init, yaw_init);
%disp(q_init);
%��ӡ��ʼŷ���ǵ�λ(��)
[roll_init, pitch_init, yaw_init] = quat2euler(q_init);
disp("��Ԫ����ʼ��ŷ����");
disp(roll_init);
disp(pitch_init);
disp(yaw_init);

%״̬�����ʼ��
state_vector = [q_init; gyro_bias];

%Э�������
p1 = 1 * ones(1, 4);
p2 = 1 * ones(1, 3);
P = diag([p1, p2]);
%disp(P);

%�۲����
Z = zeros(L, 6);

estimate_roll = zeros(L, 1);
acc_roll = zeros(L, 1);
estimate_pitch = zeros(L, 1);
acc_pitch = zeros(L, 1);

%% ����ǿ������˲�����

bx = 0.5500;% bxָ��
bz = 0.8351;% bzָ���
    
X1 = [yaw_init*deg2rad;0];
P1 = zeros(2);
%�Ƕȹ��������ͽ��ٶȹ�������
Q1 = diag([1e-6, 1e-8]);
R1 = 0.1;
%�۲�����
Z1 = zeros(L);
estimate_yaw = zeros(L, 1);
mag_yaw = zeros(L, 1);

%% EKF�˲�
for k=1:L
    T = abs(GAP_TIME(k,1));
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end
    
     %% ���ݼ��ٶȼƺʹ����Ƶõ���̬��
    [acc_roll(k, 1), acc_pitch(k, 1), mag_yaw(k, 1)] = acc2euler(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    mag_yaw(k) = mag_yaw(k, 1) * rad2deg;
    
    %��һ���۲�����  
    Z(k, :) = [acc(k, :) / norm(acc(k, :)), mag(k, :) / norm(mag(k, :))];
    
    %% Ԥ�����
    %��ʵ���ٶ�
    gyro_x_bias_current = state_vector(5);
    gyro_y_bias_current = state_vector(6);
    gyro_z_bias_current = state_vector(7);
    w_truth = [gyro(k, 1)-gyro_x_bias_current, gyro(k, 2)-gyro_y_bias_current, gyro(k, 3)-gyro_z_bias_current];
    
    %������4x4
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
      
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    %״̬Ԥ�ⷽ��
    %q_k = q_k_1 + q^ * deltaT
    %q^ = 0.5 * omega * q_k-1;
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = [gyro_x_bias_current; gyro_y_bias_current; gyro_z_bias_current];
    next_state_vector = [q_new; w_bias_new];

    %��һ��
    next_state_vector(1:4) = next_state_vector(1:4) / norm(next_state_vector(1:4));
    
    %line����4x3
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
        
    %״̬ת�ƾ���F
    Ak = [omega, line;
          zeros(3, 7)];
    F = eye(7) + 0.5 * T * Ak;
    
    %Э�������Ԥ��
    P_next = F * P * F' + Q;
    
    %% ���¹���
    %����۲����H
    %hkȫ��������NED��*[0, 0, -1]
    Hb1 = next_state_vector(1)*bx-next_state_vector(3)*bz;
    Hb2 = next_state_vector(2)*bx+next_state_vector(4)*bz;
    Hb3 = -next_state_vector(3)*bx-next_state_vector(1)*bz;
    Hb4 = -next_state_vector(4)*bx+next_state_vector(2)*bz;
    H = 2 * [next_state_vector(3), -next_state_vector(4),  next_state_vector(1), -next_state_vector(2), 0, 0, 0;
            -next_state_vector(2), -next_state_vector(1), -next_state_vector(4), -next_state_vector(3), 0, 0, 0;
            -next_state_vector(1),  next_state_vector(2),  next_state_vector(3), -next_state_vector(4), 0, 0, 0;
            Hb1,    Hb2,    Hb3,    Hb4,    0, 0, 0;
            Hb4,    -Hb3,   Hb2,    -Hb1,   0, 0, 0;
            -Hb3,   -Hb4,   Hb1,    Hb2,    0, 0, 0;];
        
    %����в�Э����
    SP = H * P_next * H' + R;
    %SP_inv = inv(SP);
    SP_inv = SP^(-1);
    
    %���㿨��������
    K = P_next * H'* SP_inv;
    
    X_ = next_state_vector(1:4);
    hk = [-2*(X_(2)*X_(4)-X_(1)*X_(3));
          -2*(X_(1)*X_(2)+X_(3)*X_(4));
          -(X_(1)^2+X_(4)^2-X_(2)^2-X_(3)^2);
          (X_(1)^2+X_(2)^2-X_(3)^2-X_(4)^2)*bx + 2*(X_(2)*X_(4) - X_(1)*X_(3))*bz;
          2*(X_(2)*X_(3)-X_(1)*X_(4))*bx + 2*(X_(1)*X_(2)-X_(3)*X_(4))*bz;
          2*(X_(1)*X_(3)+X_(2)*X_(4))*bx + (X_(1)^2-X_(2)^2-X_(3)^2+X_(4)^2)*bz];
    
    %����в�
    S = Z(k, :)' - hk;
    
    %����״̬����
    state_vector = next_state_vector + K * S;
    %��һ��
    state_vector(1:4) = state_vector(1:4) / norm(state_vector(1:4));
    
    %����Э�������
    P = (eye(7) - K * H) * P_next;
    
    %% ������ƺ����̬
    estimate_q = [state_vector(1), state_vector(2), state_vector(3), state_vector(4)];
    [estimate_roll(k, 1), estimate_pitch(k, 1), estimate_yaw(k, 1)] = quat2euler(estimate_q);
    
    
%     %% ����KF�˲�����ƫ����
%     %��_k=��_k-1 + (Gyro - w_bias_k-1)*delta_t
%     %w_bias_k = w_bias_k-1
%     A = [1, -T;
%          0, 1];
%     B = [T;0];
%     w_z = gyro(k, 3);
%     H1 = [1,0];
%     I = eye(2);
%     %% KFԤ��
%     
%     %״̬Ԥ��
%     X_next = A*X1 + B*w_z;
%     %Э����Ԥ��
%     P1_next = A*P1*A' + Q1;
%     
%     Z1(k) = getYaw(mag(k,:), estimate_roll(k, 1)*deg2rad, estimate_pitch(k, 1)*deg2rad);
%     %�в�
%     S1 = Z1(k) - H1*X_next;
%     
%     %����������
%     SP1 = H1 * P1_next * H1' + R1;
%     K1 = P1_next * H1' * SP1^(-1);
%     
%     %״̬����
%     X1 = X1 + K1*S1;
%     
%     %Э�������
%     P1 = (I - K1*H1)*P1_next;
%     
%     estimate_yaw(k, 1) = X1(1)*rad2deg;
end

% ��ͼ
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



