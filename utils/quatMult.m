% 测试欧拉角到四元数之间的转换
% addpath('utils'); %函数路径
 
syms roll0   % x
syms pitch0  % y
syms yaw0    % z 
q_x=[cos(roll0/2),1*sin(roll0/2),0,0]; % x轴：[1,0,0] 绕x轴旋转roll0
q_y=[cos(pitch0/2),0,1*sin(pitch0/2),0];   % y轴：[0,1,0] 绕y轴旋转pitch0
q_z=[cos(yaw0/2),0,0,1*sin(yaw0/2)];     % z轴：[0,0,1] 绕z轴旋转yaw0
 
q=quaternProd(quaternProd(q_z, q_y),q_x)

% q的最终结果
% cos(pitch0/2)*cos(roll0/2)*cos(yaw0/2) - sin(pitch0/2)*sin(roll0/2)*sin(yaw0/2),
% cos(roll0/2)*cos(yaw0/2)*sin(pitch0/2) - cos(pitch0/2)*sin(roll0/2)*sin(yaw0/2), 
% cos(pitch0/2)*cos(yaw0/2)*sin(roll0/2) + cos(roll0/2)*sin(pitch0/2)*sin(yaw0/2), 
% cos(pitch0/2)*cos(roll0/2)*sin(yaw0/2) + cos(yaw0/2)*sin(pitch0/2)*sin(roll0/2)]
 
 
 
% 下面函数是四元数乘法
% 四元数相乘
function ab = quaternProd(a, b)
 
    ab(1) = a(1).*b(1)-a(2).*b(2)-a(3).*b(3)-a(4).*b(4);
    ab(2) = a(1).*b(2)+a(2).*b(1)+a(3).*b(4)-a(4).*b(3);
    ab(3) = a(1).*b(3)-a(2).*b(4)+a(3).*b(1)+a(4).*b(2);
    ab(4) = a(1).*b(4)+a(2).*b(3)-a(3).*b(2)+a(4).*b(1);
    
end
 