close all; clear all; clc;
% Chuyển toàn bộ về tham chiếu toàn cục

%% XÁC ĐỊNH CÁC TRỤC TỌA ĐỘ

% Toàn cục
% Hệ tọa độ toàn cục tại điểm [0,0,0] với trục z hướng lên, quaternion chuẩn là q_ref_Global = [1 0 0 0] 
Global_User = [0, 0, 0];
Global_x = [1, 0, 0];
Global_y = [0, 1, 0];
Global_z = [0, 0, 1];

% Gốc - (hệ tọa độ trung gian)
% Hệ tọa độ gốc (hệ tọa độ trung gian) tham chiếu đến hệ tọa độ toàn cục
User_Global_Position = [2, 2, 2];% Vị trí hiện tại của hệ tọa độ gốc trong hệ tọa độ toàn cục
theta_x_0  = 0*pi/180;
theta_y_0  = 0*pi/180;
theta_z_0  = 0*pi/180;
% User_Global_Rotation = eul2quat([theta_x_0 theta_y_0 theta_z_0], 'XYZ');
User_Global_Rotation = [0, 1, 0, 0]; % Chọn Hướng hiện tại của hệ tọa độ gốc trong hệ tọa độ toàn cục trùng vs toàn cục
User_Global_Rotation = quatnormalize(User_Global_Rotation); % Chuẩn hóa quaternions đã chọn
User_x = quatrotate(User_Global_Rotation, Global_x);
User_y = quatrotate(User_Global_Rotation, Global_y);
User_z = quatrotate(User_Global_Rotation, Global_z);

% Vật thể 
% Hệ tọa độ vật thể tham chiếu đến hệ tọa độ gốc

%------------------------------Nhập vị trí bắt đầu của vật thể-------------------------------%
Object_User_Position = [1, 1, 1]; % Vị trí hiện tại của vật thể trong hệ tọa độ gốc
Object_User_Rotation = [1, 0, 0, 0]; % Hướng hiện tại của vật thể trong hệ tọa độ gốc
%____________________________________________________________________________________________%

Object_User_Rotation = quatnormalize(Object_User_Rotation); % Chuẩn hóa quaternions
% Hệ tọa độ vật thể tham chiếu đến hệ tọa độ toàn cục
Object_User_Position = quatrotate(User_Global_Rotation, Object_User_Position);
Object_Global_Position = Object_User_Position + User_Global_Position;   % Vị trí hiện tại của vật thể về hệ tọa độ toàn cục

Object_Global_Rotation = quatmultiply(User_Global_Rotation, Object_User_Rotation); % Hướng hiện tại của vật thể về hệ tọa độ toàn cục
%Object_Global_Rotation = quatmultiply(quatconj(User_Global_Rotation),Object_User_Rotation); % Hướng hiện tại của vật thể về hệ tọa độ toàn cục

% Object_x = quatrotate(Object_User_Rotation, User_x);
% Object_y = quatrotate(Object_User_Rotation, User_y);
% Object_z = quatrotate(Object_User_Rotation, User_z);
Object_x = quatrotate(Object_Global_Rotation, Global_x);
Object_y = quatrotate(Object_Global_Rotation, Global_y);
Object_z = quatrotate(Object_Global_Rotation, Global_z);

% Vẽ các hệ tọa độ
figure;
hold on;
quiver3(Global_User(1), Global_User(2), Global_User(3), Global_x(1), Global_x(2), Global_x(3), 'r');
quiver3(Global_User(1), Global_User(2), Global_User(3), Global_y(1), Global_y(2), Global_y(3), 'g');
quiver3(Global_User(1), Global_User(2), Global_User(3), Global_z(1), Global_z(2), Global_z(3), 'b');

quiver3(User_Global_Position(1), User_Global_Position(2), User_Global_Position(3), User_x(1), User_x(2), User_x(3), 'r');
quiver3(User_Global_Position(1), User_Global_Position(2), User_Global_Position(3), User_y(1), User_y(2), User_y(3), 'g');
quiver3(User_Global_Position(1), User_Global_Position(2), User_Global_Position(3), User_z(1), User_z(2), User_z(3), 'b');

quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_x(1), Object_x(2), Object_x(3), 'r');
quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_y(1), Object_y(2), Object_y(3), 'g');
quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_z(1), Object_z(2), Object_z(3), 'b');

% % Đặt các giới hạn trục tọa độ
% xlim([-20, 20]);
% ylim([-20, 20]);
% zlim([-20, 20]);

% Hiển thị dưới dạng xem 3D
view(3);
axis equal; 
grid on;

% Đặt tên cho biểu đồ
title('Hệ tọa độ');

%% QUY HOẠCH CHUYỂN ĐỘNG
% Vị trí hiện tại của vật thể tham chiếu về hệ trục tạo độ toàn cục
p_init = Object_Global_Position;
q_init = Object_Global_Rotation;

%---------------------------Nhập vị trí đến------------------------%
    % Vị trí đích đến của vật thể tham chiếu về hệ trục tọa độ gốc
    p_End  = [5, 5, -5];
    theta_x_End  = 90*pi/180;
    theta_y_End  =  0*pi/180;
    theta_z_End  =  0*pi/180;
%     q_End = eul2quat([theta_x_End theta_y_End theta_z_End],'XYZ')
    q_End  = [ -0.7071, 0.7071, 0, 0];
    quatconj(q_End);
%__________________________________________________________________%

% Vị trí đích đến của vật thể tham chiếu về hệ trục tọa độ toàn cục
q_End = quatnormalize(q_End); % Chuẩn hóa quaternions
Object_Global_Position_Target = User_Global_Position + quatrotate(User_Global_Rotation, p_End);

Object_Global_Rotation_Target = quatmultiply(User_Global_Rotation,q_End);
%Object_Global_Rotation_Target = quatmultiply(quatconj(User_Global_Rotation),q_End);

p_Target  = Object_Global_Position_Target;
q_Target  = Object_Global_Rotation_Target;

%---------------------------Nhập thông số------------------------%
    % Thông số quá trình di chuyển
    t_s = 0.1; % Thời gian giữa các bước
    t_total = 2; % Thời gian hoàn thành 
%________________________________________________________________%

num_steps = t_total / t_s; % Số lượng bước nhỏ
t_step = linspace(0, t_total, num_steps+1); % Mảng thời gian nhỏ

% Tính toán quaternion tương ứng với mỗi bước thời gian nhỏ
for i = 1:num_steps+1   
    t_current = t_step(i);  
    t = t_current/t_total;      

    % Tính toán vị trí tương ứng với thời gian hiện tại
    % Sử dụng thuật toán nội suy giữa p_init và p_Target
    p_interpolated = (1-t) * p_init + t * p_Target;
    Object_Global_Position = p_interpolated;

    % Tính toán quaternion tương ứng với thời gian hiện tại
    % Sử dụng thuật toán Slerp nội suy giữa q_init và q_Target
    q_interpolated = slerp(q_init, q_Target, t ,0);

    % Xoay vector x,y,z
    Object_x = quatrotate(q_interpolated, Global_x);
    Object_y = quatrotate(q_interpolated, Global_y);
    Object_z = quatrotate(q_interpolated, Global_z);
    % Vẽ vector x,y,z
    quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_x(1), Object_x(2), Object_x(3), 'r');
    quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_y(1), Object_y(2), Object_y(3), 'g');
    quiver3(Object_Global_Position(1), Object_Global_Position(2), Object_Global_Position(3), Object_z(1), Object_z(2), Object_z(3), 'b');
    pause(t_s/10);
end