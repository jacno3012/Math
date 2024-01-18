clc; clear all; close all;
% Các thông số đầu vào
q_init = [1, 0, 0, 0]; % Quaternion ban đầu
q_target = [0, 1, 0, 0]; % Quaternion mục tiêu
t_step = 0.01; % Thời gian giữa các bước
t_total = 1; % Thời gian hoàn thành phép xoay

% Tách quá trình xoay thành các phép xoay nhỏ
num_steps = t_total / t_step; % Số lượng bước nhỏ
t_step = linspace(0, t_total, num_steps+1); % Mảng thời gian nhỏ

% Tính toán quaternion tương ứng với mỗi bước thời gian nhỏ
quaternions = zeros(num_steps+1, 4); % Mảng chứa các quaternion
for i = 1:num_steps+1
    t_current = t_step(i);
    % Tính toán quaternion tương ứng với thời gian hiện tại
    % Sử dụng thuật toán nội suy giữa q_init và q_target
    q_interpolated = slerp(q_init, q_target, t_current/t_total,0);
    quaternions(i, :) = q_interpolated;
end

% Hiển thị sự thay đổi hướng trên giao diện đồ họa
figure;
hold on;

% Vẽ trục tọa độ ban đầu
quiver3(0, 0, 0, q_init(2), q_init(3), q_init(4), 'b', 'LineWidth', 2);
quiver3(0, 0, 0, q_init(1), q_init(3), -q_init(4), 'r', 'LineWidth', 2);
quiver3(0, 0, 0, q_init(1), q_init(2), q_init(4), 'g', 'LineWidth', 2);

% Vẽ trục tọa độ mục tiêu
quiver3(0, 0, 0, q_target(2), q_target(3), q_target(4), 'b--', 'LineWidth', 2);
quiver3(0, 0, 0, q_target(1), q_target(3), -q_target(4), 'r--', 'LineWidth', 2);
quiver3(0, 0, 0, q_target(1), q_target(2), q_target(4), 'g--', 'LineWidth', 2);

% Vẽ trục tọa độ tương ứng với từng bước thời gian
for i = 1:num_steps+1
    quiver3(0, 0, 0, quaternions(i, 2), quaternions(i, 3), quaternions(i, 4), 'b');
    quiver3(0, 0, 0, quaternions(i, 1), quaternions(i, 3), -quaternions(i, 4), 'r');
    quiver3(0, 0, 0, quaternions(i, 1), quaternions(i, 2), quaternions(i, 4), 'g');
end

% Đặt tên cho trục tọa độ
xlabel('X');
ylabel('Y');
zlabel('Z');

% Hiển thị lưới
grid on;

% Tùy chỉnh khung nhìn
axis equal;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);

% Hiển thị
title('Thay đổi hướng của trục tọa độ trong quá trình quay');
hold off;