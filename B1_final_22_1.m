clear all; clc; 
t=0;
ts = 0.01;
LastCycle=0;
%% Khai bao cac bien nho
mode = 0;
v1_pre = 0;
S1_pre = 0;
v2_pre = 0;
S2_pre = 0;
v3_pre = 0;
S3_pre = 0;
%% Toa do diem dau
x_a   = 0; %[mm]
y_a   = 0;
z_a   = 0;
%% Toa do diem cuoi
x_b   = 300; %[mm]
y_b   = 300;
z_b   = 300;
%% Quan duong di duoc

AB_MD   = sqrt(((x_b-x_a)^2)+((y_b-y_a)^2)+((z_b-z_a)^2));

%% Khai bao gia tri ban dau
t=0;
x_0       = 0;            %% trang thai cua diem dau
x_dot_0   = 0;
x_2dot_0  = 0;

x_F       = AB_MD;        %% trang thai cua diem cuoi
x_dot_F   = 0;
x_2dot_F  = 0;

v_0 = x_dot_0;             %% van toc diem dau
v_f = x_dot_F;             %% van toc diem cuoi

v_qd  = 1000;    %% [mm]/[s]
t_up = 0.1;
t_dw =0.2;
acc_max = 10000;   %[mm/s]
t_min = v_qd / acc_max;
if(t_up < t_min) 
    t_up = t_min;    
end
if(t_dw < t_min)  
    t_dw = t_min; 
end
a_up = (v_qd - v_0) / t_up;
a_dw = (v_f - v_qd) / t_dw;
a_giutoc = 0;
S_up =  v_0 * t_up + 0.5 * a_up * t_up * t_up;
S_dw =  v_qd * t_dw + 0.5 * a_dw * t_dw * t_dw;
S_giutoc = AB_MD - (S_up + S_dw);
t_giutoc = S_giutoc / v_qd;
tf = t_giutoc + t_up + t_dw;
if (t_up > tf/2 )
    t_up = tf/2;
end

if (t_dw > t_giutoc/2 )
    t_dw = tf/2;
end

while (1)
    % chia giai doan
    if (t <= t_up)
      mode = 1;
      a = a_up;
      v = v_0 + a_up * t;
      S = x_0 + (v_0 * t) + (0.5 * a_up * t * t);
    else
        if (t <= t_giutoc + t_up )
      mode = 2;
      t2 = t - t_up;
      a = 0; 
      v = v_qd + a * t2;
      S = (x_0 + v_0 * t_up + 0.5 * a_up * t_up * t_up) + (v_qd * t2) ;
        else
            if (t > t_giutoc + t_up )
      mode = 3;
      t3 = t - t_giutoc - t_up;
      a = a_dw;
      v = v_qd + a_dw * t3;
      S = (x_0 + v_0 * t_up + 0.5 * a_up * t_up * t_up + (v_qd * t_giutoc)) + (v_qd * t3) + 0.5 * a * t3 * t3 ; 
            end
        end
    end
%   switch mode
%     case 1
%       a = a_up;
%       v = a_up * t;
%       S = v_0 * t + 0.5 * a_up * t * t ; %
%       v1_f = v;
%       S1_f = S;
%     case 2
%       t2 = t - t_up;
%       a = 0; 
%       v = v1_f + a * t2;
%       S = S1_f + v * t2; % + 0.5 * a * t2 * t2 ; 
%       v2_f = v;
%       S2_f = S;
%     case 3
%       t3 = t - t_giutoc - t_up;
%       a = a_dw;
%       v = v2_f + a_dw * t3;
%       S = S2_f + ( v2_f* t3)+ 0.5 * a_dw * t3 * t3 ;;  %v * t3 + S2_f +
%       v3_f = v;
%       S3_f = S;
%       otherwise
% end 
    P_x_TCP   = x_a + (x_b - x_a)*S/AB_MD;
    P_y_TCP   = y_a + (y_b - y_a)*S/AB_MD;
    P_z_TCP   = z_a + (z_b - z_a)*S/AB_MD;
    
    subplot(1,4,1);
    plot(t,a,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    grid on;
    hold on;
    
    subplot(1,4,2);
    plot(t,v,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    grid on;
    hold on;
    
    subplot(1,4,3);
    plot(t,S,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 tf]);
    grid on;
    hold on;
    
    subplot(1,4,4);
    plot3(P_x_TCP,P_y_TCP,P_z_TCP,'-o','LineWidth',1,'MarkerEdgeColor','b')
    xlim([0 400]);
    ylim([0 400]);
    zlim([0 400]);
    view(3);
    axis equal; 
    grid on;
    hold on;
       
    if(LastCycle==1)
        break;
    end
    
    t=t+ts;
    if(t>tf)
         t=tf;
         LastCycle = 1;
    end
    pause(0.01);
end

sprintf("Simulation Done")
