%% 绘制末端执行器在世界系下的四元数变化曲线
clear; clc;

%% 1. 读取 tracking.csv（机体 B 在世界系 W 下的姿态）
% tracking.csv 列格式：t, qx, qy, qz, qw（机体）
data = readtable('tracking.csv');

t  = data.t;      % 时间
qx_B = data.qx;   % Body quaternion x
qy_B = data.qy;   % Body quaternion y
qz_B = data.qz;   % Body quaternion z
qw_B = data.qw;   % Body quaternion w

N = length(t);

%% 2. 常数：末端执行器在机体系下的旋转 B_R_D(t)
omega = 0.5;          % rad/s
alpha = pi/12;        % 弧度
sa = sin(alpha);
ca = cos(alpha);

%% 3. 结果数组：末端执行器在世界系下的四元数 q_WD
qx_D = zeros(N,1);
qy_D = zeros(N,1);
qz_D = zeros(N,1);
qw_D = zeros(N,1);

for k = 1:N
    tk = t(k);

    % ---------- 3.1 机体四元数（归一化，整理成 [w x y z] ----------
    q_WB = [qw_B(k); qx_B(k); qy_B(k); qz_B(k)];
    q_WB = q_WB / norm(q_WB);

    % ---------- 3.2 计算 B_R_D(tk) ----------
    c = cos(omega * tk);
    s = sin(omega * tk);

    R_BD = [ c,     -s*ca,   s*sa;
             s,      c*ca,  -c*sa;
             0,       sa,     ca ];

    % ---------- 3.3 R_BD -> 四元数 q_BD = [w x y z] ----------
    q_BD = rotm2quat_custom(R_BD);   % 下面自己写的函数

    % ---------- 3.4 合成 q_WD = q_WB ⊗ q_BD ----------
    q_WD = quat_multiply(q_WB, q_BD);   % 自己写的函数

    % ---------- 3.5 保证 qw > 0 （避免符号跳变） ----------
    if q_WD(1) < 0
        q_WD = -q_WD;
    end

    % ---------- 3.6 存储 ----------
    qw_D(k) = q_WD(1);
    qx_D(k) = q_WD(2);
    qy_D(k) = q_WD(3);
    qz_D(k) = q_WD(4);
end

%% 4. 绘制四元数随时间变化曲线
figure;
subplot(4,1,1);
plot(t, qw_D, 'LineWidth', 1.5);
ylabel('q_w'); grid on;
title('End-effector Quaternion in World Frame');

subplot(4,1,2);
plot(t, qx_D, 'LineWidth', 1.5);
ylabel('q_x'); grid on;

subplot(4,1,3);
plot(t, qy_D, 'LineWidth', 1.5);
ylabel('q_y'); grid on;

subplot(4,1,4);
plot(t, qz_D, 'LineWidth', 1.5);
ylabel('q_z'); xlabel('t [s]'); grid on;
