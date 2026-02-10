
clear all; close all; clc;
rng(42);

fprintf('\n');
fprintf('========================================================================\n');
fprintf('  Port-Hamiltonian 飞艇控制系统 \n');
fprintf('========================================================================\n');

%% ========================================================================
%% 全局配置
%% ========================================================================

CFG.SAVE = true;
CFG.DIR = './results_v3/';
CFG.VERBOSE = true;
CFG.PLOT_SINGLE = false;
CFG.N_MC = 30;

if CFG.SAVE && ~exist(CFG.DIR, 'dir')
    mkdir(CFG.DIR);
    fprintf('  创建结果目录: %s\n\n', CFG.DIR);
end

%% ========================================================================
%% 初始化参数
%% ========================================================================

fprintf('>>> 步骤1: 初始化物理参数...\n');
P = init_physics_params();
fprintf('  艇体质量: %.1f kg, 侧向附加质量比: %.1f%%\n', ...
    P.m_hull, P.ma_y_true/P.m_hull*100);
fprintf('  附加质量真值: [%.2f, %.2f, %.2f]\n', ...
    P.ma_x_true, P.ma_y_true, P.Ia_psi_true);
fprintf('  附加质量初值: [%.2f, %.2f, %.2f]\n', ...
    P.ma_x_init, P.ma_y_init, P.Ia_psi_init);

fprintf('>>> 步骤2: 初始化控制参数...\n');
C = init_control_params();
fprintf('  PHGL增益: Ke=diag([%.0f,%.0f,%.0f]), Kd=diag([%.0f,%.0f,%.0f])\n', ...
    diag(C.K_e), diag(C.K_d));
fprintf('  OC-EKF阈值: alpha_PE = %.1e\n', C.alpha_PE);

fprintf('>>> 步骤3: 初始化仿真参数...\n');
S = init_simulation_params();
fprintf('  仿真时长: %.0f秒, 时间步长: %.3f秒\n', S.T_total, S.dt);
fprintf('  总步数: %d, 保存间隔: %d步\n\n', S.N_steps, S.save_interval);

%% ========================================================================
%% 蒙特卡洛仿真
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  开始蒙特卡洛仿真（%d次独立运行）\n', CFG.N_MC);
fprintf('========================================================================\n\n');

MC_results_PHGL = cell(CFG.N_MC, 1);
MC_results_ABC  = cell(CFG.N_MC, 1);
MC_rmse_PHGL    = zeros(CFG.N_MC, 1);
MC_rmse_ABC     = zeros(CFG.N_MC, 1);
MC_control_PHGL = zeros(CFG.N_MC, 1);
MC_control_ABC  = zeros(CFG.N_MC, 1);

tic_total = tic;

for mc_run = 1:CFG.N_MC
    fprintf('  运行 %d/%d  ', mc_run, CFG.N_MC);
    
    % 随机化
    S_mc = S;
    S_mc.q0 = randn(3,1) .* [1.5; 1.5; 10*pi/180];
    S_mc.wind_seed = 123 + mc_run;
    
    % --- PHGL ---
    tic_s = tic;
    result_PHGL = run_controller('PHGL', P, C, S_mc);
    t1 = toc(tic_s);
    
    % --- ABC ---
    tic_s = tic;
    result_ABC = run_controller('ABC', P, C, S_mc);
    t2 = toc(tic_s);
    
    MC_results_PHGL{mc_run} = result_PHGL;
    MC_results_ABC{mc_run}  = result_ABC;
    MC_rmse_PHGL(mc_run)    = result_PHGL.rmse;
    MC_rmse_ABC(mc_run)     = result_ABC.rmse;
    MC_control_PHGL(mc_run) = result_PHGL.control_effort;
    MC_control_ABC(mc_run)  = result_ABC.control_effort;
    
    elapsed = toc(tic_total);
    remaining = elapsed / mc_run * (CFG.N_MC - mc_run);
    fprintf('PHGL=%.3fm  ABC=%.3fm  (%.1fs+%.1fs)  剩余~%.0fs\n', ...
        result_PHGL.rmse, result_ABC.rmse, t1, t2, remaining);
end

total_time = toc(tic_total);
fprintf('\n  蒙特卡洛完成！总用时 %.1f分钟\n\n', total_time/60);

%% ========================================================================
%% 统计分析
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  统计分析结果\n');
fprintf('========================================================================\n\n');

fprintf('跟踪精度 RMSE [m]:\n');
fprintf('  方法    均值     标准差    最小值    最大值\n');
fprintf('  ------  -------  --------  --------  --------\n');
fprintf('  PHGL    %.4f   %.4f    %.4f    %.4f\n', ...
    mean(MC_rmse_PHGL), std(MC_rmse_PHGL), min(MC_rmse_PHGL), max(MC_rmse_PHGL));
fprintf('  ABC     %.4f   %.4f    %.4f    %.4f\n\n', ...
    mean(MC_rmse_ABC), std(MC_rmse_ABC), min(MC_rmse_ABC), max(MC_rmse_ABC));

% Welch t检验
[h_rmse, p_rmse, ci_rmse, stats_rmse] = ttest2(MC_rmse_ABC, MC_rmse_PHGL);
if p_rmse < 0.001
    sig_label = '(*** p<0.001)';
elseif p_rmse < 0.01
    sig_label = '(** p<0.01)';
elseif p_rmse < 0.05
    sig_label = '(* p<0.05)';
else
    sig_label = '(n.s.)';
end
fprintf('Welch t检验: p=%.6f %s\n', p_rmse, sig_label);
fprintf('  t=%.3f, df=%.1f, 95%%CI=[%.4f, %.4f]\n', ...
    stats_rmse.tstat, stats_rmse.df, ci_rmse(1), ci_rmse(2));

pooled_std = sqrt(((CFG.N_MC-1)*std(MC_rmse_PHGL)^2 + ...
    (CFG.N_MC-1)*std(MC_rmse_ABC)^2) / (2*CFG.N_MC-2));
cohens_d = (mean(MC_rmse_ABC) - mean(MC_rmse_PHGL)) / pooled_std;
fprintf('  Cohen''s d = %.3f\n', cohens_d);

improvement = (mean(MC_rmse_ABC) - mean(MC_rmse_PHGL)) / mean(MC_rmse_ABC) * 100;
fprintf('  PHGL相比ABC误差降低: %.2f%%\n\n', improvement);

%% ========================================================================
%% 保存结果
%% ========================================================================

if CFG.SAVE
    save([CFG.DIR 'MC_Results_Complete.mat'], ...
        'MC_results_PHGL', 'MC_results_ABC', ...
        'MC_rmse_PHGL', 'MC_rmse_ABC', ...
        'MC_control_PHGL', 'MC_control_ABC', ...
        'P', 'C', 'S', 'CFG', ...
        'h_rmse', 'p_rmse', 'stats_rmse', 'cohens_d', 'improvement');
    fprintf('  已保存: %sMC_Results_Complete.mat\n\n', CFG.DIR);
end

%% ========================================================================
%% 可视化
%% ========================================================================

fprintf('>>> 生成对比图表...\n');

% --- 图1: 箱线图 ---
figure('Position', [100 100 1200 500], 'Color', 'w');

subplot(1,2,1);
boxplot([MC_rmse_PHGL, MC_rmse_ABC], 'Labels', {'PHGL','ABC'});
ylabel('RMSE [m]', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('跟踪精度对比 (%d次MC)', CFG.N_MC), 'FontSize', 13);
grid on;
y_max = max([MC_rmse_PHGL; MC_rmse_ABC]);
text(1.5, y_max*1.08, sprintf('p=%.4f', p_rmse), ...
    'HorizontalAlignment', 'center', 'FontSize', 11);

subplot(1,2,2);
boxplot([MC_control_PHGL, MC_control_ABC], 'Labels', {'PHGL','ABC'});
ylabel('平均控制幅值 [N]', 'FontSize', 12, 'FontWeight', 'bold');
title('控制能耗对比', 'FontSize', 13);
grid on;

if CFG.SAVE
    saveas(gcf, [CFG.DIR 'Fig1_MC_Boxplot.png']);
    saveas(gcf, [CFG.DIR 'Fig1_MC_Boxplot.fig']);
end
fprintf('  图1: 箱线图\n');

% --- 图2: 单次轨迹 ---
[~, idx_med] = min(abs(MC_rmse_PHGL - median(MC_rmse_PHGL)));
res_ph = MC_results_PHGL{idx_med};
res_ab = MC_results_ABC{idx_med};

figure('Position', [100 100 1200 800], 'Color', 'w');

subplot(2,2,1);
plot(res_ph.q_d(1,:), res_ph.q_d(2,:), 'k--', 'LineWidth', 2, 'DisplayName', '参考');
hold on;
plot(res_ph.q(1,:), res_ph.q(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'PHGL');
plot(res_ab.q(1,:), res_ab.q(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ABC');
xlabel('x [m]'); ylabel('y [m]');
title('2D轨迹对比（中位数结果）'); legend('Location','best'); grid on; axis equal;

subplot(2,2,2);
err_ph = sqrt(sum((res_ph.q(1:2,:) - res_ph.q_d(1:2,:)).^2, 1));
err_ab = sqrt(sum((res_ab.q(1:2,:) - res_ab.q_d(1:2,:)).^2, 1));
plot(res_ph.time, err_ph, 'b-', 'LineWidth', 1.5, 'DisplayName', 'PHGL');
hold on;
plot(res_ab.time, err_ab, 'r-', 'LineWidth', 1.5, 'DisplayName', 'ABC');
xlabel('时间 [s]'); ylabel('位置误差 [m]');
title('跟踪误差'); legend('Location','best'); grid on;

subplot(2,2,3);
plot(res_ph.time, res_ph.theta(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ma_x');
hold on;
plot(res_ph.time, res_ph.theta(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'ma_y');
plot(res_ph.time, res_ph.theta(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ia_\psi');
yline(P.ma_x_true, 'r--', 'LineWidth', 1);
yline(P.ma_y_true, 'g--', 'LineWidth', 1);
yline(P.Ia_psi_true, 'b--', 'LineWidth', 1);
xlabel('时间 [s]'); ylabel('参数估计值');
title('OC-EKF参数收敛（虚线=真值）'); legend('Location','best'); grid on;

subplot(2,2,4);
semilogy(res_ph.time, res_ph.alpha_obs, 'b-', 'LineWidth', 1.5);
hold on;
yline(C.alpha_PE, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('\\alpha_{PE}=%.0e', C.alpha_PE));
xlabel('时间 [s]'); ylabel('\alpha_{obs}');
title('可观性指标'); legend('Location','best'); grid on;

if CFG.SAVE
    saveas(gcf, [CFG.DIR 'Fig2_Example_Trajectories.png']);
    saveas(gcf, [CFG.DIR 'Fig2_Example_Trajectories.fig']);
end
fprintf('  图2: 轨迹与诊断\n\n');

fprintf('========================================================================\n');
fprintf('  Part 1 完成！  总用时 %.1f分钟\n', total_time/60);
fprintf('  核心发现: PHGL vs ABC 误差降低 %.2f%% (p=%.4f, d=%.2f)\n', ...
    improvement, p_rmse, cohens_d);
fprintf('========================================================================\n\n');


%% ########################################################################
%% ########################################################################
%%                         函 数 定 义 区
%% ########################################################################
%% ########################################################################

%% ========================================================================
%% 参数初始化
%% ========================================================================

function P = init_physics_params()
    P.m_hull  = 15.0;           % 艇体质量 [kg]
    P.I_zz    = 8.0;            % 偏航刚体惯量 [kg·m^2]
    P.V_env   = 25.0;           % 囊体体积 [m^3]
    P.rho_air = 0.088;          % 平流层空气密度 [kg/m^3]
    P.g       = 9.8;            % 重力加速度

    % ---- 附加质量/惯量真值 ----
    %   论文: theta_true = [0.11, 1.98, 2.4]
    m_fluid = P.rho_air * P.V_env;          % 2.2 kg
    P.ma_x_true   = 0.05 * m_fluid;         % 0.11 kg  (Lamb k1≈0.05)
    P.ma_y_true   = 0.90 * m_fluid;         % 1.98 kg  (Lamb k2≈0.90)
    P.Ia_psi_true = 2.4;                    % 2.4 kg·m^2 

    % ---- 初始估计----
    P.ma_x_init   = 0.08;       % 真值0.11, 误差27%
    P.ma_y_init   = 1.50;       % 真值1.98, 误差24%
    P.Ia_psi_init = 2.00;       % 真值2.40, 误差17%

    % 阻尼 (Rayleigh)
    P.D_x   = 0.5;
    P.D_y   = 1.0;
    P.D_psi = 0.2;

    % 偏航恢复力矩
    P.k_psi = 10.0;

    % 浮心
    P.z_b = 0.5;                % 浮心到质心距离 [m]

    % 热动力学
    P.T_day       = 273 + 30;   % 日间氦气温度 [K]
    P.T_night     = 273 - 40;   % 夜间温度 [K]
    P.tau_thermal = 7200;       % 热时间常数 [s]

    % 数值
    P.cond_threshold = 1e8;
    P.reg_factor     = 1e-6;
end

function C = init_control_params()
    % --- PHGL 控制增益 ---
    C.K_e = diag([100, 100, 50]);       % 势能整形增益
    C.K_d = diag([40, 40, 20]);         % 阻尼注入增益

    % --- OC-EKF ---
    C.Q_theta   = 1e-5 * eye(3);       % 过程噪声（增大→EKF更灵敏）
    C.R_theta   = diag([0.01, 0.01, 0.005]);  % 测量噪声协方差
    C.P_theta_0 = diag([0.05, 1.0, 1.0]);     % 初始协方差（按参数量级缩放）
    C.alpha_PE  = 5e-4;                 % 持续激励阈值
    C.v_noise_std = [0.02; 0.02; 0.01]; % 速度测量噪声标准差 [m/s, m/s, rad/s]
    C.T_window  = 100;                  % 观测历史窗口（观测块数）
    C.OBS_MIN_SAMPLES  = 10;            % 最小观测块数
    C.GRAMIAN_REG      = 1e-4;          % Gramian 正则化系数
    C.EKF_update_interval = 10;         % EKF更新间隔 [步]

    % --- [Fix-2] 稀疏GP参数 ---
    C.sigma_f    = sqrt(5);             % 信号标准差
    C.ell_s      = 500;                 % 空间长度尺度 [m]
    C.ell_t      = 100;                 % 时间长度尺度 [s]
    C.GP_noise_var = 1.0;               % GP观测噪声方差
    C.GP_N_window  = 50;                % GP观测窗口大小
    C.GP_update_interval = 500;         % GP更新间隔 [步] (=10s at dt=0.02)

    % --- ABC 基线 ---
    C.ABC_kp       = 80;
    C.ABC_kd       = 35;
    C.ABC_Gamma    = diag([0.1, 0.1, 0.05]);
    C.ABC_sigma    = 0.01;
    C.ABC_proj_min = [0.01; 0.01; 0.01];
    C.ABC_proj_max = [5.0; 5.0; 5.0];

    % --- 执行器 ---
    C.u_max = [15; 15; 10];
    C.u_min = -C.u_max;
end

function S = init_simulation_params()
    S.T_total = 3600;           % 仿真时长 [s]
    S.dt      = 0.02;           % 步长 [s]
    S.N_steps = S.T_total / S.dt;
    S.save_interval = 200;      % 保存间隔（保持~900个保存点）

    % 8字形轨迹
    S.A_x     = 50;             % x幅值 [m]
    S.A_y     = 50;             % y幅值 [m]
    S.omega_x = 2*pi / 300;
    S.omega_y = 2*pi / 150;

    % [Fix-9] 风场参数
    S.wind_base_amp = [6.0; 4.0; 0.8];  % 确定性风力幅值 [N]
    S.wind_ou_sigma = [2.0; 2.0; 0.3];  % OU噪声标准差 [N]
    S.wind_ou_tau   = 30;                % OU相关时间 [s]
    S.wind_max      = 15;                % 风力饱和 [N]
    S.wind_seed     = 123;

    % 初始条件
    S.q0 = [0; 0; 0];
    S.p0 = [0; 0; 0];
end

%% ========================================================================
%% 主控制循环
%% ========================================================================

function result = run_controller(method_name, P, C, S)

    q = S.q0(:);
    p = S.p0(:);
    theta_est = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
    theta_true = [P.ma_x_true; P.ma_y_true; P.Ia_psi_true];
    T_He = P.T_day;
    B    = P.rho_air * P.V_env * P.g;

    ctrl_data = init_controller_data(method_name, C, P);

    % [Fix-9] 初始化OU风噪声状态
    rng(S.wind_seed);
    wind_noise_state = zeros(3, 1);

    % 预分配
    n_save = floor(S.N_steps / S.save_interval) + 1;
    result.time      = zeros(1, n_save);
    result.q         = zeros(3, n_save);
    result.p         = zeros(3, n_save);
    result.u         = zeros(3, n_save);
    result.theta     = zeros(3, n_save);
    result.error     = zeros(3, n_save);
    result.q_d       = zeros(3, n_save);
    if strcmp(method_name, 'PHGL')
        result.alpha_obs   = zeros(1, n_save);
        result.d_wind_est  = zeros(3, n_save);
    end
    save_idx = 1;

    error_sq_sum  = 0;
    control_sum   = 0;

    for step = 1:S.N_steps
        t = (step - 1) * S.dt;

        % 参考轨迹
        [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S);

        % [Fix-9] 风场扰动（连续OU过程）
        [d_wind, wind_noise_state] = generate_wind_disturbance( ...
            t, wind_noise_state, S);

        % 控制
        switch method_name
            case 'PHGL'
                % [Fix-EKF] 模拟速度传感器测量
                M_true_sensor = mass_matrix(q(3), theta_true, P);
                v_measured = M_true_sensor \ p + C.v_noise_std .* randn(3,1);

                [u, ctrl_data] = control_PHGL( ...
                    q, p, ctrl_data, q_d, dq_d, ddq_d, B, t, P, C, S, v_measured);
                theta_est = ctrl_data.theta;

            case 'ABC'
                [u, ctrl_data] = control_ABC( ...
                    q, p, ctrl_data, q_d, dq_d, ddq_d, B, P, C, S);
                theta_est = ctrl_data.theta_abc;
        end

        u = apply_saturation(u, C);

        % 保存旧状态（用于GP残差计算）
        p_old = p;

        % [Fix-3] 辛积分器（PH梯度修正版）
        [q, p] = symplectic_integrator(q, p, u, d_wind, theta_true, B, P, S.dt);

        % [Fix-2] 为GP计算风场残差
        if strcmp(method_name, 'PHGL')
            wind_residual = estimate_wind_residual( ...
                p_old, p, q, u, theta_est, B, P, S.dt);
            ctrl_data = gp_add_observation(ctrl_data, q(1:2), t, wind_residual, C);
        end

        % 热动力学
        [T_He, B] = update_thermal_dynamics(T_He, t, P, S.dt);

        % 累计误差
        error_sq_sum = error_sq_sum + norm(q(1:2) - q_d(1:2))^2 * S.dt;
        control_sum  = control_sum + norm(u) * S.dt;

        % 保存
        if mod(step, S.save_interval) == 0
            result.time(save_idx)     = t;
            result.q(:, save_idx)     = q;
            result.p(:, save_idx)     = p;
            result.u(:, save_idx)     = u;
            result.theta(:, save_idx) = theta_est;
            result.error(:, save_idx) = q - q_d;
            result.q_d(:, save_idx)   = q_d;
            if strcmp(method_name, 'PHGL')
                result.alpha_obs(save_idx)     = ctrl_data.alpha_obs;
                result.d_wind_est(:, save_idx) = ctrl_data.d_wind_est;
            end
            save_idx = save_idx + 1;
        end
    end

    result.rmse = sqrt(error_sq_sum / S.T_total);
    result.control_effort = control_sum / S.T_total;
    result.final_param_error = norm(theta_est - theta_true) / norm(theta_true);
end

%% ========================================================================
%% 控制器初始化
%% ========================================================================

function ctrl_data = init_controller_data(method_name, C, P)
    if strcmp(method_name, 'PHGL')
        ctrl_data.theta     = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
        ctrl_data.P_theta   = C.P_theta_0;
        ctrl_data.Obs_history = [];          % N_blocks×9 矩阵（每行是展平的3×3 H）
        ctrl_data.alpha_obs   = 0;
        ctrl_data.d_wind_est  = zeros(3, 1);

        % [Fix-2] GP 观测缓冲
        ctrl_data.GP_obs_X     = zeros(2, 0);  % 位置
        ctrl_data.GP_obs_T     = zeros(1, 0);  % 时间
        ctrl_data.GP_obs_Y     = zeros(3, 0);  % 风力残差
        ctrl_data.GP_last_update_time = -inf;

    elseif strcmp(method_name, 'ABC')
        ctrl_data.theta_abc = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
    end
end

%% ========================================================================
%% PHGL 控制器
%% ========================================================================

function [u, ctrl_data] = control_PHGL(q, p, ctrl_data, ...
        q_d, dq_d, ddq_d, B, t, P, C, S, v_measured)
    % 论文控制律 (13):
    % u = -Ke*eq - Kd*M^{-1}*ep + M(ψd)*q̈d + C(ψd,q̇d)*q̇d
    %     - d̂_wind + f_comp

    q   = q(:);   p   = p(:);
    q_d = q_d(:); dq_d = dq_d(:); ddq_d = ddq_d(:);
    theta = ctrl_data.theta(:);

    % 误差
    e_q = q - q_d;
    M_d = mass_matrix(q_d(3), theta, P);
    p_d = M_d * dq_d;
    e_p = p - p_d;

    M_curr = mass_matrix(q(3), theta, P);
    M_inv  = robust_inv(M_curr, P);

    % (1) 比例项（势能整形）
    u1 = -C.K_e * e_q;

    % (2) 阻尼项（阻尼注入）
    u2 = -C.K_d * (M_inv * e_p);

    % (3) 前馈（期望轨迹动力学）
    C_des = coriolis_matrix(q_d(3), dq_d, theta, P);
    u3 = M_d * ddq_d + C_des * dq_d;

    % (4) 风场补偿
    u4 = -ctrl_data.d_wind_est;

    % (5) 浮力补偿力矩
    f_buoy_psi = (B - P.m_hull * P.g) * P.z_b * sin(q(3));
    u5 = [0; 0; -f_buoy_psi];

    u = u1 + u2 + u3 + u4 + u5;
    u = u(:);

    % ---- OC-EKF 参数更新 ----
    step_now = round(t / S.dt) + 1;
    if mod(step_now, C.EKF_update_interval) == 0
        psi = q(3);   
        [theta_new, P_new, alpha_obs] = observability_constrained_EKF( ...
            ctrl_data.theta, ctrl_data.P_theta, ...
            psi, p, v_measured, ctrl_data.Obs_history, P, C);
        H = compute_observation_jacobian(psi, p, ctrl_data.theta, P);
        H_flat = H(:)';   % 1×9
        ctrl_data.Obs_history = [ctrl_data.Obs_history; H_flat];
        if size(ctrl_data.Obs_history, 1) > C.T_window
            ctrl_data.Obs_history(1, :) = [];   
        end

        ctrl_data.theta   = theta_new;
        ctrl_data.P_theta = P_new;
        ctrl_data.alpha_obs = alpha_obs;
    end

    % ---- [Fix-2] GP 风场预测 ----
    if (t - ctrl_data.GP_last_update_time) >= C.GP_update_interval * S.dt
        ctrl_data.d_wind_est = gp_predict_wind(q(1:2), t, ctrl_data, C);
        ctrl_data.GP_last_update_time = t;
    end
end

%% ========================================================================
%% OC-EKF（可观性约束扩展卡尔曼滤波）
%% ========================================================================

function [theta_new, P_new, alpha_obs] = observability_constrained_EKF( ...
        theta, P_theta, psi, p, v, Obs_history, P, C)
    % 论文公式 (18)-(19), 算法 2

    theta = theta(:);

    % == 预测步 ==
    theta_pred = theta;
    P_pred = P_theta + C.Q_theta;

    % == 测量更新 ==
    M_pred = mass_matrix(psi, theta_pred, P);
    v_pred = M_pred \ p;
    innovation = v - v_pred;

    H = compute_observation_jacobian(psi, p, theta_pred, P);

    % == [Fix-5] 可观性 Gramian（归一化按观测块数）==
    alpha_obs = 0;
    P_obs = eye(3);

    n_blocks = size(Obs_history, 1);   
    if n_blocks >= C.OBS_MIN_SAMPLES
        % 重建 Gramian: G = (1/n) Σ H_i^T H_i
        G_obs = zeros(3, 3);
        for i = 1:n_blocks
            Hi = reshape(Obs_history(i, :), 3, 3);
            G_obs = G_obs + Hi' * Hi;
        end
        G_obs = G_obs / n_blocks;

        % 正则化
        reg = C.GRAMIAN_REG * trace(G_obs) / 3;
        G_obs = G_obs + reg * eye(3);

        % SVD
        [U, Sigma, ~] = svd(G_obs);
        sv = diag(Sigma);
        alpha_obs = sv(end) / (sv(1) + 1e-12);

        % 投影矩阵：保留奇异值 > alpha_PE * sigma_max 的方向
        mask = sv > C.alpha_PE * sv(1);
        U_r = U(:, mask);
        P_obs = U_r * U_r';
    end

    % == Kalman 增益 ==
    S_inn = H * P_pred * H' + C.R_theta;
    if cond(S_inn) > 1e8
        S_inn = S_inn + 1e-6 * trace(S_inn)/3 * eye(3);
    end
    K = P_pred * H' / S_inn;

    % == [Fix-4/5] 条件更新（论文公式19）==
    if alpha_obs > C.alpha_PE
        theta_new = theta_pred + P_obs * K * innovation;
    else
        theta_new = theta_pred;
    end

    % 物理约束投影
    theta_new = max(theta_new, [0.01; 0.01; 0.01]);
    theta_new = min(theta_new, [5.0; 5.0; 10.0]);

    % Joseph 形式协方差更新
    IKH = eye(3) - K * H;
    P_new = IKH * P_pred * IKH' + K * C.R_theta * K';
    P_new = (P_new + P_new') / 2 + 1e-8 * eye(3);
end

function H = compute_observation_jacobian(psi, p, theta, P)
    % 观测雅可比 ∂v/∂θ（数值中心差分）
    % [Fix-4] psi 为实际偏航角
    theta = theta(:);
    eps = 1e-7;
    H = zeros(3, 3);

    M_0 = mass_matrix(psi, theta, P);
    v_0 = M_0 \ p;

    for i = 1:3
        th_p = theta; th_p(i) = th_p(i) + eps;
        th_m = theta; th_m(i) = th_m(i) - eps;
        v_p = mass_matrix(psi, th_p, P) \ p;
        v_m = mass_matrix(psi, th_m, P) \ p;
        H(:, i) = (v_p - v_m) / (2 * eps);   % 中心差分更精确
    end
end

%% ========================================================================
%% [Fix-2] 高斯过程在线风场推断
%% ========================================================================

function ctrl_data = gp_add_observation(ctrl_data, x_pos, t, y_wind, C)
    % 向GP缓冲区添加一个风场残差观测
    ctrl_data.GP_obs_X = [ctrl_data.GP_obs_X, x_pos(:)];
    ctrl_data.GP_obs_T = [ctrl_data.GP_obs_T, t];
    ctrl_data.GP_obs_Y = [ctrl_data.GP_obs_Y, y_wind(:)];

    % 保持窗口大小
    if size(ctrl_data.GP_obs_X, 2) > C.GP_N_window
        ctrl_data.GP_obs_X(:, 1) = [];
        ctrl_data.GP_obs_T(1)    = [];
        ctrl_data.GP_obs_Y(:, 1) = [];
    end
end

function d_pred = gp_predict_wind(x_query, t_query, ctrl_data, C)
    % GP后验均值预测
    % 使用滑动窗口内的观测数据，RBF时空核

    N = size(ctrl_data.GP_obs_X, 2);
    if N < 5
        d_pred = zeros(3, 1);
        return;
    end

    X = ctrl_data.GP_obs_X;   % 2×N
    T = ctrl_data.GP_obs_T;   % 1×N
    Y = ctrl_data.GP_obs_Y;   % 3×N

    % 核矩阵 K(N×N) + 核向量 k*(N×1)
    K_mat  = zeros(N, N);
    k_star = zeros(N, 1);

    sf2  = C.sigma_f^2;
    ls2  = C.ell_s^2;
    lt2  = C.ell_t^2;

    for i = 1:N
        dx_i = x_query - X(:, i);
        dt_i = t_query - T(i);
        k_star(i) = sf2 * exp(-0.5*(dx_i'*dx_i)/ls2 - 0.5*dt_i^2/lt2);
        for j = i:N
            dx_ij = X(:,i) - X(:,j);
            dt_ij = T(i) - T(j);
            val = sf2 * exp(-0.5*(dx_ij'*dx_ij)/ls2 - 0.5*dt_ij^2/lt2);
            K_mat(i,j) = val;
            K_mat(j,i) = val;
        end
    end

    K_mat = K_mat + C.GP_noise_var * eye(N);

    % Cholesky 求解（数值稳定）
    [L, flag] = chol(K_mat, 'lower');
    if flag ~= 0
        K_mat = K_mat + 1e-4 * eye(N);
        L = chol(K_mat, 'lower');
    end

    d_pred = zeros(3, 1);
    for dim = 1:3
        alpha = L' \ (L \ Y(dim, :)');
        d_pred(dim) = k_star' * alpha;
    end
end

function wind_res = estimate_wind_residual(p_old, p_new, q, u, theta_est, B, P, dt)
    % 从前后动量差估计风力残差
    % dp/dt ≈ -∇_q H - D_p v + u + d_wind + f_buoy
    % => d_wind ≈ (p_new-p_old)/dt - (-∇_q H - D_p v + u + f_buoy)

    M_c = mass_matrix(q(3), theta_est, P);
    v = M_c \ p_old;

    % ∂H/∂q（使用估计参数）
    dH_dpsi = compute_dH_dpsi(q(3), p_old, theta_est, P);
    grad_H_q = [0; 0; dH_dpsi];

    D_v = [P.D_x; P.D_y; P.D_psi] .* v;
    f_buoy = [0; 0; (B - P.m_hull*P.g) * P.z_b * sin(q(3))];

    dp_dt_approx = (p_new - p_old) / dt;
    f_model = -grad_H_q - D_v + u(:) + f_buoy;

    wind_res = dp_dt_approx - f_model;
end

%% ========================================================================
%% ABC 基线控制器
%% ========================================================================

function [u, ctrl_data] = control_ABC(q, p, ctrl_data, q_d, dq_d, ddq_d, B, P, C, S)

    q = q(:); p = p(:);
    q_d = q_d(:); dq_d = dq_d(:); ddq_d = ddq_d(:);
    theta_abc = ctrl_data.theta_abc(:);

    e_q = q - q_d;
    M_c = mass_matrix(q(3), theta_abc, P);
    v   = M_c \ p;
    e_v = v - dq_d;

    % 自适应律（σ-修正）
    phi = [v(1)*e_v(1); v(2)*e_v(2); v(3)*e_v(3)];
    theta_dot = C.ABC_Gamma * phi - C.ABC_sigma * theta_abc;
    ctrl_data.theta_abc = theta_abc + S.dt * theta_dot;
    ctrl_data.theta_abc = max(ctrl_data.theta_abc, C.ABC_proj_min);
    ctrl_data.theta_abc = min(ctrl_data.theta_abc, C.ABC_proj_max);

    % 控制律
    M_est = mass_matrix(q_d(3), ctrl_data.theta_abc, P);
    C_est = coriolis_matrix(q_d(3), dq_d, ctrl_data.theta_abc, P);
    f_buoy_comp = [0; 0; -(B - P.m_hull*P.g) * P.z_b * sin(q(3))];

    u = -C.ABC_kp * e_q - C.ABC_kd * e_v ...
        + M_est * ddq_d + C_est * dq_d + f_buoy_comp;
    u = u(:);
end

%% ========================================================================
%% 动力学模型
%% ========================================================================

function M_total = mass_matrix(psi, theta, P)
    % [Fix-1] 完整质量矩阵 M(ψ) = M_RB + R(ψ)*M_A*R(ψ)^T

    theta = theta(:);

    M_RB = diag([P.m_hull, P.m_hull, P.I_zz]);

    % 2D旋转（平移部分）
    c = cos(psi); s = sin(psi);
    R2 = [c, -s; s, c];

    M_A_2d = diag([theta(1), theta(2)]);         % [ma_x, ma_y]
    M_A_trans = R2 * M_A_2d * R2';

    M_total = M_RB;
    M_total(1:2, 1:2) = M_total(1:2, 1:2) + M_A_trans;
    M_total(3,3)       = M_total(3,3) + theta(3);  
end

function C_mat = coriolis_matrix(psi, dq, theta, P)
    % C(ψ, q̇) = ∂M/∂ψ · ψ̇
    % 用于控制律前馈项 C(ψd, q̇d)*q̇d
    dq = dq(:); theta = theta(:);
    dpsi = dq(3);
    delta_ma = theta(2) - theta(1);  % m_a^y - m_a^x

    % ∂M_{12}/∂ψ = -(m_a^y - m_a^x)*cos(2ψ)   (论文公式29)
    % C_{ij} = ∂M_{ij}/∂ψ * dψ/dt

    dM12 = -delta_ma * cos(2*psi);      % ∂M_{12}/∂ψ (论文公式29)
    dM11 =  delta_ma * sin(2*psi);     % ∂M_{11}/∂ψ = (ma_y-ma_x)sin(2ψ)
    dM22 = -delta_ma * sin(2*psi);     % ∂M_{22}/∂ψ = -(ma_y-ma_x)sin(2ψ)

    C_mat = dpsi * [dM11,  dM12,  0;
                    dM12,  dM22,  0;
                    0,     0,     0];
end

function dH_dpsi = compute_dH_dpsi(psi, p, theta, P)
    % [Fix-3] 完整计算 ∂H/∂ψ = 1/2 * p^T * ∂M^{-1}/∂ψ * p + k_ψ * ψ
    % 使用中心差分数值计算 ∂M^{-1}/∂ψ

    eps_psi = 1e-7;
    M_plus  = mass_matrix(psi + eps_psi, theta, P);
    M_minus = mass_matrix(psi - eps_psi, theta, P);
    dMinv_dpsi = (M_plus \ eye(3) - M_minus \ eye(3)) / (2 * eps_psi);

    dH_dpsi = 0.5 * p' * dMinv_dpsi * p + P.k_psi * psi;
end

function [q_new, p_new] = symplectic_integrator(q, p, u, d_wind, theta, B, P, dt)
    % [Fix-3] Störmer-Verlet 辛积分器
    % 使用 PH 框架的 ∂H/∂q 梯度（不再单独加 C*v 项）
    %
    % 半步 q → 全步 p → 半步 q

    q = q(:); p = p(:); u = u(:); d_wind = d_wind(:); theta = theta(:);

    % --- 第1步：位置半步 ---
    M_c  = mass_matrix(q(3), theta, P);
    M_inv = robust_inv(M_c, P);
    v = M_inv * p;
    q_half = q + 0.5 * dt * v;

    % --- 第2步：动量全步 ---
    % [Fix-3] ∂H/∂q 完整计算（含动能对ψ梯度）
    dH_dpsi = compute_dH_dpsi(q_half(3), p, theta, P);
    grad_H_q = [0; 0; dH_dpsi];

    D_v = [P.D_x; P.D_y; P.D_psi] .* v;  % Rayleigh 阻尼
    f_buoy = [0; 0; (B - P.m_hull*P.g) * P.z_b * sin(q_half(3))];

    % PH动力学: dp/dt = -∂H/∂q - D_p·v + u + d_wind + f_buoy
    f_total = -grad_H_q - D_v + u + d_wind + f_buoy;
    p_new = p + dt * f_total;

    % --- 第3步：位置剩余半步 ---
    M_half = mass_matrix(q_half(3), theta, P);
    v_new  = robust_inv(M_half, P) * p_new;
    q_new  = q_half + 0.5 * dt * v_new;
end

%% ========================================================================
%% 辅助函数
%% ========================================================================

function [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S)
    q_d   = [S.A_x * sin(S.omega_x * t);
             S.A_y * sin(S.omega_y * t);
             0];
    dq_d  = [S.A_x * S.omega_x * cos(S.omega_x * t);
             S.A_y * S.omega_y * cos(S.omega_y * t);
             0];
    ddq_d = [-S.A_x * S.omega_x^2 * sin(S.omega_x * t);
             -S.A_y * S.omega_y^2 * sin(S.omega_y * t);
              0];
end

function [d_wind, noise_state] = generate_wind_disturbance(t, noise_state, S)
    % [Fix-9] 连续风场模型
    %   确定性成分：低频正弦（模拟大尺度气象变化）
    %   随机成分：Ornstein-Uhlenbeck 过程（时间相关）
    %   输出：广义力 [N, N, N·m]

    omega1 = 2*pi / 600;    % 10分钟周期
    omega2 = 2*pi / 300;    % 5分钟周期

    d_base = [S.wind_base_amp(1) * sin(omega1*t);
              S.wind_base_amp(2) * cos(omega2*t);
              S.wind_base_amp(3) * sin(omega1*t + pi/3)];

    % OU 过程演化：dx = -x/tau*dt + sigma*sqrt(2/tau)*dW
    tau = S.wind_ou_tau;
    decay = exp(-S.dt / tau);
    diffusion = S.wind_ou_sigma .* sqrt(1 - decay^2);
    noise_state = noise_state * decay + diffusion .* randn(3, 1);

    d_wind = d_base + noise_state;

    % 力饱和
    wind_norm = norm(d_wind);
    if wind_norm > S.wind_max
        d_wind = d_wind / wind_norm * S.wind_max;
    end
end

function [T_He_new, B_new] = update_thermal_dynamics(T_He, t, P, dt)
    hour = mod(t/3600, 24);
    if hour >= 6 && hour <= 18
        T_target = P.T_day;
    else
        T_target = P.T_night;
    end
    T_He_new = T_He + (T_target - T_He) / P.tau_thermal * dt;
    B_new = P.rho_air * P.V_env * P.g * (T_He_new / (273 + 15));
end

function u = apply_saturation(u, C)
    u = u(:);
    u = max(min(u, C.u_max), C.u_min);
end

function M_inv = robust_inv(M, P)
    % 带条件数检查的鲁棒矩阵求逆
    if cond(M) > P.cond_threshold
        reg = P.reg_factor * trace(M) / size(M, 1);
        M = M + reg * eye(size(M));
    end
    M_inv = M \ eye(size(M));
end