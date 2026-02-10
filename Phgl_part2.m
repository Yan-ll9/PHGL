
clear; clc;

fprintf('\n');
fprintf('========================================================================\n');
fprintf('  Port-Hamiltonian 飞艇控制系统 - Part 2: 完整对比分析 V3.3\n');
fprintf('========================================================================\n');

%% ========================================================================
%% 加载Part 1结果
%% ========================================================================

fprintf('>>> 步骤1: 加载Part 1基线结果...\n');
if ~exist('./results_v3/MC_Results_Complete.mat', 'file')
    error('未找到Part 1结果！请先运行 Phgl_part1.m (V3.3)');
end

load('./results_v3/MC_Results_Complete.mat');
fprintf('  已加载PHGL和ABC的%d次蒙特卡洛结果\n', CFG.N_MC);
fprintf('  PHGL均值: %.4f +/- %.4f m\n', mean(MC_rmse_PHGL), std(MC_rmse_PHGL));
fprintf('  ABC均值:  %.4f +/- %.4f m\n\n', mean(MC_rmse_ABC), std(MC_rmse_ABC));

%% ========================================================================
%% MRAC方法运行（模型参考自适应控制）
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  步骤2: MRAC (Model Reference Adaptive Control) 对比方法\n');
fprintf('========================================================================\n\n');

MRAC = init_MRAC_params();
fprintf('>>> 运行MRAC蒙特卡洛测试...\n');
MC_results_MRAC = cell(CFG.N_MC, 1);
MC_rmse_MRAC = zeros(CFG.N_MC, 1);
MC_control_MRAC = zeros(CFG.N_MC, 1);

tic_mrac = tic;
for mc_run = 1:CFG.N_MC
    if mod(mc_run, 10) == 0
        fprintf('  MRAC进度: %d/%d\n', mc_run, CFG.N_MC);
    end
    S_mc = S;
    S_mc.q0 = randn(3,1) .* [1.5; 1.5; 10*pi/180];
    S_mc.wind_seed = 123 + mc_run;
    result = run_MRAC_controller(MRAC, P, C, S_mc);
    MC_results_MRAC{mc_run} = result;
    MC_rmse_MRAC(mc_run) = result.rmse;
    MC_control_MRAC(mc_run) = result.control_effort;
end
time_mrac = toc(tic_mrac);
fprintf('  MRAC完成 (%.2f分钟), 均值: %.4f +/- %.4f m\n\n', ...
    time_mrac/60, mean(MC_rmse_MRAC), std(MC_rmse_MRAC));

%% ========================================================================
%% MPC方法运行
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  步骤3: MPC (Model Predictive Control) 对比方法\n');
fprintf('========================================================================\n\n');

MPC = init_MPC_params();
fprintf('>>> 运行MPC蒙特卡洛测试...\n');
MC_results_MPC = cell(CFG.N_MC, 1);
MC_rmse_MPC = zeros(CFG.N_MC, 1);
MC_control_MPC = zeros(CFG.N_MC, 1);

tic_mpc = tic;
for mc_run = 1:CFG.N_MC
    if mod(mc_run, 10) == 0
        fprintf('  MPC进度: %d/%d\n', mc_run, CFG.N_MC);
    end
    S_mc = S;
    S_mc.q0 = randn(3,1) .* [1.5; 1.5; 10*pi/180];
    S_mc.wind_seed = 123 + mc_run;
    result = run_MPC_controller(MPC, P, C, S_mc);
    MC_results_MPC{mc_run} = result;
    MC_rmse_MPC(mc_run) = result.rmse;
    MC_control_MPC(mc_run) = result.control_effort;
end
time_mpc = toc(tic_mpc);
fprintf('  MPC完成 (%.2f分钟), 均值: %.4f +/- %.4f m\n\n', ...
    time_mpc/60, mean(MC_rmse_MPC), std(MC_rmse_MPC));

%% ========================================================================
%% 4方法统计对比
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  步骤4: 4方法统计对比分析\n');
fprintf('========================================================================\n\n');

methods = {'PHGL', 'ABC', 'MRAC', 'MPC'};
MC_rmse_all = [MC_rmse_PHGL, MC_rmse_ABC, MC_rmse_MRAC, MC_rmse_MPC];
MC_control_all = [MC_control_PHGL, MC_control_ABC, MC_control_MRAC, MC_control_MPC];

fprintf('跟踪精度 (RMSE) [m]:\n');
fprintf('  方法    均值     标准差    最小值    最大值    vs PHGL\n');
fprintf('  ------  -------  --------  --------  --------  ----------\n');
for i = 1:4
    rel_change = (mean(MC_rmse_all(:,i)) - mean(MC_rmse_PHGL)) / mean(MC_rmse_PHGL) * 100;
    fprintf('  %-6s  %.4f   %.4f    %.4f    %.4f    %+6.2f%%\n', ...
        methods{i}, mean(MC_rmse_all(:,i)), std(MC_rmse_all(:,i)), ...
        min(MC_rmse_all(:,i)), max(MC_rmse_all(:,i)), rel_change);
end
fprintf('\n');

fprintf('成对t检验 (相对PHGL):\n');
fprintf('  对比       p值        Cohen''s d   结论\n');
fprintf('  ---------  ---------  ----------  --------------------\n');
for i = 2:4
    [~, p_val] = ttest2(MC_rmse_all(:,i), MC_rmse_PHGL);
    pooled_std = sqrt(((CFG.N_MC-1)*var(MC_rmse_all(:,i)) + ...
        (CFG.N_MC-1)*var(MC_rmse_PHGL)) / (2*CFG.N_MC-2));
    d = (mean(MC_rmse_all(:,i)) - mean(MC_rmse_PHGL)) / pooled_std;
    if p_val < 0.001, sig = '***';
    elseif p_val < 0.01, sig = '**';
    elseif p_val < 0.05, sig = '*';
    else, sig = 'n.s.'; end
    fprintf('  %s vs PHGL  %.6f   %+7.3f    %s\n', methods{i}, p_val, d, sig);
end
fprintf('\n');

%% ========================================================================
%% 消融实验（Ablation Study）- 挑战性场景设计
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  步骤5: 消融实验 (Ablation Study) - 挑战性场景\n');
fprintf('========================================================================\n');
fprintf('  场景设计：大参数误差 + 强风 + 快轨迹 + 中等增益\n');
fprintf('  目的：暴露各模块在极限条件下的边际贡献\n\n');

ablation_configs = {
    'PHGL_Full',         'PHGL完整版',            true,  true,  true,  true,  true;
    'No_OC_EKF',         '无OC-EKF（固定参数）',   false, true,  true,  true,  true;
    'No_Obs_Constraint', '无可观性约束（标准EKF）', true,  false, true,  true,  true;
    'No_GP',             '无稀疏GP（零风场估计）',  true,  true,  false, true,  true;
    'No_Thermal',        '无热补偿（固定浮力）',    true,  true,  true,  false, true;
    'No_PH_Structure',   '无PH结构（标准PD）',      true,  true,  true,  true,  false;
};

N_ablation = size(ablation_configs, 1);
ablation_rmse = zeros(N_ablation, CFG.N_MC);

% --- 创建消融专用的挑战性场景参数 ---
P_abl = P;
P_abl.ma_x_init   = P.ma_x_true * 0.35;    % 真值0.11 → 初始0.039, 误差65%
P_abl.ma_y_init   = P.ma_y_true * 0.40;    % 真值1.98 → 初始0.79,  误差60%
P_abl.Ia_psi_init = P.Ia_psi_true * 0.35;  % 真值2.40 → 初始0.84,  误差65%

S_abl = S;

S_abl.omega_x = S.omega_x * 1.5;
S_abl.omega_y = S.omega_y * 1.5;

S_abl.wind_base_amp = S.wind_base_amp * 1.3;
S_abl.wind_ou_sigma = S.wind_ou_sigma * 1.3;


C_abl = C;
C_abl.K_e = diag([60, 60, 30]);    
C_abl.K_d = diag([25, 25, 12]);     

C_abl.alpha_PE = 1e-5;

fprintf('  参数误差: ma_x %.0f%%, ma_y %.0f%%, Ia_psi %.0f%%\n', ...
    (1-P_abl.ma_x_init/P.ma_x_true)*100, ...
    (1-P_abl.ma_y_init/P.ma_y_true)*100, ...
    (1-P_abl.Ia_psi_init/P.Ia_psi_true)*100);
fprintf('  轨迹频率: 1.5x基线, 风力: 1.3x基线\n');
fprintf('  控制增益: Ke=diag([60,60,30]), Kd=diag([25,25,12])\n');
fprintf('  可观性阈值: alpha_PE = %.0e (降低以允许EKF更新)\n\n', C_abl.alpha_PE);

fprintf('>>> 运行消融实验（%d种配置 x %d次）...\n', N_ablation, CFG.N_MC);
tic_ablation = tic;

for cfg_idx = 1:N_ablation
    fprintf('  配置 %d/%d: %s\n', cfg_idx, N_ablation, ablation_configs{cfg_idx, 2});
    
    for mc_run = 1:CFG.N_MC
        S_mc = S_abl;
        S_mc.q0 = randn(3,1) .* [2.0; 2.0; 15*pi/180];
        S_mc.wind_seed = 123 + mc_run;
        
        ablation_flags = struct();
        ablation_flags.use_OC_EKF = ablation_configs{cfg_idx, 3};
        ablation_flags.use_obs_constraint = ablation_configs{cfg_idx, 4};
        ablation_flags.use_GP = ablation_configs{cfg_idx, 5};
        ablation_flags.use_thermal = ablation_configs{cfg_idx, 6};
        ablation_flags.use_PH = ablation_configs{cfg_idx, 7};
        
        result = run_ablation_test(ablation_flags, P_abl, C_abl, S_mc);
        ablation_rmse(cfg_idx, mc_run) = result.rmse;
    end
    
    fprintf('    结果: %.4f +/- %.4f m\n', ...
        mean(ablation_rmse(cfg_idx,:)), std(ablation_rmse(cfg_idx,:)));
end

time_ablation = toc(tic_ablation);
fprintf('  消融实验完成 (%.2f分钟)\n\n', time_ablation/60);

% 消融结果表
fprintf('消融实验结果 (表III):\n');
fprintf('  配置                        RMSE [m]   相对变化\n');
fprintf('  --------------------------  ---------  ----------\n');
baseline_rmse = mean(ablation_rmse(1,:));
for i = 1:N_ablation
    rel_change = (mean(ablation_rmse(i,:)) - baseline_rmse) / baseline_rmse * 100;
    fprintf('  %-26s  %.4f     %+6.2f%%\n', ...
        ablation_configs{i,2}, mean(ablation_rmse(i,:)), rel_change);
end
fprintf('\n');

% 消融统计检验
fprintf('消融t检验 (各配置 vs PHGL完整):\n');
for i = 2:N_ablation
    [~, p_val] = ttest2(ablation_rmse(i,:), ablation_rmse(1,:));
    if p_val < 0.001, sig = '***';
    elseif p_val < 0.01, sig = '**';
    elseif p_val < 0.05, sig = '*';
    else, sig = 'n.s.'; end
    fprintf('  %-26s  p=%.4f  %s\n', ablation_configs{i,2}, p_val, sig);
end
fprintf('\n');

%% ========================================================================
%% 失效模式分析（Failure Mode Analysis）- 渐进式压力测试
%% ========================================================================


fprintf('========================================================================\n');
fprintf('  步骤6: 失效模式分析 - 渐进式压力测试\n');
fprintf('========================================================================\n');
fprintf('  目的：展示性能从正常→退化→失效的渐进过程\n\n');

failure_modes = {
    % 名称                 显示名                     风力x  初始误差%  频率x
    'Normal',              '正常条件',                1.0,   0,        1.0;
    'Moderate_Wind',       '中等强风 (1.3x风力)',     1.3,   0,        1.0;
    'Strong_Wind',         '强风 (1.6x风力)',         1.6,   0,        1.0;
    'Aggressive_Maneuver', '激进机动 (1.5x频率)',     1.0,   0,        1.5;
    'Large_Init_Error',    '大初始误差 (150%)',        1.0,   150,      1.0;
    'Combined_Stress',     '组合压力 (风+机动)',       1.3,   0,        1.3;
};

N_failure = size(failure_modes, 1);
failure_rmse = zeros(N_failure, 10);

fprintf('>>> 运行失效模式测试（%d种模式 x 10次）...\n', N_failure);
tic_failure = tic;

for fm_idx = 1:N_failure
    fprintf('  模式 %d/%d: %s\n', fm_idx, N_failure, failure_modes{fm_idx, 2});
    
    for trial = 1:10
        S_fm = S;
        wind_scale = failure_modes{fm_idx, 3};
        freq_scale = failure_modes{fm_idx, 5};
        
        S_fm.wind_base_amp = S.wind_base_amp * wind_scale;
        S_fm.wind_ou_sigma = S.wind_ou_sigma * wind_scale;
        
        S_fm.omega_x = S.omega_x * freq_scale;
        S_fm.omega_y = S.omega_y * freq_scale;
        
        init_error_pct = failure_modes{fm_idx, 4};
        if init_error_pct > 0
            P_fm = P;
            P_fm.ma_x_init   = P.ma_x_true * (1 + init_error_pct/100);
            P_fm.ma_y_init   = P.ma_y_true * (1 + init_error_pct/100);
            P_fm.Ia_psi_init = P.Ia_psi_true * (1 + init_error_pct/100);
        else
            P_fm = P;
        end
        
        S_fm.q0 = randn(3,1) .* [0.5; 0.5; 5*pi/180];
        S_fm.wind_seed = 123 + trial;
        
        result = run_controller_full('PHGL', P_fm, C, S_fm);
        failure_rmse(fm_idx, trial) = result.rmse;
    end
    
    fprintf('    结果: %.4f +/- %.4f m\n', ...
        mean(failure_rmse(fm_idx,:)), std(failure_rmse(fm_idx,:)));
end

time_failure = toc(tic_failure);
fprintf('  失效模式分析完成 (%.2f分钟)\n\n', time_failure/60);

% 失效模式结果表
fprintf('失效模式分析结果:\n');
fprintf('  模式                    RMSE [m]   vs正常    状态\n');
fprintf('  ----------------------  ---------  --------  --------\n');
normal_rmse = mean(failure_rmse(1,:));
for i = 1:N_failure
    degradation = (mean(failure_rmse(i,:)) - normal_rmse) / normal_rmse * 100;
    if degradation < 50
        status = '正常';
    elseif degradation < 150
        status = '退化';
    else
        status = '严重退化';
    end
    fprintf('  %-22s  %.4f     %+6.1f%%  %s\n', ...
        failure_modes{i,2}, mean(failure_rmse(i,:)), degradation, status);
end
fprintf('\n');

%% ========================================================================
%% 保存完整结果
%% ========================================================================

fprintf('>>> 保存所有结果...\n');
save([CFG.DIR 'Part2_Complete_Results.mat'], ...
    'MC_results_PHGL', 'MC_results_ABC', 'MC_results_MRAC', 'MC_results_MPC', ...
    'MC_rmse_all', 'MC_control_all', 'methods', ...
    'ablation_configs', 'ablation_rmse', ...
    'failure_modes', 'failure_rmse', ...
    'P', 'C', 'S', 'CFG', '-v7.3');
fprintf('  已保存: Part2_Complete_Results.mat\n\n');

%% ========================================================================
%% 完整可视化
%% ========================================================================

fprintf('========================================================================\n');
fprintf('  步骤7: 生成完整对比图表\n');
fprintf('========================================================================\n\n');

% 图1: 4方法箱线图对比
generate_4method_boxplot(MC_rmse_all, MC_control_all, methods, CFG);
fprintf('  图1: 4方法箱线图对比\n');

% 图2: 单次轨迹对比
generate_trajectory_comparison({MC_results_PHGL{1}, MC_results_ABC{1}, ...
    MC_results_MRAC{1}, MC_results_MPC{1}}, methods, S, CFG);
fprintf('  图2: 单次轨迹对比\n');

% 图3: 误差时间序列对比
generate_error_timeseries({MC_results_PHGL{1}, MC_results_ABC{1}, ...
    MC_results_MRAC{1}, MC_results_MPC{1}}, methods, CFG);
fprintf('  图3: 误差时间序列\n');

% 图4: 消融实验结果
generate_ablation_plot(ablation_configs, ablation_rmse, CFG);
fprintf('  图4: 消融实验结果\n');

% 图5: 失效模式分析
generate_failure_mode_plot(failure_modes, failure_rmse, CFG);
fprintf('  图5: 失效模式分析\n');

% 图6: PHGL诊断图
generate_PHGL_diagnostics(MC_results_PHGL{1}, P, C, CFG);
fprintf('  图6: PHGL诊断图\n\n');

fprintf('========================================================================\n');
fprintf('  Part 2 完成！\n');
fprintf('========================================================================\n');
fprintf('  所有图表已保存到: %s\n', CFG.DIR);
fprintf('  完整结果文件: Part2_Complete_Results.mat\n');
fprintf('========================================================================\n\n');


%% ########################################################################
%% ########################################################################
%%                         函 数 定 义 区
%% ########################################################################
%% ########################################################################

%% ========================================================================
%% MRAC 相关函数
%% ========================================================================

function MRAC = init_MRAC_params()
    MRAC.kp = 80;
    MRAC.kd = 30;
    MRAC.Gamma = diag([1.5, 1.5, 0.8]);
    MRAC.sigma = 0.02;
    MRAC.theta_min = [0.01; 0.01; 0.01];
    MRAC.theta_max = [5.0; 5.0; 10.0];
end

function result = run_MRAC_controller(MRAC, P, C, S)
    q = S.q0(:);
    p = S.p0(:);
    theta_mrac = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
    T_He = P.T_day;
    B = P.rho_air * P.V_env * P.g;

    rng(S.wind_seed);
    wind_noise_state = zeros(3, 1);

    n_save = floor(S.N_steps / S.save_interval) + 1;
    result.time  = zeros(1, n_save);
    result.q     = zeros(3, n_save);
    result.error = zeros(3, n_save);
    result.q_d   = zeros(3, n_save);
    save_idx = 1;
    error_sq_sum = 0;
    control_sum  = 0;

    for step = 1:S.N_steps
        t = (step - 1) * S.dt;
        [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S);
        [d_wind, wind_noise_state] = generate_wind_disturbance(t, wind_noise_state, S);

        e_q = q - q_d;
        M_c = mass_matrix(q(3), theta_mrac, P);
        v = M_c \ p;
        e_v = v - dq_d;

        u = -MRAC.kp * e_q - MRAC.kd * e_v;

        regressor = [v(1)*e_v(1); v(2)*e_v(2); v(3)*e_v(3)];
        theta_dot = MRAC.Gamma * regressor - MRAC.sigma * theta_mrac;
        theta_mrac = theta_mrac + S.dt * theta_dot;
        theta_mrac = max(theta_mrac, MRAC.theta_min);
        theta_mrac = min(theta_mrac, MRAC.theta_max);

        u = apply_saturation(u, C);
        [q, p] = symplectic_integrator(q, p, u, d_wind, ...
            [P.ma_x_true; P.ma_y_true; P.Ia_psi_true], B, P, S.dt);
        [T_He, B] = update_thermal_dynamics(T_He, t, P, S.dt);

        error_sq_sum = error_sq_sum + norm(q(1:2) - q_d(1:2))^2 * S.dt;
        control_sum  = control_sum + norm(u) * S.dt;

        if mod(step, S.save_interval) == 0
            result.time(save_idx)  = t;
            result.q(:, save_idx)  = q;
            result.error(:, save_idx) = q - q_d;
            result.q_d(:, save_idx) = q_d;
            save_idx = save_idx + 1;
        end
    end
    result.rmse = sqrt(error_sq_sum / S.T_total);
    result.control_effort = control_sum / S.T_total;
end

%% ========================================================================
%% MPC 相关函数
%% ========================================================================

function MPC = init_MPC_params()
    MPC.N_horizon = 10;
    MPC.N_control = 5;
    MPC.Q = diag([50, 50, 30]);
    MPC.R = diag([1, 1, 1]);
    MPC.Qf_factor = 10;
end

function result = run_MPC_controller(MPC, P, C, S)
    q = S.q0(:);
    p = S.p0(:);
    theta_mpc = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
    T_He = P.T_day;
    B = P.rho_air * P.V_env * P.g;

    rng(S.wind_seed);
    wind_noise_state = zeros(3, 1);

    n_save = floor(S.N_steps / S.save_interval) + 1;
    result.time  = zeros(1, n_save);
    result.q     = zeros(3, n_save);
    result.error = zeros(3, n_save);
    result.q_d   = zeros(3, n_save);
    save_idx = 1;
    error_sq_sum = 0;
    control_sum  = 0;

    for step = 1:S.N_steps
        t = (step - 1) * S.dt;
        [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S);
        [d_wind, wind_noise_state] = generate_wind_disturbance(t, wind_noise_state, S);

        e_q = q - q_d;
        M_c = mass_matrix(q(3), theta_mpc, P);
        v = M_c \ p;
        e_v = v - dq_d;
        M_d = mass_matrix(q_d(3), theta_mpc, P);
        C_d = coriolis_matrix(q_d(3), dq_d, theta_mpc, P);

        u = -MPC.Q * e_q - diag([10,10,5]) * e_v + M_d * ddq_d + C_d * dq_d;
        u = apply_saturation(u, C);

        [q, p] = symplectic_integrator(q, p, u, d_wind, ...
            [P.ma_x_true; P.ma_y_true; P.Ia_psi_true], B, P, S.dt);
        [T_He, B] = update_thermal_dynamics(T_He, t, P, S.dt);

        error_sq_sum = error_sq_sum + norm(q(1:2) - q_d(1:2))^2 * S.dt;
        control_sum  = control_sum + norm(u) * S.dt;

        if mod(step, S.save_interval) == 0
            result.time(save_idx)  = t;
            result.q(:, save_idx)  = q;
            result.error(:, save_idx) = q - q_d;
            result.q_d(:, save_idx) = q_d;
            save_idx = save_idx + 1;
        end
    end
    result.rmse = sqrt(error_sq_sum / S.T_total);
    result.control_effort = control_sum / S.T_total;
end

%% ========================================================================
%% 消融实验函数
%% ========================================================================

function result = run_ablation_test(flags, P, C, S)
    q = S.q0(:);
    p = zeros(3, 1);

    theta = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];

    T_He = P.T_day;
    B = P.rho_air * P.V_env * P.g;

    ctrl_data = init_controller_data('PHGL', C, P);
    ctrl_data.theta = theta;

    rng(S.wind_seed);
    wind_noise_state = zeros(3, 1);

    error_sq_sum = 0;

    for step = 1:S.N_steps
        t = (step - 1) * S.dt;
        [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S);
        [d_wind, wind_noise_state] = generate_wind_disturbance(t, wind_noise_state, S);

        % ---------- 控制律 ----------
        e_q = q - q_d;
        M_c = mass_matrix(q(3), ctrl_data.theta, P);
        M_inv = robust_inv(M_c, P);
        M_d = mass_matrix(q_d(3), ctrl_data.theta, P);
        p_d = M_d * dq_d;
        e_p = p - p_d;

        if flags.use_PH
            u1 = -C.K_e * e_q;
            u2 = -C.K_d * (M_inv * e_p);
            C_d = coriolis_matrix(q_d(3), dq_d, ctrl_data.theta, P);
            u3 = M_d * ddq_d + C_d * dq_d;
        else
            v = M_inv * p;
            u1 = -C.K_e * e_q;
            u2 = -C.K_d * (v - dq_d);
            u3 = zeros(3,1);
        end

        if flags.use_GP
            u4 = -ctrl_data.d_wind_est;
        else
            u4 = zeros(3,1);
        end

        if flags.use_thermal
            f_buoy = (B - P.m_hull*P.g) * P.z_b * sin(q(3));
        else
            f_buoy = 0;
        end
        u5 = [0; 0; -f_buoy];

        u = u1 + u2 + u3 + u4 + u5;
        u = apply_saturation(u, C);

        p_old = p;
        [q, p] = symplectic_integrator(q, p, u, d_wind, ...
            [P.ma_x_true; P.ma_y_true; P.Ia_psi_true], B, P, S.dt);

        if flags.use_thermal
            [T_He, B] = update_thermal_dynamics(T_He, t, P, S.dt);
        end

        % ---------- 参数更新 ----------
        if flags.use_OC_EKF && mod(step, C.EKF_update_interval) == 0
            psi = q(3);
            theta_true_vec = [P.ma_x_true; P.ma_y_true; P.Ia_psi_true];
            M_true_sensor = mass_matrix(psi, theta_true_vec, P);
            v_measured = M_true_sensor \ p + C.v_noise_std .* randn(3,1);

            [ctrl_data.theta, ctrl_data.P_theta, ctrl_data.alpha_obs] = ...
                observability_constrained_EKF( ...
                    ctrl_data.theta, ctrl_data.P_theta, ...
                    psi, p, v_measured, ctrl_data.Obs_history, ...
                    P, C, flags.use_obs_constraint);

            H = compute_observation_jacobian(psi, p, ctrl_data.theta, P);
            H_flat = H(:)';
            ctrl_data.Obs_history = [ctrl_data.Obs_history; H_flat];
            if size(ctrl_data.Obs_history, 1) > C.T_window
                ctrl_data.Obs_history(1, :) = [];
            end
        end

        % ---------- GP风场 ----------
        if flags.use_GP
            wind_residual = estimate_wind_residual( ...
                p_old, p, q, u, ctrl_data.theta, B, P, S.dt);
            ctrl_data = gp_add_observation(ctrl_data, q(1:2), t, wind_residual, C);

            if (t - ctrl_data.GP_last_update_time) >= C.GP_update_interval * S.dt
                ctrl_data.d_wind_est = gp_predict_wind(q(1:2), t, ctrl_data, C);
                ctrl_data.GP_last_update_time = t;
            end
        end

        error_sq_sum = error_sq_sum + norm(q(1:2) - q_d(1:2))^2 * S.dt;
    end

    result.rmse = sqrt(error_sq_sum / S.T_total);
end

%% ========================================================================
%% 完整控制器循环（用于失效模式测试等）
%% ========================================================================

function result = run_controller_full(method_name, P, C, S)
    q = S.q0(:);
    p = zeros(3, 1);
    theta_est = [P.ma_x_init; P.ma_y_init; P.Ia_psi_init];
    theta_true = [P.ma_x_true; P.ma_y_true; P.Ia_psi_true];
    T_He = P.T_day;
    B = P.rho_air * P.V_env * P.g;

    ctrl_data = init_controller_data(method_name, C, P);

    rng(S.wind_seed);
    wind_noise_state = zeros(3, 1);

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
    error_sq_sum = 0;
    control_sum  = 0;

    for step = 1:S.N_steps
        t = (step - 1) * S.dt;
        [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S);
        [d_wind, wind_noise_state] = generate_wind_disturbance(t, wind_noise_state, S);

        switch method_name
            case 'PHGL'
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
        p_old = p;
        [q, p] = symplectic_integrator(q, p, u, d_wind, theta_true, B, P, S.dt);

        if strcmp(method_name, 'PHGL')
            wind_residual = estimate_wind_residual( ...
                p_old, p, q, u, theta_est, B, P, S.dt);
            ctrl_data = gp_add_observation(ctrl_data, q(1:2), t, wind_residual, C);
        end

        [T_He, B] = update_thermal_dynamics(T_He, t, P, S.dt);

        error_sq_sum = error_sq_sum + norm(q(1:2) - q_d(1:2))^2 * S.dt;
        control_sum  = control_sum + norm(u) * S.dt;

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
        ctrl_data.Obs_history = [];
        ctrl_data.alpha_obs   = 0;
        ctrl_data.d_wind_est  = zeros(3, 1);
        ctrl_data.GP_obs_X     = zeros(2, 0);
        ctrl_data.GP_obs_T     = zeros(1, 0);
        ctrl_data.GP_obs_Y     = zeros(3, 0);
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
    q   = q(:);   p   = p(:);
    q_d = q_d(:); dq_d = dq_d(:); ddq_d = ddq_d(:);
    theta = ctrl_data.theta(:);

    e_q = q - q_d;
    M_d = mass_matrix(q_d(3), theta, P);
    p_d = M_d * dq_d;
    e_p = p - p_d;
    M_curr = mass_matrix(q(3), theta, P);
    M_inv  = robust_inv(M_curr, P);

    u1 = -C.K_e * e_q;
    u2 = -C.K_d * (M_inv * e_p);
    C_des = coriolis_matrix(q_d(3), dq_d, theta, P);
    u3 = M_d * ddq_d + C_des * dq_d;
    u4 = -ctrl_data.d_wind_est;
    f_buoy_psi = (B - P.m_hull * P.g) * P.z_b * sin(q(3));
    u5 = [0; 0; -f_buoy_psi];

    u = u1 + u2 + u3 + u4 + u5;
    u = u(:);

    % OC-EKF 参数更新
    step_now = round(t / S.dt) + 1;
    if mod(step_now, C.EKF_update_interval) == 0
        psi = q(3);
        [theta_new, P_new, alpha_obs] = observability_constrained_EKF( ...
            ctrl_data.theta, ctrl_data.P_theta, ...
            psi, p, v_measured, ctrl_data.Obs_history, P, C, true);

        H = compute_observation_jacobian(psi, p, ctrl_data.theta, P);
        H_flat = H(:)';
        ctrl_data.Obs_history = [ctrl_data.Obs_history; H_flat];
        if size(ctrl_data.Obs_history, 1) > C.T_window
            ctrl_data.Obs_history(1, :) = [];
        end

        ctrl_data.theta   = theta_new;
        ctrl_data.P_theta = P_new;
        ctrl_data.alpha_obs = alpha_obs;
    end

    % GP 风场预测
    if (t - ctrl_data.GP_last_update_time) >= C.GP_update_interval * S.dt
        ctrl_data.d_wind_est = gp_predict_wind(q(1:2), t, ctrl_data, C);
        ctrl_data.GP_last_update_time = t;
    end
end

%% ========================================================================
%% OC-EKF
%% ========================================================================

function [theta_new, P_new, alpha_obs] = observability_constrained_EKF( ...
        theta, P_theta, psi, p, v, Obs_history, P, C, use_constraint)
    theta = theta(:);
    theta_pred = theta;
    P_pred = P_theta + C.Q_theta;

    M_pred = mass_matrix(psi, theta_pred, P);
    v_pred = M_pred \ p;
    innovation = v - v_pred;

    H = compute_observation_jacobian(psi, p, theta_pred, P);

    alpha_obs = 0;
    P_obs = eye(3);

    n_blocks = size(Obs_history, 1);
    if n_blocks >= C.OBS_MIN_SAMPLES
        G_obs = zeros(3, 3);
        for i = 1:n_blocks
            Hi = reshape(Obs_history(i, :), 3, 3);
            G_obs = G_obs + Hi' * Hi;
        end
        G_obs = G_obs / n_blocks;
        reg = C.GRAMIAN_REG * trace(G_obs) / 3;
        G_obs = G_obs + reg * eye(3);

        [U, Sigma, ~] = svd(G_obs);
        sv = diag(Sigma);
        alpha_obs = sv(end) / (sv(1) + 1e-12);
        mask = sv > C.alpha_PE * sv(1);
        U_r = U(:, mask);
        P_obs = U_r * U_r';
    end

    S_inn = H * P_pred * H' + C.R_theta;
    if cond(S_inn) > 1e8
        S_inn = S_inn + 1e-6 * trace(S_inn)/3 * eye(3);
    end
    K = P_pred * H' / S_inn;

    if use_constraint
        if alpha_obs > C.alpha_PE
            theta_new = theta_pred + P_obs * K * innovation;
        else
            theta_new = theta_pred;
        end
    else
        theta_new = theta_pred + K * innovation;
    end

    theta_new = max(theta_new, [0.01; 0.01; 0.01]);
    theta_new = min(theta_new, [5.0; 5.0; 10.0]);

    IKH = eye(3) - K * H;
    P_new = IKH * P_pred * IKH' + K * C.R_theta * K';
    P_new = (P_new + P_new') / 2 + 1e-8 * eye(3);
end

function H = compute_observation_jacobian(psi, p, theta, P)
    theta = theta(:);
    eps_val = 1e-7;
    H = zeros(3, 3);
    for i = 1:3
        th_p = theta; th_p(i) = th_p(i) + eps_val;
        th_m = theta; th_m(i) = th_m(i) - eps_val;
        v_p = mass_matrix(psi, th_p, P) \ p;
        v_m = mass_matrix(psi, th_m, P) \ p;
        H(:, i) = (v_p - v_m) / (2 * eps_val);
    end
end

%% ========================================================================
%% GP 在线风场推断
%% ========================================================================

function ctrl_data = gp_add_observation(ctrl_data, x_pos, t, y_wind, C)
    ctrl_data.GP_obs_X = [ctrl_data.GP_obs_X, x_pos(:)];
    ctrl_data.GP_obs_T = [ctrl_data.GP_obs_T, t];
    ctrl_data.GP_obs_Y = [ctrl_data.GP_obs_Y, y_wind(:)];
    if size(ctrl_data.GP_obs_X, 2) > C.GP_N_window
        ctrl_data.GP_obs_X(:, 1) = [];
        ctrl_data.GP_obs_T(1)    = [];
        ctrl_data.GP_obs_Y(:, 1) = [];
    end
end

function d_pred = gp_predict_wind(x_query, t_query, ctrl_data, C)
    N = size(ctrl_data.GP_obs_X, 2);
    if N < 5
        d_pred = zeros(3, 1);
        return;
    end
    X = ctrl_data.GP_obs_X;
    T = ctrl_data.GP_obs_T;
    Y = ctrl_data.GP_obs_Y;

    K_mat  = zeros(N, N);
    k_star = zeros(N, 1);
    sf2 = C.sigma_f^2; ls2 = C.ell_s^2; lt2 = C.ell_t^2;

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
    M_c = mass_matrix(q(3), theta_est, P);
    v = M_c \ p_old;
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

    phi = [v(1)*e_v(1); v(2)*e_v(2); v(3)*e_v(3)];
    theta_dot = C.ABC_Gamma * phi - C.ABC_sigma * theta_abc;
    ctrl_data.theta_abc = theta_abc + S.dt * theta_dot;
    ctrl_data.theta_abc = max(ctrl_data.theta_abc, C.ABC_proj_min);
    ctrl_data.theta_abc = min(ctrl_data.theta_abc, C.ABC_proj_max);

    M_est = mass_matrix(q_d(3), ctrl_data.theta_abc, P);
    C_est = coriolis_matrix(q_d(3), dq_d, ctrl_data.theta_abc, P);
    f_buoy_comp = [0; 0; -(B - P.m_hull*P.g) * P.z_b * sin(q(3))];

    u = -C.ABC_kp * e_q - C.ABC_kd * e_v ...
        + M_est * ddq_d + C_est * dq_d + f_buoy_comp;
    u = u(:);
end

%% ========================================================================
%% 动力学模型（与 Part 1 完全一致）
%% ========================================================================

function M_total = mass_matrix(psi, theta, P)
    theta = theta(:);
    M_RB = diag([P.m_hull, P.m_hull, P.I_zz]);
    c = cos(psi); s = sin(psi);
    R2 = [c, -s; s, c];
    M_A_2d = diag([theta(1), theta(2)]);
    M_A_trans = R2 * M_A_2d * R2';
    M_total = M_RB;
    M_total(1:2, 1:2) = M_total(1:2, 1:2) + M_A_trans;
    M_total(3,3)       = M_total(3,3) + theta(3);   % [Fix-1]
end

function C_mat = coriolis_matrix(psi, dq, theta, P)
    dq = dq(:); theta = theta(:);
    dpsi = dq(3);
    delta_ma = theta(2) - theta(1);
    dM12 = -delta_ma * cos(2*psi);
    dM11 =  delta_ma * sin(2*psi);
    dM22 = -delta_ma * sin(2*psi);
    C_mat = dpsi * [dM11, dM12, 0; dM12, dM22, 0; 0, 0, 0];
end

function dH_dpsi = compute_dH_dpsi(psi, p, theta, P)
    eps_psi = 1e-7;
    M_plus  = mass_matrix(psi + eps_psi, theta, P);
    M_minus = mass_matrix(psi - eps_psi, theta, P);
    dMinv_dpsi = (M_plus \ eye(3) - M_minus \ eye(3)) / (2 * eps_psi);
    dH_dpsi = 0.5 * p' * dMinv_dpsi * p + P.k_psi * psi;
end

function [q_new, p_new] = symplectic_integrator(q, p, u, d_wind, theta, B, P, dt)
    q = q(:); p = p(:); u = u(:); d_wind = d_wind(:); theta = theta(:);
    M_c  = mass_matrix(q(3), theta, P);
    M_inv = robust_inv(M_c, P);
    v = M_inv * p;
    q_half = q + 0.5 * dt * v;

    dH_dpsi = compute_dH_dpsi(q_half(3), p, theta, P);
    grad_H_q = [0; 0; dH_dpsi];
    D_v = [P.D_x; P.D_y; P.D_psi] .* v;
    f_buoy = [0; 0; (B - P.m_hull*P.g) * P.z_b * sin(q_half(3))];

    f_total = -grad_H_q - D_v + u + d_wind + f_buoy;
    p_new = p + dt * f_total;

    M_half = mass_matrix(q_half(3), theta, P);
    v_new  = robust_inv(M_half, P) * p_new;
    q_new  = q_half + 0.5 * dt * v_new;
end

%% ========================================================================
%% 辅助函数
%% ========================================================================

function [q_d, dq_d, ddq_d] = get_reference_trajectory(t, S)
    q_d   = [S.A_x * sin(S.omega_x * t);
             S.A_y * sin(S.omega_y * t); 0];
    dq_d  = [S.A_x * S.omega_x * cos(S.omega_x * t);
             S.A_y * S.omega_y * cos(S.omega_y * t); 0];
    ddq_d = [-S.A_x * S.omega_x^2 * sin(S.omega_x * t);
             -S.A_y * S.omega_y^2 * sin(S.omega_y * t); 0];
end

function [d_wind, noise_state] = generate_wind_disturbance(t, noise_state, S)
    omega1 = 2*pi / 600; omega2 = 2*pi / 300;
    d_base = [S.wind_base_amp(1) * sin(omega1*t);
              S.wind_base_amp(2) * cos(omega2*t);
              S.wind_base_amp(3) * sin(omega1*t + pi/3)];
    tau = S.wind_ou_tau;
    decay = exp(-S.dt / tau);
    diffusion = S.wind_ou_sigma .* sqrt(1 - decay^2);
    noise_state = noise_state * decay + diffusion .* randn(3, 1);
    d_wind = d_base + noise_state;
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
    if cond(M) > P.cond_threshold
        reg = P.reg_factor * trace(M) / size(M, 1);
        M = M + reg * eye(size(M));
    end
    M_inv = M \ eye(size(M));
end

%% ========================================================================
%% 可视化函数
%% ========================================================================

function generate_4method_boxplot(rmse_all, control_all, methods, CFG)
    figure('Position', [100, 100, 1400, 600], 'Color', 'w');
    subplot(1,2,1);
    boxplot(rmse_all, 'Labels', methods);
    ylabel('RMSE [m]', 'FontSize', 13, 'FontWeight', 'bold');
    title(sprintf('跟踪精度对比（%d次MC）', CFG.N_MC), 'FontSize', 14);
    grid on; set(gca, 'FontSize', 11);
    subplot(1,2,2);
    boxplot(control_all, 'Labels', methods);
    ylabel('平均控制幅值 [N]', 'FontSize', 13, 'FontWeight', 'bold');
    title(sprintf('控制能耗对比（%d次MC）', CFG.N_MC), 'FontSize', 14);
    grid on; set(gca, 'FontSize', 11);
    saveas(gcf, [CFG.DIR 'Fig1_4Method_Boxplot.png']);
    saveas(gcf, [CFG.DIR 'Fig1_4Method_Boxplot.fig']);
end

function generate_trajectory_comparison(results, methods, S, CFG)
    figure('Position', [100, 100, 1600, 600], 'Color', 'w');
    colors = {'b', 'r', 'm', 'g'};
    t_ref = linspace(0, S.T_total, 1000);
    q_ref_x = S.A_x * sin(S.omega_x * t_ref);
    q_ref_y = S.A_y * sin(S.omega_y * t_ref);
    for i = 1:4
        subplot(2,2,i);
        plot(q_ref_x, q_ref_y, 'k--', 'LineWidth', 1.5, 'DisplayName', '参考轨迹');
        hold on;
        plot(results{i}.q(1,:), results{i}.q(2,:), colors{i}, 'LineWidth', 1.5, ...
            'DisplayName', methods{i});
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('%s (RMSE=%.3fm)', methods{i}, results{i}.rmse), 'FontSize', 12);
        legend('Location', 'best'); grid on; axis equal;
    end
    saveas(gcf, [CFG.DIR 'Fig2_Trajectory_Comparison.png']);
    saveas(gcf, [CFG.DIR 'Fig2_Trajectory_Comparison.fig']);
end

function generate_error_timeseries(results, methods, CFG)
    figure('Position', [100, 100, 1400, 800], 'Color', 'w');
    colors = {'b', 'r', 'm', 'g'};
    labels = {'e_x [m]', 'e_y [m]', 'e_\psi [rad]'};
    for i = 1:3
        subplot(3,1,i);
        for j = 1:4
            plot(results{j}.time, results{j}.error(i,:), colors{j}, ...
                'LineWidth', 1.2, 'DisplayName', methods{j});
            hold on;
        end
        ylabel(labels{i}, 'FontSize', 11, 'FontWeight', 'bold');
        if i == 1
            title('跟踪误差时间序列', 'FontSize', 13);
            legend('Location', 'best', 'Orientation', 'horizontal');
        end
        grid on;
        if i == 3, xlabel('时间 [s]', 'FontSize', 11); end
    end
    saveas(gcf, [CFG.DIR 'Fig3_Error_Timeseries.png']);
    saveas(gcf, [CFG.DIR 'Fig3_Error_Timeseries.fig']);
end

function generate_ablation_plot(configs, rmse_matrix, CFG)
    figure('Position', [100, 100, 1200, 600], 'Color', 'w');
    means = mean(rmse_matrix, 2);
    stds = std(rmse_matrix, 0, 2);
    
    % 使用颜色区分
    colors = [0.2 0.6 0.2;    % PHGL完整 - 绿色
              0.8 0.2 0.2;    % 无OC-EKF - 红色
              0.9 0.4 0.1;    % 无可观性 - 橙色
              0.2 0.4 0.8;    % 无GP - 蓝色
              0.6 0.6 0.6;    % 无热补偿 - 灰色
              0.7 0.2 0.7];   % 无PH - 紫色
    
    b = bar(means);
    b.FaceColor = 'flat';
    for i = 1:length(means)
        b.CData(i,:) = colors(i,:);
    end
    hold on;
    errorbar(1:size(configs,1), means, stds, 'k.', 'LineWidth', 1.5);
    
    labels = cell(size(configs, 1), 1);
    for i = 1:size(configs, 1)
        labels{i} = configs{i, 2};
    end
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 15);
    ylabel('RMSE [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('消融实验结果（挑战性场景，各模块贡献）', 'FontSize', 13);
    grid on; set(gca, 'FontSize', 10);
    
    % 相对变化标签
    base = means(1);
    for i = 1:length(means)
        pct = (means(i) - base) / base * 100;
        if abs(pct) < 0.5
            lbl = sprintf('%.3f\n(基线)', means(i));
        else
            lbl = sprintf('%.3f\n(+%.1f%%)', means(i), pct);
        end
        text(i, means(i) + stds(i) + 0.02, lbl, ...
            'HorizontalAlignment', 'center', 'FontSize', 9);
    end
    
    saveas(gcf, [CFG.DIR 'Fig4_Ablation_Study.png']);
    saveas(gcf, [CFG.DIR 'Fig4_Ablation_Study.fig']);
end

function generate_failure_mode_plot(modes, rmse_matrix, CFG)
    figure('Position', [100, 100, 1200, 600], 'Color', 'w');
    means = mean(rmse_matrix, 2);
    stds = std(rmse_matrix, 0, 2);
    
    bar(means);
    hold on;
    errorbar(1:size(modes,1), means, stds, 'k.', 'LineWidth', 1.5);
    
    labels = cell(size(modes, 1), 1);
    for i = 1:size(modes, 1)
        labels{i} = modes{i, 2};
    end
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 15);
    ylabel('RMSE [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('失效模式分析（渐进式压力测试）', 'FontSize', 13);
    grid on;
    
    normal_rmse_val = means(1);
    yline(normal_rmse_val, 'k--', '正常水平', 'LineWidth', 1.5);
    yline(normal_rmse_val*1.5, 'y--', '退化阈值(50%)', 'LineWidth', 1);
    yline(normal_rmse_val*2.5, 'r--', '严重退化(150%)', 'LineWidth', 1);
    
    for i = 1:length(means)
        pct = (means(i) - normal_rmse_val) / normal_rmse_val * 100;
        text(i, means(i) + stds(i) + 0.01, ...
            sprintf('%.3f\n(%+.0f%%)', means(i), pct), ...
            'HorizontalAlignment', 'center', 'FontSize', 9);
    end
    
    saveas(gcf, [CFG.DIR 'Fig5_Failure_Mode.png']);
    saveas(gcf, [CFG.DIR 'Fig5_Failure_Mode.fig']);
end

function generate_PHGL_diagnostics(result, P, C, CFG)
    figure('Position', [100, 100, 1400, 800], 'Color', 'w');
    
    subplot(2,2,1);
    plot(result.time, result.theta(1,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'ma_x');
    hold on;
    plot(result.time, result.theta(2,:), 'g', 'LineWidth', 1.5, 'DisplayName', 'ma_y');
    plot(result.time, result.theta(3,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ia_\psi');
    yline(P.ma_x_true, 'r--', 'LineWidth', 1);
    yline(P.ma_y_true, 'g--', 'LineWidth', 1);
    yline(P.Ia_psi_true, 'b--', 'LineWidth', 1);
    xlabel('时间 [s]'); ylabel('参数估计 [kg, kg*m^2]');
    title('参数收敛曲线（OC-EKF）'); legend('Location', 'best'); grid on;
    
    subplot(2,2,2);
    semilogy(result.time, result.alpha_obs, 'b', 'LineWidth', 1.5);
    hold on;
    yline(C.alpha_PE, 'r--', sprintf('alpha_{PE}=%.0e', C.alpha_PE), 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('alpha_{obs}');
    title('可观性时间序列'); legend('Location', 'best'); grid on;
    
    subplot(2,2,3);
    plot(result.time, vecnorm(result.d_wind_est), 'b', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('风场估计幅值 [N]');
    title('稀疏GP风场推断'); grid on;
    
    subplot(2,2,4);
    plot(result.time, vecnorm(result.u), 'k', 'LineWidth', 1.5);
    hold on;
    yline(15, 'r--', '饱和限制', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('控制幅值 [N]');
    title('控制输入时间序列'); legend('Location', 'best'); grid on;
    
    saveas(gcf, [CFG.DIR 'Fig6_PHGL_Diagnostics.png']);
    saveas(gcf, [CFG.DIR 'Fig6_PHGL_Diagnostics.fig']);
end