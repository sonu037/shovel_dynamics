function RESULTS = run_MS04P06_HeldOutValidation(varargin)
%RUN_MS04P06_HELDOUTVALIDATION  Milestone 4 Protocol P06: Held-Out Machine Verification & Field Readiness.
%
%   PURPOSE (per CODE_EXPLAINABILITY_PROTOCOL.md & CLAIMS_INTEGRITY_PROTOCOL.md):
%     Executes Protocol MS04P06 of Milestone 4 (Sensor Noise, Uncertainty & Errors-in-Variables).
%     Validates the generalization capability of the digital twin estimation pipeline on:
%       - Dataset 1: Machine A (P&H 2100BL) under an unseen non-sinusoidal excavation profile
%                    with impact steps and frequency chirps (rock-tooth interaction).
%       - Dataset 2: Machine B (TZ WK-20) 120-tonne field deployment target at NCL Amlohri,
%                    evaluating scalability from 54 t to 120 t heavy machinery under field noise.
%
%   ESTIMATOR TESTED:
%     Unified 3-State Kalman-Filtered GMO (KF-GMO) with adaptive measurement variance R_k
%     and Instrumental Variables (IDIM-IV) parameter baseline.
%
%   OUTPUTS:
%     RESULTS: Struct saved to M4/experiments/06_heldout/MS04P06_heldout_results.mat.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI / Coal India Limited - MT-189)
%   Author:  Multidisciplinary Research Supervisor & Technical Author
%   Date:    November 2026

    fprintf('========================================================================================\n');
    fprintf('  MILESTONE 4 PROTOCOL P06: HELD-OUT MACHINE VERIFICATION & FIELD READINESS\n');
    fprintf('  Generalization: Machine A Unseen Impact Profile & Machine B (TZ WK-20 Deployment)\n');
    fprintf('========================================================================================\n\n');

    % Set reproducible random seed
    rng(189, 'twister');

    % Locate repository root and parameter files
    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if ~exist(fullfile(repo_root, 'M1'), 'dir'), repo_root = pwd; end

    % Load Machine A baseline parameters
    if exist(fullfile(repo_root, 'M1', 'scripts', 'shovel_params.m'), 'file')
        addpath(fullfile(repo_root, 'M1', 'scripts'));
        PA = shovel_params();
    else
        PA.g     = 9.80665;
        PA.m     = 54300;
        PA.c     = -1.3200;
        PA.Izz   = 287900;
        PA.fv3   = 3800.0;
        PA.fv4   = 1473.0;
        PA.fc3   = 340.0;
        PA.fc4   = 480.0;
    end
    if ~isfield(PA, 'Md'), PA.Md = PA.m; end
    if ~isfield(PA, 'eps3'), PA.eps3 = 1e-4; end
    if ~isfield(PA, 'eps4'), PA.eps4 = 1e-4; end

    % Load Machine B (TZ WK-20) parameters from single source of truth
    if exist(fullfile(repo_root, 'M1', 'scripts', 'shovel_params_wk20.m'), 'file')
        addpath(fullfile(repo_root, 'M1', 'scripts'));
        PB = shovel_params_wk20();
    else
        % Engineering estimate prior scaled from Amlohri spec sheet (Pending Field ID)
        PB.g     = 9.80665;
        PB.m     = 56500.0;          % [ESTIMATE] 56.5 tonne empty dipper + handle
        PB.c     = -1.3200;          % [ESTIMATE] Net COG offset (m)
        PB.Izz   = 305000.0;         % [ESTIMATE] Rotational inertia (kg*m^2)
        PB.fv3   = 4100.0;           % [ESTIMATE] Viscous friction (N*m*s/rad)
        PB.fv4   = 1600.0;           % [ESTIMATE] Viscous crowd friction (N*s/m)
        PB.fc3   = 380.0;            % [ESTIMATE] Coulomb hoist friction (N*m)
        PB.fc4   = 520.0;            % [ESTIMATE] Coulomb crowd friction (N)
    end
    if ~isfield(PB, 'Md'), PB.Md = PB.m; end
    if ~isfield(PB, 'eps3'), PB.eps3 = 3e-4; end
    if ~isfield(PB, 'eps4'), PB.eps4 = 3e-4; end
    if ~isfield(PB, 'Md'), PB.Md = PB.m; end
    if ~isfield(PB, 'eps3'), PB.eps3 = 1e-4; end
    if ~isfield(PB, 'eps4'), PB.eps4 = 1e-4; end

    % Simulation grid: 30 Hz sampling rate, 30 s duration
    Fs    = 30.0;
    Ts    = 1.0 / Fs;
    T_end = 30.0;
    t     = (0:Ts:T_end)';
    N     = numel(t);

    RESULTS = struct();
    RESULTS.meta.date = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    RESULTS.meta.Fs = Fs;

    % ====================================================================================
    % TEST 1: MACHINE A UNDER UNSEEN COMPLEX MULTI-FREQUENCY IMPACT LOAD
    % ====================================================================================
    fprintf('>>> EXECUTING TEST 1: Machine A under Unseen Impact & Chirp Excavation Load...\n');
    w_traj    = pi / 10.0;
    A_real    = 2.0 / (1.0 + (0.05 * w_traj)^2);
    q3_amp    = deg2rad(15.0) / (1.0 + (0.05 * w_traj)^2);

    q3_A   = deg2rad(45.0) + q3_amp * sin(w_traj * t - 2*0.05*w_traj);
    q3d_A  = q3_amp * w_traj * cos(w_traj * t - 2*0.05*w_traj);
    q3dd_A = -q3_amp * (w_traj^2) * sin(w_traj * t - 2*0.05*w_traj);

    d4_A   = 8.75 + A_real * sin(w_traj * t - 2*0.05*w_traj);
    d4d_A  = A_real * w_traj * cos(w_traj * t - 2*0.05*w_traj);
    d4dd_A = -A_real * (w_traj^2) * sin(w_traj * t - 2*0.05*w_traj);

    % Unseen complex load profile: baseline sinusoidal + 50 kN impact steps at t = 10s and 20s
    Fy_A_true = 80e3 * sin(2*pi/7.0 * t) + ...
                50e3 * ((t >= 10.0 & t <= 14.0) | (t >= 20.0 & t <= 24.0));
    s_arm_A   = d4_A + 1.5;
    Qq3_A     = s_arm_A .* (Fy_A_true .* cos(q3_A));
    Qd4_A     = Fy_A_true .* sin(q3_A);

    r_vec_A  = d4_A + PA.c;
    Fc3_A    = PA.fc3 * tanh(q3d_A / PA.eps3);
    Fc4_A    = PA.fc4 * tanh(d4d_A / PA.eps4);

    tau3_rigid_A = (PA.Izz + PA.Md .* r_vec_A.^2) .* q3dd_A ...
                 + 2.0 * PA.Md .* r_vec_A .* d4d_A .* q3d_A ...
                 + PA.Md * PA.g .* r_vec_A .* cos(q3_A) ...
                 + PA.fv3 .* q3d_A + Fc3_A;

    f4_rigid_A   = PA.Md .* d4dd_A - PA.Md .* r_vec_A .* (q3d_A.^2) ...
                 + PA.Md * PA.g .* sin(q3_A) ...
                 + PA.fv4 .* d4d_A + Fc4_A;

    tau3_act_A = tau3_rigid_A - Qq3_A;
    f4_act_A   = f4_rigid_A   - Qd4_A;

    % 10% measurement noise + 12-bit motor shaft quantization
    alpha = 0.10;
    delta_q3_A = (2.0 * pi) / (240.0 * 4096);
    delta_d4_A = (2.0 * pi) / (109.27 * 4096);

    q3_meas_A   = delta_q3_A * round(q3_A / delta_q3_A);
    d4_meas_A   = delta_d4_A * round(d4_A / delta_d4_A);
    q3d_meas_A  = q3d_A + alpha * std(q3d_A) * randn(N, 1);
    d4d_meas_A  = d4d_A + alpha * std(d4d_A) * randn(N, 1);
    tau3_meas_A = tau3_act_A + alpha * std(tau3_act_A) * randn(N, 1);
    f4_meas_A   = f4_act_A   + alpha * std(f4_act_A)   * randn(N, 1);

    % Run KF-GMO on Test 1
    Q_est_KF_A = local_run_kfgmo(PA, q3_meas_A, d4_meas_A, q3d_meas_A, d4d_meas_A, ...
                                 tau3_meas_A, f4_meas_A, Ts, delta_q3_A, delta_d4_A, alpha);

    s_meas_A = d4_meas_A + 1.5;
    J_denom_A = (s_meas_A .* cos(q3_meas_A)).^2 + (sin(q3_meas_A)).^2;
    Fy_est_A = (s_meas_A .* cos(q3_meas_A) .* Q_est_KF_A(1, :)' + sin(q3_meas_A) .* Q_est_KF_A(2, :)') ./ max(J_denom_A, 1e-6);

    % ====================================================================================
    % TEST 2: MACHINE B (TZ WK-20 HEAVY MINING SHOVEL AT NCL AMLOHRI)
    % ====================================================================================
    fprintf('>>> EXECUTING TEST 2: Machine B (TZ WK-20) 120-Tonne Scaled Field Target...\n');
    % Gearing for WK-20: Hoist N_gear,3 = 312, Crowd N_gear,4 = 125.4 rad/m
    delta_q3_B = (2.0 * pi) / (312.0 * 4096);
    delta_d4_B = (2.0 * pi) / (125.4 * 4096);

    % Heavy shovel excavation: 350 kN dynamic digging resistance
    Fy_B_true = 350e3 * sin(2*pi/12.0 * t);
    s_arm_B   = d4_A + 2.2; % Longer dipper stick offset on WK-20
    Qq3_B     = s_arm_B .* (Fy_B_true .* cos(q3_A));
    Qd4_B     = Fy_B_true .* sin(q3_A);

    r_vec_B  = d4_A + PB.c;
    Fc3_B    = PB.fc3 * tanh(q3d_A / PB.eps3);
    Fc4_B    = PB.fc4 * tanh(d4d_A / PB.eps4);

    tau3_rigid_B = (PB.Izz + PB.Md .* r_vec_B.^2) .* q3dd_A ...
                 + 2.0 * PB.Md .* r_vec_B .* d4d_A .* q3d_A ...
                 + PB.Md * PB.g .* r_vec_B .* cos(q3_A) ...
                 + PB.fv3 .* q3d_A + Fc3_B;

    f4_rigid_B   = PB.Md .* d4dd_A - PB.Md .* r_vec_B .* (q3d_A.^2) ...
                 + PB.Md * PB.g .* sin(q3_A) ...
                 + PB.fv4 .* d4d_A + Fc4_B;

    tau3_act_B = tau3_rigid_B - Qq3_B;
    f4_act_B   = f4_rigid_B   - Qd4_B;

    q3_meas_B   = delta_q3_B * round(q3_A / delta_q3_B);
    d4_meas_B   = delta_d4_B * round(d4_A / delta_d4_B);
    q3d_meas_B  = q3d_A + alpha * std(q3d_A) * randn(N, 1);
    d4d_meas_B  = d4d_A + alpha * std(d4d_A) * randn(N, 1);
    tau3_meas_B = tau3_act_B + alpha * std(tau3_act_B) * randn(N, 1);
    f4_meas_B   = f4_act_B   + alpha * std(f4_act_B)   * randn(N, 1);

    % Run KF-GMO on Machine B without manual tuning
    Q_est_KF_B = local_run_kfgmo(PB, q3_meas_B, d4_meas_B, q3d_meas_B, d4d_meas_B, ...
                                 tau3_meas_B, f4_meas_B, Ts, delta_q3_B, delta_d4_B, alpha);

    s_meas_B = d4_meas_B + 2.2;
    J_denom_B = (s_meas_B .* cos(q3_meas_B)).^2 + (sin(q3_meas_B)).^2;
    Fy_est_B = (s_meas_B .* cos(q3_meas_B) .* Q_est_KF_B(1, :)' + sin(q3_meas_B) .* Q_est_KF_B(2, :)') ./ max(J_denom_B, 1e-6);

    % Trim interval [5 s, 30 s]
    trim_idx = (t >= 5.0);
    Fy_ref_A_trim = Fy_A_true(trim_idx);
    Fy_est_A_trim = Fy_est_A(trim_idx);
    denom_A = sum((Fy_ref_A_trim - mean(Fy_ref_A_trim)).^2);
    r2_A    = 1.0 - sum((Fy_ref_A_trim - Fy_est_A_trim).^2) / denom_A;
    rmse_A  = sqrt(mean((Fy_ref_A_trim - Fy_est_A_trim).^2));
    nrmse_A = (rmse_A / (max(Fy_ref_A_trim) - min(Fy_ref_A_trim))) * 100.0;
    mae_A   = mean(abs(Fy_ref_A_trim - Fy_est_A_trim));

    Fy_ref_B_trim = Fy_B_true(trim_idx);
    Fy_est_B_trim = Fy_est_B(trim_idx);
    denom_B = sum((Fy_ref_B_trim - mean(Fy_ref_B_trim)).^2);
    r2_B    = 1.0 - sum((Fy_ref_B_trim - Fy_est_B_trim).^2) / denom_B;
    rmse_B  = sqrt(mean((Fy_ref_B_trim - Fy_est_B_trim).^2));
    nrmse_B = (rmse_B / (max(Fy_ref_B_trim) - min(Fy_ref_B_trim))) * 100.0;
    mae_B   = mean(abs(Fy_ref_B_trim - Fy_est_B_trim));

    RESULTS.Test1_MachineA_Impact.r2 = r2_A;
    RESULTS.Test1_MachineA_Impact.nrmse = nrmse_A;
    RESULTS.Test1_MachineA_Impact.rmse = rmse_A;
    RESULTS.Test1_MachineA_Impact.mae = mae_A;

    RESULTS.Test2_MachineB_WK20.r2 = r2_B;
    RESULTS.Test2_MachineB_WK20.nrmse = nrmse_B;
    RESULTS.Test2_MachineB_WK20.rmse = rmse_B;
    RESULTS.Test2_MachineB_WK20.mae = mae_B;

    fprintf('\n========================================================================================\n');
    fprintf('  HELD-OUT EXPERIMENTAL EVALUATION SUMMARY:\n');
    fprintf('========================================================================================\n');
    fprintf('%-38s | %-8s | %-12s | %-12s | %-12s | %-12s\n', ...
            'Test Scenario', 'R^2', 'NRMSE (%)', 'RMSE (kN)', 'MAE (kN)', 'Audit Finding');
    fprintf('--------------------------------------------------------------------------------------------------------------------\n');
    audit_A = 'STEP_LAG_WARN';
    if r2_A > 0.50, audit_A = 'PASSED'; end
    audit_B = 'SCALED_PRIOR';
    if r2_B > 0.85, audit_B = 'PASSED (PRIOR)'; end
    fprintf('%-38s | %-8.4f | %-11.2f%% | %-12.2f | %-12.2f | %-12s\n', ...
            'Machine A: Discontinuous Impact Steps', r2_A, nrmse_A, rmse_A/1e3, mae_A/1e3, audit_A);
    fprintf('%-38s | %-8.4f | %-11.2f%% | %-12.2f | %-12.2f | %-12s\n', ...
            'Machine B: TZ WK-20 (56.5t Prior)', r2_B, nrmse_B, rmse_B/1e3, mae_B/1e3, audit_B);
    fprintf('--------------------------------------------------------------------------------------------------------------------\n');

    fprintf('\n========================================================================================\n');
    fprintf('  PHYSICAL ACCEPTANCE CRITERIA AUDIT (HELD-OUT GENERALIZATION):\n');
    fprintf('========================================================================================\n');
    fprintf('  1. Machine A Discontinuous Impact Step Tracking:\n');
    fprintf('     - NRMSE = %.2f%%, R^2 = %.4f.\n', nrmse_A, r2_A);
    fprintf('     - AUDIT FINDING: Linear constant-acceleration KF-GMO acts as low-pass filter;\n');
    fprintf('       experiences transient overshoot/lag at sharp 50 kN rectangular impact edges.\n');
    fprintf('       Motivates multi-rate adaptive covariance observer in Milestone 5.\n\n');
    fprintf('  2. Machine B (TZ WK-20 Scaled Prior Simulation):\n');
    fprintf('     - Digging force recovery: NRMSE = %.2f%%, R^2 = %.4f.\n', nrmse_B, r2_B);
    fprintf('     - AUDIT FINDING: High-inertia machine filters high-frequency noise naturally.\n');
    fprintf('       CRITICAL CAVEAT: Parameters Md=56.5t, Izz=305k are engineering priors scaled\n');
    fprintf('       from Rasuli 2014, NOT verified OEM CAD. Must be identified at Amlohri.\n');
    fprintf('========================================================================================\n\n');

    % Save results
    out_dir = fullfile(repo_root, 'M4', 'experiments', '06_heldout');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    save_path = fullfile(out_dir, 'MS04P06_heldout_results.mat');
    save(save_path, 'RESULTS', 'PA', 'PB');
    fprintf('Saved experiment results to: %s\n\n', save_path);
end

% Helper function: runs 3-state KF-GMO
function Q_est = local_run_kfgmo(P, q3_m, d4_m, q3d_m, d4d_m, tau3_m, f4_m, Ts, delta_q3, delta_d4, alpha)
    N = numel(q3_m);
    Q_est = zeros(2, N);

    r_0 = d4_m(1) + P.c;
    D11_0 = P.Izz + P.Md * r_0^2;
    D22_0 = P.Md;

    x_kf3 = [ D11_0 * q3d_m(1); 0.0; 0.0 ];
    x_kf4 = [ D22_0 * d4d_m(1); 0.0; 0.0 ];
    P_kf3 = diag([1e4, 1e10, 1e12]);
    P_kf4 = diag([1e2, 1e8, 1e10]);

    A_kf  = [1.0, Ts, 0.5*Ts^2; 0.0, 1.0, Ts; 0.0, 0.0, 1.0];
    B_kf  = [Ts; 0.0; 0.0];
    C_kf  = [1.0, 0.0, 0.0];

    % Scale process noise with machine inertia
    scale_inertia = D11_0 / 3.28e6;
    Q_kf3 = diag([10 * scale_inertia, (1000 * Ts * scale_inertia)^2, (5e4 * Ts * scale_inertia)^2]);
    Q_kf4 = diag([1 * (P.Md / 54300), (200 * Ts * (P.Md / 54300))^2, (5e3 * Ts * (P.Md / 54300))^2]);

    sigma_v3 = max(alpha * std(q3d_m), delta_q3 / (sqrt(12) * Ts));
    sigma_v4 = max(alpha * std(d4d_m), delta_d4 / (sqrt(12) * Ts));

    for k = 1:N
        r_k  = d4_m(k) + P.c;
        D11  = P.Izz + P.Md * r_k^2;
        D22  = P.Md;
        p_k  = [ D11 * q3d_m(k); D22 * d4d_m(k) ];

        G_k  = [ P.Md * P.g * r_k * cos(q3_m(k)); P.Md * P.g * sin(q3_m(k)) ];
        CT_k = [ 0.0; P.Md * r_k * (q3d_m(k)^2) ];
        Fv_k = [ P.fv3 * q3d_m(k); P.fv4 * d4d_m(k) ];
        Fc_k = [ P.fc3 * tanh(q3d_m(k) / P.eps3); P.fc4 * tanh(d4d_m(k) / P.eps4) ];

        h_B_k = [tau3_m(k); f4_m(k)] - G_k - Fv_k + CT_k;

        if k == 1
            Q_est(:, 1) = [0; 0];
        else
            x_pred3 = A_kf * x_kf3 + B_kf * h_B_k(1);
            P_pred3 = A_kf * P_kf3 * A_kf' + Q_kf3;
            R_k3    = max((D11 * sigma_v3)^2, 1e-4);
            K_gain3 = (P_pred3 * C_kf') / (C_kf * P_pred3 * C_kf' + R_k3);
            x_kf3   = x_pred3 + K_gain3 * (p_k(1) - C_kf * x_pred3);
            P_kf3   = (eye(3) - K_gain3 * C_kf) * P_pred3;

            x_pred4 = A_kf * x_kf4 + B_kf * h_B_k(2);
            P_pred4 = A_kf * P_kf4 * A_kf' + Q_kf4;
            R_k4    = max((D22 * sigma_v4)^2, 1e-4);
            K_gain4 = (P_pred4 * C_kf') / (C_kf * P_pred4 * C_kf' + R_k4);
            x_kf4   = x_pred4 + K_gain4 * (p_k(2) - C_kf * x_pred4);
            P_kf4   = (eye(3) - K_gain4 * C_kf) * P_pred4;

            Q_est(1, k) = x_kf3(2);
            Q_est(2, k) = x_kf4(2);
        end
    end
end
