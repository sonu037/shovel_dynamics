function P08_TD = build_M2_TorqueDriven(varargin)
%BUILD_M2_TORQUEDRIVEN  Programmatically builds M2/model/M2_TorqueDriven.slx and executes P08-TD validation.
%
%   PURPOSE (per CODE_EXPLAINABILITY_PROTOCOL.md & EXPERIMENT_DEFENSIBILITY_PROTOCOL.md):
%     Resolves Audit Finding #4 (MS02P08 Held-Out Validation) and Risk R-01
%     (Closed-Loop Instability under Stalling Load).
%
%     CRITICAL PHYSICAL DISCOVERY (Caught by Forward-Dynamic ODE Execution):
%       At d_4 in [5.5, 8.5] m (r_COG in [4.18, 7.18] m), the static gravity
%       torque of the empty 54,300 kg dipper+handle assembly alone is:
%         G_3(q) = M_d * g * r * cos(q_3) = 2.23 to 3.82 MN*m (and up to 4.4 MN*m
%         at d_4 = 9.5 m, as recorded in research_log.md 2026-07-06 session 2).
%       Setting a joint-space saddle torque limit of 1.25 MN*m (which confuses
%       hoist-drum shaft torque with saddle-pivot moment s_bail * F_hoist) is
%       smaller than static gravity G_3(q), causing 100% actuator saturation and
%       immediate gravity collapse of the boom/handle!
%       On the actual machine:
%         - Amlohri WK-20 max hoist force = 1,540 kN at s_bail*sin(alpha_3) ~ 6-8.5 m
%           -> Joint-space saddle torque ceiling tau_{3,max} = 9.50 MN*m (WK-20)
%              to 12.50 MN*m (WK-55, Bi et al. 2020 Table 2: F_hoist = 1,987 kN).
%         - Axial gravity on crowd is M_d * g * sin(q_3) = 377 kN at q_3 = 45 deg
%           -> Crowd force ceiling F_{4,max} = 745 kN (WK-20) to 1,002 kN (WK-55).
%
%   USAGE:
%     P08_TD = build_M2_TorqueDriven();
%     P08_TD = build_M2_TorqueDriven('MachineProfile', 'WK20');
%
%   Repository: M2/scripts/build_M2_TorqueDriven.m
%   Project:    IIT/SRIC/R/AEH/2026/104 (CMPDI) - IIT Kharagpur

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    if ~exist(fullfile(repo_root, 'M1'), 'dir'), repo_root = pwd; end

    ip = inputParser;
    ip.addParameter('BaseModel',      fullfile(repo_root, 'M2', 'model', 'M2_LoadInjection.slx'), @ischar);
    ip.addParameter('TargetModel',    fullfile(repo_root, 'M2', 'model', 'M2_TorqueDriven.slx'),  @ischar);
    ip.addParameter('MachineProfile', 'PH2100', @(x) any(validatestring(x, {'PH2100', 'WK20'})));
    ip.addParameter('Wn_Hz',          1.5,      @(x) isscalar(x) && x > 0);
    ip.addParameter('Zeta',           1.0,      @(x) isscalar(x) && x > 0);
    ip.addParameter('StopTime',       30.0,     @(x) isscalar(x) && x > 5);
    ip.addParameter('TrimStart',      5.0,      @(x) isscalar(x) && x >= 0);
    ip.addParameter('ObserverGain',   20.0,     @(x) isscalar(x) && x > 0);
    ip.addParameter('OutFile',        fullfile(repo_root, 'M2', 'experiments', '08_held_out', 'M2_p08_torque_driven_results.mat'), @ischar);
    ip.addParameter('Save',           true,     @islogical);
    ip.parse(varargin{:});
    opt = ip.Results;

    fprintf('======================================================================\n');
    fprintf('  M2 P08-TD: TORQUE-DRIVEN FORWARD DYNAMICS & DEFLECTION VALIDATION\n');
    fprintf('  Machine Profile: %s | Controller: Computed-Torque + PD (wn=%.2f Hz, zeta=%.1f)\n', ...
        opt.MachineProfile, opt.Wn_Hz, opt.Zeta);
    fprintf('======================================================================\n\n');

    %% 1. Load Physical Parameters & Verified Joint-Space Actuator Limits
    P = load_machine_profile(opt.MachineProfile);
    wn = 2.0 * pi * opt.Wn_Hz;
    ctrl = struct();
    ctrl.wn_rad_s  = wn;
    ctrl.zeta      = opt.Zeta;
    ctrl.Kp        = diag([wn^2, wn^2]);
    ctrl.Kv        = diag([2.0 * opt.Zeta * wn, 2.0 * opt.Zeta * wn]);
    ctrl.tau3_max  = P.tau3_max;
    ctrl.f4_max    = P.f4_max;

    % Static gravity sanity gate (prevents Risk R-01 gravity collapse):
    max_static_G3 = P.Md * P.g * (8.50 + P.c);
    if ctrl.tau3_max <= 1.2 * max_static_G3
        error('build_M2_TorqueDriven:InsufficientSaddleTorqueLimit', ...
            'tau3_max (%.2f MNm) is below 1.2x static gravity torque (%.2f MNm); handle will collapse.', ...
            ctrl.tau3_max/1e6, max_static_G3/1e6);
    end

    %% 2. Programmatically Clone & Configure M2_TorqueDriven.slx (if Simulink present)
    slx_status = configure_simulink_torque_model(opt.BaseModel, opt.TargetModel, P, ctrl);

    %% 3. Define the 6 P08-TD Calibration & Held-Out Test Cases
    cases = {
        'P08_CAL_01',            'Calibration: 100 kN Harmonic Bail Load (1/9 Hz)';
        'P08_CAL_02',            'Calibration: Multi-Step Load (0 -> 150 -> 300 kN)';
        'P08_HELD_STEP_IMPACT',  'Held-Out 1: Severe 450 kN Impact Step (20 ms ramp)';
        'P08_HELD_STOCHASTIC',   'Held-Out 2: Band-Limited Stochastic Cutting Load';
        'P08_HELD_OBLIQUE_BANK', 'Held-Out 3: Oblique Cutting Vector (Fx=120 kN, Fy=280 kN)';
        'P08_HELD_REVERSAL_STALL','Held-Out 4: Velocity Reversal Stall & Torque Saturation'
    };

    P08_TD = struct();
    P08_TD.metadata.date            = '2026-11-09';
    P08_TD.metadata.machine_profile = opt.MachineProfile;
    P08_TD.metadata.controller      = ctrl;
    P08_TD.metadata.slx_status      = slx_status;
    P08_TD.cases                    = struct();

    fprintf('%-24s | %-11s | %-11s | %-12s | %-12s | %-12s | %-12s\n', ...
        'Case ID', 'Max |dq3|', 'Max |dd4|', 'M2 NRMSE_q3', 'M2 NRMSE_d4', 'M3 LagNRMSE3', 'M3 LagNRMSE4');
    fprintf('%s\n', repmat('-', 1, 108));

    for idx = 1:size(cases, 1)
        cid  = cases{idx, 1};
        desc = cases{idx, 2};
        res  = simulate_torque_driven_case(cid, desc, P, ctrl, opt);
        P08_TD.cases.(cid) = res;

        fprintf('%-24s | %7.3f deg | %8.4f m | %10.3e %% | %10.3e %% | %10.3f %% | %10.3f %%\n', ...
            cid, res.deflection.max_abs_dq3_deg, res.deflection.max_abs_dd4_m, ...
            res.m2_sensed_ddq.nrmse_q3_pct, res.m2_sensed_ddq.nrmse_d4_pct, ...
            res.m3_30Hz_foh.lag_comp_nrmse_q3_pct, res.m3_30Hz_foh.lag_comp_nrmse_d4_pct);
    end

    if opt.Save
        out_dir = fileparts(opt.OutFile);
        if ~isempty(out_dir) && ~exist(out_dir, 'dir'), mkdir(out_dir); end
        save(opt.OutFile, 'P08_TD');
        fprintf('\nSaved P08-TD Torque-Driven Validation results to: %s\n', opt.OutFile);
    end
end

%% ========================================================================
%  HELPER 1: MACHINE PARAMETER & ACTUATOR ENVELOPE LOADER
%  ========================================================================
function P = load_machine_profile(profile_name)
    if strcmpi(profile_name, 'WK20') && exist('shovel_params_wk20', 'file') == 2
        P = shovel_params_wk20();
        % Joint-space saddle torque ceiling = s_bail * sin(alpha3) * F_hoist_max (~9.50 MN*m)
        P.tau3_max = 9.50e6;
        P.f4_max   = P.F_crowd_max; % 745 kN (Amlohri WK-20 spec sheet)
    else
        if exist('shovel_params', 'file') == 2
            P = shovel_params();
        else
            P = struct('g', 9.80665, 'm', 54300, 'L_COG', 2.82, 'x_ref', 1.50, ...
                       'c', -1.32, 'Izz', 287900, 'fv3', 3800, 'fc3', 340, ...
                       'fv4', 1473, 'fc4', 480, 'model_has_coulomb', false);
        end
        P.Md       = P.m;
        P.eps3     = 3e-4;
        P.eps4     = 3e-4;
        % Joint-space saddle torque ceiling from hoist rope (F_hoist_max = 1,540-1,987 kN at ~6.2 m arm):
        P.tau3_max = 9.50e6; % [N*m] 9.50 MN*m joint-space saddle torque limit
        P.f4_max   = 7.45e5; % [N]   745 kN crowd force limit (exceeds 377 kN gravity + cutting load)
    end
end

%% ========================================================================
%  HELPER 2: NON-DESTRUCTIVE SIMULINK MODEL CLONER & RE-WIRER
%  ========================================================================
function status_msg = configure_simulink_torque_model(base_slx, target_slx, P, ctrl)
    if exist('load_system', 'builtin') == 0 && exist('load_system', 'file') == 0
        status_msg = 'Standalone ODE mode (Simulink runtime not loaded in current session)';
        return;
    end
    if exist(base_slx, 'file') ~= 2
        [~, fn, ext] = fileparts(base_slx);
        if exist([fn ext], 'file') == 2
            base_slx = [fn ext];
        else
            status_msg = sprintf('Base model "%s" not on path; ran verified forward ODE solver.', base_slx);
            return;
        end
    end

    target_dir = fileparts(target_slx);
    if ~isempty(target_dir) && ~exist(target_dir, 'dir'), mkdir(target_dir); end
    copyfile(base_slx, target_slx, 'f');

    [~, mdl_name] = fileparts(target_slx);
    if bdIsLoaded(mdl_name), close_system(mdl_name, 0); end
    load_system(target_slx);

    rev_blks  = find_system(mdl_name, 'ReferenceBlock', sprintf('sm_lib/Joints/Revolute\nJoint'));
    pris_blks = find_system(mdl_name, 'ReferenceBlock', sprintf('sm_lib/Joints/Prismatic\nJoint'));
    if ~isempty(rev_blks)
        set_param(rev_blks{1}, 'TorqueActuationMode', 'InputTorque', 'MotionActuationMode', 'ComputedMotion');
    end
    if ~isempty(pris_blks)
        set_param(pris_blks{1}, 'ForceActuationMode', 'InputForce', 'MotionActuationMode', 'ComputedMotion');
    end

    mw = get_param(mdl_name, 'ModelWorkspace');
    mw.assignin('Kp_q3',    ctrl.Kp(1,1));
    mw.assignin('Kp_d4',    ctrl.Kp(2,2));
    mw.assignin('Kv_q3',    ctrl.Kv(1,1));
    mw.assignin('Kv_d4',    ctrl.Kv(2,2));
    mw.assignin('tau3_max', ctrl.tau3_max);
    mw.assignin('f4_max',   ctrl.f4_max);
    mw.assignin('P_struct', P);

    save_system(mdl_name, target_slx);
    close_system(mdl_name, 0);
    status_msg = sprintf('Cloned and configured "%s" with Torque/Force joint actuation.', target_slx);
end

%% ========================================================================
%  HELPER 3: FORWARD-DYNAMIC CLOSED-LOOP SIMULATOR (RK4 AT dt = 1.0 ms)
%  ========================================================================
function res = simulate_torque_driven_case(cid, desc, P, ctrl, opt)
    dt = 0.001; % 1.0 ms continuous plant integration step (1000 Hz)
    t  = (0:dt:opt.StopTime)';
    N  = numel(t);

    [q3_d, d4_d, q3d_d, d4d_d, q3dd_d, d4dd_d] = generate_desired_trajectory(cid, t);
    [Fx, Fy, station_offset] = generate_external_force_profile(cid, t);

    x = [q3_d(1); d4_d(1); q3d_d(1); d4d_d(1)];
    q3   = zeros(N, 1); d4   = zeros(N, 1);
    q3d  = zeros(N, 1); d4d  = zeros(N, 1);
    q3dd = zeros(N, 1); d4dd = zeros(N, 1);
    tau3_a = zeros(N, 1); f4_a = zeros(N, 1);
    Qq3_true = zeros(N, 1); Qd4_true = zeros(N, 1);
    sat_flag = false(N, 1);

    p_prev = zeros(2, 1);
    h_prev = zeros(2, 1);

    for k = 1:N
        q_k  = x(1:2);
        dq_k = x(3:4);
        qd_k   = [q3_d(k); d4_d(k)];
        dqd_k  = [q3d_d(k); d4d_d(k)];
        ddqd_k = [q3dd_d(k); d4dd_d(k)];

        [D_k, C_k, G_k, Fv_k, Fc_k] = plant_matrices(q_k, dq_k, P);
        v_ctrl = ddqd_k + ctrl.Kv * (dqd_k - dq_k) + ctrl.Kp * (qd_k - q_k);
        tau_unsat = D_k * v_ctrl + C_k * dq_k + G_k + Fv_k;
        tau_cmd = [min(ctrl.tau3_max, max(-ctrl.tau3_max, tau_unsat(1)));
                   min(ctrl.f4_max,   max(-ctrl.f4_max,   tau_unsat(2)))];
        if abs(tau_unsat(1)) > ctrl.tau3_max || abs(tau_unsat(2)) > ctrl.f4_max
            sat_flag(k) = true;
        end

        s_k = q_k(2) + station_offset;
        Q_k = [ s_k * (Fy(k) * cos(q_k(1)) - Fx(k) * sin(q_k(1)));
                Fx(k) * cos(q_k(1)) + Fy(k) * sin(q_k(1)) ];

        ddq_k = D_k \ (tau_cmd + Q_k - C_k * dq_k - G_k - Fv_k - Fc_k);

        q3(k) = q_k(1);   d4(k) = q_k(2);
        q3d(k) = dq_k(1); d4d(k) = dq_k(2);
        q3dd(k) = ddq_k(1); d4dd(k) = ddq_k(2);
        tau3_a(k) = tau_cmd(1); f4_a(k) = tau_cmd(2);
        Qq3_true(k) = Q_k(1);   Qd4_true(k) = Q_k(2);

        if k < N
            F_k = [Fx(k); Fy(k)];
            x = rk4_plant_step(x, qd_k, dqd_k, ddqd_k, F_k, station_offset, P, ctrl, dt);
        end
    end

    % Evaluate M2 Algebraic Recovery on Deflected Trajectory (with Sensed ddq):
    r_vec = d4 + P.c;
    tau3_model = (P.Izz + P.Md .* r_vec.^2) .* q3dd ...
               + 2.0 * P.Md .* r_vec .* d4d .* q3d ...
               + P.Md * P.g .* r_vec .* cos(q3) ...
               + P.fv3 .* q3d + P.fc3 .* tanh(q3d / P.eps3);
    f4_model   = P.Md .* d4dd - P.Md .* r_vec .* (q3d.^2) ...
               + P.Md * P.g .* sin(q3) ...
               + P.fv4 .* d4d + P.fc4 .* tanh(d4d / P.eps4);

    Q3_hat_m2 = tau3_model - tau3_a;
    Q4_hat_m2 = f4_model   - f4_a;

    % Evaluate Native 30 Hz Discrete FOH Momentum Observer (NO ddq used):
    Ts_obs = 1.0 / 30.0;
    t_30   = (0:Ts_obs:opt.StopTime)';
    N_30   = numel(t_30);
    q3_30  = interp1(t, q3, t_30, 'linear');
    d4_30  = interp1(t, d4, t_30, 'linear');
    q3d_30 = interp1(t, q3d, t_30, 'linear');
    d4d_30 = interp1(t, d4d, t_30, 'linear');
    tau3_30 = interp1(t, tau3_a, t_30, 'linear');
    f4_30   = interp1(t, f4_a, t_30, 'linear');
    Qq3_30  = interp1(t, Qq3_true, t_30, 'linear');
    Qd4_30  = interp1(t, Qd4_true, t_30, 'linear');

    kO = opt.ObserverGain;
    phi_d = exp(-kO * Ts_obs);
    beta1 = (1.0 - (1.0 - phi_d) / (kO * Ts_obs)) / kO;
    beta0 = (1.0 - phi_d) / kO - beta1;

    z_foh = zeros(2, N_30);
    tau_hat_foh = zeros(2, N_30);
    tau_true_filt = zeros(2, N_30);

    for k = 1:N_30
        r_k = d4_30(k) + P.c;
        p_k = [ (P.Izz + P.Md * r_k^2) * q3d_30(k);
                 P.Md * d4d_30(k) ];
        G_k = [ P.Md * P.g * r_k * cos(q3_30(k));
                P.Md * P.g * sin(q3_30(k)) ];
        CT_dq_k = [ 0.0;
                    P.Md * r_k * (q3d_30(k)^2) ];
        Fv_k = [ P.fv3 * q3d_30(k);
                 P.fv4 * d4d_30(k) ];
        Fc_k = [ P.fc3 * tanh(q3d_30(k) / P.eps3);
                 P.fc4 * tanh(d4d_30(k) / P.eps4) ];
        h_k  = [tau3_30(k); f4_30(k)] - G_k - Fv_k - Fc_k + CT_dq_k;

        if k == 1
            z_foh(:, k) = p_k;
            tau_true_filt(:, k) = [Qq3_30(k); Qd4_30(k)];
        else
            u_prev = h_prev + kO * p_prev;
            u_k    = h_k    + kO * p_k;
            z_foh(:, k) = phi_d * z_foh(:, k-1) + beta0 * u_prev + beta1 * u_k;
            tau_true_filt(:, k) = phi_d * tau_true_filt(:, k-1) ...
                        + (beta0 * kO) * [Qq3_30(k-1); Qd4_30(k-1)] ...
                        + (beta1 * kO) * [Qq3_30(k); Qd4_30(k)];
        end
        tau_hat_foh(:, k) = kO * (p_k - z_foh(:, k));
        p_prev = p_k;
        h_prev = h_k;
    end

    mask_1k = (t >= opt.TrimStart);
    mask_30 = (t_30 >= opt.TrimStart);

    pk_q3 = max(abs(Qq3_true(mask_1k)));
    pk_d4 = max(abs(Qd4_true(mask_1k)));

    err_m2_q3 = Q3_hat_m2(mask_1k) - Qq3_true(mask_1k);
    err_m2_d4 = Q4_hat_m2(mask_1k) - Qd4_true(mask_1k);

    err_m3_raw_q3 = tau_hat_foh(1, mask_30)' - Qq3_30(mask_30);
    err_m3_raw_d4 = tau_hat_foh(2, mask_30)' - Qd4_30(mask_30);
    err_m3_lag_q3 = tau_hat_foh(1, mask_30)' - tau_true_filt(1, mask_30)';
    err_m3_lag_d4 = tau_hat_foh(2, mask_30)' - tau_true_filt(2, mask_30)';

    res = struct();
    res.case_id     = cid;
    res.description = desc;
    res.deflection.max_abs_dq3_rad = max(abs(q3 - q3_d));
    res.deflection.max_abs_dq3_deg = rad2deg(max(abs(q3 - q3_d)));
    res.deflection.max_abs_dd4_m   = max(abs(d4 - d4_d));
    res.deflection.max_norm_dq     = max(hypot(q3 - q3_d, d4 - d4_d));
    res.deflection.actuator_sat_pct = 100.0 * mean(double(sat_flag));

    res.m2_sensed_ddq.rms_q3_Nm    = sqrt(mean(err_m2_q3.^2));
    res.m2_sensed_ddq.rms_d4_N     = sqrt(mean(err_m2_d4.^2));
    res.m2_sensed_ddq.nrmse_q3_pct = 100.0 * res.m2_sensed_ddq.rms_q3_Nm / max(1.0, pk_q3);
    res.m2_sensed_ddq.nrmse_d4_pct = 100.0 * res.m2_sensed_ddq.rms_d4_N  / max(1.0, pk_d4);

    res.m3_30Hz_foh.raw_rms_q3_Nm       = sqrt(mean(err_m3_raw_q3.^2));
    res.m3_30Hz_foh.raw_rms_d4_N        = sqrt(mean(err_m3_raw_d4.^2));
    res.m3_30Hz_foh.raw_nrmse_q3_pct    = 100.0 * res.m3_30Hz_foh.raw_rms_q3_Nm / max(1.0, pk_q3);
    res.m3_30Hz_foh.raw_nrmse_d4_pct    = 100.0 * res.m3_30Hz_foh.raw_rms_d4_N  / max(1.0, pk_d4);
    res.m3_30Hz_foh.lag_comp_rms_q3_Nm  = sqrt(mean(err_m3_lag_q3.^2));
    res.m3_30Hz_foh.lag_comp_rms_d4_N   = sqrt(mean(err_m3_lag_d4.^2));
    res.m3_30Hz_foh.lag_comp_nrmse_q3_pct = 100.0 * res.m3_30Hz_foh.lag_comp_rms_q3_Nm / max(1.0, pk_q3);
    res.m3_30Hz_foh.lag_comp_nrmse_d4_pct = 100.0 * res.m3_30Hz_foh.lag_comp_rms_d4_N  / max(1.0, pk_d4);
end

function [D, C, G, Fv, Fc] = plant_matrices(q, dq, P)
    q3  = q(1);  d4  = q(2);
    q3d = dq(1); d4d = dq(2);
    r   = d4 + P.c;
    D   = [P.Izz + P.Md * r^2, 0.0;
           0.0,                P.Md];
    C   = [P.Md * r * d4d,  P.Md * r * q3d;
          -P.Md * r * q3d,  0.0];
    G   = [P.Md * P.g * r * cos(q3);
           P.Md * P.g * sin(q3)];
    Fv  = [P.fv3 * q3d;
           P.fv4 * d4d];
    Fc  = [P.fc3 * tanh(q3d / P.eps3);
           P.fc4 * tanh(d4d / P.eps4)];
end

function x_next = rk4_plant_step(x, qd, dqd, ddqd, F_ext, st_off, P, ctrl, dt)
    f = @(st) plant_rhs(st, qd, dqd, ddqd, F_ext, st_off, P, ctrl);
    k1 = f(x);
    k2 = f(x + 0.5 * dt * k1);
    k3 = f(x + 0.5 * dt * k2);
    k4 = f(x + dt * k3);
    x_next = x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
end

function dx = plant_rhs(x, qd, dqd, ddqd, F_ext, st_off, P, ctrl)
    q  = x(1:2);
    dq = x(3:4);
    [D, C, G, Fv, Fc] = plant_matrices(q, dq, P);
    v_ctrl = ddqd + ctrl.Kv * (dqd - dq) + ctrl.Kp * (qd - q);
    tau_u  = D * v_ctrl + C * dq + G + Fv;
    tau_a  = [min(ctrl.tau3_max, max(-ctrl.tau3_max, tau_u(1)));
              min(ctrl.f4_max,   max(-ctrl.f4_max,   tau_u(2)))];
    s = q(2) + st_off;
    Q_ext = [ s * (F_ext(2) * cos(q(1)) - F_ext(1) * sin(q(1)));
              F_ext(1) * cos(q(1)) + F_ext(2) * sin(q(1)) ];
    ddq = D \ (tau_a + Q_ext - C * dq - G - Fv - Fc);
    dx  = [dq; ddq];
end

function [q3_d, d4_d, q3d_d, d4d_d, q3dd_d, d4dd_d] = generate_desired_trajectory(cid, t)
    w = pi / 10.0;
    switch cid
        case 'P08_HELD_STEP_IMPACT'
            q3_d   = deg2rad(35.0) + deg2rad(10.0) * sin(0.20 * t);
            q3d_d  = deg2rad(10.0) * 0.20 * cos(0.20 * t);
            q3dd_d = -deg2rad(10.0) * (0.20^2) * sin(0.20 * t);
            d4_d   = 7.80 + 0.15 * sin(0.15 * t);
            d4d_d  = 0.15 * 0.15 * cos(0.15 * t);
            d4dd_d = -0.15 * (0.15^2) * sin(0.15 * t);
        case 'P08_HELD_OBLIQUE_BANK'
            q3_d   = deg2rad(42.0) + deg2rad(14.0) * sin(w * t) + deg2rad(3.0) * cos(2*w*t);
            q3d_d  = deg2rad(14.0)*w*cos(w*t) - deg2rad(3.0)*2*w*sin(2*w*t);
            q3dd_d = -deg2rad(14.0)*(w^2)*sin(w*t) - deg2rad(3.0)*4*(w^2)*cos(2*w*t);
            d4_d   = 7.00 + 1.40 * sin(w * t - 0.3);
            d4d_d  = 1.40 * w * cos(w * t - 0.3);
            d4dd_d = -1.40 * (w^2) * sin(w * t - 0.3);
        otherwise
            q3_d   = deg2rad(45.0) + deg2rad(15.0) * sin(w * t);
            q3d_d  = deg2rad(15.0) * w * cos(w * t);
            q3dd_d = -deg2rad(15.0) * (w^2) * sin(w * t);
            d4_d   = 7.00 + 1.50 * sin(w * t);
            d4d_d  = 1.50 * w * cos(w * t);
            d4dd_d = -1.50 * (w^2) * sin(w * t);
    end
end

function [Fx, Fy, st_off] = generate_external_force_profile(cid, t)
    st_off = 1.50;
    wF = 2.0 * pi / 9.0;
    Fx = zeros(size(t));
    Fy = zeros(size(t));
    switch cid
        case 'P08_CAL_01'
            Fy = -100e3 * sin(wF * t);
        case 'P08_CAL_02'
            Fy(t >= 8.0 & t < 16.0)  = -150e3;
            Fy(t >= 16.0 & t < 25.0) = -300e3;
        case 'P08_HELD_STEP_IMPACT'
            ramp = min(1.0, max(0.0, (t - 10.0) / 0.020)) - min(1.0, max(0.0, (t - 22.0) / 0.050));
            Fy   = -450e3 * ramp;
        case 'P08_HELD_STOCHASTIC'
            Fy = -180e3 * (1.0 + 0.35*sin(1.7*t) + 0.25*cos(3.9*t) + 0.18*sin(7.3*t) + 0.12*cos(12.1*t));
            Fx = -65e3  * (1.0 + 0.30*cos(2.1*t) + 0.20*sin(5.5*t));
        case 'P08_HELD_OBLIQUE_BANK'
            Fx = -120e3 * sin(wF * t + 0.4);
            Fy = -280e3 * sin(wF * t);
        case 'P08_HELD_REVERSAL_STALL'
            pulse = exp(-((t - 15.0) / 2.5).^2);
            Fy = -620e3 * pulse - 120e3 * sin(wF * t);
            Fx = -180e3 * pulse;
    end
end
