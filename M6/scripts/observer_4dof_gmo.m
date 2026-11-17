function out = observer_4dof_gmo(u_in, y_meas, dt, state, P)
%OBSERVER_4DOF_GMO  Dual-Layer De Luca-Spong 4-DOF Momentum Observer for WK-20.
%
%   MS06P03: Implements the discrete-time (30 Hz) dual-layer generalized momentum
%   observer decoupling high-frequency hoist wire-rope compliance (Layer A) from
%   3D link-side swing/crowd/saddle external disturbance estimation (Layer B)
%   for the Taiyuan Heavy Industry WK-20 mining rope shovel.
%
%   Observer Architecture:
%     Layer A (Winch Drum Tension Observer):
%       p_drum = M_drum * x_drum_dot
%       dot(z_drum) = F_hoist_mot - F_f_drum + K_O_drum * (p_drum - z_drum)
%       hat_T_rope  = K_O_drum * (p_drum - z_drum)
%
%     Layer B (3D Link-Side Momentum Observer):
%       q_L = [q1; q3; d4], p_L = M_L(q_L) * qdot_L
%       tau_L = [tau_swing; -hat_T_rope*(dell/dq3); F_crowd - hat_T_rope*(dell/dd4)] - tau_f_L
%       dot(z_L) = tau_L + C_L^T*qdot_L - G_L + K_O_L * (p_L - z_L)
%       hat_tau_ext_raw = K_O_L * (p_L - z_L)
%       hat_tau_ext_L   = Notch_Filter(hat_tau_ext_raw, 4.35 Hz)
%
%     3D Tooth-Tip Force Inversion:
%       F_tip_3D = inv(J_tip_3D') * hat_tau_ext_L
%       F_lat = hat_tau_ext_1 / rho_tip(q3, d4) [Lateral side-raking bank force]
%
%   USAGE:
%     out = observer_4dof_gmo(); % Standalone verification suite (MS06P03)
%     out = observer_4dof_gmo(u_in, y_meas, dt, state, P);
%
%   See also KINEMATICS_DYNAMICS_4DOF, ELASTIC_ROPE_SUBSYSTEM, SHOVEL_PARAMS_WK20.

    persistent path_added;
    if isempty(path_added)
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        addpath(fullfile(repo_root, 'M1', 'scripts'));
        addpath(fullfile(repo_root, 'M6', 'scripts'));
        path_added = true;
    end

    % Standalone verification mode
    if nargin == 0
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        out = run_ms06p03_verification(repo_root);
        return;
    end

    if nargin < 5 || isempty(P), P = shovel_params_wk20(); end
    if nargin < 3 || isempty(dt), dt = P.Ts_observer; end % 1/30 s

    % Parse inputs:
    % u_in: Actuator commands [tau_swing; F_crowd; F_hoist]
    % y_meas: Sensor measurements [q (4x1); qdot (4x1)]
    q = y_meas(1:4);
    qdot = y_meas(5:8);

    % Observer gains
    K_O_drum = 200.0; % [rad/s] Layer A winch drum observer gain
    K_O_L    = diag([20.0, 5.0, 20.0]); % [rad/s] Layer B link-side observer gains

    % -------------------------------------------------------------------------
    % 1. Plant Kinematics & Dynamics Evaluation
    % -------------------------------------------------------------------------
    dyn = kinematics_dynamics_4dof(q, qdot, P);
    rope = elastic_rope_subsystem(q, qdot, P);

    M_L = dyn.M(1:3, 1:3);
    C_L = dyn.C(1:3, 1:3);
    G_L = dyn.G(1:3);
    M_drum = dyn.M(4,4);

    % Link and drum generalized momenta
    p_L    = M_L * qdot(1:3);
    p_drum = M_drum * qdot(4);

    % Notch filter coefficients for 4.35 Hz bounce suppression
    f0 = 4.35;
    w0 = 2 * pi * f0 * dt;
    r_notch = 0.85;
    b_notch = [1.0, -2.0 * cos(w0), 1.0];
    a_notch = [1.0, -2.0 * r_notch * cos(w0), r_notch^2];
    b_notch = b_notch * (sum(a_notch) / sum(b_notch)); % Exact unity DC gain

    % Initialize state if empty
    if nargin < 4 || isempty(state)
        state = struct();
        state.z_drum = p_drum;
        state.z_L    = p_L;
        F_f_init = 1500.0 * qdot(4) + 2500.0 * tanh(qdot(4) / 0.01);
        state.u_B_drum_prev = u_in(3) - F_f_init + K_O_drum * p_drum;
        tau_f_init = [ 12000.0 * qdot(1) + 8000.0 * tanh(qdot(1) / 0.005);
                       25000.0 * qdot(2) + 15000.0 * tanh(qdot(2) / 0.005);
                       18000.0 * qdot(3) + 12000.0 * tanh(qdot(3) / 0.005) ];
        tau_L_init = [ u_in(1); -rope.T_rope * rope.dell_dq3; u_in(2) - rope.T_rope * rope.dell_dd4 ] - tau_f_init;
        state.u_B_L_prev     = tau_L_init + (C_L' * qdot(1:3)) - G_L + K_O_L * p_L;
        state.raw_hist       = zeros(3, 2);
        state.filt_hist      = zeros(3, 2);
        state.hat_T_rope     = 0.0;
        state.hat_tau_ext_L  = zeros(3, 1);
    end

    % -------------------------------------------------------------------------
    % 2. Layer A: Winch Drum Tension Observer (hat_T_rope)
    % -------------------------------------------------------------------------
    F_f_drum = 1500.0 * qdot(4) + 2500.0 * tanh(qdot(4) / 0.01);
    F_hoist_mot = u_in(3);

    % FOH discrete update for Layer A
    u_B_drum = F_hoist_mot - F_f_drum + K_O_drum * p_drum;

    phi_drum = exp(-K_O_drum * dt);
    beta1_drum = (1.0 - (1.0 - phi_drum) / (K_O_drum * dt)) / K_O_drum;
    beta0_drum = (1.0 - phi_drum) / K_O_drum - beta1_drum;

    z_drum_next = phi_drum * state.z_drum + beta0_drum * state.u_B_drum_prev + beta1_drum * u_B_drum;

    % Synchronized Drum Momentum Residual
    hat_T_rope = K_O_drum * (p_drum - z_drum_next);

    % -------------------------------------------------------------------------
    % 3. Layer B: 3D Link-Side Momentum Observer (hat_tau_ext_L)
    % -------------------------------------------------------------------------
    tau_swing_mot = u_in(1);
    F_crowd_mot   = u_in(2);

    tau_f_swing = 12000.0 * qdot(1) + 8000.0 * tanh(qdot(1) / 0.005);
    tau_f_pitch = 25000.0 * qdot(2) + 15000.0 * tanh(qdot(2) / 0.005);
    F_f_crowd   = 18000.0 * qdot(3) + 12000.0 * tanh(qdot(3) / 0.005);
    tau_f_L = [tau_f_swing; tau_f_pitch; F_f_crowd];

    tau_L = [ tau_swing_mot;
             -hat_T_rope * rope.dell_dq3;
              F_crowd_mot - hat_T_rope * rope.dell_dd4 ] - tau_f_L;

    % FOH discrete update for Layer B
    u_B_L = tau_L + (C_L' * qdot(1:3)) - G_L + K_O_L * p_L;

    z_L_next = zeros(3, 1);
    for m = 1:3
        k_m = K_O_L(m, m);
        phi_m = exp(-k_m * dt);
        beta1_m = (1.0 - (1.0 - phi_m) / (k_m * dt)) / k_m;
        beta0_m = (1.0 - phi_m) / k_m - beta1_m;
        z_L_next(m) = phi_m * state.z_L(m) + beta0_m * state.u_B_L_prev(m) + beta1_m * u_B_L(m);
    end

    % Synchronized Raw Momentum Residual
    hat_tau_ext_raw = K_O_L * (p_L - z_L_next);

    % Apply 4.35 Hz Notch Filter to eliminate wire-rope bounce ringing
    hat_tau_ext_L = zeros(3, 1);
    raw_hist_next = zeros(3, 2);
    filt_hist_next = zeros(3, 2);

    for m = 1:3
        x_m = hat_tau_ext_raw(m);
        x_m_1 = state.raw_hist(m, 1);
        x_m_2 = state.raw_hist(m, 2);
        y_m_1 = state.filt_hist(m, 1);
        y_m_2 = state.filt_hist(m, 2);

        y_m = (b_notch(1)*x_m + b_notch(2)*x_m_1 + b_notch(3)*x_m_2 ...
               - a_notch(2)*y_m_1 - a_notch(3)*y_m_2) / a_notch(1);

        hat_tau_ext_L(m) = y_m;
        raw_hist_next(m, :)  = [x_m, x_m_1];
        filt_hist_next(m, :) = [y_m, y_m_1];
    end

    % -------------------------------------------------------------------------
    % 4. 3D Spatial Tooth-Tip Force Inversion
    % -------------------------------------------------------------------------
    J_tip = dyn.J_tip_3D;
    F_tip_3D = (J_tip') \ hat_tau_ext_L;

    % Lateral side-raking bank force (perpendicular to handle pitch plane)
    F_lat = hat_tau_ext_L(1) / dyn.rho_tip;

    % Package updated state
    next_state = struct();
    next_state.z_drum         = z_drum_next;
    next_state.z_L            = z_L_next;
    next_state.u_B_drum_prev  = u_B_drum;
    next_state.u_B_L_prev     = u_B_L;
    next_state.raw_hist       = raw_hist_next;
    next_state.filt_hist      = filt_hist_next;
    next_state.hat_T_rope     = hat_T_rope;
    next_state.hat_tau_ext_L  = hat_tau_ext_L;

    out = struct();
    out.hat_T_rope     = hat_T_rope;
    out.hat_tau_ext_L  = hat_tau_ext_L;
    out.F_tip_3D       = F_tip_3D;
    out.F_lat          = F_lat;
    out.rho_tip        = dyn.rho_tip;
    out.state          = next_state;
end

%% ========================================================================
%  STANDALONE PROTOCOL VERIFICATION SUITE (MS06P03)
%  ========================================================================
function report = run_ms06p03_verification(repo_root)
    fprintf('========================================================================================\n');
    fprintf(' RUNNING PROTOCOL MS06P03: DUAL-LAYER DE LUCA-SPONG 4-DOF MOMENTUM OBSERVER\n');
    fprintf(' Target Equipment: Taiyuan Heavy Industry WK-20 (Discrete 30 Hz PLC Execution)\n');
    fprintf('========================================================================================\n\n');

    P = shovel_params_wk20();
    dt = P.Ts_observer; % 1/30 s = 33.333 ms

    % -------------------------------------------------------------------------
    % Test 1: Layer A Winch Drum Tension Tracking
    % -------------------------------------------------------------------------
    fprintf('--- [Test 1] Layer A Winch Drum Tension Observer (Step Load 450 kN) ---\n');
    dt_plant = 1e-3;
    t_end = 2.0;
    t_sim = 0:dt_plant:t_end;
    N_steps = numel(t_sim);

    M_drum = 229520.0;
    x_drum = 0.0;
    x_drum_dot = 0.0;

    T_step_mag = 450000.0; % [N]
    T_true_hist = zeros(size(t_sim));
    T_hat_hist  = zeros(size(t_sim));

    obs_state = [];
    t_last_obs = -dt;

    for i = 1:N_steps
        t_now = t_sim(i);
        if t_now >= 0.5
            T_true = T_step_mag;
            F_hoist_mot = -T_step_mag; % Motor balances load
        else
            T_true = 0.0;
            F_hoist_mot = 0.0;
        end
        T_true_hist(i) = T_true;

        % Plant acceleration: M_drum * xddot = F_hoist_mot + T_true - F_f_drum
        F_f_drum = 1500.0 * x_drum_dot + 2500.0 * tanh(x_drum_dot / 0.01);
        x_drum_ddot = (F_hoist_mot + T_true - F_f_drum) / M_drum;

        x_drum_dot = x_drum_dot + dt_plant * x_drum_ddot;
        x_drum     = x_drum + dt_plant * x_drum_dot;

        % Run observer at 30 Hz
        if (t_now - t_last_obs) >= (dt - 1e-6)
            q_meas = [0.0; 0.0; 6.50; x_drum];
            qdot_meas = [0.0; 0.0; 0.0; x_drum_dot];
            u_in = [0.0; 0.0; F_hoist_mot];

            obs_out = observer_4dof_gmo(u_in, [q_meas; qdot_meas], dt, obs_state, P);
            obs_state = obs_out.state;
            t_last_obs = t_now;
        end
        if ~isempty(obs_state)
            T_hat_hist(i) = obs_state.hat_T_rope;
        end
    end

    % Evaluate steady-state error (t in [1.5, 2.0] s)
    ss_idx = find(t_sim >= 1.5);
    err_T_ss = mean(abs(T_hat_hist(ss_idx) - T_true_hist(ss_idx)));
    rel_err_T_ss = (err_T_ss / T_step_mag) * 100.0;

    fprintf('  Applied Hoist Tension Step        : %8.1f kN\n', T_step_mag * 1e-3);
    fprintf('  Identified Steady-State Tension   : %8.1f kN\n', mean(T_hat_hist(ss_idx)) * 1e-3);
    fprintf('  Layer A Steady-State Error        : %8.4f %% (Tolerance: < 0.10 %%)\n', rel_err_T_ss);

    pass_layerA = (rel_err_T_ss < 0.10);

    % -------------------------------------------------------------------------
    % Test 2: Dual-Layer 3D Disturbance Tracking at 30 Hz
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 2] Dual-Layer 3D Link-Side Disturbance Tracking (30 Hz Discrete) ---\n');
    % Applied multiaxial external wrench:
    % tau_ext,1 = 250 kNm (swing bank force)
    % tau_ext,3 = 350 kNm (pitch boulder moment)
    % F_ext,4   = 180 kN  (crowd penetration resistance)
    tau_ext_true = [250000.0; 350000.0; 180000.0];

    % Posture: q1 = 0.2 rad, q3 = 0.1 rad, d4 = 7.00 m, x_drum = 8.00 m
    q_bench = [0.2; 0.1; 7.00; 8.00];
    qdot_bench = [0.05; -0.02; 0.10; 0.0];

    tau_f_swing = 12000.0 * qdot_bench(1) + 8000.0 * tanh(qdot_bench(1) / 0.005);
    tau_f_pitch = 25000.0 * qdot_bench(2) + 15000.0 * tanh(qdot_bench(2) / 0.005);
    F_f_crowd   = 18000.0 * qdot_bench(3) + 12000.0 * tanh(qdot_bench(3) / 0.005);
    tau_f_L = [tau_f_swing; tau_f_pitch; F_f_crowd];

    obs_state_L = [];
    N_obs_steps = round(2.5 / dt);
    tau_hat_hist = zeros(3, N_obs_steps);
    q_curr = q_bench;

    for k = 1:N_obs_steps
        dyn_k = kinematics_dynamics_4dof(q_curr, qdot_bench, P);
        rope_k = elastic_rope_subsystem(q_curr, qdot_bench, P);

        % Plant required effort to sustain motion under disturbance:
        tau_req_k = dyn_k.C(1:3, 1:3)*qdot_bench(1:3) + dyn_k.G(1:3) + tau_f_L - tau_ext_true;

        % Unactuated Joint 2 -> tension balances pitch requirement:
        T_rope_k = tau_req_k(2) / (-rope_k.dell_dq3);

        % Joint 3 crowd motor force:
        F_crowd_k = tau_req_k(3) + T_rope_k * rope_k.dell_dd4;

        % Joint 1 swing motor torque:
        tau_swing_k = tau_req_k(1);

        % Hoist motor effort balancing rope tension:
        F_f_drum_k = 1500.0 * qdot_bench(4) + 2500.0 * tanh(qdot_bench(4) / 0.01);
        F_hoist_k = -T_rope_k + F_f_drum_k;

        u_k = [tau_swing_k; F_crowd_k; F_hoist_k];

        obs_out_k = observer_4dof_gmo(u_k, [q_curr; qdot_bench], dt, obs_state_L, P);
        obs_state_L = obs_out_k.state;
        tau_hat_hist(:, k) = obs_out_k.hat_tau_ext_L;

        % Advance state along trajectory
        q_curr = q_curr + dt * qdot_bench;
    end

    hat_tau_ss = tau_hat_hist(:, end);
    rel_err_L = abs(hat_tau_ss - tau_ext_true) ./ tau_ext_true * 100.0;

    fprintf('  Channel 1 (Swing Torque) Disturbance: True = %8.1f kNm | Est = %8.1f kNm | Error: %6.4f %%\n', ...
        tau_ext_true(1)*1e-3, hat_tau_ss(1)*1e-3, rel_err_L(1));
    fprintf('  Channel 3 (Pitch Torque) Disturbance: True = %8.1f kNm | Est = %8.1f kNm | Error: %6.4f %%\n', ...
        tau_ext_true(2)*1e-3, hat_tau_ss(2)*1e-3, rel_err_L(2));
    fprintf('  Channel 4 (Crowd Force)  Disturbance: True = %8.1f kN  | Est = %8.1f kN  | Error: %6.4f %%\n', ...
        tau_ext_true(3)*1e-3, hat_tau_ss(3)*1e-3, rel_err_L(3));

    pass_layerB = all(rel_err_L < 0.10);

    % -------------------------------------------------------------------------
    % Test 3: 4.35 Hz Wire-Rope Axial Resonance Decoupling
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 3] 4.35 Hz Wire-Rope Bounce Decoupling & Attenuation ---\n');
    f_res = 4.35;
    t_res_vec = 0:dt:2.5;
    N_res = numel(t_res_vec);
    tau_pitch_dist = zeros(size(t_res_vec));

    dyn_nom = kinematics_dynamics_4dof(q_bench, qdot_bench, P);
    rope_nom = elastic_rope_subsystem(q_bench, qdot_bench, P);

    % Nominal equilibrium cable tension supporting dipper
    T_eq = (dyn_nom.G(2) + dyn_nom.C(2,:)*qdot_bench(:) + tau_f_pitch) / (-rope_nom.dell_dq3);
    delta_T = 50000.0; % [N] 4.35 Hz bounce amplitude

    M_drum = dyn_nom.M(4,4);
    x_drum_sim = 8.0;
    x_drum_dot_sim = 0.0;
    F_hoist_nom = -T_eq;

    obs_state_res = [];
    dt_sub = 1e-3;

    for k = 1:N_res
        t_k = t_res_vec(k);
        for sub = 1:round(dt / dt_sub)
            t_sub = t_k - dt + sub * dt_sub;
            T_true = T_eq + delta_T * sin(2 * pi * f_res * t_sub);
            F_f = 1500.0 * x_drum_dot_sim + 2500.0 * tanh(x_drum_dot_sim / 0.01);
            xddot = (F_hoist_nom + T_true - F_f) / M_drum;
            x_drum_dot_sim = x_drum_dot_sim + dt_sub * xddot;
            x_drum_sim     = x_drum_sim + dt_sub * x_drum_dot_sim;
        end

        tau_req_link = dyn_nom.C(1:3, 1:3)*qdot_bench(1:3) + dyn_nom.G(1:3) + tau_f_L;
        tau_swing_cmd = tau_req_link(1);
        F_crowd_cmd   = tau_req_link(3) + T_true * rope_nom.dell_dd4;
        u_k = [tau_swing_cmd; F_crowd_cmd; F_hoist_nom];

        q_meas = [q_bench(1:3); x_drum_sim];
        qdot_meas = [qdot_bench(1:3); x_drum_dot_sim];

        obs_out_k = observer_4dof_gmo(u_k, [q_meas; qdot_meas], dt, obs_state_res, P);
        obs_state_res = obs_out_k.state;
        tau_pitch_dist(k) = obs_out_k.hat_tau_ext_L(2);
    end

    eval_idx = find(t_res_vec >= 1.5);
    p2p_pitch_kNm = (max(tau_pitch_dist(eval_idx)) - min(tau_pitch_dist(eval_idx))) * 1e-3;
    undecoupled_p2p_kNm = 2.0 * delta_T * abs(rope_nom.dell_dq3) * 1e-3;
    attenuation_dB = 20 * log10(undecoupled_p2p_kNm / max(1e-3, p2p_pitch_kNm));

    fprintf('  Undecoupled False Pitch Ringing : %8.1f kNm\n', undecoupled_p2p_kNm);
    fprintf('  Dual-Layer Decoupled Ringing    : %8.4f kNm\n', p2p_pitch_kNm);
    fprintf('  Resonance Rejection Attenuation : %8.2f dB (Must be > 25 dB)\n', attenuation_dB);

    pass_res = (attenuation_dB > 25.0);

    % -------------------------------------------------------------------------
    % Test 4: 3D Tooth-Tip Force Inversion & Lateral Side-Raking Bank Force
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 4] 3D Spatial Tooth-Tip Inversion & Lateral Bank Force ---\n');
    F_lat_true = 25000.0; % [N] 25 kN lateral bank force
    obs_state_lat = [];
    q_curr = q_bench;

    for k = 1:round(2.5 / dt)
        dyn_k = kinematics_dynamics_4dof(q_curr, qdot_bench, P);
        rope_k = elastic_rope_subsystem(q_curr, qdot_bench, P);

        tau_ext_lat = [dyn_k.rho_tip * F_lat_true; 0.0; 0.0];
        tau_req_k = dyn_k.C(1:3, 1:3)*qdot_bench(1:3) + dyn_k.G(1:3) + tau_f_L - tau_ext_lat;
        T_rope_k = tau_req_k(2) / (-rope_k.dell_dq3);
        F_crowd_k = tau_req_k(3) + T_rope_k * rope_k.dell_dd4;
        tau_swing_k = tau_req_k(1);
        F_f_drum_k = 1500.0 * qdot_bench(4) + 2500.0 * tanh(qdot_bench(4) / 0.01);
        F_hoist_k = -T_rope_k + F_f_drum_k;

        u_k = [tau_swing_k; F_crowd_k; F_hoist_k];

        obs_lat = observer_4dof_gmo(u_k, [q_curr; qdot_bench], dt, obs_state_lat, P);
        obs_state_lat = obs_lat.state;

        q_curr = q_curr + dt * qdot_bench;
    end

    F_lat_est = obs_lat.F_lat;
    rel_err_lat = abs(F_lat_est - F_lat_true) / F_lat_true * 100.0;

    fprintf('  True Applied Lateral Bank Force   : %8.2f kN\n', F_lat_true * 1e-3);
    fprintf('  Identified Lateral Bank Force     : %8.2f kN\n', F_lat_est * 1e-3);
    fprintf('  Lateral Force Inversion Error     : %8.4f %% (Tolerance: < 0.05 %%)\n', rel_err_lat);

    pass_lat = (rel_err_lat < 0.05);

    pass_all = pass_layerA && pass_layerB && pass_res && pass_lat;

    fprintf('\n========================================================================================\n');
    fprintf(' MS06P03 SCORECARD: ');
    if pass_all
        fprintf('PASS (Dual-Layer De Luca-Spong 4-DOF Momentum Observer Verified!)\n');
    else
        fprintf('FAIL (One or more metrics out of tolerance)\n');
    end
    fprintf('========================================================================================\n');

    % Archive results
    exp_dir = fullfile(repo_root, 'M6', 'experiments');
    if ~exist(exp_dir, 'dir'), mkdir(exp_dir); end
    mat_file = fullfile(exp_dir, 'MS06P03_4dof_observer_results.mat');
    save(mat_file, 'rel_err_T_ss', 'rel_err_L', 'attenuation_dB', 'rel_err_lat', 'pass_all');
    fprintf(' Saved verification archive to: %s\n\n', mat_file);

    report = struct();
    report.rel_err_T_ss   = rel_err_T_ss;
    report.rel_err_L      = rel_err_L;
    report.attenuation_dB = attenuation_dB;
    report.rel_err_lat    = rel_err_lat;
    report.pass_all       = pass_all;
end
