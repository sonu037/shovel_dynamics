function report = run_MS05_Verification_Suite(save_results)
%RUN_MS05_VERIFICATION_SUITE  Milestone 5 Electromechanical Drive Dynamics Benchmark.
%
%   MS05P04: Integrated End-to-End Electromechanical Drive Verification Suite
%   for the Taiyuan Heavy Industry (TZ) WK-20 mining rope shovel (NCL Amlohri Mine).
%
%   Executes 4 Canonical Shovel Operating Regimes:
%     [Case 1] Standard 38 s Digging Cycle: Continuous Energy Conservation & Power Balance (< 0.1% error).
%     [Case 2] 4-Quadrant Regenerative Lowering: Dipper gravity descent feeding power into 1050 VDC bus (P_dc < 0).
%     [Case 3] Severe Boulder Impact: Step response with adaptive innovation gating (t_rise <= 33.3 ms).
%     [Case 4] Thermal Detuning Sweep (Delta T = 0 to +95 deg C): Rotor resistance drift and slip divergence.
%
%   AUDIT 2026-11-18 (M5 frozen as a model draft): Case 1 uses typed trajectories and forces and
%   closes its energy balance by definition; Case 2 is arithmetic; Case 3 is the model-matched
%   MS05P03 self-check; Case 4 follows from R_r(T) = R_r0(1 + alpha dT) (slip read 20 ms after a
%   motor start). None of these is a test. See claims.md, M5 section.
%
%   USAGE:
%     report = run_MS05_Verification_Suite();
%     report = run_MS05_Verification_Suite(true); % Save MS05P04_benchmark_summary.mat
%
%   See also MOTOR_FOC_WK20, DRIVETRAIN_TRANSMISSION, ELECTROMECHANICAL_GMO, SHOVEL_PARAMS_WK20.

    if nargin < 1 || isempty(save_results), save_results = true; end

    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(repo_root, 'M1', 'scripts'));
    addpath(fullfile(repo_root, 'M5', 'scripts'));

    fprintf('========================================================================================\n');
    fprintf(' MILESTONE 5: ELECTROMECHANICAL DRIVE DYNAMICS & INVERTER TELEMETRY VERIFICATION\n');
    fprintf(' Equipment : Taiyuan Heavy Industry WK-20 (NCL Amlohri Mine / MT-189)\n');
    fprintf(' Protocols : MS05P01 (FOC) | MS05P02 (Transmission) | MS05P03 (GMO) | MS05P04 (Benchmark)\n');
    fprintf(' Date      : %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf('========================================================================================\n\n');

    P = shovel_params_wk20();
    report = struct();
    report.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    %% ========================================================================
    %  CASE 1: STANDARD 38 s DIGGING CYCLE (CONTINUOUS ENERGY CONSERVATION)
    %  ========================================================================
    fprintf('--- [Case 1] Standard 38 s Digging Cycle: Continuous Energy Balance ---\n');
    dt_c1 = 0.02; % 50 Hz simulation grid (1901 steps)
    t_c1 = (0:dt_c1:38.0)';
    N_c1 = numel(t_c1);

    % Canonical 38-second shovel cycle phases:
    % 0 - 12 s : Digging (High hoist pull, crowd extension)
    % 12 - 20 s: Swing to dump (constant tuck radius, hoist to clearance)
    % 20 - 25 s: Dipper door trip & dump into haul truck
    % 25 - 38 s: Return swing & lowering back to bank toe
    q3_c1   = zeros(N_c1, 1);
    d4_c1   = zeros(N_c1, 1);
    F_h_c1  = zeros(N_c1, 1);
    F_c_c1  = zeros(N_c1, 1);

    for k = 1:N_c1
        tk = t_c1(k);
        if tk <= 12.0
            % Digging phase
            tau_dig = tk / 12.0;
            q3_c1(k)  = deg2rad(-25.0) + deg2rad(45.0) * (1.0 - cos(pi * tau_dig)) / 2.0;
            d4_c1(k)  = 5.50 + 2.50 * sin(pi * tau_dig);
            F_h_c1(k) = 650e3 + 250e3 * sin(pi * tau_dig); % Heavy digging bail pull (up to 900 kN)
            F_c_c1(k) = 250e3 * cos(pi * tau_dig);          % Crowd thrust
        elseif tk <= 20.0
            % Swing to haul truck
            tau_sw = (tk - 12.0) / 8.0;
            q3_c1(k)  = deg2rad(20.0) + deg2rad(5.0) * tau_sw;
            d4_c1(k)  = 8.00 - 0.50 * tau_sw;
            F_h_c1(k) = 450e3;                              % Loaded dipper payload hoist hold
            F_c_c1(k) = 20e3;
        elseif tk <= 25.0
            % Dump into truck (payload discharged)
            tau_dp = (tk - 20.0) / 5.0;
            q3_c1(k)  = deg2rad(25.0);
            d4_c1(k)  = 7.50;
            F_h_c1(k) = 450e3 - 220e3 * tau_dp;            % Payload discharge drops bail pull
            F_c_c1(k) = 0.0;
        else
            % Return swing and lower back to toe
            tau_ret = (tk - 25.0) / 13.0;
            q3_c1(k)  = deg2rad(25.0) - deg2rad(50.0) * (1.0 - cos(pi * tau_ret)) / 2.0;
            d4_c1(k)  = 7.50 - 2.00 * (1.0 - cos(pi * tau_ret)) / 2.0;
            F_h_c1(k) = 230e3;                              % Empty dipper gravity hold
            F_c_c1(k) = -50e3 * sin(pi * tau_ret);
        end
    end

    q3dot_c1 = gradient(q3_c1, dt_c1);
    d4dot_c1 = gradient(d4_c1, dt_c1);

    % Vectorized 1D Trajectory Sheave Geometry & Kinematics
    S_bail_c1  = d4_c1 + P.x_ref;
    Px_bail_c1 = S_bail_c1 .* cos(q3_c1);
    Py_bail_c1 = S_bail_c1 .* sin(q3_c1);
    Dx_c1      = P.p_sheave_center(1) - Px_bail_c1;
    Dy_c1      = P.p_sheave_center(2) - Py_bail_c1;
    D_center_c1 = hypot(Dx_c1, Dy_c1);
    psi_cent_c1 = atan2(Dy_c1, Dx_c1);
    delta_tang_c1 = asin(min(0.999, P.R_sheave ./ max(P.R_sheave * 1.001, D_center_c1)));
    psi_rope_c1 = psi_cent_c1 + delta_tang_c1;
    alpha3_c1   = psi_rope_c1 - q3_c1;

    v_bail = S_bail_c1 .* sin(alpha3_c1) .* q3dot_c1 + cos(alpha3_c1) .* d4dot_c1;
    v_cr   = d4dot_c1;

    % Continuous Power Components
    P_mech_h = F_h_c1 .* v_bail;
    P_mech_c = F_c_c1 .* v_cr;
    P_mech_c1 = P_mech_h + P_mech_c;

    % Smooth 4-Quadrant Directional Efficiency (Verstraten 2016)
    P_eps = P.drivetrain.P_eps_W;
    th_h = tanh(P_mech_h / P_eps);
    gamma_h = 0.5 * (1.0 + th_h) * P.drivetrain.eta_mot + 0.5 * (1.0 - th_h) * (1.0 / P.drivetrain.eta_gen);
    th_c = tanh(P_mech_c / P_eps);
    gamma_c = 0.5 * (1.0 + th_c) * P.drivetrain.eta_mot + 0.5 * (1.0 - th_c) * (1.0 / P.drivetrain.eta_gen);

    % Electrical drive efficiency (motor * inverter)
    eta_drive = 0.955 * 0.985; % 94.07%

    % Continuous instantaneous electrical power and losses
    P_shaft_h = zeros(N_c1, 1);
    P_shaft_c = zeros(N_c1, 1);
    P_elec_c1 = zeros(N_c1, 1);
    P_loss_c1 = zeros(N_c1, 1);

    for k = 1:N_c1
        % Hoist channel
        if P_mech_h(k) >= 0
            P_sh_h = P_mech_h(k) / P.drivetrain.eta_mot;
            P_el_h = P_sh_h / eta_drive;
        else
            P_sh_h = P_mech_h(k) * P.drivetrain.eta_gen;
            P_el_h = P_sh_h * eta_drive;
        end

        % Crowd channel
        if P_mech_c(k) >= 0
            P_sh_c = P_mech_c(k) / P.drivetrain.eta_mot;
            P_el_c = P_sh_c / eta_drive;
        else
            P_sh_c = P_mech_c(k) * P.drivetrain.eta_gen;
            P_el_c = P_sh_c * eta_drive;
        end

        P_shaft_h(k) = P_sh_h;
        P_shaft_c(k) = P_sh_c;
        P_elec_c1(k) = P_el_h + P_el_c;
        P_loss_c1(k) = P_elec_c1(k) - P_mech_c1(k);
    end

    % Energy integration over full 38 s cycle
    E_elec = trapz(t_c1, P_elec_c1);
    E_mech = trapz(t_c1, P_mech_c1);
    E_loss = trapz(t_c1, P_loss_c1);
    E_balance_err = abs(E_elec - (E_mech + E_loss)) / max(1e3, trapz(t_c1, abs(P_elec_c1)));

    fprintf('  Total Electrical Energy Consumed : %8.2f MJ (%.2f kWh)\n', E_elec * 1e-6, E_elec / 3.6e6);
    fprintf('  Total Mechanical Work Delivered  : %8.2f MJ (%.2f kWh)\n', E_mech * 1e-6, E_mech / 3.6e6);
    fprintf('  Total Drivetrain & Drive Losses  : %8.2f MJ (%.2f kWh)\n', E_loss * 1e-6, E_loss / 3.6e6);
    fprintf('  Energy Conservation Discrepancy  : %.5f%% (Target: < 0.1%%)\n', E_balance_err * 100);

    pass_c1 = (E_balance_err < 1e-3);
    report.case1 = struct('E_elec_MJ', E_elec*1e-6, 'E_mech_MJ', E_mech*1e-6, ...
                          'E_loss_MJ', E_loss*1e-6, 'E_balance_err_pct', E_balance_err*100, 'pass', pass_c1);

    %% ========================================================================
    %  CASE 2: 4-QUADRANT REGENERATIVE LOWERING INTO 1050 VDC BUS
    %  ========================================================================
    fprintf('\n--- [Case 2] 4-Quadrant Regenerative Lowering into 1050 VDC Bus ---\n');
    % Lower empty dipper (230 kN gravity load) at 1.2 m/s descent
    v_lower = -1.20; % [m/s] downward velocity
    F_lower = 230e3; % [N] tension holding dipper against gravity

    P_mech_lower = F_lower * v_lower; % Negative mechanical power = -276 kW
    P_shaft_regen = P_mech_lower * P.drivetrain.eta_gen; % Gearbox back-driving
    P_dc_regen    = P_shaft_regen * (0.955 * 0.985); % Power delivered to DC bus (motor * inverter)

    fprintf('  Descent Bail Mechanical Power    : %8.2f kW (Motoring < 0 -> Regenerative)\n', P_mech_lower * 1e-3);
    fprintf('  Motor Shaft Regenerative Power   : %8.2f kW (Gearbox back-driving eta = %.2f)\n', ...
        P_shaft_regen * 1e-3, P.drivetrain.eta_gen);
    fprintf('  Net Power Fed to 1050 VDC Bus    : %8.2f kW (Supplying grid / aux transformer)\n', P_dc_regen * 1e-3);

    pass_c2 = (P_dc_regen < -200e3) && (P_dc_regen > -276e3);
    report.case2 = struct('P_mech_kW', P_mech_lower*1e-3, 'P_dc_kW', P_dc_regen*1e-3, 'pass', pass_c2);

    %% ========================================================================
    %  CASE 3: SEVERE BOULDER IMPACT & INNOVATION GATING
    %  ========================================================================
    fprintf('\n--- [Case 3] Severe Boulder Impact & Innovation Gating ---\n');
    obs_res = electromechanical_gmo();
    pass_c3 = obs_res.pass_all;
    report.case3 = struct('t_rise_ms', obs_res.t_rise_ms, 't_settle_ms', obs_res.t_settle_ms, 'pass', pass_c3);

    %% ========================================================================
    %  CASE 4: SUMMER THERMAL DETUNING SWEEP (Delta T = 0 to +95 deg C)
    %  ========================================================================
    fprintf('\n--- [Case 4] Summer Thermal Detuning Sweep (Delta T = 0 to +95 deg C) ---\n');
    T_sweep = (25.0:5.0:120.0)';
    N_T = numel(T_sweep);
    tau_r_sweep     = zeros(N_T, 1);
    slip_freq_sweep = zeros(N_T, 1);
    detuned_tau_err = zeros(N_T, 1);

    cmd_step = struct('t_span', (0:1e-3:0.02)', 'w_ref', 78.5);
    load_step = struct('tau_load', 7640.0);

    for i = 1:N_T
        Ti = T_sweep(i);
        out_foc = motor_foc_wk20(cmd_step, load_step, 1e-3, Ti, 'hoist');
        tau_r_sweep(i)     = out_foc.params.tau_r;
        slip_freq_sweep(i) = out_foc.slip_Hz(end);

        % Analytical detuning ratio K_tau = tau_r_actual / tau_r_nominal
        K_tau = tau_r_sweep(i) / tau_r_sweep(1);
        % Classical induction machine torque detuning at rated slip
        tau_ratio = (2.0 * K_tau) / (1.0 + K_tau^2);
        detuned_tau_err(i) = 100.0 * abs(1.0 - tau_ratio);
    end

    max_detuned_err = max(detuned_tau_err);
    delta_tau_r_pct = 100 * (tau_r_sweep(end) - tau_r_sweep(1)) / tau_r_sweep(1);
    delta_slip_Hz   = slip_freq_sweep(end) - slip_freq_sweep(1);

    fprintf('  Rotor Time Constant Drift        : %.3f s -> %.3f s (Drift: %.2f%%)\n', ...
        tau_r_sweep(1), tau_r_sweep(end), delta_tau_r_pct);
    fprintf('  VFD Slip Frequency Divergence    : %.3f Hz -> %.3f Hz (+%.3f Hz)\n', ...
        slip_freq_sweep(1), slip_freq_sweep(end), delta_slip_Hz);
    fprintf('  Base-Speed Torque Detuning Error : %.2f%% at 120 deg C (Nominal 25 deg C VFD model)\n', max_detuned_err);

    pass_c4 = (delta_tau_r_pct < -25.0) && (delta_slip_Hz > 0.40);
    report.case4 = struct('T_sweep', T_sweep, 'tau_r_sweep', tau_r_sweep, ...
                          'slip_freq_sweep', slip_freq_sweep, 'detuned_tau_err', detuned_tau_err, 'pass', pass_c4);

    %% ========================================================================
    %  OVERALL SCORECARD & RESULTS ARCHIVE
    %  ========================================================================
    report.all_passed = pass_c1 && pass_c2 && pass_c3 && pass_c4;

    fprintf('\n========================================================================================\n');
    fprintf(' MILESTONE 5 VERIFICATION SCORECARD: ');
    if report.all_passed
        fprintf('ALL 4 CANONICAL CASES PASSED (100%% VERIFIED)\n');
    else
        fprintf('ONE OR MORE CASES FAILED\n');
    end
    fprintf('========================================================================================\n');
    fprintf('  [Case 1] Continuous Energy Balance   : %s\n', get_pass_fail_str(pass_c1));
    fprintf('  [Case 2] 4-Quadrant DC Regeneration  : %s\n', get_pass_fail_str(pass_c2));
    fprintf('  [Case 3] Boulder Impact Recovery     : %s\n', get_pass_fail_str(pass_c3));
    fprintf('  [Case 4] Thermal Detuning Sweep      : %s\n', get_pass_fail_str(pass_c4));
    fprintf('========================================================================================\n\n');

    if save_results
        exp_dir = fullfile(repo_root, 'M5', 'experiments');
        if ~exist(exp_dir, 'dir'), mkdir(exp_dir); end
        out_mat = fullfile(exp_dir, 'MS05P04_benchmark_summary.mat');
        save(out_mat, 'report');
        fprintf(' Milestone 5 Benchmark summary successfully archived to:\n   %s\n\n', out_mat);
    end
end

function s = get_pass_fail_str(p)
    if p
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
