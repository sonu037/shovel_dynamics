function out = motor_foc_wk20(cmd, load, dt, T_motor, drive_type)
%MOTOR_FOC_WK20  CRRC Yongji AC Induction Motor & Siemens S120 Inverter FOC Model for WK-20.
%
%   Simulates Field-Oriented Control (FOC) dynamics in the synchronous d-q rotor-flux
%   reference frame for the CRRC Yongji 690 VAC electric shovel motors:
%     - Hoist Drive: Dual YJ56B1 motors (2 x 600 kW rated, 2 x 750 kW peak, 25.4 Hz base)
%     - Crowd Drive: Single Yongji motor (350 kW rated, 437.5 kW peak, 4-pole)
%
%   Features:
%     1) Dynamic rotor flux lag (d(psi_r)/dt) with rotor time constant tau_r(T).
%     2) Field-weakening control above rated base frequency (25.4 Hz for hoist).
%     3) Thermal drift of rotor resistance: R_r(T) = R_r0 * (1 + alpha_Cu * Delta_T).
%     4) Active Line Module (ALM) DC bus energy balance and regenerative power flow.
%
%   USAGE:
%     out = motor_foc_wk20(); % Runs standalone verification suite (MS05P01)
%     out = motor_foc_wk20(cmd, load, dt, T_motor, drive_type);
%
%   See also SHOVEL_PARAMS_WK20, DRIVETRAIN_TRANSMISSION, ELECTROMECHANICAL_GMO.

    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(repo_root, 'M1', 'scripts'));

    if nargin < 5 || isempty(drive_type), drive_type = 'hoist'; end
    if nargin < 4 || isempty(T_motor),    T_motor = 25.0;       end % [deg C]
    if nargin < 3 || isempty(dt),         dt = 1e-3;            end % [s] 1 kHz internal loop
    if nargin < 2 || isempty(load),       load = struct();      end
    if nargin < 1 || isempty(cmd),        cmd = struct();       end

    % Standalone verification mode
    if isempty(fieldnames(cmd))
        out = run_ms05p01_verification(repo_root);
        return;
    end

    P_all = shovel_params_wk20();
    m_params = get_motor_electrical_params(drive_type, T_motor, P_all);

    out = step_foc_simulation(cmd, load, dt, m_params);
end

%% ========================================================================
%  INTERNAL FOC PARAMETER INITIALIZER
%  ========================================================================
function m = get_motor_electrical_params(drive_type, T_degC, P_all)
    m = struct();
    m.drive_type = drive_type;
    m.T_degC     = T_degC;
    m.T0_degC    = 25.0;
    m.alpha_Cu   = 0.00393; % [1/K] Copper temperature coefficient
    temp_factor  = 1.0 + m.alpha_Cu * (m.T_degC - m.T0_degC);

    m.V_dc_nom   = P_all.inverter.V_dc_nom; % 1050 VDC
    m.p          = 2;                       % 4 poles = 2 pole pairs

    if strcmpi(drive_type, 'hoist')
        % Dual CRRC Yongji YJ56B1 motors
        m.n_motors       = 2;
        m.P_rated_motor  = P_all.hoist_motor.P_rated_kW * 1e3;   % 600 kW per motor
        m.P_peak_motor   = P_all.hoist_motor.P_peak_kW * 1e3;    % 750 kW per motor
        m.P_rated_total  = m.n_motors * m.P_rated_motor;         % 1,200 kW total
        m.P_peak_total   = m.n_motors * m.P_peak_motor;          % 1,500 kW total
        m.f_base         = P_all.hoist_motor.f_rated_Hz;         % 25.4 Hz
        m.omega_m_base   = P_all.hoist_motor.omega_m_base;       % 79.796 rad/s (762 rpm)
        m.tau_rated_tot  = m.n_motors * P_all.hoist_motor.tau_rated_Nm; % ~15,038 - 15,280 Nm
        m.tau_peak_tot   = m.tau_rated_tot * (m.P_peak_total / m.P_rated_total); % ~19,000 Nm
        m.J_rotor_tot    = m.n_motors * P_all.drivetrain.J_m_hoist; % 2 x 26.5 = 53.0 kg*m^2

        % Equivalent single-phase Y parameters at 25 deg C (scaled for dual 600 kW)
        m.R_s0           = 0.0125 / m.n_motors;
        m.R_r0           = 0.0098 / m.n_motors;
        m.L_ls           = 0.00035 / m.n_motors;
        m.L_lr           = 0.00035 / m.n_motors;
        m.L_m            = 0.01150 / m.n_motors;
        m.psi_r_rated    = 1.850; % [Wb] Rated rotor flux linkage
    else
        % Single CRRC Yongji Crowd Motor
        m.n_motors       = 1;
        m.P_rated_motor  = P_all.crowd_motor.P_rated_kW * 1e3;   % 350 kW
        m.P_peak_motor   = P_all.crowd_motor.P_peak_kW * 1e3;    % 437.5 kW
        m.P_rated_total  = m.P_rated_motor;
        m.P_peak_total   = m.P_peak_motor;
        m.f_base         = 24.5;                                 % [Hz]
        m.omega_m_base   = P_all.crowd_motor.omega_m_base;       % 77.0 rad/s
        m.tau_rated_tot  = P_all.crowd_motor.tau_rated_Nm;       % 4,545.5 Nm
        m.tau_peak_tot   = m.tau_rated_tot * (m.P_peak_total / m.P_rated_total); % 5,681.8 Nm
        m.J_rotor_tot    = P_all.drivetrain.J_m_crowd;           % 14.8 kg*m^2

        m.R_s0           = 0.0210;
        m.R_r0           = 0.0165;
        m.L_ls           = 0.00060;
        m.L_lr           = 0.00060;
        m.L_m            = 0.01950;
        m.psi_r_rated    = 1.720; % [Wb]
    end

    % Apply temperature scaling on rotor resistance
    m.R_s = m.R_s0 * temp_factor;
    m.R_r = m.R_r0 * temp_factor;
    m.L_s = m.L_m + m.L_ls;
    m.L_r = m.L_m + m.L_lr;
    m.sigma = 1.0 - (m.L_m^2) / (m.L_s * m.L_r);
    m.tau_r = m.L_r / m.R_r; % [s] Rotor time constant
    m.tau_r0 = m.L_r / m.R_r0;

    % FOC Controller Gains (Current PI loop)
    m.omega_cc = 2 * pi * 250.0; % [rad/s] 250 Hz current loop bandwidth
    m.Kp_i = m.sigma * m.L_s * m.omega_cc;
    m.Ki_i = m.R_s * m.omega_cc;

    % Speed PI Gains
    m.Kp_w = 2.0 * m.J_rotor_tot * (2 * pi * 10.0); % 10 Hz outer speed loop
    m.Ki_w = m.Kp_w * (2 * pi * 2.0);
end

%% ========================================================================
%  FOC STEP INTEGRATOR
%  ========================================================================
function sim = step_foc_simulation(cmd, load, dt, m)
    % Unpack inputs
    t_span = cmd.t_span;
    N = numel(t_span);

    w_ref = cmd.w_ref(:);
    if numel(w_ref) == 1, w_ref = repmat(w_ref, N, 1); end

    tau_load = load.tau_load(:);
    if numel(tau_load) == 1, tau_load = repmat(tau_load, N, 1); end

    % State memory allocations
    omega_m   = zeros(N, 1); % [rad/s] Rotor mechanical speed
    psi_r     = zeros(N, 1); % [Wb] Rotor flux linkage
    tau_e     = zeros(N, 1); % [Nm] Electromagnetic torque
    i_sd      = zeros(N, 1); % [A] Direct flux-producing stator current
    i_sq      = zeros(N, 1); % [A] Quadrature torque-producing stator current
    v_sd      = zeros(N, 1); % [V] d-axis stator voltage
    v_sq      = zeros(N, 1); % [V] q-axis stator voltage
    P_elec_kW = zeros(N, 1); % [kW] 3-phase electrical power
    P_mech_kW = zeros(N, 1); % [kW] Shaft mechanical power
    P_dc_kW   = zeros(N, 1); % [kW] Inverter DC-link power
    slip_Hz   = zeros(N, 1); % [Hz] Slip frequency

    % Initial conditions
    w_curr     = 0.0;
    psi_curr   = m.psi_r_rated;
    isd_curr   = m.psi_r_rated / m.L_m;
    isq_curr   = 0.0;
    speed_int  = 0.0;
    iq_int     = 0.0;
    id_int     = 0.0;

    for k = 1:N
        w_target = w_ref(k);
        t_load   = tau_load(k);

        % 1. Field Weakening Profile
        abs_w = max(abs(w_curr), 1e-3);
        if abs_w > m.omega_m_base
            psi_ref = m.psi_r_rated * (m.omega_m_base / abs_w);
        else
            psi_ref = m.psi_r_rated;
        end
        isd_ref = psi_ref / m.L_m;

        % 2. Outer Speed PI Controller -> Torque Reference
        w_err = w_target - w_curr;
        speed_int = speed_int + w_err * dt;
        tau_ref = m.Kp_w * w_err + m.Ki_w * speed_int;

        % Clamp torque to field-weakened pull-out capability
        tau_lim = m.tau_peak_tot * min(1.0, m.omega_m_base / abs_w);
        tau_ref = max(-tau_lim, min(tau_lim, tau_ref));

        % 3. FOC Torque-to-Current Equation: tau_e = 1.5 * p * (L_m/L_r) * psi_r * isq
        % Note: For dual motors, total torque is split across the motors
        factor_kT = 1.5 * m.p * (m.L_m / m.L_r) * max(psi_curr, 0.1);
        isq_ref = tau_ref / factor_kT;

        % 4. Inner Current Control (PI + Back-EMF Feedforward)
        id_err = isd_ref - isd_curr;
        iq_err = isq_ref - isq_curr;
        id_int = id_int + id_err * dt;
        iq_int = iq_int + iq_err * dt;

        omega_e = m.p * w_curr + (m.L_m / (m.tau_r * max(psi_curr, 0.1))) * isq_curr;
        v_d_cmd = m.Kp_i * id_err + m.Ki_i * id_int - omega_e * m.sigma * m.L_s * isq_curr;
        v_q_cmd = m.Kp_i * iq_err + m.Ki_i * iq_int + omega_e * m.sigma * m.L_s * isd_curr + omega_e * (m.L_m / m.L_r) * psi_curr;

        % Voltage limit (Inverter PWM saturation)
        v_max = m.V_dc_nom / sqrt(3); % Space vector limit
        v_mag = hypot(v_d_cmd, v_q_cmd);
        if v_mag > v_max
            v_d_cmd = v_d_cmd * (v_max / v_mag);
            v_q_cmd = v_q_cmd * (v_max / v_mag);
        end

        % 5. Continuous Induction Motor Electrical Dynamics
        % Stator current derivatives:
        disd_dt = (1.0 / (m.sigma * m.L_s)) * (v_d_cmd - m.R_s * isd_curr + omega_e * m.sigma * m.L_s * isq_curr - (m.L_m / m.L_r) * (-(1.0/m.tau_r)*psi_curr + (m.L_m/m.tau_r)*isd_curr));
        disq_dt = (1.0 / (m.sigma * m.L_s)) * (v_q_cmd - m.R_s * isq_curr - omega_e * m.sigma * m.L_s * isd_curr - omega_e * (m.L_m / m.L_r) * psi_curr);

        % Rotor flux derivative:
        dpsi_dt = -(1.0 / m.tau_r) * psi_curr + (m.L_m / m.tau_r) * isd_curr;

        % State updates (Euler step at dt = 1 ms)
        isd_curr = isd_curr + disd_dt * dt;
        isq_curr = isq_curr + disq_dt * dt;
        psi_curr = psi_curr + dpsi_dt * dt;

        % Generated electromagnetic torque
        tau_em = 1.5 * m.p * (m.L_m / m.L_r) * psi_curr * isq_curr;

        % 6. Mechanical Shaft Dynamics: J_rotor * d(omega_m)/dt = tau_em - tau_load - B_m*omega_m
        B_visc = 0.5; % [Nm*s/rad] Motor bearing viscous drag
        dw_dt = (tau_em - t_load - B_visc * w_curr) / m.J_rotor_tot;
        w_curr = w_curr + dw_dt * dt;

        % 7. Power and Telemetry Metrics
        p_mech = tau_em * w_curr;                          % [W] Mechanical shaft power
        p_elec = 1.5 * (v_d_cmd * isd_curr + v_q_cmd * isq_curr); % [W] Electrical stator power
        eta_inv = 0.985; % Inverter IGBT switching efficiency
        if p_elec >= 0
            p_dc = p_elec / eta_inv; % Motoring: DC link feeds motor
        else
            p_dc = p_elec * eta_inv; % Regenerating: Motor feeds DC link
        end

        % Store telemetry
        omega_m(k)   = w_curr;
        psi_r(k)     = psi_curr;
        tau_e(k)     = tau_em;
        i_sd(k)      = isd_curr;
        i_sq(k)      = isq_curr;
        v_sd(k)      = v_d_cmd;
        v_sq(k)      = v_q_cmd;
        P_elec_kW(k) = p_elec * 1e-3;
        P_mech_kW(k) = p_mech * 1e-3;
        P_dc_kW(k)   = p_dc * 1e-3;
        slip_Hz(k)   = ((m.L_m / (m.tau_r * max(psi_curr, 0.1))) * isq_curr) / (2 * pi);
    end

    sim = struct();
    sim.t_s       = t_span;
    sim.omega_m   = omega_m;
    sim.n_rpm     = omega_m * (60 / (2 * pi));
    sim.psi_r     = psi_r;
    sim.tau_e     = tau_e;
    sim.i_sd      = i_sd;
    sim.i_sq      = i_sq;
    sim.P_elec_kW = P_elec_kW;
    sim.P_mech_kW = P_mech_kW;
    sim.P_dc_kW   = P_dc_kW;
    sim.slip_Hz   = slip_Hz;
    sim.params    = m;
end

%% ========================================================================
%  STANDALONE PROTOCOL VERIFICATION SUITE (MS05P01)
%  ========================================================================
function report = run_ms05p01_verification(repo_root)
    fprintf('================================================================================\n');
    fprintf(' RUNNING PROTOCOL MS05P01: CRRC YONGJI AC MOTOR & SIEMENS S120 FOC VERIFICATION\n');
    fprintf(' Target Equipment: Taiyuan Heavy Industry WK-20 (NCL Amlohri Mine)\n');
    fprintf('================================================================================\n\n');

    dt = 1e-3; % 1 ms sampling
    t_end = 10.0;
    t = (0:dt:t_end)';
    N = numel(t);

    % Test Case 1: Hoist Acceleration, Rated Stall Torque & Field Weakening
    cmd1 = struct();
    cmd1.t_span = t;
    % Commanded speed profile: ramp to rated base speed (762 rpm = 79.8 rad/s), then into field weakening (950 rpm = 99.5 rad/s)
    w_ref = zeros(N, 1);
    for i = 1:N
        ti = t(i);
        if ti <= 2.0
            w_ref(i) = (79.8 / 2.0) * ti; % Accel to base speed
        elseif ti <= 5.0
            w_ref(i) = 79.8;              % Base speed dwell
        elseif ti <= 7.0
            w_ref(i) = 79.8 + ((99.5 - 79.8) / 2.0) * (ti - 5.0); % Field weakening ramp
        else
            w_ref(i) = 99.5;              % High speed dwell
        end
    end
    cmd1.w_ref = w_ref;

    % Load profile: apply rated load (15.28 kNm) at t = 3.0s, step to stall limit (19.0 kNm) at t = 4.0s
    tau_load = zeros(N, 1);
    for i = 1:N
        ti = t(i);
        if ti >= 3.0 && ti < 4.0
            tau_load(i) = 15280.0; % Rated torque
        elseif ti >= 4.0 && ti < 5.0
            tau_load(i) = 19000.0; % Peak stall limit
        elseif ti >= 7.0
            tau_load(i) = 12000.0; % Field weakened load
        end
    end
    load1 = struct('tau_load', tau_load);

    fprintf('--- [Test 1] Simulating Dual Hoist Motors (2 x 600 kW YJ56B1)...\n');
    sim_hoist = motor_foc_wk20(cmd1, load1, dt, 25.0, 'hoist');

    % Metric checks
    max_tau_hoist = max(sim_hoist.tau_e);
    p_peak_kW     = max(sim_hoist.P_mech_kW);
    psi_at_fw     = sim_hoist.psi_r(end);

    fprintf('  Peak Hoist Torque Produced   : %8.2f Nm (Target: ~19,000 Nm peak stall)\n', max_tau_hoist);
    fprintf('  Peak Shaft Mechanical Power : %8.2f kW (Target: ~1,500 kW combined peak)\n', p_peak_kW);
    fprintf('  Rotor Flux at 99.5 rad/s    : %8.4f Wb (Rated: 1.8500 Wb | Target: ~1.48 Wb)\n', psi_at_fw);

    % Test Case 2: Crowd Drive Torque and 4-Quadrant Reversal
    fprintf('\n--- [Test 2] Simulating Crowd Motor (350 kW Yongji) with Direction Reversal...\n');
    cmd2 = struct();
    cmd2.t_span = t;
    % Forward crowd at 77 rad/s, reverse to -77 rad/s
    cmd2.w_ref = 77.0 * sin(2 * pi * 0.2 * t);
    load2 = struct('tau_load', 4545.5 * sign(sin(2 * pi * 0.2 * t)));
    sim_crowd = motor_foc_wk20(cmd2, load2, dt, 25.0, 'crowd');

    fprintf('  Crowd Torque Range          : [%7.2f, %7.2f] Nm (Target: +/- 4,545.5 Nm)\n', min(sim_crowd.tau_e), max(sim_crowd.tau_e));
    fprintf('  Crowd Regenerative Power    : %8.2f kW into DC Link during reversal\n', min(sim_crowd.P_dc_kW));

    % Test Case 3: Thermal Drift of Rotor Resistance (Delta T = +95 C)
    fprintf('\n--- [Test 3] Simulating Thermal Detuning (Delta T = +95 C copper heating)...\n');
    sim_hot = motor_foc_wk20(cmd1, load1, dt, 120.0, 'hoist'); % 120 deg C (Delta T = +95 C)
    tau_slip_diff = max(abs(sim_hoist.slip_Hz - sim_hot.slip_Hz));
    fprintf('  Hot Rotor Time Constant tau_r: %7.4f s (Nominal 25 C: %7.4f s | -27.2%% shift)\n', sim_hot.params.tau_r, sim_hoist.params.tau_r);
    fprintf('  Slip Frequency Divergence    : %7.3f Hz under thermal detuning\n', tau_slip_diff);

    % Acceptance verification
    pass_hoist = (max_tau_hoist >= 18500.0) && (p_peak_kW >= 1450.0) && (psi_at_fw < 1.60);
    pass_crowd = (max(sim_crowd.tau_e) >= 4400.0) && (min(sim_crowd.P_dc_kW) < -100.0);
    pass_temp  = (sim_hot.params.R_r > 1.30 * sim_hoist.params.R_r);

    pass_all = pass_hoist && pass_crowd && pass_temp;

    fprintf('\n================================================================================\n');
    fprintf(' MS05P01 SCORECARD: ');
    if pass_all
        fprintf('PASS (All motor electromagnetic & FOC specifications verified!)\n');
    else
        fprintf('FAIL (One or more metrics out of tolerance)\n');
    end
    fprintf('================================================================================\n');

    % Save results archive
    exp_dir = fullfile(repo_root, 'M5', 'experiments');
    if ~exist(exp_dir, 'dir'), mkdir(exp_dir); end
    mat_file = fullfile(exp_dir, 'MS05P01_motor_foc_results.mat');
    save(mat_file, 'sim_hoist', 'sim_crowd', 'sim_hot', 'pass_all');
    fprintf(' Saved verification archive to: %s\n\n', mat_file);

    report = struct();
    report.sim_hoist = sim_hoist;
    report.sim_crowd = sim_crowd;
    report.sim_hot   = sim_hot;
    report.pass_all  = pass_all;
end
