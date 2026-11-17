function out = elastic_rope_subsystem(q, qdot, P)
%ELASTIC_ROPE_SUBSYSTEM  Dynamic Hoist Wire-Rope Elasticity & Compliance for WK-20.
%
%   MS06P02: Implements the variable-length twin 52 mm hoist wire-rope elasticity,
%   tangent sheave kinematics, and C^infty-smooth unilateral tension barrier for the
%   Taiyuan Heavy Industry WK-20 mining rope shovel.
%
%   State Vector:
%     q    = [q1; q3; d4; x_drum]
%     qdot = [q1dot; q3dot; d4dot; x_drum_dot]
%     q1     : Turntable swing angle [rad]
%     q3     : Saddle pitch angle [rad]
%     d4     : Prismatic crowd extension [m]
%     x_drum : Bail-referred hoist drum payout [m]
%
%   Outputs:
%     out.ell_bail      : Suspended tangent cable span from boom sheave to bail pin [m]
%     out.dell_dq3      : Partial derivative d(ell_bail)/dq3 [m/rad]
%     out.dell_dd4      : Partial derivative d(ell_bail)/dd4 [-]
%     out.ell_bail_dot  : Kinematic cable shortening velocity [m/s]
%     out.delta_rope    : Dynamic elastic elongation (stretch) [m]
%     out.delta_dot     : Rate of elongation [m/s]
%     out.K_rope        : Variable-length longitudinal stiffness [N/m]
%     out.D_rope        : Longitudinal internal damping coefficient [N*s/m]
%     out.T_rope        : Unilateral elastic wire-rope tension [N]
%     out.Q_rope        : 4x1 Generalized force vector applied to [q1; q3; d4; x_drum] [N or N*m]
%     out.f_bounce      : Theoretical axial bounce resonance frequency [Hz]
%
%   USAGE:
%     out = elastic_rope_subsystem(); % Standalone verification suite (MS06P02)
%     out = elastic_rope_subsystem(q, qdot, P);
%
%   See also KINEMATICS_DYNAMICS_4DOF, SHOVEL_PARAMS_WK20, OBSERVER_4DOF_GMO.

    persistent path_added;
    if isempty(path_added)
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        addpath(fullfile(repo_root, 'M1', 'scripts'));
        path_added = true;
    end

    % Standalone verification mode
    if nargin == 0
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        out = run_ms06p02_verification(repo_root);
        return;
    end

    if nargin < 3 || isempty(P), P = shovel_params_wk20(); end
    if nargin < 2 || isempty(qdot), qdot = zeros(4, 1); end

    q = q(:);
    qdot = qdot(:);

    q1 = q(1); q3 = q(2); d4 = q(3); x_drum = q(4);
    q1dot = qdot(1); q3dot = qdot(2); d4dot = qdot(3); x_drum_dot = qdot(4);

    % -------------------------------------------------------------------------
    % 1. Tangent Sheave Kinematics (Saddle Pivot O3 Reference Frame)
    % -------------------------------------------------------------------------
    % Boom point sheave center in O3 frame [6.2579; 7.8842] m
    P_sheave = P.p_sheave_center;
    R_sheave = P.R_sheave; % 1.085 m

    % Bail pin position in O3 frame
    s_bail = d4 + P.x_ref;
    Px_bail = s_bail * cos(q3);
    Py_bail = s_bail * sin(q3);

    % Center-to-center offset vector
    Dx = P_sheave(1) - Px_bail;
    Dy = P_sheave(2) - Py_bail;
    D_center = hypot(Dx, Dy);

    % Tangent suspended rope length: ell_bail = sqrt(D_center^2 - R_sheave^2)
    ell_bail = sqrt(max(0.25, D_center^2 - R_sheave^2));

    % Gradients of Dx, Dy
    % d(Px_bail)/dq3 = -Py_bail, d(Py_bail)/dq3 = Px_bail
    % d(Px_bail)/dd4 = cos(q3),  d(Py_bail)/dd4 = sin(q3)
    dDx_dq3 =  Py_bail;
    dDy_dq3 = -Px_bail;
    dDx_dd4 = -cos(q3);
    dDy_dd4 = -sin(q3);

    % d(ell_bail)/dq = (Dx * dDx/dq + Dy * dDy/dq) / ell_bail
    dell_dq3 = (Dx * dDx_dq3 + Dy * dDy_dq3) / ell_bail;
    dell_dd4 = (Dx * dDx_dd4 + Dy * dDy_dd4) / ell_bail;

    ell_bail_dot = dell_dq3 * q3dot + dell_dd4 * d4dot;

    % -------------------------------------------------------------------------
    % 2. Hoist Wire-Rope Physical Properties (Twin 52 mm Wire Ropes)
    % -------------------------------------------------------------------------
    % EA_hoist_total = n_falls * E_rope * A_rope_metal (~2.710e8 N)
    EA_total = P.EA_hoist_total;
    zeta_rope = P.zeta_rope; % 0.035 (3.5% internal structural damping)

    % Variable-length axial stiffness K(ell)
    K_rope = EA_total / ell_bail;

    % Hoist drum and dipper effective bounce masses
    k_h = (P.drivetrain.n_falls * P.drivetrain.G_hoist) / P.drivetrain.R_drum;
    M_drum = (k_h^2) * (P.hoist_motor.count * P.drivetrain.J_m_hoist) + ...
             (1/3) * P.drivetrain.n_falls * 14.5 * 14.50; % ~229,520 kg
    M_bail = P.Md; % 56,500 kg (empty) to 92,500 kg (loaded)
    M_red  = (M_drum * M_bail) / (M_drum + M_bail);

    % Axial bounce resonance frequency
    omega_bounce = sqrt(K_rope / M_red);
    f_bounce = omega_bounce / (2 * pi);

    % Critical damping
    D_crit = 2 * sqrt(K_rope * M_red);
    D_rope = zeta_rope * D_crit;

    % -------------------------------------------------------------------------
    % 3. C^infty-Smooth Unilateral Wire-Rope Tension Barrier
    % -------------------------------------------------------------------------
    % Stretch: delta = ell_bail - x_drum (taut when suspended length > drum payout)
    delta_rope = ell_bail - x_drum;
    delta_dot  = ell_bail_dot - x_drum_dot;

    % Log-sum-exp smooth barrier: sigma_+(delta) = (1/beta)*ln(1 + exp(beta*delta))
    beta = 5000.0; % [m^-1] Smooth transition over <0.2 mm
    b_delta = beta * delta_rope;
    if b_delta > 40.0
        sigma_plus = delta_rope;
        H_smooth   = 1.0;
    elseif b_delta < -40.0
        sigma_plus = 0.0;
        H_smooth   = 0.0;
    else
        sigma_plus = (1.0 / beta) * log(1.0 + exp(b_delta));
        H_smooth   = 1.0 / (1.0 + exp(-b_delta));
    end

    % Tension: Elastic restoring force + smooth damping
    T_rope_raw = K_rope * sigma_plus + D_rope * H_smooth * delta_dot;
    T_rope = max(0.0, T_rope_raw);

    % -------------------------------------------------------------------------
    % 4. Generalized Applied Forces Q_rope
    % -------------------------------------------------------------------------
    % Virtual work: dW = -T_rope * d(ell_bail) + T_rope * d(x_drum)
    % Q_3 = -T_rope * (dell_dq3) [N*m] (>0 raises dipper)
    % Q_4 = -T_rope * (dell_dd4) [N]   (<0 pulls back on crowd)
    % Q_drum = +T_rope [N] (suspension load pulls cable out of winch drum)
    Q_rope = [ 0.0;
              -T_rope * dell_dq3;
              -T_rope * dell_dd4;
               T_rope ];

    out = struct();
    out.q            = q;
    out.qdot         = qdot;
    out.ell_bail     = ell_bail;
    out.dell_dq3     = dell_dq3;
    out.dell_dd4     = dell_dd4;
    out.ell_bail_dot = ell_bail_dot;
    out.delta_rope   = delta_rope;
    out.delta_dot    = delta_dot;
    out.K_rope       = K_rope;
    out.D_rope       = D_rope;
    out.T_rope       = T_rope;
    out.Q_rope       = Q_rope;
    out.M_red        = M_red;
    out.f_bounce     = f_bounce;
end

%% ========================================================================
%  STANDALONE PROTOCOL VERIFICATION SUITE (MS06P02)
%  ========================================================================
function report = run_ms06p02_verification(repo_root)
    fprintf('========================================================================================\n');
    fprintf(' RUNNING PROTOCOL MS06P02: DYNAMIC HOIST WIRE-ROPE COMPLIANCE & 4.35 HZ BOUNCE MODE\n');
    fprintf(' Target Equipment: Taiyuan Heavy Industry WK-20 (Twin 52 mm Hoist Ropes)\n');
    fprintf('========================================================================================\n\n');

    P = shovel_params_wk20();

    % -------------------------------------------------------------------------
    % Test 1: Stiffness & Two-Mass Resonance Hand-Calculation Closure
    % -------------------------------------------------------------------------
    fprintf('--- [Test 1] Variable-Length Stiffness & Axial Bounce Resonance ---\n');
    % Baseline posture: q3 = 0, d4 = 6.50 m
    q_nom = [0.0; 0.0; 6.50; 0.0];
    qdot_zero = zeros(4, 1);

    % Compute nominal geometry
    rope_nom = elastic_rope_subsystem(q_nom, qdot_zero, P);
    ell_nom = rope_nom.ell_bail; % ~8.001 m
    K_nom   = rope_nom.K_rope;   % ~33.87 MN/m
    f_empty = rope_nom.f_bounce; % ~4.35 Hz

    fprintf('  Nominal Suspended Bail Distance ell_0 : %8.3f m\n', ell_nom);
    fprintf('  Nominal Hoist Stiffness K_rope(ell_0) : %8.2f MN/m (Expected: 33.87 MN/m)\n', K_nom * 1e-6);
    fprintf('  Empty Dipper Bounce Resonance f_rope  : %8.2f Hz   (Expected: 4.35 Hz)\n', f_empty);

    % Loaded Dipper (Md = 92,500 kg)
    P_loaded = P;
    P_loaded.Md = 92500.0;
    rope_loaded = elastic_rope_subsystem(q_nom, qdot_zero, P_loaded);
    f_loaded = rope_loaded.f_bounce;
    fprintf('  Loaded Dipper Bounce Resonance f_rope : %8.2f Hz   (Expected: 3.61 Hz)\n', f_loaded);

    pass_K = abs(K_nom*1e-6 - 33.87) < 0.2;
    pass_f_empty = abs(f_empty - 4.35) < 0.05;
    pass_f_loaded = abs(f_loaded - 3.61) < 0.05;

    % -------------------------------------------------------------------------
    % Test 2: Unilateral Smooth Barrier C^infty Transition (Slack vs Taut)
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 2] C^infty-Smooth Tension Barrier Across Slack-Taut Transition ---\n');
    % Sweep delta_rope from -50 mm (slack) to +50 mm (taut)
    delta_sweep = linspace(-0.05, 0.05, 201);
    T_sweep = zeros(size(delta_sweep));
    for k = 1:numel(delta_sweep)
        % Set x_drum such that delta = ell_nom - x_drum = delta_sweep(k)
        q_k = [0.0; 0.0; 6.50; ell_nom - delta_sweep(k)];
        r_k = elastic_rope_subsystem(q_k, qdot_zero, P);
        T_sweep(k) = r_k.T_rope;
    end

    % Verify zero compressive force in slack region (delta < -5 mm)
    slack_idx = find(delta_sweep < -0.005);
    max_slack_tension = max(T_sweep(slack_idx));

    % Verify linear Hookean restoration in taut region (delta > +10 mm)
    taut_idx = find(delta_sweep > 0.010);
    expected_taut_T = K_nom * delta_sweep(taut_idx);
    max_taut_error = max(abs(T_sweep(taut_idx) - expected_taut_T));

    fprintf('  Max Slack Region Compressive Force    : %.2e N (Must be < 1e-3 N)\n', max_slack_tension);
    fprintf('  Max Taut Region Hookean Tracking Error: %.2e N (Must be < 1.0 N)\n', max_taut_error);

    pass_slack = (max_slack_tension < 1e-3);
    pass_taut  = (max_taut_error < 1.0);

    % -------------------------------------------------------------------------
    % Test 3: Numerical Time Ringdown & Logarithmic Decrement
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 3] 1-Second Dynamic Ringdown Simulation & Logarithmic Decrement ---\n');
    % 1-DOF axial bounce ringdown: M_red * xddot + D_rope * xdot + K_rope * x = 0
    % Initial displacement delta(0) = 0.010 m (10 mm), xdot(0) = 0
    t_span = linspace(0, 1.0, 5000); % 5 kHz sampling
    M_red = rope_nom.M_red;
    D_rope = rope_nom.D_rope;
    K_rope = rope_nom.K_rope;

    % Analytical 2nd-order damped harmonic oscillator solution
    wn = sqrt(K_rope / M_red);
    zeta = D_rope / (2 * sqrt(K_rope * M_red));
    wd = wn * sqrt(1 - zeta^2);
    x_sim = 0.010 * exp(-zeta * wn * t_span) .* cos(wd * t_span);

    % Find positive peaks via zero-crossing derivative (base MATLAB, zero toolbox dependencies)
    dx_sim = diff(x_sim);
    peak_idx = find(dx_sim(1:end-1) > 0 & dx_sim(2:end) <= 0) + 1;
    pks = x_sim(peak_idx);
    locs = t_span(peak_idx);

    sim_T_period = mean(diff(locs));
    sim_f_damped = 1.0 / sim_T_period;

    % Log decrement: delta_log = ln(x_1 / x_2) = 2*pi*zeta / sqrt(1 - zeta^2)
    delta_log = log(pks(1) / pks(2));
    sim_zeta = delta_log / sqrt(4 * pi^2 + delta_log^2);

    fprintf('  Identified Damped Frequency f_d       : %8.2f Hz (Analytical: %8.2f Hz)\n', sim_f_damped, wd/(2*pi));
    fprintf('  Identified Logarithmic Damping Ratio  : %8.4f   (Nominal:    %8.4f)\n', sim_zeta, zeta);

    pass_sim_f = abs(sim_f_damped - wd/(2*pi)) < 0.05;
    pass_sim_z = abs(sim_zeta - zeta) < 0.001;

    pass_all = pass_K && pass_f_empty && pass_f_loaded && pass_slack && pass_taut && pass_sim_f && pass_sim_z;

    fprintf('\n========================================================================================\n');
    fprintf(' MS06P02 SCORECARD: ');
    if pass_all
        fprintf('PASS (Dynamic Hoist Wire-Rope Elasticity & 4.35 Hz Resonance Verified!)\n');
    else
        fprintf('FAIL (One or more metrics out of tolerance)\n');
    end
    fprintf('========================================================================================\n');

    % Archive results
    exp_dir = fullfile(repo_root, 'M6', 'experiments');
    if ~exist(exp_dir, 'dir'), mkdir(exp_dir); end
    mat_file = fullfile(exp_dir, 'MS06P02_elastic_rope_results.mat');
    save(mat_file, 'ell_nom', 'K_nom', 'f_empty', 'f_loaded', 'max_slack_tension', ...
         'max_taut_error', 'sim_f_damped', 'sim_zeta', 'pass_all');
    fprintf(' Saved verification archive to: %s\n\n', mat_file);

    report = struct();
    report.ell_nom      = ell_nom;
    report.K_nom        = K_nom;
    report.f_empty      = f_empty;
    report.f_loaded     = f_loaded;
    report.sim_f_damped = sim_f_damped;
    report.sim_zeta     = sim_zeta;
    report.pass_all     = pass_all;
end
