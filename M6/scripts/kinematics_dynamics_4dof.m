function out = kinematics_dynamics_4dof(q, qdot, P)
%KINEMATICS_DYNAMICS_4DOF  3D 4-DOF Euler-Lagrange Dynamics & Passivity for WK-20.
%
%   MS06P01: Computes the exact 4x4 configuration-dependent mass matrix M(q),
%   Christoffel tensor C(q, qdot), gravity vector G(q), and spatial Jacobian
%   J_tip,3D for the 4-DOF WK-20 rope shovel model:
%     q = [q1; q3; d4; x_drum]
%     q1     : Turntable swing angle about vertical kingpin +Z0 [rad]
%     q3     : Saddle pitch angle CCW from horizontal +X0 [rad]
%     d4     : Prismatic crowd handle extension along +X4 [m]
%     x_drum : Bail-referred hoist drum payout [m]
%
%   Features:
%     1) Full 3D non-spherical attitude inertia J_rot(q3) = Ixx*sin^2(q3) + Iyy*cos^2(q3).
%     2) Gyroscopic pitch nutation moment (Ixx - Iyy)*sin(2*q3)*q1dot^2 / 2.
%     3) Exact skew-symmetry proof: z^T * (Mdot - 2*C) * z = 0 for all z in R^4.
%     4) Rigorous centrifugal crowd pull (F4_cent = 148.48 kN) and Coriolis swing torque.
%
%   USAGE:
%     out = kinematics_dynamics_4dof(); % Standalone verification suite (MS06P01)
%     out = kinematics_dynamics_4dof(q, qdot, P);
%
%   See also SHOVEL_PARAMS_WK20, ELASTIC_ROPE_SUBSYSTEM, OBSERVER_4DOF_GMO.

    persistent path_added;
    if isempty(path_added)
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        addpath(fullfile(repo_root, 'M1', 'scripts'));
        path_added = true;
    end

    % Standalone verification mode
    if nargin == 0
        repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        out = run_ms06p01_verification(repo_root);
        return;
    end

    if nargin < 3 || isempty(P), P = shovel_params_wk20(); end
    if nargin < 2 || isempty(qdot), qdot = zeros(4, 1); end

    q = q(:);
    qdot = qdot(:);

    q1 = q(1); q3 = q(2); d4 = q(3); x_drum = q(4);
    q1dot = qdot(1); q3dot = qdot(2); d4dot = qdot(3); x_drum_dot = qdot(4);

    % Geometry parameters
    X_O3 = 8.117; % [m] Saddle pivot radial distance from swing axis
    Z_O3 = 5.250; % [m] Saddle pivot vertical height
    r_COG = d4 + P.c; % [m] Lever arm to composite dipper COM (c = -1.32 m)
    rho_COM = X_O3 + r_COG * cos(q3); % [m] Horizontal radius from swing axis

    % 3D Inertia Tensors
    % Dipper handle principal inertias: Ixx (longitudinal roll), Iyy (transverse pitch), Izz (yaw)
    I_xx = 45000.0;  % [kg*m^2] Slender handle roll inertia
    I_yy = 285000.0; % [kg*m^2] Pitch plane transverse inertia
    I_zz = P.Izz;    % [kg*m^2] 305,000 kg*m^2 pitch inertia about COM

    % Swing drive reflected inertia: G_swing^2 * (2 * J_m_swing)
    G_swing = 124.0;
    J_m_swing = 1.955; % [kg*m^2] Dual CRRC Yongji YJ80A1 motors
    I_refl_swing = (G_swing^2) * (2 * J_m_swing); % ~60,121 kg*m^2
    I_upper_zz = 2500000.0; % [kg*m^2] Revolving upper deck + boom + counterweight

    % Crowd reflected mass: (G_c / R_p)^2 * J_m_c
    k_c = P.drivetrain.G_crowd / P.drivetrain.R_pinion; % 142.9412 m^-1
    M_refl_c = (k_c^2) * P.drivetrain.J_m_crowd;       % 302,401.8 kg

    % Hoist drum reflected mass: (n_falls * G_h / R_drum)^2 * (2 * J_m_h)
    k_h = (P.drivetrain.n_falls * P.drivetrain.G_hoist) / P.drivetrain.R_drum; % 65.7868 m^-1
    M_refl_h = (k_h^2) * (P.hoist_motor.count * P.drivetrain.J_m_hoist);      % 229,379.7 kg
    M_drum = M_refl_h + (1/3) * P.drivetrain.n_falls * 14.5 * 14.50;          % 229,520 kg

    % -------------------------------------------------------------------------
    % 1. 4x4 Mass Matrix M(q)
    % -------------------------------------------------------------------------
    % 3D Attitude Inertia J_rot(q3)
    J_rot_11 = I_xx * (sin(q3)^2) + I_yy * (cos(q3)^2);

    D11 = I_upper_zz + I_refl_swing + J_rot_11 + P.Md * (rho_COM^2);
    D22 = I_zz + P.Md * (r_COG^2);
    D33 = P.Md + M_refl_c;
    D44 = M_drum;

    M = diag([D11, D22, D33, D44]);

    % -------------------------------------------------------------------------
    % 2. 4x4 Christoffel Matrix C(q, qdot)
    % -------------------------------------------------------------------------
    % Gradients of D11
    dJrot_dq3 = (I_xx - I_yy) * sin(2 * q3);
    dD11_dq3  = dJrot_dq3 - 2 * P.Md * rho_COM * r_COG * sin(q3);
    dD11_dd4  = 2 * P.Md * rho_COM * cos(q3);

    % Gradients of D22
    dD22_dd4  = 2 * P.Md * r_COG;

    % Time derivative of Mass Matrix Mdot(q, qdot)
    dD11_dt = dD11_dq3 * q3dot + dD11_dd4 * d4dot;
    dD22_dt = dD22_dd4 * d4dot;
    Mdot = diag([dD11_dt, dD22_dt, 0.0, 0.0]);

    C12 = 0.5 * dD11_dq3 * q1dot;
    C13 = 0.5 * dD11_dd4 * q1dot;
    C11 = 0.5 * dD11_dt;

    C21 = -C12;
    C22 = 0.5 * dD22_dt;
    C23 = 0.5 * dD22_dd4 * q3dot;

    C31 = -C13;
    C32 = -C23;
    C33 = 0.0;

    C = [ C11, C12, C13, 0.0;
          C21, C22, C23, 0.0;
          C31, C32, C33, 0.0;
          0.0, 0.0, 0.0, 0.0 ];

    % -------------------------------------------------------------------------
    % 3. Gravity Vector G(q)
    % -------------------------------------------------------------------------
    G = [ 0.0;
          P.Md * P.g * r_COG * cos(q3);
          P.Md * P.g * sin(q3);
          0.0 ];

    % -------------------------------------------------------------------------
    % 5. 3D Spatial Tooth-Tip Position and Jacobian J_tip,3D
    % -------------------------------------------------------------------------
    L_tip = 2.85; % [m] Dipper tooth tip offset forward of bail pin
    s_tip = d4 + P.x_ref + L_tip;
    rho_tip = X_O3 + s_tip * cos(q3);
    z_tip   = Z_O3 + s_tip * sin(q3);

    r_tip = [ rho_tip * cos(q1);
              rho_tip * sin(q1);
              z_tip ];

    % Jacobian relating qdot_L = [q1dot; q3dot; d4dot] to v_tip in World Frame {0}
    J_tip = [ -rho_tip * sin(q1), -s_tip * sin(q3) * cos(q1), cos(q3) * cos(q1);
               rho_tip * cos(q1), -s_tip * sin(q3) * sin(q1), cos(q3) * sin(q1);
               0.0,                s_tip * cos(q3),           sin(q3) ];

    out = struct();
    out.q         = q;
    out.qdot      = qdot;
    out.M         = M;
    out.Mdot      = Mdot;
    out.C         = C;
    out.G         = G;
    out.r_tip     = r_tip;
    out.J_tip_3D  = J_tip;
    out.rho_COM   = rho_COM;
    out.rho_tip   = rho_tip;
    out.F4_cent   = C31 * q1dot;
    out.tau3_cent = C21 * q1dot;
    out.tau1_coriolis = 2 * (C12 * q3dot + C13 * d4dot);
end

%% ========================================================================
%  STANDALONE PROTOCOL VERIFICATION SUITE (MS06P01)
%  ========================================================================
function report = run_ms06p01_verification(repo_root)
    fprintf('========================================================================================\n');
    fprintf(' RUNNING PROTOCOL MS06P01: 3D 4-DOF VARIATIONAL MECHANICS & PASSIVITY SKEW-SYMMETRY\n');
    fprintf(' Target Equipment: Taiyuan Heavy Industry WK-20 (NCL Amlohri Mine)\n');
    fprintf('========================================================================================\n\n');

    P = shovel_params_wk20();

    % -------------------------------------------------------------------------
    % Test 1: Machine-Precision Skew-Symmetry Verification (Mdot - 2*C)
    % -------------------------------------------------------------------------
    fprintf('--- [Test 1] Analytical Passivity Skew-Symmetry Proof: z^T*(Mdot - 2*C)*z == 0 ---\n');
    rng(202604, 'twister'); % Deterministic seed
    N_trials = 100;
    skew_errors = zeros(N_trials, 1);
    sym_part_norms = zeros(N_trials, 1);

    for i = 1:N_trials
        % Random generalized state within physical operational limits
        q_test = [ (rand - 0.5) * pi;          % q1 in [-90, +90] deg
                   deg2rad(-30 + 55 * rand);   % q3 in [-30, +25] deg
                   5.0 + 4.5 * rand;           % d4 in [5.0, 9.5] m
                   10.0 + 5.0 * rand ];        % x_drum in [10, 15] m
        qdot_test = [ (rand - 0.5) * 0.7;      % q1dot in [-0.35, +0.35] rad/s (up to 3.5 rpm)
                      (rand - 0.5) * 0.3;      % q3dot in [-0.15, +0.15] rad/s
                      (rand - 0.5) * 1.0;      % d4dot in [-0.50, +0.50] m/s
                      (rand - 0.5) * 2.0 ];    % x_drum_dot in [-1.0, +1.0] m/s

        dyn = kinematics_dynamics_4dof(q_test, qdot_test, P);
        M = dyn.M;
        Mdot = dyn.Mdot;
        C = dyn.C;

        % Skew-symmetry matrix: S = Mdot - 2*C
        S = Mdot - 2 * C;

        % Exact Frobenius norm of symmetric part: (S + S') == 0
        sym_part_norms(i) = norm(S + S', 'fro');

        % Quadratic form: z^T * S * z (pure floating point summation)
        z = randn(4, 1);
        skew_errors(i) = abs(z' * S * z);
    end

    max_sym_part = max(sym_part_norms);
    max_skew_err = max(skew_errors);
    fprintf('  Maximum ||S + S^T||_F (Exact Skew-Symmetry) across %d states: %.4e\n', N_trials, max_sym_part);
    fprintf('  Maximum |z^T*S*z| Quadratic Form Roundoff across %d states: %.4e\n', N_trials, max_skew_err);
    pass_skew = (max_sym_part < 1e-12) && (max_skew_err < 1e-8);

    % -------------------------------------------------------------------------
    % Test 2: Certified Hand-Calculation Benchmarks on Loaded WK-20
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 2] Certified 3D Cross-Coupling Hand Calculations (Amlohri Loaded WK-20) ---\n');
    % Loaded dipper: Payload 36 t -> Md = 56,500 + 36,000 = 92,500 kg
    P_loaded = P;
    P_loaded.Md = 92500.0;
    q_bench = [0.0; 0.0; 7.50; 12.0]; % q1 = 0, q3 = 0, d4 = 7.50 m
    % At q3 = 0, r_COG = 7.50 - 1.32 = 6.18 m -> rho_COM = 8.117 + 6.18 = 14.297 m
    q1dot_target = 0.3351032; % 3.2 rpm = 3.2 * 2*pi / 60
    d4dot_target = 0.54;      % Rated crowd speed 0.54 m/s
    qdot_bench = [q1dot_target; 0.0; d4dot_target; 0.0];

    dyn_bench = kinematics_dynamics_4dof(q_bench, qdot_bench, P_loaded);

    % Expected centrifugal crowd pull: Md * rho_COM * (q1dot^2)
    expected_F4_cent = 92500.0 * 14.297 * (q1dot_target^2); % 148,477.5 N
    actual_F4_cent   = abs(dyn_bench.F4_cent);
    err_F4_cent      = abs(actual_F4_cent - expected_F4_cent);

    % Expected Coriolis swing torque: 2 * Md * rho_COM * d4dot * q1dot
    expected_tau1_cor = 2.0 * 92500.0 * 14.297 * d4dot_target * q1dot_target; % 478,690.6 Nm
    actual_tau1_cor   = abs(dyn_bench.tau1_coriolis);
    err_tau1_cor      = abs(actual_tau1_cor - expected_tau1_cor);

    fprintf('  Centrifugal Crowd Pull |F4,cent|   : %8.2f kN (Expected: %8.2f kN | Error: %.2f N)\n', ...
        actual_F4_cent * 1e-3, expected_F4_cent * 1e-3, err_F4_cent);
    fprintf('  Coriolis Swing Reaction Torque      : %8.2f kNm (Expected: %8.2f kNm | Error: %.2f Nm)\n', ...
        actual_tau1_cor * 1e-3, expected_tau1_cor * 1e-3, err_tau1_cor);

    pass_cent = (err_F4_cent < 1.0);
    pass_cor  = (err_tau1_cor < 1.0);

    % -------------------------------------------------------------------------
    % Test 3: 3D Attitude Inertia & Gyroscopic Pitch Nutation
    % -------------------------------------------------------------------------
    fprintf('\n--- [Test 3] 3D Non-Spherical Attitude Inertia & Pitch Nutation Moment ---\n');
    % Pitch angle sweep q3 from -30 deg to +30 deg at q1dot = 0.3351 rad/s
    q3_sweep = deg2rad(linspace(-30, 30, 100))';
    tau3_nutation = zeros(size(q3_sweep));
    for j = 1:numel(q3_sweep)
        dyn_j = kinematics_dynamics_4dof([0.0; q3_sweep(j); 7.50; 12.0], [q1dot_target; 0.0; 0.0; 0.0], P);
        tau3_nutation(j) = dyn_j.tau3_cent;
    end
    max_nutation_kNm = max(abs(tau3_nutation)) * 1e-3;
    fprintf('  Peak Gyroscopic Pitch Restoring Moment : %8.2f kNm\n', max_nutation_kNm);
    pass_nutation = (max_nutation_kNm > 50.0);

    pass_all = pass_skew && pass_cent && pass_cor && pass_nutation;

    fprintf('\n========================================================================================\n');
    fprintf(' MS06P01 SCORECARD: ');
    if pass_all
        fprintf('PASS (3D 4-DOF Variational Mechanics & Skew-Symmetry Verified!)\n');
    else
        fprintf('FAIL (One or more metrics out of tolerance)\n');
    end
    fprintf('========================================================================================\n');

    % Archive results
    exp_dir = fullfile(repo_root, 'M6', 'experiments');
    if ~exist(exp_dir, 'dir'), mkdir(exp_dir); end
    mat_file = fullfile(exp_dir, 'MS06P01_4dof_dynamics_results.mat');
    save(mat_file, 'max_skew_err', 'actual_F4_cent', 'actual_tau1_cor', 'max_nutation_kNm', 'pass_all');
    fprintf(' Saved verification archive to: %s\n\n', mat_file);

    report = struct();
    report.max_skew_err     = max_skew_err;
    report.actual_F4_cent   = actual_F4_cent;
    report.actual_tau1_cor  = actual_tau1_cor;
    report.max_nutation_kNm = max_nutation_kNm;
    report.pass_all         = pass_all;
end
