function [P, geom_table] = shovel_params_wk20(q3_eval, d4_eval)
%SHOVEL_PARAMS_WK20  Target field machine parameter source & hoist-rope sheave geometry for NCL Amlohri TZ WK-20.
%
%   PURPOSE (per CODE_EXPLAINABILITY_PROTOCOL.md):
%     1) Encodes the verified physical, geometric, kinematic, and electrical
%        drive specifications of the Taiyuan Heavy Industry (TZ) WK-20
%        Electric Rope Shovel deployed at Northern Coalfields Limited (NCL),
%        Amlohri Opencast Project (Project IIT/SRIC/R/AEH/2026/104 / MT-189).
%     2) Preserves strict separation (per CLAIMS_INTEGRITY_PROTOCOL.md)
%        between:
%          - [V] Verified OEM / Nameplate Source Facts (from '3. AMLOHRI
%                WK-20 TZ ROPE SHOVEL - Technical Specifications (1).docx'
%                and Oct 2026 field visit nameplate photos IMG20261009104021_01.jpg),
%          - [D] Derived geometric & transmission quantities, and
%          - [ESTIMATE] Scaled inertial/friction priors pending OEM CAD
%                or free-swing field identification at Amlohri.
%     3) Computes the exact planar hoist-rope-to-handle sheave geometry
%        mapping alpha_3(q_3, d_4), free rope length l_rope(q_3, d_4),
%        length-dependent axial rope stiffness K_rope(q_3, d_4), and the
%        2x2 actuator transmission matrix T_rope(q_3, d_4) mapping hoist
%        rope tension F_hoist and crowd rack force F_crowd into generalized
%        joint efforts [tau_{a,3}; F_{a,4}]:
%
%            tau_{a,3} =  s_bail(d_4) * sin(alpha_3(q_3, d_4)) * F_hoist
%            F_{a,4}   =  cos(alpha_3(q_3, d_4)) * F_hoist + F_crowd
%
%   COORDINATE & ANGLE CONVENTION (Reconciled with shovel_params.m, Rasuli 2014 Fig. 6, & Fu 2022 Eq. 17):
%     - Origin O_3 = [0; 0] is the saddle-block revolute pivot axis.
%     - +x is horizontal forward toward the digging bank; +y is vertical upward.
%     - q_3 [rad] is the handle angle measured CCW from the horizontal (+x) axis
%       (so gravity torque is M_d * g * r * cos(q_3) and axial gravity component
%       along the handle is +M_d * g * sin(q_3), identical to equations_master.tex).
%       NOTE ON PHYSICAL DIGGING ARC vs. SIMSCAPE TEST ARC:
%         In physical operation (Rasuli 2014 Fig. 6), the handle pitches from
%         q_3 = -45 deg (toe of bank, pointing forward-downward) through q_3 = 0 deg
%         (horizontal) up to q_3 = +20 deg (bank exit / boom bumper limit), because
%         the fixed boom lies at +45 deg. Evaluating q_3 >= +35 deg at large d_4
%         approaches the physical boom-point sheave (flagged via geom.boom_interference).
%     - d_4 [m] is the prismatic crowd extension along the handle axis.
%     - alpha_3(q_3, d_4) [rad] is the interior angle between the outward handle
%       unit vector e_r(q_3) = [cos(q_3); sin(q_3)] and the hoist rope segment
%       running from the bail pin B(q_3, d_4) to the boom-point sheave tangent.
%
%   USAGE:
%     P = shovel_params_wk20();
%     [P, geom] = shovel_params_wk20(deg2rad([-45, -30, -15, 0, 15, 25]), [4.50, 6.50, 8.50]);
%
%   Repository: scripts/shovel_params_wk20.m
%   See also SHOVEL_PARAMS, BUILD_M2_TORQUEDRIVEN, BUILD_M3_DISCRETEOBSERVER_30HZ.

    %% 1. Gravity & Environment [V: Standard]
    P.g              = 9.80665;                 % [m/s^2] Standard gravity
    P.machine_name   = 'Taiyuan Heavy Industry (TZ) WK-20 Electric Rope Shovel';
    P.mine_site      = 'NCL Amlohri Opencast Mine, Singrauli';
    P.project_code   = 'IIT/SRIC/R/AEH/2026/104 (MT-189)';

    %% 2. Verified Machine General Specifications [V: Amlohri WK-20 Spec Sheet]
    P.V_dipper_nom   = 20.0;                    % [m^3]   Struck SAE rated dipper capacity (range 16-34 m^3) [V]
    P.m_payload_nom  = 36000.0;                 % [kg]    Nominal rated payload (36 tonnes) [V]
    P.F_payload_nom  = P.m_payload_nom * P.g;   % [N]     353,039.4 N static weight of nominal 36 t payload [D]
    P.m_working_1321 = 779000.0;                % [kg]    Working weight with 1321 mm crawler shoes (779 t) [V]
    P.m_working_1778 = 792000.0;                % [kg]    Working weight with 1778 mm crawler shoes (792 t) [V]
    P.m_counterwt    = 135000.0;                % [kg]    Counterweight mass (135 t) [V]
    P.T_cycle_nom    = 30.0;                    % [s]     Rated bucket cycle time for 90 deg swing [V]

    %% 3. Verified Boom, Sheave & Handle Geometry [V: Amlohri WK-20 Spec Sheet]
    P.L_boom         = 15.50;                   % [m]     Boom length (foot pin to boom-point sheave) [V]
    P.theta_boom_deg = 45.0;                    % [deg]   Fixed boom inclination angle from horizontal [V]
    P.theta_boom     = deg2rad(P.theta_boom_deg);
    P.D_sheave       = 2.17;                    % [m]     Boom top point sheave diameter [V]
    P.R_sheave       = P.D_sheave / 2.0;        % [m]     1.085 m boom point sheave radius [D]
    P.L_handle_eff   = 9.70;                    % [m]     Effective dipper handle length [V]
    P.H_sheave_clear = 16.00;                   % [m]     Clearance height of boom point sheave above ground [V]
    P.R_sheave_clear = 15.46;                   % [m]     Clearance radius of boom point sheave from swing axis [V]
    P.R_max_dig      = 21.02;                   % [m]     Maximum digging radius [V]
    P.H_max_cut      = 13.54;                   % [m]     Maximum height of cut [V]
    P.R_floor_level  = 13.77;                   % [m]     Floor level digging radius [V]
    P.R_max_dump     = 17.50;                   % [m]     Maximum dumping radius [V]
    P.H_max_dump     = 9.07;                    % [m]     Maximum dumping height (door open) [V]

    % Saddle-block pivot to boom-point sheave center in O_3 frame:
    % With L_boom = 15.50 m and saddle pivot at L_boom_foot_to_saddle = 5.50 m from boom foot,
    % the distance along the 45 deg boom from O_3 to the top sheave is 10.00 m, plus a
    % 1.15 m upward/forward sheave-head offset [ESTIMATE: pending OEM GA drawing B2]:
    P.L_boom_foot_to_saddle = 5.50;             % [m]     [ESTIMATE: boom foot to saddle block pivot]
    P.L_saddle_to_sheave    = P.L_boom - P.L_boom_foot_to_saddle; % = 10.00 m along 45 deg boom
    P.h_sheave_offset       = 1.15;             % [m]     [ESTIMATE: sheave center offset normal to boom axis]
    e_boom_para = [ cos(P.theta_boom); sin(P.theta_boom)];
    e_boom_norm = [-sin(P.theta_boom); cos(P.theta_boom)];
    P.p_sheave_center = P.L_saddle_to_sheave * e_boom_para + P.h_sheave_offset * e_boom_norm; % [6.2579; 7.8842] m

    %% 4. Verified Kinematic & Effort Ceilings [V: Amlohri WK-20 Spec Sheet]
    P.v_hoist_max    = 1.58;                    % [m/s]   Maximum hoist rope speed [V]
    P.d4dot_max      = 0.54;                    % [m/s]   Maximum crowd extension speed (stricter than P&H 0.70 m/s) [V]
    P.F_hoist_max    = 1540e3;                  % [N]     Maximum hoisting force (1,540 kN) [V]
    P.F_crowd_max    = 745e3;                   % [N]     Maximum crowding force (745 kN) [V]
    P.d4ddot_max     = 0.50;                    % [m/s^2] Crowd acceleration design limit [D: WK-55 prior, Bi et al. 2020]
    P.q3dot_max      = 0.45;                    % [rad/s] Saddle angular rate limit [D]
    P.q3_dig_min_deg = -45.0;                   % [deg]   Bank toe entry pitch angle (Rasuli 2014 Fig. 6) [D]
    P.q3_dig_max_deg =  25.0;                   % [deg]   Bank exit / upper pitch limit before boom interference [D]

    % Admissible crowd stroke in project datum (d_4 measured to reference datum x_ref = 1.50 m upstream of bail):
    P.x_ref          = 1.50;                    % [m]     Project datum offset upstream of bail pin [D: locked to M1/M2]
    P.d4_min         = 4.20;                    % [m]     Minimum retracted crowd extension in project datum [ESTIMATE]
    P.d4_max         = 8.80;                    % [m]     Maximum extended crowd extension in project datum [ESTIMATE]
    P.x_tip          = 2.65;                    % [m]     Tooth-tip station offset (s_tip = d_4 + 2.65 m) [ESTIMATE]

    %% 5. Dipper + Handle Inertial & Friction Priors [ESTIMATE: Scaled from Rasuli 2014 Pending Field ID]
    P.m              = 56500.0;                 % [kg]        [ESTIMATE] Empty dipper + handle mass M_d
    P.Md             = P.m;                     % [kg]        Alias for M3 scripts
    P.L_COG          = 2.82;                    % [m]         [ESTIMATE] Distance from bail pin to empty COM
    P.c              = P.x_ref - P.L_COG;       % [m]         -1.32 m net lever offset: r_COG(t) = d_4(t) + c
    P.Izz            = 305000.0;                % [kg*m^2]    [ESTIMATE] Rotational inertia about COM
    P.fv3            = 4100.0;                  % [N*m*s/rad] [ESTIMATE] Saddle revolute viscous damping
    P.fv3_degs       = P.fv3 * pi / 180.0;      % [N*m/(deg/s)] Simscape Revolute Joint dialog unit
    P.fc3            = 380.0;                   % [N*m]       [ESTIMATE] Saddle revolute Coulomb friction
    P.fv4            = 1600.0;                  % [N*s/m]     [ESTIMATE] Crowd prismatic viscous damping
    P.fc4            = 520.0;                   % [N]         [ESTIMATE] Crowd prismatic Coulomb friction
    P.eps3           = 3e-4;                    % [rad/s]     Regularization threshold for tanh(dq3/eps3)
    P.eps4           = 3e-4;                    % [m/s]       Regularization threshold for tanh(dd4/eps4)
    P.model_has_coulomb = false;                % Boolean     Flip to true when Coulomb block is active in .slx

    %% 6. Hoist Rope Elasticity Parameters [ESTIMATE: Twin-Fall 52 mm Wire Rope]
    P.n_falls        = 2;                       % [-]     Twin hoist rope falls wrapping bail & boom sheave
    P.d_rope         = 0.052;                   % [m]     52 mm wire rope nominal diameter [ESTIMATE]
    P.A_rope_metal   = 0.58 * pi * (P.d_rope/2)^2; % [m^2] Metallic cross-sectional area (~1.232e-3 m^2 per rope)
    P.E_rope         = 1.10e11;                 % [Pa]    Effective wire-rope elastic modulus (110 GPa)
    P.EA_hoist_total = P.n_falls * P.E_rope * P.A_rope_metal; % [N] ~2.710e8 N axial rigidity (K_rope = EA / l_rope)
    P.zeta_rope      = 0.035;                   % [-]     Structural damping ratio of hoist wire rope

    %% 7. Verified Electrical Drive & Motor Nameplate Data [V: Spec Sheet + Oct 2026 Nameplates]
    P.drive_system   = 'Siemens SINAMICS S120 AC VFD (CU320-2)';
    P.plc_cpu        = 'Siemens SIMATIC S7-300 / S7-400 CPU 319-3 PN/DP';
    P.fieldbus       = 'PROFIBUS DP (12 Mbps) + Distributed ET200 I/O';
    P.V_supply_nom   = 6600.0;                  % [V]     6.6 kV 3-phase 50 Hz trailing cable supply [V]
    P.V_motor_nom    = 690.0;                   % [V]     690 VAC inverter-fed induction motor bus [V]
    P.Fs_observer    = 30.0;                    % [Hz]    Standard 30 Hz observer sampling rate
    P.Ts_observer    = 1.0 / P.Fs_observer;     % [s]     33.3333 ms sample period

    % Motor Power Ratings [V: Amlohri WK-20 Spec Sheet]:
    P.hoist_motor.make        = 'CRRC Yongji Electric Co., Ltd.';
    P.hoist_motor.count       = 2;              % Dual hoist motors driving common hoist drum
    P.hoist_motor.P_rated_kW  = 600.0;          % [kW]    2 x 600 kW = 1,200 kW continuous rated power [V]
    P.hoist_motor.P_peak_kW   = 750.0;          % [kW]    2 x 750 kW = 1,500 kW peak power [V]
    P.hoist_motor.V_rated_V   = 690.0;          % [V]     Rated stator voltage [V]

    P.crowd_motor.make        = 'CRRC Yongji Electric Co., Ltd.';
    P.crowd_motor.count       = 1;
    P.crowd_motor.P_rated_kW  = 350.0;          % [kW]    350 kW continuous rated power [V]
    P.crowd_motor.P_peak_kW   = 437.5;          % [kW]    437.5 kW peak power [V]
    P.crowd_motor.V_rated_V   = 690.0;          % [V]     Rated stator voltage [V]

    % Swing Motor Nameplate [V: Verified from Oct 2026 Field Photo IMG20261009104021_01.jpg]:
    P.swing_motor.make        = 'CRRC Yongji Electric Co., Ltd.';
    P.swing_motor.model       = 'YI80A1';       % [V]     Read directly from nameplate photo
    P.swing_motor.serial_1    = '290A1120018';  % [V]     Motor 1 serial (Date: 2016-09)
    P.swing_motor.serial_2    = '380A1120018';  % [V]     Motor 2 serial (Date: 2016-09)
    P.swing_motor.count       = 2;
    P.swing_motor.P_rated_kW  = 260.0;          % [kW]    2 x 260 kW rated output [V]
    P.swing_motor.P_peak_kW   = 325.0;          % [kW]    2 x 325 kW peak power [V]
    P.swing_motor.I_rated_A   = 281.0;          % [A]     Rated stator current [V]
    P.swing_motor.V_rated_V   = 690.0;          % [V]     Rated voltage [V]
    P.swing_motor.bearing_DE  = 'NU324 ECM/C3 VL0241'; % [V] Drive-end cylindrical roller bearing
    P.swing_motor.bearing_NDE = 'NU224 ECM/C3';        % [V] Non-drive-end bearing
    P.swing_motor.grease      = 'Mobilith SHC 100 (25 g / 8000 h)'; % [V]

    P.propel_motor.make       = 'CRRC Yongji Electric Co., Ltd.';
    P.propel_motor.count      = 2;
    P.propel_motor.P_rated_kW = 450.0;          % [kW]    2 x 450 kW rated power [V]
    P.propel_motor.P_peak_kW  = 562.5;          % [kW]    2 x 562.5 kW peak power [V]

    %% 8. Evaluate Hoist-Rope Sheave Geometry Mapping if Requested
    if nargin < 1 || isempty(q3_eval)
        q3_eval = deg2rad([-45, -30, -15, 0, 15, 25]);
    end
    if nargin < 2 || isempty(d4_eval)
        d4_eval = [4.50, 6.50, 8.50];
    end
    geom_table = compute_wk20_sheave_map(q3_eval, d4_eval, P);
end

function geom = compute_wk20_sheave_map(q3_vec, d4_vec, P)
%COMPUTE_WK20_SHEAVE_MAP  Closed-form hoist-rope angle alpha_3(q3, d4) and transmission map.
    [Q3, D4] = ndgrid(q3_vec(:), d4_vec(:));
    S_bail   = D4 + P.x_ref;
    Px_bail  = S_bail .* cos(Q3);
    Py_bail  = S_bail .* sin(Q3);

    Dx       = P.p_sheave_center(1) - Px_bail;
    Dy       = P.p_sheave_center(2) - Py_bail;
    D_center = hypot(Dx, Dy);

    psi_center = atan2(Dy, Dx);
    delta_tang = asin(min(0.999, P.R_sheave ./ max(P.R_sheave * 1.001, D_center)));
    psi_rope   = psi_center + delta_tang;
    L_rope     = max(0.50, sqrt(max(0.0, D_center.^2 - P.R_sheave^2)));
    Alpha3     = psi_rope - Q3;
    K_rope     = P.EA_hoist_total ./ L_rope;

    % Moment arm on q3 and axial coupling factor on d4:
    Arm_hoist_q3   = S_bail .* sin(Alpha3);             % [m]   d(tau_{a,3}) / d(F_hoist)
    Coupling_d4    = cos(Alpha3);                       % [-]   d(F_{a,4})   / d(F_hoist)
    Tau3_max_MNm   = 1e-6 * Arm_hoist_q3 * P.F_hoist_max; % [MN*m] Max hoist-induced saddle torque

    geom = struct();
    geom.q3_rad            = Q3;
    geom.q3_deg            = rad2deg(Q3);
    geom.d4_m              = D4;
    geom.s_bail_m          = S_bail;
    geom.alpha3_rad        = Alpha3;
    geom.alpha3_deg        = rad2deg(Alpha3);
    geom.l_rope_m          = L_rope;
    geom.K_rope_N_per_m    = K_rope;
    geom.arm_hoist_m       = Arm_hoist_q3;
    geom.crowd_coupling    = Coupling_d4;
    geom.tau3_max_MNm      = Tau3_max_MNm;
    geom.boom_interference = (rad2deg(Q3) > P.q3_dig_max_deg) | (L_rope < 1.50);
end
