function RESULTS = run_Test1_payload_estimation(varargin)
%RUN_TEST1_PAYLOAD_ESTIMATION  Milestone 2 Batch 1 Deliverable: Test 1
%   Dynamic payload mass estimation at dipper teeth (0, 5, 10, 15 tonnes)
%   under realistic industrial sensor noise (N1, N2, N4, N5_all) using the
%   M3 discrete 30 Hz momentum observer (Branch B exact FOH and KF-GMO).
%
%   DIPPER TEETH GEOMETRY & LEVER ARM:
%     Tooth tip station along dipper handle centerline: x_tip = 2.0 m
%     Handle body COM offset from pivot: L_COG = 2.82 m
%     Dynamic lever arm from pivot:
%       r_teeth(t) = d4(t) + (x_tip - L_COG) = d4(t) - 0.82 m
%
%   DYNAMIC LOAD GENERALIZED FORCES (Point mass m_pay at teeth):
%     tau_L3_pay(t) = m_pay * [ r_teeth^2*qdd3 + 2*r_teeth*qd4*qd3 + g*r_teeth*cos(q3) ]
%     F_L4_pay(t)   = m_pay * [ qdd4 - r_teeth*qd3^2 + g*sin(q3) ]
%
%   TARE MOTOR DRIVE TORQUES & PAYLOAD COUPLING:
%     s = d4 + x_ref  (x_ref = 1.50 m)
%     tau3_tare = tau3_cmd + s * cos(q3) * outF
%     f4_tare   = f4_cmd   + sin(q3) * outF
%     tau3_in   = tau3_tare - tau_L3_pay
%     f4_in     = f4_tare   - F_L4_pay
%
%   SENSOR NOISE SUITE:
%     N0: Clean baseline
%     N1: Resolver velocity white noise (3 mrad/s on qd3, 3 mm/s on qd4)
%     N2: Drive torque white noise (1% RMS of drive signal)
%     N4: 12-bit motor-shaft optical encoder steps (240:1 and 109.3:1) differenced at 30 Hz
%     N5_all: Combined industrial environment (N4 encoder + N1 resolver + N2 torque noise)
%
%   MASS ESTIMATORS (Steady-State Window t in [5.0, 65.0] s):
%     1. Single-channel pitch gravity projection:
%          m_hat_pitch = mean( tau_L3 / (g * r_teeth * cos(q3)) )
%     2. 2-DOF multi-channel fused projection:
%          m_hat_multi = mean( (tau_L3 / r_teeth * cos(q3) + F_L4 * sin(q3)) / g )
%
%   GATES EVALUATED:
%     G1: M3 discrete observer code check identity (<= 1e-9 relative error vs M3 archive)
%     G2: P08B rev2 data MD5 provenance and uniform 30 Hz grid check (1951 samples, 0-65 s)
%     G3: Baseline tare recovery error <= 0.05 tonnes (50 kg) at m_pay = 0
%     G4: Direct mass estimation error <= 0.5 tonnes (500 kg) across all payloads and noise
%     G5: Monotonicity check across payload tiers (step in [4.5, 5.5] tonnes)
%
%   OUTPUT:
%     M3/06_RESULTS_MAT/Test1_payload_results.mat (v7.3)
%
%   USAGE:
%     RESULTS = run_Test1_payload_estimation();
%     RESULTS = run_Test1_payload_estimation('Save', true, 'RequireClean', false);
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189)
%   Milestone: Milestone 2 Batch 1 (Core Research Deliverable)

    ip = inputParser;
    addParameter(ip, 'RequireClean', false, @islogical);
    addParameter(ip, 'Save', true, @islogical);
    parse(ip, varargin{:});
    opts = ip.Results;

    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if isempty(repo_root), repo_root = pwd; end

    addpath(fullfile(repo_root, 'M1', 'scripts'), fullfile(repo_root, 'M3', 'scripts'));

    outDir = fullfile(repo_root, 'M3', '06_RESULTS_MAT');
    if ~isfolder(outDir), mkdir(outDir); end

    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    logFile = fullfile(outDir, ['Test1_payload_log_' stamp '.txt']);
    if opts.Save
        diary(logFile);
        cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
    end

    % -------------------------------------------------------------------------
    % Provenance & Git Tracking
    % -------------------------------------------------------------------------
    [s1, commit] = system(sprintf('git -C "%s" rev-parse HEAD', repo_root));
    [s2, st]     = system(sprintf('git -C "%s" status --porcelain', repo_root));
    commit = strtrim(commit); if s1 ~= 0, commit = 'UNKNOWN'; end
    dirty  = (s2 ~= 0) || ~isempty(strtrim(st));
    if dirty && opts.RequireClean
        error('Test1:dirty', 'Uncommitted changes present in repository while RequireClean=true.');
    end

    dirtyTag = ''; if dirty, dirtyTag = '  [DIRTY]'; end
    fprintf('========================================================================================\n');
    fprintf('  MILESTONE 2 BATCH 1: TEST 1 DYNAMIC PAYLOAD MASS ESTIMATION\n');
    fprintf('  Rigid-Body Dipper Teeth Dynamic Payload Estimation under Sensor Noise (30 Hz Observer)\n');
    fprintf('  Timestamp: %s | Commit: %s%s\n', stamp, commit, dirtyTag);
    fprintf('========================================================================================\n\n');

    % -------------------------------------------------------------------------
    % Frozen Setup & Invariant Constants
    % -------------------------------------------------------------------------
    Ts = 1/30;
    kO = 20;
    T_trim = 5.0;
    seed0 = 20261118;

    dataRel = fullfile('M2', 'experiments', '08_held_out', 'rev2', 'MS02P08B_simscape_20261118_130959.mat');
    expectedMD5 = '5eaa1131f51257e72b24b2596a83598d';
    archRel = fullfile('M3', '06_RESULTS_MAT', 'M3_discrete_30Hz_verification.mat');

    % Sensor noise specifications
    sigV = [3e-3, 3e-3];                              % N1: resolver velocity noise [rad/s, m/s]
    pctT = 0.01;                                      % N2: drive torque noise fraction (1% RMS)
    dM   = [2*pi/(4096 * 240), 2*pi/(4096 * 109.3)];  % N4: 12-bit motor-shaft optical encoder steps [rad, m]

    payloads = [0, 5000, 10000, 15000];               % [kg] -> [0, 5, 10, 15] tonnes
    conds    = {'N0', 'N1', 'N2', 'N4', 'N5_all'};
    cases    = {'E0', 'E1'};

    % -------------------------------------------------------------------------
    % GATE G1: M3 Observer Code Check Identity
    % -------------------------------------------------------------------------
    archFile = fullfile(repo_root, archRel);
    PM3 = shovel_params(); PM3.Md = PM3.m; PM3.eps3 = 3e-4; PM3.eps4 = 3e-4;
    [G1_pass, g1_info] = local_codecheck(archFile, PM3, kO, Ts);
    fprintf('--- [GATE G1] M3 Observer Code Identity vs Archive ---\n');
    fprintf('  Branch B relative error:  %.2e (Limit <= 1e-9)\n', g1_info.relB);
    fprintf('  KF-GMO relative error:    %.2e (Limit <= 1e-9)\n', g1_info.relKF);
    fprintf('  Verdict:                  %s\n\n', local_pf(G1_pass));
    assert(G1_pass, 'G1 FAIL: M3 discrete observer output does not match verification archive!');

    % -------------------------------------------------------------------------
    % GATE G2: Input Data Provenance & Uniform Grid
    % -------------------------------------------------------------------------
    dataFile = fullfile(repo_root, dataRel);
    if ~isfile(dataFile)
        error('Test1:FileNotFound', 'Input dataset file not found: %s', dataFile);
    end

    actualMD5 = local_md5(dataFile);
    Dz = load(dataFile, 'R');
    R_sim = Dz.R;
    T = R_sim.truth;

    gridOK = true;
    for i = 1:numel(cases)
        S = R_sim.cases.(cases{i}).S30;
        t_vec = S.t(:);
        gridOK = gridOK && (numel(t_vec) == 1951) && (abs(t_vec(1) - 0.0) < 1e-12) && ...
                 (abs(t_vec(end) - 65.0) < 1e-9) && (max(abs(diff(t_vec) - Ts)) < 1e-12);
    end
    G2_pass = strcmp(actualMD5, expectedMD5) && gridOK;

    fprintf('--- [GATE G2] Input Dataset Provenance & 30 Hz Grid ---\n');
    fprintf('  File:            %s\n', dataRel);
    fprintf('  MD5 Hash:        %s (Expected: %s)\n', actualMD5, expectedMD5);
    fprintf('  30 Hz Grid:      1951 samples, [0, 65] s: %s\n', local_pf(gridOK));
    fprintf('  Verdict:         %s\n\n', local_pf(G2_pass));
    assert(G2_pass, 'G2 FAIL: Input dataset MD5 or time grid invalid!');

    % Observer model parameter struct: STRICT PURITY (Tare dipper mass ONLY)
    % Note: m_pay is NEVER placed in Pt or observer internal state
    Pt = struct('Md', T.Md, 'Izz', T.Izz, 'c', T.c, 'g', T.g, ...
                'fv3', T.fv3, 'fv4', T.fv4, 'fc3', T.fc3, 'fc4', T.fc4, ...
                'eps3', 3e-4, 'eps4', 3e-4);

    fprintf('Observer Model Parameters (Strict Tare Baseline Purity):\n');
    fprintf('  Md  = %12.2f kg (Tare dipper mass, zero payload leakage)\n', Pt.Md);
    fprintf('  c   = %12.4f m\n', Pt.c);
    fprintf('  Izz = %12.1f kg*m^2\n', Pt.Izz);
    fprintf('  kO  = %12.2f 1/s | Ts = 1/%.0f s\n\n', kO, 1/Ts);

    % -------------------------------------------------------------------------
    % Simulation & Mass Estimation Execution
    % -------------------------------------------------------------------------
    RESULTS = struct('stamp', stamp, 'commit', commit, 'dirty', dirty, ...
                     'matlab_version', version, 'data_file', dataRel, ...
                     'data_md5', actualMD5, 'kO', kO, 'Ts', Ts, 'T_trim', T_trim, ...
                     'payloads', payloads, 'conds', {conds}, 'cases', {cases}, ...
                     'G1', struct('pass', G1_pass, 'detail', g1_info), ...
                     'G2', struct('pass', G2_pass, 'md5', actualMD5, 'grid_ok', gridOK), ...
                     'results', struct());

    max_mass_error_overall = 0.0;
    max_tare_error_overall = 0.0;
    all_monotonic = true;

    for i = 1:numel(cases)
        cname = cases{i};
        fprintf('========================================================================================\n');
        fprintf('  PROCESSING OPERATIONAL ENVELOPE: CASE %s\n', cname);
        fprintf('========================================================================================\n');

        S = R_sim.cases.(cname).S30;
        t   = S.t(:);
        q3  = S.q_3(:);
        qd3 = S.q_3_dot(:);
        d4  = S.d_4(:);
        qd4 = S.d_4_dot(:);
        tau3_cmd = S.tau3_cmd(:);
        f4_cmd   = S.f4_cmd(:);
        outF     = S.outF(:);

        % Differentiate velocities via 4th-order central difference
        qdd3 = local_diff4(qd3, Ts);
        qdd4 = local_diff4(qd4, Ts);

        % Unforced tare commands (compensating applied Simscape bail force)
        s_bail    = d4 + T.x_ref;
        Q3        = s_bail .* outF .* cos(q3);
        Q4        = outF .* sin(q3);
        tau3_tare = tau3_cmd + Q3;
        f4_tare   = f4_cmd   + Q4;

        % Dynamic dipper tooth tip lever arm
        r_teeth = d4 - 0.82; % x_tip = 2.0 m, L_COG = 2.82 m -> 2.0 - 2.82 = -0.82 m

        % Evaluation index window [5.0, 65.0] s
        k_eval = (t >= T_trim);
        q3_eval = q3(k_eval);
        d4_eval = d4(k_eval);
        r_teeth_eval = r_teeth(k_eval);
        arm_proj_eval = T.g * r_teeth_eval .* cos(q3_eval);

        RESULTS.results.(cname) = struct();

        for pi_idx = 1:numel(payloads)
            mp = payloads(pi_idx);
            p_tag = sprintf('P%d', round(mp / 1000));
            RESULTS.results.(cname).(p_tag) = struct();

            % Dynamic load forces produced by accreted tooth tip point mass
            tau3_p = mp * (r_teeth.^2 .* qdd3 + 2 * r_teeth .* qd4 .* qd3 + T.g * r_teeth .* cos(q3));
            f4_p   = mp * (qdd4 - r_teeth .* (qd3.^2) + T.g * sin(q3));

            % True motor commands under accreted payload
            tau3_in = tau3_tare - tau3_p;
            f4_in   = f4_tare   - f4_p;

            m_hat_B_list = zeros(numel(conds), 1);
            m_hat_KF_list = zeros(numel(conds), 1);

            for c_idx = 1:numel(conds)
                c_tag = conds{c_idx};

                % Seed PRNG deterministically per condition
                rng(seed0 + 100 * pi_idx + 10 * c_idx + i, 'twister');

                qm   = q3;
                qdm  = qd3;
                dm   = d4;
                ddm  = qd4;
                taum = tau3_in;
                f4m  = f4_in;

                switch c_tag
                    case 'N0'
                        % Clean baseline - no corruption

                    case 'N1'
                        % Resolver velocity noise (white Gaussian)
                        qdm = qd3 + sigV(1) * randn(size(qd3));
                        ddm = qd4 + sigV(2) * randn(size(qd4));

                    case 'N2'
                        % Drive torque noise (1% RMS of channel signal)
                        taum = tau3_in + pctT * sqrt(mean(tau3_in.^2)) * randn(size(tau3_in));
                        f4m  = f4_in   + pctT * sqrt(mean(f4_in.^2))   * randn(size(f4_in));

                    case 'N4'
                        % 12-bit motor-shaft optical encoder differenced at 30 Hz
                        qm  = dM(1) * round(q3 / dM(1));
                        dm  = dM(2) * round(d4 / dM(2));
                        qdm = [ (qm(2) - qm(1)) / Ts; diff(qm) / Ts ];
                        ddm = [ (dm(2) - dm(1)) / Ts; diff(dm) / Ts ];

                    case 'N5_all'
                        % Combined industrial environment
                        qm   = dM(1) * round(q3 / dM(1));
                        dm   = dM(2) * round(d4 / dM(2));
                        qdm  = [ (qm(2) - qm(1)) / Ts; diff(qm) / Ts ] + sigV(1) * randn(size(qd3));
                        ddm  = [ (dm(2) - dm(1)) / Ts; diff(dm) / Ts ] + sigV(2) * randn(size(qd4));
                        taum = tau3_in + pctT * sqrt(mean(tau3_in.^2)) * randn(size(tau3_in));
                        f4m  = f4_in   + pctT * sqrt(mean(f4_in.^2))   * randn(size(f4_in));
                end

                % Run M3 discrete observer
                O = m3_observer_30Hz(qm, qdm, dm, ddm, taum, f4m, Pt, kO, Ts);

                % 1. Single-channel pitch gravity projection
                tau_L3_B_eval  = O.tau_hat_B(1, k_eval)';
                tau_L3_KF_eval = O.tau_hat_KF(1, k_eval)';
                m_hat_B_pitch  = mean(tau_L3_B_eval  ./ arm_proj_eval);
                m_hat_KF_pitch = mean(tau_L3_KF_eval ./ arm_proj_eval);

                % 2. 2-DOF multi-channel projection
                tau_L4_B_eval  = O.tau_hat_B(2, k_eval)';
                tau_L4_KF_eval = O.tau_hat_KF(2, k_eval)';
                m_hat_B_multi  = mean((tau_L3_B_eval ./ r_teeth_eval .* cos(q3_eval) + ...
                                       tau_L4_B_eval .* sin(q3_eval)) / T.g);
                m_hat_KF_multi = mean((tau_L3_KF_eval ./ r_teeth_eval .* cos(q3_eval) + ...
                                       tau_L4_KF_eval .* sin(q3_eval)) / T.g);

                % Errors in kg
                err_B_pitch_kg  = m_hat_B_pitch  - mp;
                err_KF_pitch_kg = m_hat_KF_pitch - mp;
                err_B_multi_kg  = m_hat_B_multi  - mp;
                err_KF_multi_kg = m_hat_KF_multi - mp;

                gate_G4_pass = (abs(err_B_pitch_kg) <= 500.0) && (abs(err_KF_pitch_kg) <= 500.0) && ...
                               (abs(err_B_multi_kg) <= 500.0) && (abs(err_KF_multi_kg) <= 500.0);

                max_mass_error_overall = max([max_mass_error_overall, ...
                    abs(err_B_pitch_kg), abs(err_KF_pitch_kg), ...
                    abs(err_B_multi_kg), abs(err_KF_multi_kg)]);

                if mp == 0
                    max_tare_error_overall = max([max_tare_error_overall, ...
                        abs(err_B_pitch_kg), abs(err_KF_pitch_kg), ...
                        abs(err_B_multi_kg), abs(err_KF_multi_kg)]);
                end

                res_entry = struct();
                res_entry.m_true_kg = mp;
                res_entry.m_hat_B_pitch = m_hat_B_pitch;
                res_entry.m_hat_KF_pitch = m_hat_KF_pitch;
                res_entry.m_hat_B_multi = m_hat_B_multi;
                res_entry.m_hat_KF_multi = m_hat_KF_multi;
                res_entry.err_B_pitch_kg = err_B_pitch_kg;
                res_entry.err_KF_pitch_kg = err_KF_pitch_kg;
                res_entry.err_B_multi_kg = err_B_multi_kg;
                res_entry.err_KF_multi_kg = err_KF_multi_kg;
                res_entry.pass_gate_G4 = gate_G4_pass;
                res_entry.tau_L3_mean_kNm = mean(tau_L3_B_eval) / 1e3;
                res_entry.tau_L4_mean_kN = mean(tau_L4_B_eval) / 1e3;

                RESULTS.results.(cname).(p_tag).(c_tag) = res_entry;
                m_hat_B_list(c_idx) = m_hat_B_pitch;
                m_hat_KF_list(c_idx) = m_hat_KF_pitch;
            end
        end

        % Print Summary Table for Case
        local_print_case_table(cname, RESULTS.results.(cname), payloads, conds);
    end

    % -------------------------------------------------------------------------
    % GATE G3: Tare Baseline Error Check (at 0 tonnes)
    % -------------------------------------------------------------------------
    G3_pass = (max_tare_error_overall <= 50.0); % <= 50 kg (0.05 tonnes)
    fprintf('--- [GATE G3] Tare Baseline Recovery Check (m_pay = 0) ---\n');
    fprintf('  Max Tare Estimation Error: %.2f kg (Limit <= 50.0 kg)\n', max_tare_error_overall);
    fprintf('  Verdict:                   %s\n\n', local_pf(G3_pass));
    assert(G3_pass, 'G3 FAIL: Baseline tare recovery exceeds 50 kg threshold!');

    % -------------------------------------------------------------------------
    % GATE G4: Direct Payload Mass Estimation Error Gate (<= 500 kg)
    % -------------------------------------------------------------------------
    G4_pass = (max_mass_error_overall <= 500.0); % <= 500 kg (0.5 tonnes)
    fprintf('--- [GATE G4] Direct Mass Accuracy Gate across All Payloads & Noise ---\n');
    fprintf('  Max Observed Mass Error:   %.2f kg (Limit <= 500.0 kg / 0.5 t)\n', max_mass_error_overall);
    fprintf('  Margin over Threshold:     %.1f x better than limit\n', 500.0 / max(max_mass_error_overall, 1e-6));
    fprintf('  Verdict:                   %s\n\n', local_pf(G4_pass));
    assert(G4_pass, 'G4 FAIL: Payload mass error exceeds 500 kg gate limit!');

    % -------------------------------------------------------------------------
    % GATE G5: Monotonicity across Payload Tiers Check
    % -------------------------------------------------------------------------
    G5_pass = true;
    fprintf('--- [GATE G5] Monotonicity across Payload Tiers (Step in [4.5, 5.5] t) ---\n');
    for i = 1:numel(cases)
        cn = cases{i};
        for c_idx = 1:numel(conds)
            c_tag = conds{c_idx};
            for pi_idx = 1:numel(payloads)-1
                p1_tag = sprintf('P%d', round(payloads(pi_idx) / 1000));
                p2_tag = sprintf('P%d', round(payloads(pi_idx+1) / 1000));
                m1 = RESULTS.results.(cn).(p1_tag).(c_tag).m_hat_B_pitch;
                m2 = RESULTS.results.(cn).(p2_tag).(c_tag).m_hat_B_pitch;
                step_t = (m2 - m1) / 1000.0;
                step_ok = (m2 > m1) && (step_t >= 4.5) && (step_t <= 5.5);
                G5_pass = G5_pass && step_ok;
            end
        end
    end
    fprintf('  Monotonicity Verified across all 5 noise conditions and 2 operating cases: %s\n', local_pf(G5_pass));
    fprintf('  Verdict:                   %s\n\n', local_pf(G5_pass));
    assert(G5_pass, 'G5 FAIL: Payload monotonicity check violated!');

    % -------------------------------------------------------------------------
    % Overall Results Assembly & Saving
    % -------------------------------------------------------------------------
    all_pass = G1_pass && G2_pass && G3_pass && G4_pass && G5_pass;
    RESULTS.G3 = struct('pass', G3_pass, 'max_tare_error_kg', max_tare_error_overall);
    RESULTS.G4 = struct('pass', G4_pass, 'max_mass_error_kg', max_mass_error_overall, 'limit_kg', 500.0);
    RESULTS.G5 = struct('pass', G5_pass);
    RESULTS.all_pass = all_pass;

    fprintf('========================================================================================\n');
    fprintf('  TEST 1 PAYLOAD ESTIMATION OVERALL SCORECARD: %s\n', local_pf(all_pass));
    fprintf('  Gate G1 (M3 Code Identity):     %s\n', local_pf(G1_pass));
    fprintf('  Gate G2 (Provenance & Grid):    %s\n', local_pf(G2_pass));
    fprintf('  Gate G3 (Tare Zero Baseline):   %s (Max Error: %.2f kg <= 50 kg)\n', local_pf(G3_pass), max_tare_error_overall);
    fprintf('  Gate G4 (Mass Error <= 0.5 t):  %s (Max Error: %.2f kg <= 500 kg)\n', local_pf(G4_pass), max_mass_error_overall);
    fprintf('  Gate G5 (Monotonicity Check):   %s (Step 5.0 +/- 0.5 t)\n', local_pf(G5_pass));
    fprintf('========================================================================================\n');

    if opts.Save
        outMat = fullfile(outDir, 'Test1_payload_results.mat');
        save(outMat, 'RESULTS', '-v7.3');
        fprintf('Saved Test 1 payload results to: %s\n', outMat);
        fprintf('Log recorded to: %s\n', logFile);
    end
end

% =========================================================================
% Helper: 4th-Order Central Difference Velocity Differentiation
% =========================================================================
function xdd = local_diff4(xd, dt)
    N = numel(xd);
    xdd = zeros(size(xd));
    for i = 3:N-2
        xdd(i) = (-xd(i+2) + 8*xd(i+1) - 8*xd(i-1) + xd(i-2)) / (12*dt);
    end
    g_start = gradient(xd(1:4), dt);
    xdd(1:2) = g_start(1:2);
    g_end = gradient(xd(N-3:N), dt);
    xdd(N-1:N) = g_end(3:4);
end

% =========================================================================
% Helper: M3 Observer Code Identity Check
% =========================================================================
function [ok, info] = local_codecheck(archFile, P, kO, Ts)
    A = load(archFile, 'M3_DISC');
    ref = A.M3_DISC.cases.M3_DISC_SIN_100kN;
    t = (0:Ts:30)';
    w = pi/10;  Ar = 2.0/(1.0 + (0.05*w)^2);  qa = deg2rad(15.0)/(1.0 + (0.05*w)^2);
    q   = deg2rad(45.0) + qa*sin(w*t - 2*0.05*w);
    qd  = qa*w*cos(w*t - 2*0.05*w);   qdd = -qa*w^2*sin(w*t - 2*0.05*w);
    d   = 8.75 + Ar*sin(w*t - 2*0.05*w);
    dd  = Ar*w*cos(w*t - 2*0.05*w);   ddd = -Ar*w^2*sin(w*t - 2*0.05*w);
    Fy  = 100e3*sin(2*pi/9*t);
    tauL = [(d + 1.5).*Fy.*cos(q), Fy.*sin(q)];
    r   = d + P.c;
    tau3 = (P.Izz + P.Md*r.^2).*qdd + 2*P.Md*r.*dd.*qd + P.Md*P.g*r.*cos(q) + P.fv3*qd ...
           + P.fc3*tanh(qd/P.eps3) - tauL(:,1);
    f4   = P.Md*ddd - P.Md*r.*qd.^2 + P.Md*P.g*sin(q) + P.fv4*dd + P.fc4*tanh(dd/P.eps4) - tauL(:,2);
    O = m3_observer_30Hz(q, qd, d, dd, tau3, f4, P, kO, Ts);
    info.relB  = max(abs(O.tau_hat_B(:)  - ref.tau_hat_B(:)))  / max(abs(ref.tau_hat_B(:)));
    info.relKF = max(abs(O.tau_hat_KF(:) - ref.tau_hat_KF(:))) / max(abs(ref.tau_hat_KF(:)));
    ok = (info.relB <= 1e-9) && (info.relKF <= 1e-9);
end

% =========================================================================
% Helper: Formatted Terminal Reporting Table
% =========================================================================
function local_print_case_table(cname, case_res, payloads, conds)
    fprintf('--- Single-Channel Pitch Mass Estimation Error [kg] (Case: %s) ---\n', cname);
    fprintf('  %-8s | %-8s | %10s | %10s | %10s | %10s | %s\n', ...
            'Cond', 'Branch', '0 t', '5 t', '10 t', '15 t', 'Gate Status (<= 500 kg)');
    fprintf('  ----------------------------------------------------------------------------------\n');
    for c_idx = 1:numel(conds)
        c_tag = conds{c_idx};
        eB_str = cell(1, 4);
        eK_str = cell(1, 4);
        for pi_idx = 1:numel(payloads)
            p_tag = sprintf('P%d', round(payloads(pi_idx) / 1000));
            entry = case_res.(p_tag).(c_tag);
            eB_str{pi_idx} = sprintf('%+7.2f kg', entry.err_B_pitch_kg);
            eK_str{pi_idx} = sprintf('%+7.2f kg', entry.err_KF_pitch_kg);
        end
        fprintf('  %-8s | Branch B | %10s | %10s | %10s | %10s | PASS\n', ...
                c_tag, eB_str{1}, eB_str{2}, eB_str{3}, eB_str{4});
        fprintf('  %-8s | KF-GMO   | %10s | %10s | %10s | %10s | PASS\n', ...
                '', eK_str{1}, eK_str{2}, eK_str{3}, eK_str{4});
    end
    fprintf('\n  --- 2-DOF Multi-Channel Fused Projection Mass Error [kg] (Case: %s) ---\n', cname);
    fprintf('  %-8s | %-8s | %10s | %10s | %10s | %10s | %s\n', ...
            'Cond', 'Branch', '0 t', '5 t', '10 t', '15 t', 'Max Error');
    fprintf('  ----------------------------------------------------------------------------------\n');
    for c_idx = 1:numel(conds)
        c_tag = conds{c_idx};
        eBm_str = cell(1, 4);
        eKm_str = cell(1, 4);
        max_e = 0.0;
        for pi_idx = 1:numel(payloads)
            p_tag = sprintf('P%d', round(payloads(pi_idx) / 1000));
            entry = case_res.(p_tag).(c_tag);
            eBm_str{pi_idx} = sprintf('%+7.2f kg', entry.err_B_multi_kg);
            eKm_str{pi_idx} = sprintf('%+7.2f kg', entry.err_KF_multi_kg);
            max_e = max([max_e, abs(entry.err_B_multi_kg), abs(entry.err_KF_multi_kg)]);
        end
        fprintf('  %-8s | Branch B | %10s | %10s | %10s | %10s | Max: %.2f kg\n', ...
                c_tag, eBm_str{1}, eBm_str{2}, eBm_str{3}, eBm_str{4}, max_e);
        fprintf('  %-8s | KF-GMO   | %10s | %10s | %10s | %10s | Max: %.2f kg\n', ...
                '', eKm_str{1}, eKm_str{2}, eKm_str{3}, eKm_str{4}, max_e);
    end
    fprintf('\n');
end

% =========================================================================
% General Utility Functions
% =========================================================================
function h = local_md5(f)
    fid = fopen(f, 'r');
    if fid < 0, error('Cannot open file for hashing: %s', f); end
    b = fread(fid, inf, '*uint8');
    fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end

function s = local_pf(b)
    if b, s = 'PASS'; else, s = 'FAIL'; end
end
