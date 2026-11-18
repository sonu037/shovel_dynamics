function R = run_MS03_IND1_on_P08B(varargin)
%RUN_MS03_IND1_ON_P08B  MS03-IND1: the M3 30 Hz observer on independent Simscape data.
%
%   Spec: M3/experiments/ind1_p08b/MS03_IND1_SPEC.md (freeze by commit before the run).
%
%   QUESTION: does the M3 30 Hz momentum observer recover the bail load from data
%   it did not generate - the torque-driven Simscape plant of MS02P08B rev 2
%   (run 20261118_130959, 30 Hz export S30) - with (i) the true parameters and
%   (ii) the P08B wrong parameters (+5 % Md, +10 % Izz, c +0.05 m)?
%
%   WHAT RUNS
%     G1  code identity: m3_observer_30Hz.m re-run on the archived M3 case
%         M3_DISC_SIN_100kN must reproduce M3/06_RESULTS_MAT/M3_discrete_30Hz_verification.mat
%         (Branch B and KF-GMO) to <= 1e-9 relative. Proves it is the M3 observer.
%     G2  data provenance: the P08B results file has the MD5 written in the spec;
%         S30 is exactly 30 Hz, 1951 samples, 0-65 s, for E0 and E1.
%     Then, for E0 and E1, the observer runs on S30 (q3, q3dot, d4, d4dot, drive
%     torque tau3_cmd, drive force f4_cmd - no acceleration) with the true and
%     with the mismatched parameters. Fy is recovered from the saddle channel,
%     Fy_hat = tau_hat_L3 / ((d4 + 1.5) cos q3), as in MS02P08B, and scored after
%     t = 5 s against the logged applied force (outF) and against the same force
%     passed through the observer's own first-order filter (lag-matched).
%     Predictions P1-P3 (spec) are printed as HOLDS / REFUTED.
%
%   USAGE (MATLAB R2025b, current folder = repository root)
%     R = run_MS03_IND1_on_P08B();                          % needs a clean git tree
%     R = run_MS03_IND1_on_P08B('RequireClean', false);     % recorded as DIRTY
%
%   OUTPUT  M3/experiments/ind1_p08b/MS03_IND1_<stamp>.mat, _log_<stamp>.txt, _<stamp>.png
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18
%   (written without MATLAB; the first run must be checked by the researcher).

    ip = inputParser;
    addParameter(ip, 'RequireClean', true, @islogical);
    addParameter(ip, 'Save', true, @islogical);
    parse(ip, varargin{:});
    o = ip.Results;

    root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    outDir = fullfile(root, 'M3', 'experiments', 'ind1_p08b');
    addpath(fullfile(root, 'M1', 'scripts'), fullfile(root, 'M3', 'scripts'));
    stamp  = datestr(now, 'yyyymmdd_HHMMSS');

    % ---------------- frozen constants (spec section "Setup")
    Ts = 1/30;  kO = 20;  T_trim = 5;
    dataRel = fullfile('M2', 'experiments', '08_held_out', 'rev2', 'MS02P08B_simscape_20261118_130959.mat');
    dataMD5 = '5eaa1131f51257e72b24b2596a83598d';
    archRel = fullfile('M3', '06_RESULTS_MAT', 'M3_discrete_30Hz_verification.mat');

    % ---------------- provenance, checked before any file is created
    [s1, commit] = system(sprintf('git -C "%s" rev-parse HEAD', root));
    [s2, st]     = system(sprintf('git -C "%s" status --porcelain', root));
    commit = strtrim(commit); if s1 ~= 0, commit = 'UNKNOWN'; end
    dirty  = (s2 ~= 0) || ~isempty(strtrim(st));
    if dirty && o.RequireClean
        error('IND1:dirty', ['Uncommitted changes in the repository. Commit the spec and scripts ' ...
              'first (the commit is the freeze), or pass ''RequireClean'', false ' ...
              '(the run is then recorded as DIRTY).\n%s'], strtrim(st));
    end
    if o.Save
        if ~isfolder(outDir), mkdir(outDir); end
        logFile = fullfile(outDir, ['MS03_IND1_log_' stamp '.txt']);
        diary(logFile); cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
    end
    dirtyTag = ''; if dirty, dirtyTag = '  [DIRTY]'; end
    fprintf('MS03-IND1  %s  commit %s%s\n', stamp, commit, dirtyTag);
    fprintf('   observer: M3 30 Hz momentum observer, k_O = %g 1/s, Ts = 1/%g s; trim %g s\n', kO, 1/Ts, T_trim);

    R = struct('stamp', stamp, 'commit', commit, 'dirty', dirty, 'matlab', version, ...
               'kO', kO, 'Ts', Ts, 'T_trim', T_trim, 'data_file', dataRel);

    % ---------------- G1 code identity with the M3 archive
    PM3 = shovel_params(); PM3.Md = PM3.m; PM3.eps3 = 3e-4; PM3.eps4 = 3e-4;   % M3 Machine A defaults
    [G1, g1] = local_codecheck(fullfile(root, archRel), PM3, kO, Ts);
    fprintf('G1 code identity vs M3 archive (M3_DISC_SIN_100kN): Branch B %.2e, KF-GMO %.2e (limit 1e-9) -> %s\n', ...
            g1.relB, g1.relKF, local_pf(G1));
    R.G1 = struct('pass', G1, 'detail', g1);

    % ---------------- G2 data provenance
    dataFile = fullfile(root, dataRel);
    md5 = local_md5(dataFile);
    Dz = load(dataFile, 'R');  D = Dz.R;
    T  = D.truth;  Pm = D.mismatch;
    gridOK = true;
    cases = {'E0', 'E1'};
    for i = 1:2
        S = D.cases.(cases{i}).S30;
        t = S.t(:);
        gridOK = gridOK && numel(t) == 1951 && abs(t(1)) < 1e-12 && abs(t(end) - 65) < 1e-9 && ...
                 max(abs(diff(t) - Ts)) < 1e-9;
    end
    G2 = strcmp(md5, dataMD5) && gridOK;
    fprintf('G2 data: %s MD5 %s (expected %s); 30 Hz grid 0-65 s, 1951 samples: %s -> %s\n', ...
            dataRel, md5, dataMD5, local_tf(gridOK, 'yes', 'NO'), local_pf(G2));
    R.G2 = struct('pass', G2, 'md5', md5, 'grid_ok', gridOK);

    % ---------------- observer parameter sets (from the P08B file, not retyped)
    Pt = struct('Md', T.Md,  'Izz', T.Izz,  'c', T.c,  'g', T.g,  'fv3', T.fv3,  'fv4', T.fv4, ...
                'fc3', T.fc3,  'fc4', T.fc4,  'eps3', 3e-4, 'eps4', 3e-4);
    Pw = struct('Md', Pm.Md, 'Izz', Pm.Izz, 'c', Pm.c, 'g', Pm.g, 'fv3', Pm.fv3, 'fv4', Pm.fv4, ...
                'fc3', Pm.fc3, 'fc4', Pm.fc4, 'eps3', 3e-4, 'eps4', 3e-4);
    fprintf('   true      : Md %.1f kg, Izz %.1f kg m^2, c %.4f m\n', Pt.Md, Pt.Izz, Pt.c);
    fprintf('   mismatch  : Md %.1f kg, Izz %.1f kg m^2, c %.4f m (as MS02P08B)\n', Pw.Md, Pw.Izz, Pw.c);

    % ---------------- run
    lagRms = 100 * (kO^-1 * (2*pi/9)) / sqrt(1 + (kO^-1 * (2*pi/9))^2) / sqrt(2);   % kN, spec P1
    RES = struct();
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 700]);
    for i = 1:2
        cn = cases{i};
        S  = D.cases.(cn).S30;
        t  = S.t(:);  q = S.q_3(:);  qd = S.q_3_dot(:);  d = S.d_4(:);  dd = S.d_4_dot(:);
        tau = S.tau3_cmd(:);  f4 = S.f4_cmd(:);  Fy = S.outF(:);
        s   = d + T.x_ref;  arm = s .* cos(q);
        Q3  = arm .* Fy;  Q4 = Fy .* sin(q);
        Q3f = local_filter(Q3, kO, Ts);           % true load through the observer's own first-order filter
        Fyf = Q3f ./ arm;
        k   = t >= T_trim - 1e-9;
        p08 = D.cases.(cn).mismatch_bias_kN;      % inverse-dynamics bias from MS02P08B on the same data

        sets = {'control', Pt; 'mismatch', Pw};
        for j = 1:2
            O = m3_observer_30Hz(q, qd, d, dd, tau, f4, sets{j,2}, kO, Ts);
            FyB = O.tau_hat_B(1,:)' ./ arm;   FyK = O.tau_hat_KF(1,:)' ./ arm;
            m = struct();
            m.B_bias_kN    = mean(FyB(k) - Fy(k)) / 1e3;
            m.B_rms_kN     = sqrt(mean((FyB(k) - Fy(k)).^2)) / 1e3;
            m.B_rmsLag_kN  = sqrt(mean((FyB(k) - Fyf(k)).^2)) / 1e3;
            m.KF_bias_kN   = mean(FyK(k) - Fy(k)) / 1e3;
            m.KF_rms_kN    = sqrt(mean((FyK(k) - Fy(k)).^2)) / 1e3;
            m.B_crowd_rms_kN  = sqrt(mean((O.tau_hat_B(2,k)' - Q4(k)).^2)) / 1e3;
            m.KF_crowd_rms_kN = sqrt(mean((O.tau_hat_KF(2,k)' - Q4(k)).^2)) / 1e3;
            m.FyB = FyB;  m.FyK = FyK;
            RES.(cn).(sets{j,1}) = m;

            subplot(2, 2, (i-1)*2 + j);
            plot(t, Fy/1e3, 'k', 'LineWidth', 1.4); hold on;
            plot(t, FyB/1e3, 'b', 'LineWidth', 1.0); plot(t, FyK/1e3, 'r:', 'LineWidth', 1.0);
            xline(T_trim, ':'); grid on; xlim([0 65]);
            title(sprintf('%s, %s parameters', cn, sets{j,1})); xlabel('t [s]'); ylabel('F_y [kN]');
            if i == 1 && j == 1, legend('true load (outF)', 'M3 Branch B', 'M3 KF-GMO', 'Location', 'southwest'); end
        end
        RES.(cn).t = t;  RES.(cn).Fy = Fy;  RES.(cn).Fy_filt = Fyf;  RES.(cn).p08b_bias_kN = p08;
    end
    sgtitle(sprintf('MS03-IND1 %s: M3 observer on MS02P08B Simscape data (30 Hz)', stamp));

    % ---------------- report and predictions
    fprintf('\n%-4s %-9s %10s %10s %12s %10s %10s %12s %12s\n', 'case', 'params', 'B bias', 'B rms', 'B rms(lag)', ...
            'KF bias', 'KF rms', 'B crowd rms', 'KF crowd rms');
    for i = 1:2
        for j = {'control', 'mismatch'}
            m = RES.(cases{i}).(j{1});
            fprintf('%-4s %-9s %8.3f kN %8.3f kN %10.3f kN %8.3f kN %8.3f kN %10.3f kN %10.3f kN\n', cases{i}, j{1}, ...
                m.B_bias_kN, m.B_rms_kN, m.B_rmsLag_kN, m.KF_bias_kN, m.KF_rms_kN, m.B_crowd_rms_kN, m.KF_crowd_rms_kN);
        end
    end

    PR = struct();
    fprintf('\nPredictions (spec; written before the run):\n');
    ok1 = true;
    for i = 1:2
        m = RES.(cases{i}).control;
        fprintf('       %s control: bias %.3f kN, rms %.3f kN, rms vs lag-filtered truth %.3f kN\n', cases{i}, m.B_bias_kN, m.B_rms_kN, m.B_rmsLag_kN);
        ok1 = ok1 && abs(m.B_bias_kN) < 0.3 && m.B_rms_kN >= 2.0 && m.B_rms_kN <= 3.0 && m.B_rmsLag_kN < 0.2;
    end
    PR.P1 = ok1;
    fprintf(['   P1  control, Branch B: |bias| < 0.3 kN, rms 2.0-3.0 kN (pure lag %.2f kN), ' ...
             'rms vs lag-filtered truth < 0.2 kN -> %s\n'], lagRms, local_hr(ok1));
    ok2 = true;
    for i = 1:2
        m = RES.(cases{i}).mismatch;  b = RES.(cases{i}).p08b_bias_kN;
        rmsPred = sqrt(b^2 + lagRms^2);
        ok2 = ok2 && abs(m.B_bias_kN - b) <= 0.10*abs(b) && abs(m.B_rms_kN - rmsPred) <= 0.10*rmsPred;
        fprintf('       %s mismatch: bias %.3f kN vs MS02P08B %.3f kN; rms %.3f kN vs predicted %.3f kN\n', ...
                cases{i}, m.B_bias_kN, b, m.B_rms_kN, rmsPred);
    end
    PR.P2 = ok2;
    fprintf('   P2  mismatch, Branch B: bias and rms within 10 %% of the MS02P08B bias / sqrt(bias^2 + lag^2) -> %s\n', local_hr(ok2));
    ok3 = true;
    for i = 1:2
        mc = RES.(cases{i}).control;  mm = RES.(cases{i}).mismatch;
        ok3 = ok3 && abs(mc.KF_bias_kN) < 0.5 && abs(mm.KF_bias_kN - mm.B_bias_kN) <= 0.10*abs(mm.B_bias_kN);
    end
    PR.P3 = ok3;
    fprintf('   P3  KF-GMO: control |bias| < 0.5 kN; mismatch bias within 10 %% of Branch B -> %s\n', local_hr(ok3));

    R.results = RES;  R.predictions = PR;
    R.valid = G1 && G2;
    fprintf('\nVALIDITY (G1 and G2): %s. Predictions may be refuted; a refutation is a result.\n', local_pf(R.valid));

    if o.Save
        exportgraphics(fig, fullfile(outDir, ['MS03_IND1_' stamp '.png']), 'Resolution', 150);
        f = fullfile(outDir, ['MS03_IND1_' stamp '.mat']);
        save(f, 'R', '-v7.3');
        fprintf('Saved %s\nLog   %s\n', f, logFile);
    end
    close(fig);
end

% =========================================================================
function [ok, info] = local_codecheck(archFile, P, kO, Ts)
% Re-create the archived M3 case M3_DISC_SIN_100kN exactly as
% build_M3_DiscreteObserver_30Hz.m does, run m3_observer_30Hz, compare.
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
    info.phi = O.phi;  info.beta0 = O.beta0;  info.beta1 = O.beta1;
    ok = info.relB <= 1e-9 && info.relKF <= 1e-9;
end

function y = local_filter(x, kO, Ts)
% Exact discrete first-order filter (gain kO, FOH input), as M3's tau_L_filt.
    phi = exp(-kO*Ts);  b1 = (1 - (1 - phi)/(kO*Ts))/kO;  b0 = (1 - phi)/kO - b1;
    y = zeros(size(x));  y(1) = x(1);
    for k = 2:numel(x)
        y(k) = phi*y(k-1) + (b0*kO)*x(k-1) + (b1*kO)*x(k);
    end
end

function h = local_md5(f)
    fid = fopen(f, 'r'); b = fread(fid, inf, '*uint8'); fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end

function s = local_pf(b), s = local_tf(b, 'PASS', 'FAIL');     end
function s = local_hr(b), s = local_tf(b, 'HOLDS', 'REFUTED'); end
function s = local_tf(b, t, f)
    if b, s = t; else, s = f; end
end
