function R = run_MS04_IND1_noise_on_P08B(varargin)
%RUN_MS04_IND1_NOISE_ON_P08B  MS04-IND1: sensor noise and encoder quantisation on the M3 observer,
%   using the torque-driven Simscape data of MS02P08B rev 2 (30 Hz export S30).
%
%   Spec: M4/experiments/ind1_p08b_noise/MS04_IND1_SPEC.md (freeze by commit before the run).
%
%   QUESTION: how much does the load estimate of the M3 observer degrade when the
%   clean Simscape signals are replaced by what real sensors deliver - noisy speed,
%   noisy drive torque, and positions from a finite-resolution encoder with speed
%   obtained by differencing at 30 Hz?
%
%   CONDITIONS (true parameters only; each for E0 and E1; fixed random seeds)
%     N0  clean (must reproduce MS03-IND1 exactly)
%     N1  white speed noise 3 mrad/s on q3dot and 3 mm/s on d4dot (M3 "noisy resolver" level)
%     N2  white drive-torque noise, 1 % of the rms of tau3_cmd and of f4_cmd
%     N3  12-bit encoder on the joint (M4 Case 1A convention): step 2*pi/4096 rad and
%         2*pi/4096 m; speeds by backward difference at 30 Hz
%     N4  12-bit encoder on the motor (M4 Case 1B design values, unsourced): steps
%         2*pi/(4096*240) rad and 2*pi/(4096*109.3) m; speeds by backward difference
%     N5  N4 + N2 together
%   Score: Fy from the saddle channel as in MS03-IND1; noise-induced error =
%   Fy_hat(condition) - Fy_hat(N0), rms after t = 5 s; Branch B and KF-GMO.
%
%   USAGE (MATLAB R2025b, current folder = repository root)
%     addpath(fullfile(pwd,'M4','scripts'))
%     R = run_MS04_IND1_noise_on_P08B();
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18
%   (written without MATLAB; the first run must be checked by the researcher).

    ip = inputParser;
    addParameter(ip, 'RequireClean', true, @islogical);
    addParameter(ip, 'Save', true, @islogical);
    parse(ip, varargin{:});
    o = ip.Results;

    root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    outDir = fullfile(root, 'M4', 'experiments', 'ind1_p08b_noise');
    addpath(fullfile(root, 'M1', 'scripts'), fullfile(root, 'M3', 'scripts'));
    stamp  = datestr(now, 'yyyymmdd_HHMMSS');

    % ---------------- frozen constants (spec)
    Ts = 1/30;  kO = 20;  T_trim = 5;
    dataRel = fullfile('M2', 'experiments', '08_held_out', 'rev2', 'MS02P08B_simscape_20261118_130959.mat');
    dataMD5 = '5eaa1131f51257e72b24b2596a83598d';
    archRel = fullfile('M3', '06_RESULTS_MAT', 'M3_discrete_30Hz_verification.mat');
    ind1Rel = fullfile('M3', 'experiments', 'ind1_p08b', 'MS03_IND1_20261118_160926.mat');
    sigV  = [3e-3 3e-3];                          % N1: rad/s, m/s
    pctT  = 0.01;                                 % N2: fraction of channel rms
    dJ    = [2*pi/4096, 2*pi/4096];               % N3: rad, m  (M4 Case 1A convention)
    dM    = [2*pi/(4096*240), 2*pi/(4096*109.3)]; % N4: rad, m  (M4 Case 1B design values)
    seed0 = 20261118;

    % ---------------- provenance, before any file is created
    [s1, commit] = system(sprintf('git -C "%s" rev-parse HEAD', root));
    [s2, st]     = system(sprintf('git -C "%s" status --porcelain', root));
    commit = strtrim(commit); if s1 ~= 0, commit = 'UNKNOWN'; end
    dirty  = (s2 ~= 0) || ~isempty(strtrim(st));
    if dirty && o.RequireClean
        error('M4IND1:dirty', ['Uncommitted changes in the repository. Commit the spec and script ' ...
              'first (the commit is the freeze), or pass ''RequireClean'', false ' ...
              '(the run is then recorded as DIRTY).\n%s'], strtrim(st));
    end
    if o.Save
        if ~isfolder(outDir), mkdir(outDir); end
        logFile = fullfile(outDir, ['MS04_IND1_log_' stamp '.txt']);
        diary(logFile); cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
    end
    dirtyTag = ''; if dirty, dirtyTag = '  [DIRTY]'; end
    fprintf('MS04-IND1  %s  commit %s%s\n', stamp, commit, dirtyTag);
    R = struct('stamp', stamp, 'commit', commit, 'dirty', dirty, 'matlab', version);

    % ---------------- G1 observer identity with the M3 archive
    PM3 = shovel_params(); PM3.Md = PM3.m; PM3.eps3 = 3e-4; PM3.eps4 = 3e-4;
    [G1, g1] = local_codecheck(fullfile(root, archRel), PM3, kO, Ts);
    fprintf('G1 observer identity vs M3 archive: Branch B %.2e, KF-GMO %.2e (limit 1e-9) -> %s\n', g1.relB, g1.relKF, local_pf(G1));

    % ---------------- G2 data provenance
    dataFile = fullfile(root, dataRel);
    md5 = local_md5(dataFile);
    Dz = load(dataFile, 'R');  D = Dz.R;  T = D.truth;
    cases = {'E0', 'E1'};
    gridOK = true;
    for i = 1:2
        t = D.cases.(cases{i}).S30.t(:);
        gridOK = gridOK && numel(t) == 1951 && abs(t(end) - 65) < 1e-9 && max(abs(diff(t) - Ts)) < 1e-9;
    end
    G2 = strcmp(md5, dataMD5) && gridOK;
    fprintf('G2 data MD5 %s (expected %s), 30 Hz grid ok: %d -> %s\n', md5, dataMD5, gridOK, local_pf(G2));

    Pt = struct('Md', T.Md, 'Izz', T.Izz, 'c', T.c, 'g', T.g, 'fv3', T.fv3, 'fv4', T.fv4, ...
                'fc3', T.fc3, 'fc4', T.fc4, 'eps3', 3e-4, 'eps4', 3e-4);

    % ---------------- predictions (spec; analytic)
    % Branch B noise-induced rms of Fy [kN], [E0 E1]. Velocity noise: g * kO * sigma_v * rms(D11/arm),
    % g = 0.8390 for white noise, 0.9356 for differenced quantisation noise; torque noise:
    % 0.4942 * sigma_tau * rms(1/arm). Exact filter gains; derivation in the spec.
    PRED.N1 = [27.10 13.17];  PRED.N2 = [1.915 1.829];  PRED.N3 = [189.3 92.0];
    PRED.N4 = [0.789 0.383];  PRED.N5 = sqrt(PRED.N4.^2 + PRED.N2.^2);
    BAND = struct('N1', 0.15, 'N2', 0.15, 'N3', 0.50, 'N4', 0.25, 'N5', 0.15);

    % ---------------- run
    I1 = load(fullfile(root, ind1Rel), 'R');  I1 = I1.R;
    conds = {'N0','N1','N2','N3','N4','N5'};
    RES = struct();  G3 = true;
    for i = 1:2
        cn = cases{i};  S = D.cases.(cn).S30;
        t = S.t(:);  q = S.q_3(:);  qd = S.q_3_dot(:);  d = S.d_4(:);  dd = S.d_4_dot(:);
        tau = S.tau3_cmd(:);  f4 = S.f4_cmd(:);  Fy = S.outF(:);
        k = t >= T_trim - 1e-9;
        for j = 1:numel(conds)
            c = conds{j};
            rng(seed0 + 10*j + i, 'twister');
            qm = q; qdm = qd; dm = d; ddm = dd; taum = tau; f4m = f4;
            switch c
                case 'N1'
                    qdm = qd + sigV(1)*randn(size(qd));  ddm = dd + sigV(2)*randn(size(dd));
                case 'N2'
                    taum = tau + pctT*local_rms(tau)*randn(size(tau));  f4m = f4 + pctT*local_rms(f4)*randn(size(f4));
                case {'N3','N4','N5'}
                    if strcmp(c, 'N3'), st_ = dJ; else, st_ = dM; end
                    qm = st_(1)*round(q/st_(1));  dm = st_(2)*round(d/st_(2));
                    qdm = [ (qm(2)-qm(1))/Ts; diff(qm)/Ts ];
                    ddm = [ (dm(2)-dm(1))/Ts; diff(dm)/Ts ];
                    if strcmp(c, 'N5')
                        taum = tau + pctT*local_rms(tau)*randn(size(tau));  f4m = f4 + pctT*local_rms(f4)*randn(size(f4));
                    end
            end
            O = m3_observer_30Hz(qm, qdm, dm, ddm, taum, f4m, Pt, kO, Ts);
            arm = (dm + T.x_ref) .* cos(qm);
            FyB = O.tau_hat_B(1,:)' ./ arm;  FyK = O.tau_hat_KF(1,:)' ./ arm;
            m = struct('FyB', FyB, 'FyK', FyK);
            m.B_rms_truth_kN  = sqrt(mean((FyB(k) - Fy(k)).^2)) / 1e3;
            m.KF_rms_truth_kN = sqrt(mean((FyK(k) - Fy(k)).^2)) / 1e3;
            if strcmp(c, 'N0')
                ref = I1.results.(cn).control;
                dB = max(abs(FyB - ref.FyB(:)));  dK = max(abs(FyK - ref.FyK(:)));
                G3 = G3 && dB <= 1e-6 && dK <= 1e-6;      % N; arrays must be the same
                m.G3diff_N = [dB dK];
                B0 = FyB;  K0 = FyK;
            end
            m.B_noise_kN  = sqrt(mean((FyB(k) - B0(k)).^2)) / 1e3;
            m.KF_noise_kN = sqrt(mean((FyK(k) - K0(k)).^2)) / 1e3;
            RES.(cn).(c) = m;
        end
    end
    fprintf('G3 clean condition reproduces MS03-IND1 (max diff B %.1e N, KF %.1e N; limit 1e-6 N) -> %s\n', ...
            max(RES.E0.N0.G3diff_N(1), RES.E1.N0.G3diff_N(1)), ...
            max(RES.E0.N0.G3diff_N(2), RES.E1.N0.G3diff_N(2)), local_pf(G3));
    R.gates = struct('G1', G1, 'G1_detail', g1, 'G2', G2, 'md5', md5, 'G3', G3);

    % ---------------- report
    fprintf('\n%-4s %-4s %14s %14s %12s %14s %14s\n', 'case', 'cond', 'B noise [kN]', 'KF noise [kN]', 'KF/B', 'B vs truth', 'KF vs truth');
    for i = 1:2
        for j = 1:numel(conds)
            m = RES.(cases{i}).(conds{j});
            ratio = m.KF_noise_kN / max(m.B_noise_kN, eps);
            fprintf('%-4s %-4s %14.3f %14.3f %12.3f %14.3f %14.3f\n', cases{i}, conds{j}, m.B_noise_kN, m.KF_noise_kN, ...
                    ratio, m.B_rms_truth_kN, m.KF_rms_truth_kN);
        end
    end

    fprintf('\nPredictions (spec; Branch B noise-induced rms, kN):\n');
    PR = struct();
    for j = 2:numel(conds)
        c = conds{j};  ok = true;
        for i = 1:2
            v = RES.(cases{i}).(c).B_noise_kN;  p = PRED.(c)(i);
            ok = ok && abs(v - p) <= BAND.(c)*p;
            fprintf('   %s %s: %.3f kN (predicted %.3f, band +/-%d %%)\n', c, cases{i}, v, p, round(100*BAND.(c)));
        end
        PR.(c) = ok;
        fprintf('   P-%s -> %s\n', c, local_hr(ok));
    end
    okKF = true;
    for i = 1:2
        okKF = okKF && RES.(cases{i}).N1.KF_noise_kN <= 0.35 * RES.(cases{i}).N1.B_noise_kN;
    end
    PR.KF_N1 = okKF;
    fprintf('   P-KF  KF-GMO noise-induced rms <= 0.35 x Branch B under N1 (M3 archive: 0.19) -> %s\n', local_hr(okKF));

    R.results = RES;  R.predictions = PR;  R.pred_values = PRED;  R.bands = BAND;
    R.valid = G1 && G2 && G3;
    fprintf('\nVALIDITY (G1, G2, G3): %s. Predictions may be refuted; a refutation is a result.\n', local_pf(R.valid));

    if o.Save
        fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 450]);
        for i = 1:2
            subplot(1, 2, i);
            yB = cellfun(@(c) RES.(cases{i}).(c).B_noise_kN,  conds(2:end));
            yK = cellfun(@(c) RES.(cases{i}).(c).KF_noise_kN, conds(2:end));
            yP = cellfun(@(c) PRED.(c)(i), conds(2:end));
            % markers, not bars: bars on a log axis start from an arbitrary base and mislead
            semilogy((1:5) - 0.12, yB, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8); hold on;
            semilogy((1:5) + 0.12, yK, 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
            semilogy(1:5, yP, 'kd', 'MarkerSize', 9, 'LineWidth', 1.2);
            semilogy([0.5 5.5], [100 100], 'k--');
            set(gca, 'XTick', 1:5, 'XTickLabel', conds(2:end), 'XLim', [0.5 5.5], 'YLim', [1e-2 1e3]);
            grid on; ylabel('noise-induced F_y error, rms [kN]');
            title(sprintf('%s (dashed: 100 kN load amplitude)', cases{i}), 'Interpreter', 'none');
            legend('M3 Branch B', 'M3 KF-GMO', 'predicted (Branch B)', 'Location', 'southwest');
        end
        sgtitle(sprintf('MS04-IND1 %s: sensor noise and encoder steps on MS02P08B Simscape data', stamp), 'Interpreter', 'none');
        exportgraphics(fig, fullfile(outDir, ['MS04_IND1_' stamp '.png']), 'Resolution', 150);
        close(fig);
        f = fullfile(outDir, ['MS04_IND1_' stamp '.mat']);
        save(f, 'R', '-v7.3');
        fprintf('Saved %s\nLog   %s\n', f, logFile);
    end
end

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
    ok = info.relB <= 1e-9 && info.relKF <= 1e-9;
end

function y = local_rms(x), y = sqrt(mean(x(:).^2)); end

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
