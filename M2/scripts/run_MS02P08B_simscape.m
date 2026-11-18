function R = run_MS02P08B_simscape(varargin)
%RUN_MS02P08B_SIMSCAPE  MS02P08B: torque-driven Simscape plant vs. the M2 residual estimator.
%
%   Spec (freeze before the first full run):
%     M2/experiments/08_held_out/rev2/MS02P08B_SPEC.md
%
%   Plant:     M2/model/M2_TorqueDriven_Validation.slx (build_M2_TorqueDriven_Validation.m),
%              Machine A parameters (Rasuli 2014 Table II), read back from the blocks.
%   Measured:  drive torque/force (controller output), sensed q, qdot, qddot.
%   Estimator: M1/M2 inverse-dynamics residual  Q_hat = tau_model(P_hat; q,qd,qdd) - tau_cmd
%              run twice: P_hat = truth (control) and P_hat = mismatch
%              (Md +5 %, Izz +10 %, c +0.05 m).
%
%   Gates (tolerances fixed in the spec; do not edit after seeing results):
%     G0  plant parameters read back from the model equal Rasuli Table II values
%     G1  applied bail force peak equals F0_y within 0.5 %
%     G2  realised trajectory: q3 mid within 0.5 deg, d4 mid within 0.02 m,
%         half-ranges within 5 % of the reference
%     G3  wiring: joint actuator torque = drive command - Coulomb, within 1e-6 relative
%     G4  control (P_hat = truth): residual equals the true generalised load,
%         max error within 1e-6 of peak |Q3_true| (q3) and of F0_y (d4)
%     G5  mismatch: Fy bias and RMS error from the saddle channel within 10 % of the
%         pre-registered prediction (E0: +21.9 / 21.9 kN; E1: +21.3 / 21.3 kN)
%
%   USAGE (MATLAB R2025b desktop):
%     run_MS02P08B_simscape('SmokeTest', true)  % 2 s, prints G3/G4 only, saves nothing
%     R = run_MS02P08B_simscape;                % full run, both cases, saves results (65 s stop, 5 s trim)
%     R = run_MS02P08B_simscape('RequireClean', false)  % allowed, but recorded as DIRTY
%
%   OUTPUT (full run): M2/experiments/08_held_out/rev2/
%     MS02P08B_simscape_<timestamp>.mat   results, read-back, gates, provenance,
%                                         plus 30 Hz signals (S30) for the M3 observer
%     MS02P08B_log_<timestamp>.txt        console log
%   Existing files are never overwritten.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18 (untested:
%   written without MATLAB; first run must be checked by the researcher).

    p = inputParser;
    addParameter(p, 'Cases', {'E0','E1'});
    addParameter(p, 'RequireClean', true, @islogical);
    addParameter(p, 'SmokeTest', false, @islogical);
    addParameter(p, 'Save', true, @islogical);
    parse(p, varargin{:});
    o = p.Results;

    here    = fileparts(mfilename('fullpath'));
    root    = fileparts(fileparts(here));
    mdl     = 'M2_TorqueDriven_Validation';
    mdlFile = fullfile(root, 'M2', 'model', [mdl '.slx']);
    outDir  = fullfile(root, 'M2', 'experiments', '08_held_out', 'rev2');
    if ~isfile(mdlFile)
        error('P08B:model', 'Model not found: %s\nRun build_M2_TorqueDriven_Validation first.', mdlFile);
    end
    doSave = o.Save && ~o.SmokeTest;
    stamp  = datestr(now, 'yyyymmdd_HHMMSS');
    if doSave
        if ~isfolder(outDir), mkdir(outDir); end
        logFile = fullfile(outDir, ['MS02P08B_log_' stamp '.txt']);
        diary(logFile); cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
    end

    % ---------------- provenance
    [s1, commit] = system(sprintf('git -C "%s" rev-parse HEAD', root));
    [s2, st]     = system(sprintf('git -C "%s" status --porcelain', root));
    commit = strtrim(commit); if s1 ~= 0, commit = 'UNKNOWN'; end
    dirty  = (s2 ~= 0) || ~isempty(strtrim(st));
    if dirty && o.RequireClean && ~o.SmokeTest
        error('P08B:dirty', ['Uncommitted changes in the repository. Commit the spec, scripts and ' ...
              'model first (the commit is the freeze), or pass ''RequireClean'', false ' ...
              '(the run is then recorded as DIRTY).']);
    end
    fprintf('MS02P08B  %s  commit %s%s\n', stamp, commit, ternary(dirty, '  [DIRTY]', ''));

    % ---------------- truth (Rasuli 2014 Table II, claims.md rows 9-16) and setup
    T = struct('Md',54300, 'c',-1.32, 'Izz',287900, 'fv3',3800, 'fc3',340, ...
               'fv4',1473, 'fc4',480, 'g',9.80665, 'x_ref',1.5, 'eps_c',1e-3);
    local_crosscheck_shovel_params(root, T);
    Pm = T; Pm.Md = 1.05*T.Md; Pm.Izz = 1.10*T.Izz; Pm.c = T.c + 0.05;      % mismatch
    F0_y = 100e3; omega_F = 2*pi/9;                                          % bail load
    T_stop = 65; T_trim = 5; Ts_log = 1/300;                                 % 65s stop, 5s trim -> 60s steady window
    if o.SmokeTest, T_stop = 2; T_trim = 0.5; end
    CASES.E0 = struct('name','E0', 'q3B_deg',45,  'q3A_deg',15, 'd4B',8.75,  'd4A',2.0,   'w',pi/10, ...
                      'pred_bias_kN',21.9, 'pred_rms_kN',21.9);
    CASES.E1 = struct('name','E1', 'q3B_deg',-12, 'q3A_deg',9,  'd4B',7.325, 'd4A',0.275, 'w',0.51, ...
                      'pred_bias_kN',21.3, 'pred_rms_kN',21.3);
    TOL = struct('force',5e-3, 'q3mid_deg',0.5, 'd4mid_m',0.02, 'halfrange',0.05, ...
                 'wiring',1e-6, 'identity',1e-6, 'pred',0.10);

    % ---------------- G0: read back the plant from the model
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(mdlFile);
    RB = local_readback(mdl);
    G0 = abs(RB.Md-T.Md) <= 1e-9*T.Md && abs(RB.c-T.c) <= 1e-9 && abs(RB.Izz-T.Izz) <= 1e-9*T.Izz && ...
         abs(RB.fv3-T.fv3) <= 1e-4*T.fv3 && abs(RB.fv4-T.fv4) <= 1e-9*T.fv4 && ...
         abs(RB.fc3-T.fc3) <= 1e-12 && abs(RB.fc4-T.fc4) <= 1e-12 && ...
         abs(RB.g-T.g) <= 1e-12 && abs(RB.eps3-T.eps_c) <= 1e-15 && abs(RB.eps4-T.eps_c) <= 1e-15 && ...
         RB.torqueInput && isempty(RB.wsVars);
    fprintf('G0 plant read-back: Md %.1f  c %.4f  Izz %.1f  fv3 %.2f  fv4 %.1f  fc3 %.1f  fc4 %.1f  g %.5f  -> %s\n', ...
        RB.Md, RB.c, RB.Izz, RB.fv3, RB.fv4, RB.fc3, RB.fc4, RB.g, pf(G0));
    if ~isempty(RB.wsVars)
        fprintf('   model workspace is not empty (%s): setVariable values could be shadowed\n', strjoin(RB.wsVars, ', '));
    end

    % ---------------- runs
    names = o.Cases; if ischar(names), names = {names}; end
    R = struct('meta',[], 'readback',RB, 'truth',T, 'mismatch',Pm, 'tol',TOL, 'cases',[]);
    R.meta = struct('stamp',stamp, 'commit',commit, 'dirty',dirty, 'matlab',version, ...
        'model',mdlFile, 'model_md5',local_md5(mdlFile), 'smoke',o.SmokeTest, ...
        'F0_y',F0_y, 'omega_F',omega_F, 'T_stop',T_stop, 'T_trim',T_trim, 'Ts_log',Ts_log, ...
        'spec','M2/experiments/08_held_out/rev2/MS02P08B_SPEC.md');
    allPass = G0;
    for i = 1:numel(names)
        cs = CASES.(names{i});
        fprintf('\n=== Case %s: q3 = %+g +/- %g deg, d4 = %g +/- %g m, omega = %.3f rad/s ===\n', ...
            cs.name, cs.q3B_deg, cs.q3A_deg, cs.d4B, cs.d4A, cs.w);
        S = local_simulate(mdl, cs, T, F0_y, omega_F, T_stop, Ts_log);
        C = local_evaluate(S, cs, T, Pm, F0_y, T_trim, TOL, o.SmokeTest);
        C.S30 = local_30Hz(S, Ts_log);
        R.cases.(cs.name) = C;
        allPass = allPass && C.pass;
    end
    close_system(mdl, 0);

    fprintf('\nOVERALL: %s%s\n', pf(allPass), ternary(o.SmokeTest, '  (smoke test: G1, G2, G5 not evaluated)', ''));
    R.meta.allPass = allPass;
    if doSave
        f = fullfile(outDir, ['MS02P08B_simscape_' stamp '.mat']);
        if isfile(f), error('P08B:exists', 'Refusing to overwrite %s', f); end
        save(f, 'R', '-v7.3');
        fprintf('Saved %s\nLog   %s\n', f, logFile);
    end
end

% =====================================================================================
function S = local_simulate(mdl, cs, T, F0_y, omega_F, T_stop, Ts_log)
    q3B = deg2rad(cs.q3B_deg); q3A = deg2rad(cs.q3A_deg);
    % controller (part of the plant/operator, uses truth; the estimator never sees it)
    wc = 15;                                   % rad/s closed-loop bandwidth
    J3 = T.Izz + T.Md*(cs.d4B + T.c)^2;
    Kp3 = J3*wc^2;   Kd3 = 2*J3*wc;   Ki3 = Kp3*wc/4;
    Kp4 = T.Md*wc^2; Kd4 = 2*T.Md*wc; Ki4 = Kp4*wc/4;
    I3_0 = T.Md*T.g*(cs.d4B + T.c)*cos(q3B)/Ki3;   % integrator starts at gravity torque
    I4_0 = T.Md*T.g*sin(q3B)/Ki4;
    V = struct('q3_B',q3B, 'q3_A',q3A, 'd4_B',cs.d4B, 'd4_A',cs.d4A, 'omega_traj',cs.w, ...
        'F0_y',F0_y, 'F0_x',0, 'omega_F',omega_F, ...
        'q3_0_deg',cs.q3B_deg, 'q3d_0_degps',cs.q3A_deg*cs.w, 'd4_0',cs.d4B, 'd4d_0',cs.d4A*cs.w, ...
        'Kp3',Kp3, 'Kd3',Kd3, 'Ki3',Ki3, 'Kp4',Kp4, 'Kd4',Kd4, 'Ki4',Ki4, ...
        'I3_0',I3_0, 'I4_0',I4_0, 'Ts_log',Ts_log, 'T_stop',T_stop);
    in = Simulink.SimulationInput(mdl);
    fn = fieldnames(V);
    for k = 1:numel(fn), in = in.setVariable(fn{k}, V.(fn{k})); end
    in = in.setModelParameter('StopTime', num2str(T_stop));
    t0 = tic;
    out = sim(in);
    if ~isempty(out.ErrorMessage), error('P08B:sim', '%s', out.ErrorMessage); end
    fprintf('   simulated %.1f s in %.1f s wall time\n', T_stop, toc(t0));
    names = {'q_3','q_3_dot','q_3_ddot','d_4','d_4_dot','d_4_ddot','tau_3','f_4','outF', ...
             'tau3_cmd','tau3_coul','f4_cmd','f4_coul','q3_ref','d4_ref'};
    for k = 1:numel(names)
        [t, x] = local_sig(out, names{k});
        if k == 1
            S.t = t;
        elseif numel(t) ~= numel(S.t) || max(abs(t - S.t)) > 1e-9
            error('P08B:time', 'Signal %s is not on the common Ts_log grid.', names{k});
        end
        S.(names{k}) = x;
    end
    S.vars = V;
end

function C = local_evaluate(S, cs, T, Pm, F0_y, T_trim, TOL, smoke)
    k  = S.t >= T_trim;
    q  = S.q_3(k);  qd = S.q_3_dot(k);  qdd = S.q_3_ddot(k);
    d  = S.d_4(k);  dd = S.d_4_dot(k);  ddd = S.d_4_ddot(k);
    t3 = S.tau3_cmd(k); f4 = S.f4_cmd(k);
    Fy = S.outF(k); Fx = zeros(size(Fy));
    s  = d + T.x_ref;                                   % bail station from the pivot
    Q3 = s.*(Fy.*cos(q) - Fx.*sin(q));                  % true generalised load (M2_README)
    Q4 = Fx.*cos(q) + Fy.*sin(q);

    % G1 force read-back, G2 trajectory read-back
    Fpk = max(abs(Fy));
    G1 = abs(Fpk - F0_y)/F0_y <= TOL.force;
    q3mid = rad2deg((max(q)+min(q))/2); q3half = rad2deg((max(q)-min(q))/2);
    d4mid = (max(d)+min(d))/2;          d4half = (max(d)-min(d))/2;
    G2 = abs(q3mid - cs.q3B_deg) <= TOL.q3mid_deg && abs(d4mid - cs.d4B) <= TOL.d4mid_m && ...
         abs(q3half/cs.q3A_deg - 1) <= TOL.halfrange && abs(d4half/cs.d4A - 1) <= TOL.halfrange;
    % G3 wiring: joint actuator torque = command - Coulomb
    w3 = max(abs(S.tau_3(k) - (t3 - S.tau3_coul(k)))) / max(abs(t3));
    w4 = max(abs(S.f_4(k)   - (f4 - S.f4_coul(k))))   / max(abs(f4));
    G3 = w3 <= TOL.wiring && w4 <= TOL.wiring;

    % estimator, control and mismatch
    [m3t, m4t] = local_model(T,  q,qd,qdd,d,dd,ddd);
    [m3m, m4m] = local_model(Pm, q,qd,qdd,d,dd,ddd);
    Q3c = m3t - t3;  Q4c = m4t - f4;                    % control residual
    Q3m = m3m - t3;                                     % mismatch residual (saddle)
    e3 = max(abs(Q3c - Q3)) / max(abs(Q3));
    e4 = max(abs(Q4c - Q4)) / F0_y;
    G4 = e3 <= TOL.identity && e4 <= TOL.identity;

    Fy_c = Q3c ./ (s.*cos(q));  Fy_m = Q3m ./ (s.*cos(q));
    ec = Fy_c - Fy; em = Fy_m - Fy;
    bias_m = mean(em)/1e3; rms_m = sqrt(mean(em.^2))/1e3;
    G5 = abs(bias_m - cs.pred_bias_kN)/cs.pred_bias_kN <= TOL.pred && ...
         abs(rms_m  - cs.pred_rms_kN)/cs.pred_rms_kN  <= TOL.pred;

    fprintf('   G1 force peak      %.1f kN (F0_y %.1f)                       -> %s\n', Fpk/1e3, F0_y/1e3, pf(G1));
    fprintf('   G2 trajectory      q3 mid %+.2f deg, half %.2f; d4 mid %.3f m, half %.3f -> %s\n', q3mid, q3half, d4mid, d4half, pf(G2));
    fprintf('   G3 wiring          %.2e (q3), %.2e (d4)                        -> %s\n', w3, w4, pf(G3));
    fprintf('   G4 identity        max err %.2e of peak Q3, %.2e of F0_y        -> %s\n', e3, e4, pf(G4));
    fprintf('   control  Fy error  bias %+.4f kN, rms %.4f kN\n', mean(ec)/1e3, sqrt(mean(ec.^2))/1e3);
    fprintf('   G5 mismatch Fy err bias %+.2f kN (pred %+.1f), rms %.2f kN (pred %.1f) -> %s\n', ...
        bias_m, cs.pred_bias_kN, rms_m, cs.pred_rms_kN, pf(G5));

    if smoke
        pass = G3 && G4;            % short run: force peak and trajectory not yet reached
    else
        pass = G1 && G2 && G3 && G4 && G5;
    end
    C = struct('case',cs, 'pass',pass, ...
        'G1',G1, 'G2',G2, 'G3',G3, 'G4',G4, 'G5',G5, ...
        'Fpeak',Fpk, 'q3mid_deg',q3mid, 'q3half_deg',q3half, 'd4mid',d4mid, 'd4half',d4half, ...
        'wiring',[w3 w4], 'identity',[e3 e4], ...
        'control_bias_kN',mean(ec)/1e3, 'control_rms_kN',sqrt(mean(ec.^2))/1e3, ...
        'mismatch_bias_kN',bias_m, 'mismatch_rms_kN',rms_m, ...
        'signals',S);
end

function [t3, f4] = local_model(P, q,qd,qdd,d,dd,ddd)
% M1 2-DOF inverse dynamics (claims.md Stage 1; r = d4 + c), tanh Coulomb with eps_c.
    r  = d + P.c;
    t3 = (P.Izz + P.Md*r.^2).*qdd + 2*P.Md*r.*dd.*qd + P.Md*P.g*r.*cos(q) ...
         + P.fv3*qd + P.fc3*tanh(qd/P.eps_c);
    f4 = P.Md*ddd - P.Md*r.*qd.^2 + P.Md*P.g*sin(q) + P.fv4*dd + P.fc4*tanh(dd/P.eps_c);
end

function S30 = local_30Hz(S, Ts_log)
% Every 10th sample of the 300 Hz log = 30 Hz drive-logging rate (input for the M3 observer).
    n = round((1/30)/Ts_log);
    i = 1:n:numel(S.t);
    f = {'t','q_3','q_3_dot','d_4','d_4_dot','tau3_cmd','f4_cmd','outF'};
    for k = 1:numel(f), S30.(f{k}) = S.(f{k})(i); end
    S30.note = 'Ts = 1/30 s. No acceleration included on purpose (M3 observer input).';
end

function RB = local_readback(mdl)
    num = @(h, prm) str2num(get_param(h, prm)); %#ok<ST2NM>
    hBody = Simulink.ID.getHandle([mdl ':31']);
    hTf   = Simulink.ID.getHandle([mdl ':36']);
    hRev  = Simulink.ID.getHandle([mdl ':29']);
    hPri  = Simulink.ID.getHandle([mdl ':37']);
    hMech = Simulink.ID.getHandle([mdl ':21']);
    assert(strcmp(get_param(hBody,'MassUnits'),'kg') && strcmp(get_param(hBody,'CenterOfMassUnits'),'m') && ...
           strcmp(get_param(hBody,'MomentsOfInertiaUnits'),'kg*m^2'), 'P08B:units', 'Unexpected body units');
    RB.Md   = num(hBody,'Mass');
    com     = num(hBody,'CenterOfMass');
    I       = num(hBody,'MomentsOfInertia');
    RB.Izz  = I(1);                                  % body x maps to the revolute axis (90 deg about +Y)
    off     = num(hTf,'TranslationCartesianOffset');
    RB.x_ref = off(1);
    RB.c    = RB.x_ref + com(3);                     % COM 2.82 m behind the bail -> c = -1.32
    v = num(hRev,'DampingCoefficient'); u = get_param(hRev,'DampingCoefficientUnits');
    switch u
        case 'N*m/(deg/s)', RB.fv3 = v*180/pi;
        case 'N*m/(rad/s)', RB.fv3 = v;
        otherwise, error('P08B:units', 'Unexpected revolute damping unit %s', u);
    end
    assert(strcmp(get_param(hPri,'DampingCoefficientUnits'),'N/(m/s)'), 'P08B:units', 'Unexpected prismatic damping unit');
    RB.fv4  = num(hPri,'DampingCoefficient');
    RB.fc3  = str2double(get_param([mdl '/fc3 plant (Rasuli 2014 Tab II)'], 'Gain'));
    RB.fc4  = str2double(get_param([mdl '/fc4 plant (Rasuli 2014 Tab II)'], 'Gain'));
    RB.eps3 = 1/str2double(get_param([mdl '/q3 inv eps_c (plant)'], 'Gain'));
    RB.eps4 = 1/str2double(get_param([mdl '/d4 inv eps_c (plant)'], 'Gain'));
    gv      = num(hMech,'GravityVector');
    RB.g    = -gv(2);
    RB.torqueInput = strcmp(get_param(hRev,'TorqueActuationMode'),'InputTorque') && ...
                     strcmp(get_param(hPri,'TorqueActuationMode'),'InputTorque') && ...
                     strcmp(get_param(hRev,'MotionActuationMode'),'ComputedMotion') && ...
                     strcmp(get_param(hPri,'MotionActuationMode'),'ComputedMotion');
    hws = get_param(mdl,'ModelWorkspace');
    w = hws.whos;
    RB.wsVars = {w.name};
end

function local_crosscheck_shovel_params(root, T)
% Compare the truth constants with M1/scripts/shovel_params.m (single source of truth).
    f = fullfile(root, 'M1', 'scripts', 'shovel_params.m');
    if ~isfile(f), fprintf('   (shovel_params.m not found; cross-check skipped)\n'); return; end
    addpath(fileparts(f));
    P = shovel_params();
    d = [P.m-T.Md, P.c-T.c, P.Izz-T.Izz, P.fv3-T.fv3, P.fc3-T.fc3, P.fv4-T.fv4, P.fc4-T.fc4, P.g-T.g];
    if any(abs(d) > 1e-9)
        error('P08B:params', 'Truth constants differ from shovel_params.m: [%s]', num2str(d));
    end
    fprintf('   truth constants match shovel_params.m\n');
end

function [t, x] = local_sig(out, name)
    v = out.get(name);
    if isa(v, 'timeseries')
        t = v.Time(:); x = squeeze(v.Data); x = x(:);
    elseif isstruct(v) && isfield(v, 'signals')
        t = v.time(:); x = v.signals.values(:);
    else
        error('P08B:log', 'Signal %s not found or in an unexpected format.', name);
    end
end

function h = local_md5(f)
    fid = fopen(f, 'r'); b = fread(fid, inf, '*uint8'); fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5'); md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'))', 1, []));
end

function s = pf(b), s = ternary(b, 'PASS', 'FAIL'); end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end
