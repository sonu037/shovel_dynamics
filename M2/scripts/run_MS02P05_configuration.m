function R = run_MS02P05_configuration(varargin)
%RUN_MS02P05_CONFIGURATION  MS02P05: configuration maps (Part 1) and Simscape spot checks (Part 2).
%
%   Spec (frozen 2026-11-17, commit 2dd61f7) with amendment 1 (2026-11-18,
%   before the first run):
%     M2/experiments/00_design/M2_Experiment_Design_Specification.md, module 05.
%
%   QUESTION (spec): across the configurations a working shovel actually occupies,
%   where does each generalized coordinate observe a vertical load at the bail
%   station, and where can that load be separated from an error in the
%   empty-dipper mass M_d when both channels are used together?
%
%   PART 1 - analytic maps (the main result). Grid q3 = -60:1:70 deg,
%     d4 = 4.0:0.1:11.0 m. Map 1 |s_F3|, Map 2 |s_F4|, Map 3 quasi-static |det|
%     and whitened column angle theta_W for L_n = 1, 3, 10 m, Map 4 the dynamic
%     separability bound near q3 = 0 for E1 and E2 rates. Envelopes E0/E1/E2
%     overlaid. Predictions P1-P5 and acceptance (c) zero locus, (d) code check
%     against the 12-case table of m2_formal_identifiability_svd.m.
%   PART 2 - Simscape spot checks on M2_LoadInjection_P05.slx (copy built by
%     build_M2_LoadInjection_P05.m), 100 kN vertical at the bail, motion-prescribed,
%     analysed with M2/scripts/recover_load.m (unchanged):
%       (a) read-back A_q3 = 15, B_q3 = 45 must reproduce M2_100kN_0p1111Hz_sp1p50
%           to every printed digit; control A_q3 = 10 must differ;
%       (b) 10 centres, A_q3 = 2 deg, A_d4 = 0.25 m, omega_traj = 0.5 rad/s, trim 0 s:
%           slope -1.000000 on both channels and error at the Module-01 floor
%           (operational form in amendment 1: max|err| / max|model effort| <= 10 x
%           the Module-01 relative floor of that channel, read from M2_null.mat).
%
%   USAGE (MATLAB R2025b desktop, current folder = repository root)
%     run_MS02P05_configuration                       % both parts; needs a clean git tree
%     run_MS02P05_configuration('Part', 'maps')       % Part 1 only
%     run_MS02P05_configuration('SmokeTest', true)    % Part 1 + read-back + control + one
%                                                     % centre; nothing saved; tree may be dirty
%
%   OUTPUT  M2/experiments/05_configuration_sweep/rev2/
%     MS02P05_<stamp>.mat          summary (maps, predictions, acceptance, spot table)
%     MS02P05_log_<stamp>.txt      console log
%     MS02P05_map1..4_<stamp>.png  figures
%     MS02P05_<stamp>_<case>.mat   one recover_load result per Simscape run
%   The 2026-11-09 pilot files in 05_configuration_sweep/ are not touched.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18
%   (written without MATLAB; the first run must be checked by the researcher).

    ip = inputParser;
    addParameter(ip, 'Part', 'all', @(s) any(strcmpi(s, {'all','maps','spots'})));
    addParameter(ip, 'RequireClean', true, @islogical);
    addParameter(ip, 'Save', true, @islogical);
    addParameter(ip, 'SmokeTest', false, @islogical);
    parse(ip, varargin{:});
    o = ip.Results;
    doMaps  = any(strcmpi(o.Part, {'all','maps'}));
    doSpots = any(strcmpi(o.Part, {'all','spots'}));
    doSave  = o.Save && ~o.SmokeTest;

    root   = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    outDir = fullfile(root, 'M2', 'experiments', '05_configuration_sweep', 'rev2');
    addpath(fullfile(root, 'M1', 'scripts'), fullfile(root, 'M2', 'scripts'));
    stamp  = datestr(now, 'yyyymmdd_HHMMSS');

    % ---------------- provenance: checked BEFORE any file is created
    [s1, commit] = system(sprintf('git -C "%s" rev-parse HEAD', root));
    [s2, st]     = system(sprintf('git -C "%s" status --porcelain', root));
    commit = strtrim(commit); if s1 ~= 0, commit = 'UNKNOWN'; end
    dirty  = (s2 ~= 0) || ~isempty(strtrim(st));
    if dirty && o.RequireClean && ~o.SmokeTest
        error('P05:dirty', ['Uncommitted changes in the repository. Commit the spec amendment, ' ...
              'scripts, model copy and receipt first (the commit is the freeze), or pass ' ...
              '''RequireClean'', false (the run is then recorded as DIRTY).\n%s'], strtrim(st));
    end
    if doSave
        if ~isfolder(outDir), mkdir(outDir); end
        logFile = fullfile(outDir, ['MS02P05_log_' stamp '.txt']);
        diary(logFile); cleanupDiary = onCleanup(@() diary('off')); %#ok<NASGU>
    end
    dirtyTag = ''; if dirty, dirtyTag = '  [DIRTY]'; end
    smokeTag = ''; if o.SmokeTest, smokeTag = '  [SMOKE TEST - nothing saved]'; end
    fprintf('MS02P05  %s  commit %s%s%s\n', stamp, commit, dirtyTag, smokeTag);

    % ---------------- parameters (single source of truth)
    P  = shovel_params();
    g  = P.g;  c = P.c;  xr = P.x_ref;  Lcog = P.L_COG;
    fprintf('   shovel_params: g %.5f, x_ref %.2f m, L_COG %.2f m, c %.2f m (s - r = %.2f m)\n', ...
            g, xr, Lcog, c, xr - c);
    if abs((xr - c) - Lcog) > 1e-12
        error('P05:geometry', 'x_ref - c (%.6f) differs from L_COG (%.6f).', xr - c, Lcog);
    end

    EN = local_envelopes();
    R  = struct('stamp', stamp, 'commit', commit, 'dirty', dirty, 'smoke', o.SmokeTest, ...
                'matlab', version, 'envelopes', EN);

    % =====================================================================
    % PART 1 - analytic maps
    % =====================================================================
    if doMaps
        fprintf('\n=== PART 1: analytic configuration maps ===\n');
        q3g = (-60:1:70)';                 % deg   (rows)
        d4g = 4.0:0.1:11.0;                % m     (columns)
        d4g = round(d4g*10)/10;            % exact 0.1 m grid values
        [Qd, D] = ndgrid(q3g, d4g);
        Q  = deg2rad(Qd);
        s  = D + xr;   r = D + c;

        sF3 = s .* cos(Q);                 % m        load column, saddle
        sF4 = sin(Q);                      % -        load column, crowd
        sM3 = g * r .* cos(Q);             % m^2/s^2  mass-error column, saddle (static part)
        sM4 = g * sin(Q);                  % m/s^2    mass-error column, crowd  (static part)
        dt  = sF3 .* sM4 - sM3 .* sF4;     % quasi-static determinant
        dtClosed = Lcog * g * sin(Q) .* cos(Q);
        errClosed = max(abs(dt(:) - dtClosed(:))) / max(abs(dtClosed(:)));
        fprintf('   det from the four columns vs closed form 2.82 g sin q3 cos q3: max rel diff %.2e\n', errClosed);

        Ln = [1 3 10];
        thW = zeros([size(Q) numel(Ln)]);
        for k = 1:numel(Ln)
            thW(:,:,k) = local_angle(sF3, Ln(k)*sF4, sM3, Ln(k)*sM4);
        end

        % Map 4: dynamic separability bound near q3 = 0, max |d4ddot - r q3dot^2| / g
        rr = abs(d4g + c);
        M4.E1 = (EN.E1.d4dd_max + rr * EN.E1.q3d_max^2) / g;
        M4.E2 = (EN.E2.d4dd_max + rr * EN.E2.q3d_max^2) / g;

        maps = struct('q3_deg', q3g, 'd4_m', d4g, 'sF3', sF3, 'sF4', sF4, 'sM3', sM3, 'sM4', sM4, ...
                      'det', dt, 'det_closed_reldiff', errClosed, 'Ln_m', Ln, 'thetaW_deg', thW, 'map4', M4);

        % ---- predictions P1-P5 (spec, written before the run)
        PR = struct();
        inE1 = local_in(Qd, D, EN.E1.q3_lift, EN.E1.d4) | local_in(Qd, D, EN.E1.q3_swing, EN.E1.d4);
        inE2 = local_in(Qd, D, EN.E2.q3, EN.E2.d4);
        minF3_E1 = min(abs(sF3(inE1)));  minF3_E2 = min(abs(sF3(inE2)));
        PR.P1 = struct('min_sF3_E1', minF3_E1, 'min_sF3_E2', minF3_E2, 'pred_min', 4.03, ...
                       'holds', min(minF3_E1, minF3_E2) >= 4.03 - 0.005);
        fprintf('   P1  min |s_F3| in E1 %.4f m, in E2 %.4f m (pred >= 4.03 m)        -> %s\n', ...
                minF3_E1, minF3_E2, local_hr(PR.P1.holds));

        i0  = find(q3g == 0);  im3 = find(q3g == -3);
        sF4_0  = max(abs(sF4(i0,:)));  sF4_m3 = abs(sF4(im3,1));
        PR.P2 = struct('max_sF4_at_0', sF4_0, 'sF4_at_m3', sF4_m3, 'pred_m3', 0.052, ...
                       'holds', sF4_0 < 1e-15 && abs(round(sF4_m3,3) - 0.052) < 1e-9);
        fprintf('   P2  max |s_F4| on q3 = 0: %.1e; |s_F4| at -3 deg: %.4f (pred 0, 0.052) -> %s\n', ...
                sF4_0, sF4_m3, local_hr(PR.P2.holds));

        det0 = max(abs(dt(i0,:)));
        dDep = max(max(dt,[],2) - min(dt,[],2)) / max(abs(dt(:)));
        th0  = max(reshape(abs(thW(i0,:,:)), 1, []));
        i45  = find(q3g == 45);
        relQ = [-21 -10 -3 0];  relP = [0.67 0.34 0.10 0];
        relV = zeros(size(relQ));
        for k = 1:numel(relQ)
            relV(k) = abs(dt(q3g == relQ(k), 1)) / abs(dt(i45, 1));
        end
        relOK = all(abs(round(relV, 2) - relP) < 1e-9);
        PR.P3 = struct('max_det_at_0', det0, 'd4_dependence', dDep, 'max_thetaW_at_0', th0, ...
                       'q3_deg', relQ, 'rel_det', relV, 'pred', relP, ...
                       'holds', det0 < 1e-9 && dDep < 1e-12 && th0 < 1e-6 && relOK);
        fprintf(['   P3  det on q3 = 0: max %.1e; d4-dependence %.1e; theta_W on q3 = 0: max %.1e deg;\n' ...
                 '       |det|/|det(45)| at -21/-10/-3/0 deg = %.4f %.4f %.4f %.4f (pred 0.67 0.34 0.10 0) -> %s\n'], ...
                det0, dDep, th0, relV, local_hr(PR.P3.holds));

        PR.P4 = struct('from', relV(1), 'to', relV(3), 'pred', [0.67 0.10], ...
                       'holds', abs(round(relV(1),2) - 0.67) < 1e-9 && abs(round(relV(3),2) - 0.10) < 1e-9);
        fprintf('   P4  E1 lifting (-21 -> -3 deg) runs from %.4f to %.4f of E0 separability (pred 0.67 -> 0.10) -> %s\n', ...
                relV(1), relV(3), local_hr(PR.P4.holds));

        e1 = d4g >= EN.E1.d4(1) - 1e-9 & d4g <= EN.E1.d4(2) + 1e-9;
        e2 = d4g >= EN.E2.d4(1) - 1e-9 & d4g <= EN.E2.d4(2) + 1e-9;
        b1 = max(M4.E1(e1)) * g;  b2 = max(M4.E2(e2)) * g;
        PR.P5 = struct('E1_ms2', b1, 'E1_pct_g', 100*b1/g, 'E2_ms2', b2, 'E2_pct_g', 100*b2/g, ...
                       'pred', [0.24 2.0], ...
                       'holds', abs(round(b1,2) - 0.24) < 1e-9 && abs(round(b2,1) - 2.0) < 1e-9);
        fprintf('   P5  dynamic bound near q3 = 0: E1 %.4f m/s^2 (%.2f %% g), E2 %.4f m/s^2 (%.2f %% g) (pred 0.24, 2.0) -> %s\n', ...
                b1, 100*b1/g, b2, 100*b2/g, local_hr(PR.P5.holds));

        % ---- acceptance (c): numeric zero locus of det within 1 grid step of q3 = 0 for every d4
        zOK = true(1, numel(d4g)); zLoc = cell(1, numel(d4g));
        for j = 1:numel(d4g)
            col = dt(:, j);
            k0  = find(col == 0);
            kc  = find(sign(col(1:end-1)) .* sign(col(2:end)) < 0);
            loc = [q3g(k0); (q3g(kc) + q3g(kc+1))/2];
            zLoc{j} = loc;
            zOK(j) = ~isempty(loc) && all(abs(loc) <= 1);
        end
        ACC.c = struct('pass', all(zOK), 'n_d4', numel(d4g), 'n_ok', sum(zOK));
        fprintf('   (c) zero locus of det within 1 deg of q3 = 0: %d of %d d4 columns -> %s\n', ...
                sum(zOK), numel(d4g), local_pf(ACC.c.pass));

        % ---- acceptance (d): code check against the 12-case table
        try
            ACC.d = local_codecheck(root, P);
        catch ME
            fprintf(2, '   (d) code check could not run: %s\n', ME.message);
            ACC.d = struct('pass', false, 'table', [], 'error', ME.message);
        end
        maps.zero_locus = zLoc;
        R.maps = maps;  R.predictions = PR;  R.acc_c = ACC.c;  R.acc_d = ACC.d;

        if doSave
            local_figures(maps, EN, outDir, stamp);
        end
    end

    % =====================================================================
    % PART 2 - Simscape spot checks
    % =====================================================================
    if doSpots
        fprintf('\n=== PART 2: Simscape spot checks (M2_LoadInjection_P05.slx) ===\n');
        mdl     = 'M2_LoadInjection_P05';
        mdlFile = fullfile(root, 'M2', 'model', [mdl '.slx']);
        rcpt    = fullfile(root, 'M2', 'model', [mdl '_BUILD.txt']);
        if ~isfile(mdlFile)
            error('P05:noModel', 'Model copy not found: %s. Run build_M2_LoadInjection_P05 first.', mdlFile);
        end
        md5now = local_md5(mdlFile);
        rtxt   = fileread(rcpt);
        if isempty(strfind(rtxt, md5now)) %#ok<STREMP>
            error('P05:receipt', ['%s (MD5 %s) does not match its build receipt. It was changed ' ...
                  'after the build; rebuild it with build_M2_LoadInjection_P05(''Overwrite'', true).'], mdlFile, md5now);
        end
        fprintf('   model %s  MD5 %s (matches build receipt)\n', mdl, md5now);

        if bdIsLoaded(mdl), close_system(mdl, 0); end
        load_system(mdlFile);
        cleanupModel = onCleanup(@() local_close(mdl)); %#ok<NASGU>   % never saved
        mw = get_param(mdl, 'ModelWorkspace');
        w0 = mw.getVariable('omega_traj');
        fprintf('   model-workspace omega_traj (baseline) = %.12g rad/s\n', w0);

        baseFile = fullfile(root, 'M2', 'experiments', '02_known_load', 'M2_100kN_0p1111Hz_sp1p50.mat');
        nullFile = fullfile(root, 'M2', 'experiments', '01_zero_force', 'M2_null.mat');
        Bs = load(baseFile, 'R');  B = Bs.R;
        Ns = load(nullFile, 'R');  N = Ns.R;
        floor3 = N.q3.relFloor;  floor4 = N.d4.relFloor;
        fprintf('   Module-01 relative floor (M2_null.mat): q3 %.3e, d4 %.3e; limit = 10x\n', floor3, floor4);

        bo   = B.opt;
        base = {'Amplitude', bo.Amplitude, 'Frequency', bo.Frequency, 'Fx0', local_f(bo,'Fx0',0), ...
                'Station', bo.Station, 'CrowdAmplitude', local_f(bo,'CrowdAmplitude',2), ...
                'CrowdBias', local_f(bo,'CrowdBias',8.75), 'TrimStart', local_f(bo,'TrimStart',5), ...
                'StopTime', local_f(bo,'StopTime',30), 'RelTol', local_f(bo,'RelTol',1e-10), ...
                'AbsTol', local_f(bo,'AbsTol',1e-12)};
        fprintf(['   read-back settings from the baseline file: Amplitude %g N, Frequency %.7f rad/s, ' ...
                 'Fx0 %g N, Station %.2f m, A_d4 %g m, B_d4 %g m, trim %g s, stop %g s, ' ...
                 'RelTol %g, AbsTol %g\n'], base{2:2:end});

        % ---- (a) read-back and control
        Ra = local_run(mdl, mw, 15, 45, w0, base, sprintf('MS02P05_%s_readback', stamp), outDir, doSave);
        Rc = local_run(mdl, mw, 10, 45, w0, base, sprintf('MS02P05_%s_control_Aq3_10', stamp), outDir, doSave);
        [same, tab] = local_compare(B, Ra);
        ctrlDiff  = ~strcmp(sprintf('%.6e', Rc.q3.oneMinusR2), sprintf('%.6e', B.q3.oneMinusR2));
        ctrlSlope = strcmp(sprintf('%.6f', Rc.q3.slope), '-1.000000') && strcmp(sprintf('%.6f', Rc.d4.slope), '-1.000000');
        fprintf('\n   (a) read-back vs M2_100kN_0p1111Hz_sp1p50, printed digits:\n');
        for k = 1:size(tab, 1)
            fprintf('       %-24s base %-16s read-back %-16s %s\n', tab{k,1}, tab{k,2}, tab{k,3}, tab{k,4});
        end
        fprintf('       control A_q3 = 10: 1-R^2 q3 %.6e (base %.6e) -> %s; slopes %.6f / %.6f\n', ...
                Rc.q3.oneMinusR2, B.q3.oneMinusR2, local_tf(ctrlDiff, 'differs', 'SAME'), Rc.q3.slope, Rc.d4.slope);
        ACC.a = struct('pass', same && ctrlDiff && ctrlSlope, 'readback_identical', same, ...
                       'control_differs', ctrlDiff, 'control_slopes_ok', ctrlSlope, 'table', {tab});
        fprintf('   (a) -> %s\n', local_pf(ACC.a.pass));
        if ~ACC.a.pass
            fprintf(['   STOP (spec failure interpretation (a)): parameter not connected or the copy differs.\n' ...
                     '   Spot checks are not run.\n']);
        end

        % ---- (b) spot checks
        C = [-40 7.3; -20 7.3; -10 7.3; -3 7.3; 0 7.3; 10 7.3; 20 7.3; 45 7.3; 0 4.5; 0 8.5];
        if o.SmokeTest, C = [0 7.3]; end
        spot = struct([]);
        if ACC.a.pass
            for k = 1:size(C, 1)
                q3c = C(k,1);  d4c = C(k,2);
                if q3c < 0, qs = sprintf('m%d', -q3c); else, qs = sprintf('p%d', q3c); end
                cn  = sprintf('MS02P05_%s_spot_q3%s_d4_%s', stamp, qs, strrep(sprintf('%.1f', d4c), '.', 'p'));
                args = {'Amplitude', 100e3, 'Frequency', 2*pi/9, 'Fx0', 0, 'Station', 1.5, ...
                        'CrowdAmplitude', 0.25, 'CrowdBias', d4c, 'TrimStart', 0, ...
                        'StopTime', local_f(bo,'StopTime',30), 'RelTol', local_f(bo,'RelTol',1e-10), ...
                        'AbsTol', local_f(bo,'AbsTol',1e-12)};
                fprintf('\n--- spot %d/%d: q3 centre %+d deg, d4 centre %.1f m ---\n', k, size(C,1), q3c, d4c);
                try
                    Rk = local_run(mdl, mw, 2, q3c, 0.5, args, cn, outDir, doSave);
                    spot = local_spot(spot, k, q3c, d4c, Rk, floor3, floor4, '');
                catch ME
                    fprintf(2, '   spot %d ERROR: %s\n', k, ME.message);
                    spot = local_spot(spot, k, q3c, d4c, [], floor3, floor4, ME.message);
                end
            end
            fprintf('\n   %-8s %-6s %-10s %-10s %-12s %-12s %-10s %-10s %-9s %-9s %s\n', 'q3 [deg]', 'd4 [m]', ...
                    'slope q3', 'slope d4', '1-R2 q3', '1-R2 d4', 'err/eff q3', 'err/eff d4', 'q3 mid', 'q3 half', 'result');
            for k = 1:numel(spot)
                S = spot(k);
                fprintf('   %+8d %6.1f %10.6f %10.6f %12.4e %12.4e %10.2e %10.2e %9.3f %9.4f %s %s\n', ...
                        S.q3c, S.d4c, S.slope(1), S.slope(2), S.oneMinusR2(1), S.oneMinusR2(2), ...
                        S.rel(1), S.rel(2), S.q3mid_deg, S.q3half_deg, local_pf(S.pass), S.err);
            end
            ACC.b = struct('pass', all([spot.pass]), 'n', numel(spot), 'n_pass', sum([spot.pass]));
            fprintf('   (b) %d of %d centres pass -> %s\n', ACC.b.n_pass, ACC.b.n, local_pf(ACC.b.pass));
            p6 = all(arrayfun(@(S) strcmp(sprintf('%.6f', S.slope(1)), '-1.000000') && ...
                                   strcmp(sprintf('%.6f', S.slope(2)), '-1.000000'), spot));
            R.predictions.P6 = struct('holds', p6);
            fprintf('   P6  slope -1.000000 on both channels at every centre, including q3 = 0 -> %s\n', local_hr(p6));
        else
            ACC.b = struct('pass', false, 'n', 0, 'n_pass', 0);
        end
        R.acc_a = ACC.a;  R.acc_b = ACC.b;  R.spot = spot;  R.model_md5 = md5now;
        R.module01_relfloor = [floor3 floor4];
    end

    % =====================================================================
    % summary
    % =====================================================================
    fprintf('\n=== MS02P05 SUMMARY (%s) ===\n', stamp);
    names = {'acc_a','(a) read-back + control'; 'acc_b','(b) spot checks'; ...
             'acc_c','(c) det zero locus'; 'acc_d','(d) 12-case code check'};
    allPass = true;
    for k = 1:size(names, 1)
        if isfield(R, names{k,1})
            fprintf('   %-26s %s\n', names{k,2}, local_pf(R.(names{k,1}).pass));
            allPass = allPass && R.(names{k,1}).pass;
        else
            fprintf('   %-26s not run\n', names{k,2});
        end
    end
    if isfield(R, 'predictions')
        pn = fieldnames(R.predictions);
        for k = 1:numel(pn)
            fprintf('   %-26s %s\n', ['prediction ' pn{k}], local_hr(R.predictions.(pn{k}).holds));
        end
    end
    fprintf('   Validity of the module = acceptance (a)-(d). Predictions may be refuted; a refutation is a result.\n');
    complete = all(isfield(R, names(:,1)')) && ~o.SmokeTest;
    R.allAcceptancePass = allPass && complete;
    if complete
        fprintf('OVERALL acceptance: %s\n', local_pf(allPass));
    else
        fprintf('OVERALL acceptance: INCOMPLETE (smoke test or not every part run; this is not a module result)\n');
    end

    if doSave
        f = fullfile(outDir, ['MS02P05_' stamp '.mat']);
        save(f, 'R', '-v7.3');
        fprintf('Saved %s\nLog   %s\n', f, logFile);
    end
end

% =========================================================================
function EN = local_envelopes()
% Envelopes exactly as in the frozen spec (module 05, Setup).
    EN.E0.q3      = [30 60];      EN.E0.d4_wide = [6.75 10.75];  EN.E0.d4_bi = [9.50 10.04];
    EN.E0.status  = 'design choice, modules 02-04';
    EN.E1.q3_lift = [-21 -3];     EN.E1.q3_swing = [-22 -18];    EN.E1.d4 = [7.05 7.60];
    EN.E1.q3d_max = 0.080;        EN.E1.d4dd_max = 0.2;
    EN.E1.status  = 'Rasuli 2014 Figs 7-9 (read from figures), d4 converted to project datum';
    EN.E2.q3      = [-45 25];     EN.E2.d4 = [4.2 8.8];
    EN.E2.q3d_max = 0.45;         EN.E2.d4dd_max = 0.5;
    EN.E2.status  = 'design choice / estimate; |d4dot| <= 0.54 m/s OEM; d4ddot Bi 2020 (WK-55)';
end

function th = local_angle(a1, a2, b1, b2)
% Angle [deg] between the 2-vectors [a1 a2] and [b1 b2], elementwise (0..90 deg).
    num = abs(a1.*b1 + a2.*b2);
    den = hypot(a1, a2) .* hypot(b1, b2);
    th  = acosd(min(1, num ./ den));
    th(den == 0) = NaN;
end

function m = local_in(Qd, D, qr, dr)
    m = Qd >= qr(1) - 1e-9 & Qd <= qr(2) + 1e-9 & D >= dr(1) - 1e-9 & D <= dr(2) + 1e-9;
end

function A = local_codecheck(root, P)
% Acceptance (d): recompute the 12-case table of m2_formal_identifiability_svd.m
% (same archives, same trim, same column definitions, s_bar = 10 m) and compare
% cond_joint with M2/experiments/06_identifiability/M2_formal_svd_summary.mat.
    dd = fullfile(root, 'M2', 'experiments');
    cases = {
        'Wide_w0.314',      fullfile(dd, '04_frequency_sweep', 'M2_04_w0.314.mat');
        'Wide_w0.500',      fullfile(dd, '04_frequency_sweep', 'M2_04_w0.500.mat');
        'Wide_w0.700',      fullfile(dd, '04_frequency_sweep', 'M2_04_w0.700.mat');
        'Wide_w1.000',      fullfile(dd, '04_frequency_sweep', 'M2_04_w1.000.mat');
        'Wide_w1.360',      fullfile(dd, '04_frequency_sweep', 'M2_04_w1.360.mat');
        'Narrow_BI_w0.314', fullfile(dd, '04_frequency_sweep', 'M2_04n_w0.314.mat');
        'Narrow_BI_w0.500', fullfile(dd, '04_frequency_sweep', 'M2_04n_w0.500.mat');
        'Narrow_BI_w0.700', fullfile(dd, '04_frequency_sweep', 'M2_04n_w0.700.mat');
        'Narrow_BI_w1.000', fullfile(dd, '04_frequency_sweep', 'M2_04n_w1.000.mat');
        'Narrow_BI_w1.360', fullfile(dd, '04_frequency_sweep', 'M2_04n_w1.360.mat');
        'Narrow_Centered',  fullfile(dd, '06_identifiability', 'M2_wrongstation_narrowband.mat');
        'Medium_Stroke_1m', fullfile(dd, '00_design',          'M2_crowdparam_control.mat')};
    ref = load(fullfile(dd, '06_identifiability', 'M2_formal_svd_summary.mat'), 'S_out');
    ref = ref.S_out;
    fprintf('   (d) code check vs M2_formal_svd_summary.mat (s_bar = 10 m):\n');
    fprintf('       %-18s %9s %10s %10s %8s\n', 'case', 'cond_2col', 'cond_joint', 'archived', 'rel diff');
    n = size(cases, 1);
    T = struct('case', cases(:,1), 'cond2', NaN, 'cond_joint', NaN, 'archived', NaN, 'reldiff', NaN);
    for k = 1:n
        tmp = load(cases{k,2}, 'R');  Rk = tmp.R;
        idx  = Rk.t >= Rk.opt.TrimStart - 1e-9;
        q3   = Rk.sig.q3(idx);   q3d = Rk.sig.q3d(idx);  q3dd = Rk.sig.q3dd(idx);
        d4   = Rk.sig.d4(idx);   d4d = Rk.sig.d4d(idx);  d4dd = Rk.sig.d4dd(idx);
        r = d4 + P.c;  s = d4 + 1.50;
        F3 = s .* cos(q3);
        M3 = (r.^2).*q3dd + 2*r.*d4d.*q3d + P.g*r.*cos(q3);
        F4 = sin(q3);
        M4 = d4dd - r.*(q3d.^2) + P.g*sin(q3);
        S2  = [F3 - mean(F3), M3 - mean(M3)];
        S2u = S2 ./ vecnorm(S2, 2, 1);   sv2 = svd(S2u);
        Sj  = [S2; 10.0 * [F4 - mean(F4), M4 - mean(M4)]];
        Sju = Sj ./ vecnorm(Sj, 2, 1);   svj = svd(Sju);
        T(k).cond2 = sv2(1)/sv2(end);  T(k).cond_joint = svj(1)/svj(end);
        T(k).archived = ref.(matlab.lang.makeValidName(cases{k,1})).cond_joint_2ch;
        T(k).reldiff  = abs(T(k).cond_joint - T(k).archived) / T(k).archived;
        fprintf('       %-18s %9.2f %10.2f %10.2f %8.1e\n', cases{k,1}, T(k).cond2, ...
                T(k).cond_joint, T(k).archived, T(k).reldiff);
    end
    A = struct('pass', all([T.reldiff] <= 0.01), 'table', T);
    fprintf('   (d) all 12 within 1 %% -> %s\n', local_pf(A.pass));
end

function Rk = local_run(mdl, mw, Aq3, Bq3, w, args, caseName, outDir, doSave)
% Set the handle-angle trajectory in the model workspace, read it back, run recover_load.
    mw.assignin('A_q3', Aq3);  mw.assignin('B_q3', Bq3);  mw.assignin('omega_traj', w);
    if mw.getVariable('A_q3') ~= Aq3 || mw.getVariable('B_q3') ~= Bq3 || mw.getVariable('omega_traj') ~= w
        error('P05:mwWrite', 'Model-workspace write of A_q3/B_q3/omega_traj did not take.');
    end
    fprintf('   model workspace: A_q3 %g deg, B_q3 %g deg, omega_traj %.12g rad/s\n', Aq3, Bq3, w);
    Rk = recover_load('Model', mdl, args{:}, 'CaseName', caseName, 'OutDir', outDir, 'Save', doSave);
    if mw.getVariable('A_q3') ~= Aq3 || mw.getVariable('B_q3') ~= Bq3 || mw.getVariable('omega_traj') ~= w
        error('P05:mwChanged', 'Model-workspace trajectory variables changed during the run.');
    end
end

function [same, tab] = local_compare(B, Ra)
% Acceptance (a): compare every metric recover_load prints, in its printed format.
    f = {'slope','%.6f'; 'intercept','%.3f'; 'R2','%.6f'; 'oneMinusR2','%.6e'; 'NRMSE_pct','%.4f'; ...
         'relMaxErr','%.3e'; 'peakMagErr_pct','%.4f'; 'peakTimeErr_s','%.6f'; 'impliedLag_s','%.6f'};
    ch = {'q3','d4'};
    tab = cell(0, 4);  same = true;  nCompared = 0;
    for i = 1:2
        for k = 1:size(f, 1)
            nm = [ch{i} '.' f{k,1}];
            if isfield(B.(ch{i}), f{k,1}) && isfield(Ra.(ch{i}), f{k,1})
                sb = sprintf(f{k,2}, B.(ch{i}).(f{k,1}));
                sr = sprintf(f{k,2}, Ra.(ch{i}).(f{k,1}));
                ok = strcmp(sb, sr);  same = same && ok;  nCompared = nCompared + 1;
                tab(end+1, :) = {nm, sb, sr, local_tf(ok, 'identical', 'DIFFERENT')}; %#ok<AGROW>
            else
                tab(end+1, :) = {nm, '-', '-', 'not in baseline'}; %#ok<AGROW>
            end
        end
    end
    same = same && nCompared >= 8;
end

function spot = local_spot(spot, k, q3c, d4c, Rk, floor3, floor4, err)
    S = struct('q3c', q3c, 'd4c', d4c, 'err', err, 'slope', [NaN NaN], 'intercept', [NaN NaN], ...
               'oneMinusR2', [NaN NaN], 'relMaxErr', [NaN NaN], 'rel', [NaN NaN], ...
               'q3mid_deg', NaN, 'q3half_deg', NaN, 'd4mid', NaN, 'd4half', NaN, ...
               'pass', false, 'file', '');
    if ~isempty(Rk)
        idx = logical(Rk.trimIdx);
        eff3 = max(abs(Rk.sig.tau3_model(idx)));  eff4 = max(abs(Rk.sig.f4_model(idx)));
        S.slope      = [Rk.q3.slope, Rk.d4.slope];
        S.intercept  = [Rk.q3.intercept, Rk.d4.intercept];
        S.oneMinusR2 = [Rk.q3.oneMinusR2, Rk.d4.oneMinusR2];
        S.relMaxErr  = [Rk.q3.relMaxErr, Rk.d4.relMaxErr];
        S.rel        = [Rk.q3.maxAbsErr/eff3, Rk.d4.maxAbsErr/eff4];
        q = rad2deg(Rk.sig.q3(idx));  d = Rk.sig.d4(idx);
        S.q3mid_deg = (max(q)+min(q))/2;  S.q3half_deg = (max(q)-min(q))/2;
        S.d4mid = (max(d)+min(d))/2;      S.d4half = (max(d)-min(d))/2;
        slopeOK = strcmp(sprintf('%.6f', S.slope(1)), '-1.000000') && strcmp(sprintf('%.6f', S.slope(2)), '-1.000000');
        floorOK = S.rel(1) <= 10*floor3 && S.rel(2) <= 10*floor4;
        S.pass = slopeOK && floorOK;
        if isfield(Rk, 'file'), S.file = Rk.file; end
    end
    if isempty(spot), spot = S; else, spot(k) = S; end
end

function local_figures(M, EN, outDir, stamp)
% Map 1-4 figures with the three envelopes overlaid.
    q = M.q3_deg;  d = M.d4_m;
    F = {abs(M.sF3), '|s_{F3}| = (d_4+1.5) cos q_3  [m]', 'map1';
         abs(M.sF4), '|s_{F4}| = |sin q_3|  [-]',          'map2'};
    for k = 1:2
        fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 560]);
        imagesc(q, d, F{k,1}'); axis xy; colorbar; hold on;
        local_env(EN); xlabel('q_3 [deg]'); ylabel('d_4 [m]'); title(['MS02P05 Map ' num2str(k) ': ' F{k,2}]);
        exportgraphics(fig, fullfile(outDir, sprintf('MS02P05_%s_%s.png', F{k,3}, stamp)), 'Resolution', 150);
        close(fig);
    end
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 800]);
    subplot(2,2,1); imagesc(q, d, abs(M.det)'); axis xy; colorbar; hold on; local_env(EN);
    title('|det| quasi-static [m^2/s^2]'); xlabel('q_3 [deg]'); ylabel('d_4 [m]');
    for k = 1:numel(M.Ln_m)
        subplot(2,2,k+1); imagesc(q, d, M.thetaW_deg(:,:,k)'); axis xy; colorbar; hold on; local_env(EN);
        title(sprintf('\\theta_W [deg], L_n = %g m', M.Ln_m(k))); xlabel('q_3 [deg]'); ylabel('d_4 [m]');
    end
    sgtitle('MS02P05 Map 3: quasi-static separability of load vs mass error');
    exportgraphics(fig, fullfile(outDir, sprintf('MS02P05_map3_%s.png', stamp)), 'Resolution', 150);
    close(fig);
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 500]);
    yl = [0, 1.1*max(100*M.map4.E2)];
    patch([EN.E2.d4 fliplr(EN.E2.d4)], [yl(1) yl(1) yl(2) yl(2)], [0.8 0.4 0.2], 'FaceAlpha', 0.08, 'EdgeColor', 'none'); hold on;
    patch([EN.E1.d4 fliplr(EN.E1.d4)], [yl(1) yl(1) yl(2) yl(2)], [0.2 0.4 0.8], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    plot(d, 100*M.map4.E1, 'b', 'LineWidth', 1.5); plot(d, 100*M.map4.E2, 'r', 'LineWidth', 1.5);
    ylim(yl); grid on; xlabel('d_4 [m]'); ylabel('max |d4ddot - r q3dot^2| / g  [% of g]');
    legend('E2 d_4 range', 'E1 d_4 range', 'E1 rates (Rasuli 2014)', 'E2 limits (WK-20)', 'Location', 'northwest');
    title('MS02P05 Map 4: dynamic separability bound near q_3 = 0');
    exportgraphics(fig, fullfile(outDir, sprintf('MS02P05_map4_%s.png', stamp)), 'Resolution', 150);
    close(fig);
end

function local_env(EN)
    rectangle('Position', [EN.E0.q3(1) EN.E0.d4_wide(1) diff(EN.E0.q3) diff(EN.E0.d4_wide)], 'EdgeColor', 'w', 'LineWidth', 1.2);
    rectangle('Position', [EN.E0.q3(1) EN.E0.d4_bi(1) diff(EN.E0.q3) diff(EN.E0.d4_bi)], 'EdgeColor', 'w', 'LineStyle', ':');
    rectangle('Position', [EN.E1.q3_lift(1) EN.E1.d4(1) diff(EN.E1.q3_lift) diff(EN.E1.d4)], 'EdgeColor', 'r', 'LineWidth', 1.5);
    rectangle('Position', [EN.E1.q3_swing(1) EN.E1.d4(1) diff(EN.E1.q3_swing) diff(EN.E1.d4)], 'EdgeColor', 'm', 'LineWidth', 1.5);
    rectangle('Position', [EN.E2.q3(1) EN.E2.d4(1) diff(EN.E2.q3) diff(EN.E2.d4)], 'EdgeColor', 'k', 'LineWidth', 1.5, 'LineStyle', '--');
    xline(0, 'y-', 'LineWidth', 1);
    text(EN.E0.q3(1), EN.E0.d4_wide(2) + 0.15, 'E0', 'Color', 'w');
    text(EN.E1.q3_lift(1), EN.E1.d4(2) + 0.15, 'E1', 'Color', 'r');
    text(EN.E2.q3(1), EN.E2.d4(2) + 0.15, 'E2', 'Color', 'k');
end

function v = local_f(s, name, dflt)
    if isfield(s, name), v = s.(name); else, v = dflt; end
end

function local_close(mdl)
    if bdIsLoaded(mdl), close_system(mdl, 0); end     % discard model-workspace changes
end

function h = local_md5(f)
    fid = fopen(f, 'r'); b = fread(fid, inf, '*uint8'); fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end

function s = local_pf(b),  s = local_tf(b, 'PASS', 'FAIL');         end
function s = local_hr(b),  s = local_tf(b, 'HOLDS', 'REFUTED');     end
function s = local_tf(b, t, f)
    if b, s = t; else, s = f; end
end
