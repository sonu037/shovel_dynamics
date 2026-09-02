function R = recover_load(varargin)
%RECOVER_LOAD  M2 residual load-recovery analysis for the cable shovel model.
%
%   Runs the M2 load-injection model, forms the residual against the frozen
%   M1 analytical model, maps the known injected force through the Jacobian,
%   and computes the module-01 / module-02 acceptance metrics.
%
%   R = RECOVER_LOAD() runs the default case (100 kN, 1/9 Hz, bail station).
%
%   R = RECOVER_LOAD('Name',Value,...) accepts:
%
%     'Amplitude'   force amplitude F0 [N].  Set to 0 for module 01
%                   (zero-force gate).  Default 100e3.
%     'Frequency'   force angular frequency [rad/s].  Default 2*pi/9.
%     'Fx0'         amplitude of the x-component [N].  Default 0.
%     'Station'     along-handle offset k such that s = d_4 + k [m].
%                   1.5   = bail station   (module 02, observable arm)
%                  -1.32  = COG station    (negative-control arm)
%                   Default 1.5.
%     'TrimStart'   discard data before this time [s].  Default 5.
%     'StopTime'    simulation stop time [s].  Default 30.
%     'RelTol'      solver relative tolerance.  Default 1e-10.
%     'AbsTol'      solver absolute tolerance.  Default 1e-12.
%     'Model'       model name.  Default 'M2_LoadInjection'.
%     'CaseName'    label used for the saved .mat.  Default auto-generated.
%     'OutDir'      results directory.  Default pwd.
%     'Save'        true/false.  Default true.
%
%   SIGN CONVENTION
%     The equation of motion is  D qdd + C qd + G + f = tau_actuator + Q_ext,
%     so  tau_actuator = tau_model - Q_ext  and therefore
%
%         residual  =  tau_measured - tau_model  =  -J' F
%
%     The reference signal against which the residual is compared is
%     -Q_pred, and the expected regression slope of residual on Q_pred is -1.
%
%   MODEL PREREQUISITES
%     The Simulink model must expose these workspace variables:
%       F0_x, F0_y   force amplitudes referenced by the source blocks
%       omega_F      force angular frequency
%     and should log the APPLIED force to the output as 'outF'.  If 'outF'
%     is absent the commanded force is reconstructed analytically and a
%     warning is issued -- results in that state are NOT a valid module-02
%     run, because any delay in the force input path appears as phase error.
%
%   Repository:  M2/scripts/recover_load.m
%
%   See also SHOVEL_PARAMS.

% -------------------------------------------------------------------------
% 1. Arguments
% -------------------------------------------------------------------------
ip = inputParser;
ip.addParameter('Amplitude', 100e3,        @(x)isscalar(x) && isnumeric(x));
ip.addParameter('Frequency', 2*pi/9,       @(x)isscalar(x) && isnumeric(x));
ip.addParameter('Fx0',       0,            @(x)isscalar(x) && isnumeric(x));
ip.addParameter('Station',   1.5,          @(x)isscalar(x) && isnumeric(x));
ip.addParameter('TrimStart', 5,            @(x)isscalar(x) && isnumeric(x));
ip.addParameter('StopTime',  30,           @(x)isscalar(x) && isnumeric(x));
ip.addParameter('RelTol',    1e-10,        @(x)isscalar(x) && isnumeric(x));
ip.addParameter('AbsTol',    1e-12,        @(x)isscalar(x) && isnumeric(x));
ip.addParameter('Model',     'M2_LoadInjection', @ischar);
ip.addParameter('CaseName',  '',           @ischar);
ip.addParameter('OutDir',    pwd,          @ischar);
ip.addParameter('Save',      true,         @islogical);
ip.parse(varargin{:});
opt = ip.Results;

isNull = (opt.Amplitude == 0) && (opt.Fx0 == 0);

if isempty(opt.CaseName)
    if isNull
        opt.CaseName = 'M2_null';
    else
        opt.CaseName = sprintf('M2_%gkN_%.4fHz_s%+.2f', ...
            opt.Amplitude/1e3, opt.Frequency/(2*pi), opt.Station);
        opt.CaseName = strrep(strrep(opt.CaseName,'.','p'),'+','p');
    end
end

% -------------------------------------------------------------------------
% 2. Parameters -- single source of truth
% -------------------------------------------------------------------------
p = local_get_params();
local_check_params(p);

% -------------------------------------------------------------------------
% 3. Run the model
% -------------------------------------------------------------------------
if ~bdIsLoaded(opt.Model), load_system(opt.Model); end

si = Simulink.SimulationInput(opt.Model);
% Model-workspace variables SHADOW Simulink.SimulationInput.setVariable.
% F0_x/F0_y/omega_F were placed in the model workspace on 2026-08-28 to make
% the model self-contained; from that point setVariable was silently ignored.
% This produced six identical "amplitude sweep" runs on 2026-09-02, all at
% 100 kN, labelled 10-1000 kN. Write to the model workspace directly, then
% read back to confirm.
mw = get_param(opt.Model,'ModelWorkspace');
mw.assignin('F0_y',    opt.Amplitude);
mw.assignin('F0_x',    opt.Fx0);
mw.assignin('omega_F', opt.Frequency);

if mw.getVariable('F0_y') ~= opt.Amplitude || ...
   mw.getVariable('F0_x') ~= opt.Fx0 || ...
   mw.getVariable('omega_F') ~= opt.Frequency
    error('recover_load:inputNotApplied', ...
        ['Model-workspace write did not take. F0_y %g (requested %g), ' ...
         'F0_x %g (requested %g), omega_F %g (requested %g).'], ...
        mw.getVariable('F0_y'), opt.Amplitude, ...
        mw.getVariable('F0_x'), opt.Fx0, ...
        mw.getVariable('omega_F'), opt.Frequency);
end
si = si.setModelParameter('StopTime', num2str(opt.StopTime));
si = si.setModelParameter('RelTol',   sprintf('%g', opt.RelTol));
si = si.setModelParameter('AbsTol',   sprintf('%g', opt.AbsTol));

out = sim(si);

% -------------------------------------------------------------------------
% 4. Extract signals and align lengths
% -------------------------------------------------------------------------
names = {'q_3','q_3_dot','q_3_ddot','d_4','d_4_dot','d_4_ddot', ...
         'tau_3','f_4'};
S = struct();
for k = 1:numel(names)
    S.(names{k}) = local_sig(out, names{k});
end
tRaw = local_sig(out, 'tout');

[t, S, alignNote] = local_align(tRaw, S, names, opt.StopTime);

q3        = S.q_3;
q3d       = S.q_3_dot;
q3dd      = S.q_3_ddot;
d4        = S.d_4;
d4d       = S.d_4_dot;
d4dd      = S.d_4_ddot;
tau3_meas = S.tau_3;
f4_meas   = S.f_4;

% -------------------------------------------------------------------------
% 5. Frozen M1 analytical model  (parameters from shovel_params.m only)
% -------------------------------------------------------------------------
r = d4 + p.c;                                  % lever arm to the COG

tau3_model = (p.I_zz + p.M_d.*r.^2).*q3dd ...
           + 2*p.M_d.*r.*d4d.*q3d ...
           + p.M_d*p.g.*r.*cos(q3) ...
           + p.f_v3.*q3d;

f4_model   = p.M_d.*d4dd ...
           - p.M_d.*r.*q3d.^2 ...
           + p.M_d*p.g.*sin(q3) ...
           + p.f_v4.*d4d;

e_tau3 = tau3_meas - tau3_model;
e_f4   = f4_meas   - f4_model;

% -------------------------------------------------------------------------
% 6. Ground-truth force:  prefer the LOGGED APPLIED force
% -------------------------------------------------------------------------
[Fx, Fy, forceSource] = local_get_force(out, t, opt);

% -------------------------------------------------------------------------
% 7. Jacobian mapping   Q = J' F,   s = d_4 + Station
% -------------------------------------------------------------------------
s = d4 + opt.Station;

Qq3_pred = s .* (Fy.*cos(q3) - Fx.*sin(q3));   % N m
Qd4_pred = Fx.*cos(q3) + Fy.*sin(q3);          % N

ref3 = -Qq3_pred;
ref4 = -Qd4_pred;

% -------------------------------------------------------------------------
% 8. Trim
% -------------------------------------------------------------------------
idx = t >= opt.TrimStart;
if ~any(idx)
    error('recover_load:trim','TrimStart exceeds the simulation length.');
end

% -------------------------------------------------------------------------
% 9. Metrics
% -------------------------------------------------------------------------
R = struct();
R.case      = opt.CaseName;
R.isNull    = isNull;
R.opt       = opt;
R.params    = p;
R.forceSrc  = forceSource;
R.alignNote = alignNote;
R.t         = t;
R.trimIdx   = idx;
R.nSamples  = numel(t);
R.nTrimmed  = sum(idx);

if isNull
    R.q3.maxAbsResidual = max(abs(e_tau3(idx)));
    R.q3.rmsResidual    = sqrt(mean(e_tau3(idx).^2));
    R.q3.torqueScale    = max(abs(tau3_model(idx)));
    R.q3.relFloor       = R.q3.maxAbsResidual / R.q3.torqueScale;

    R.d4.maxAbsResidual = max(abs(e_f4(idx)));
    R.d4.rmsResidual    = sqrt(mean(e_f4(idx).^2));
    R.d4.forceScale     = max(abs(f4_model(idx)));
    R.d4.relFloor       = R.d4.maxAbsResidual / R.d4.forceScale;
else
    R.q3 = local_metrics(Qq3_pred(idx), ref3(idx), e_tau3(idx), ...
                         t(idx), opt.Frequency);
    R.d4 = local_metrics(Qd4_pred(idx), ref4(idx), e_f4(idx), ...
                         t(idx), opt.Frequency);
end

R.sig = struct('q3',q3,'d4',d4,'q3d',q3d,'d4d',d4d, ...
               'q3dd',q3dd,'d4dd',d4dd, ...
               'tau3_meas',tau3_meas,'tau3_model',tau3_model,'e_tau3',e_tau3, ...
               'f4_meas',f4_meas,'f4_model',f4_model,'e_f4',e_f4, ...
               'Fx',Fx,'Fy',Fy,'Qq3_pred',Qq3_pred,'Qd4_pred',Qd4_pred, ...
               'ref3',ref3,'ref4',ref4,'s',s,'r',r);

% -------------------------------------------------------------------------
% 10. Provenance
% -------------------------------------------------------------------------
R.prov = local_provenance(opt);

% -------------------------------------------------------------------------
% 11. Report and save
% -------------------------------------------------------------------------
local_report(R);

if opt.Save
    if ~exist(opt.OutDir,'dir'), mkdir(opt.OutDir); end
    fn = fullfile(opt.OutDir, [opt.CaseName '.mat']);
    save(fn, 'R', '-v7.3');
    fprintf('Saved: %s\n', fn);
    R.file = fn;
end

end % ===================== end main =====================


% =========================================================================
%  Local functions
% =========================================================================

function v = local_sig(out, name)
%LOCAL_SIG  Fetch one logged signal as a column vector.
try
    raw = out.(name);
catch
    error('recover_load:missingSignal', ...
        ['Signal "%s" is not in the simulation output. Check the To ' ...
         'Workspace / outport names in the model.'], name);
end
if isa(raw,'timeseries')
    v = raw.Data(:);
elseif isnumeric(raw)
    v = raw(:);
else
    error('recover_load:badSignal', ...
        'Signal "%s" has unsupported class %s.', name, class(raw));
end
end


function [t, S, note] = local_align(tRaw, S, names, stopTime)
%LOCAL_ALIGN  Reconcile a time vector that disagrees with the signal length.
%
%   Under tight solver tolerances the solver time vector can grow while
%   To Workspace blocks logging at a fixed sample time do not. Rather than
%   fail, reconstruct a uniform time base matching the signals and say so.

L = zeros(1,numel(names));
for k = 1:numel(names)
    L(k) = numel(S.(names{k}));
end

if any(L ~= L(1))
    n = min(L);
    warning('recover_load:signalLengths', ...
        ['Logged signals have differing lengths (%s). Truncating all to ' ...
         '%d samples. Give every To Workspace block the same sample ' ...
         'time before trusting these numbers.'], mat2str(L), n);
    for k = 1:numel(names)
        S.(names{k}) = S.(names{k})(1:n);
    end
else
    n = L(1);
end

if numel(tRaw) == n
    t = tRaw;
    note = 'time taken from tout';
elseif numel(tRaw) > n
    t = linspace(0, stopTime, n).';
    note = sprintf(['tout had %d points against %d signal samples; a ' ...
        'uniform time base was reconstructed over 0 to %g s. This is ' ...
        'correct only if the outports log at a fixed sample time.'], ...
        numel(tRaw), n, stopTime);
    warning('recover_load:timeBase','%s', note);
else
    n = numel(tRaw);
    for k = 1:numel(names)
        S.(names{k}) = S.(names{k})(1:n);
    end
    t = tRaw;
    note = sprintf('signals truncated to the %d points in tout', n);
    warning('recover_load:timeBase','%s', note);
end
end


function p = local_get_params()
%LOCAL_GET_PARAMS  Pull the frozen M1 parameters from shovel_params.m.

if exist('shovel_params','file') ~= 2
    error('recover_load:noParams', ...
        ['shovel_params.m not found on the path. Add the repository ' ...
         'root to the MATLAB path before running.']);
end

try
    cand = shovel_params();
    if isstruct(cand)
        p = local_normalise_params(cand);
        return
    end
catch
end

run('shovel_params');
vars = who;
tmp = struct();
for k = 1:numel(vars)
    tmp.(vars{k}) = eval(vars{k});   %#ok<EV2IN>
end
p = local_normalise_params(tmp);
end


function p = local_normalise_params(sIn)
%LOCAL_NORMALISE_PARAMS  Map whatever names are used onto canonical ones.

alias = { ...
    'M_d',  {'M_d','Md','m','m_d','mass_dipper_handle'}; ...
    'I_zz', {'I_zz','Izz','I_zz_cog','inertia_zz'}; ...
    'c',    {'c','c_offset','lever_const'}; ...
    'f_v3', {'f_v3','fv3','b3','damp_saddle'}; ...
    'f_v4', {'f_v4','fv4','b4','damp_crowd'}; ...
    'g',    {'g','g0','gravity'} };

p = struct();
for k = 1:size(alias,1)
    canon = alias{k,1};
    nm    = alias{k,2};
    found = false;
    for j = 1:numel(nm)
        if isfield(sIn, nm{j})
            p.(canon) = sIn.(nm{j});
            found = true;
            break
        end
    end
    if ~found
        error('recover_load:missingParam', ...
            ['Parameter "%s" not found in shovel_params.m. Add it, or ' ...
             'add its actual name to the alias table in ' ...
             'local_normalise_params.'], canon);
    end
end
end


function local_check_params(p)
%LOCAL_CHECK_PARAMS  Warn if parameters drift from the M2 nomenclature.
%   These reference values are for CHECKING ONLY and never enter a
%   computation.

ref = struct('M_d',54300,'I_zz',287900,'c',-1.32, ...
             'f_v3',3800,'f_v4',1473,'g',9.80665);
fn = fieldnames(ref);
for k = 1:numel(fn)
    if abs(p.(fn{k}) - ref.(fn{k})) > 1e-6*max(1,abs(ref.(fn{k})))
        warning('recover_load:paramDrift', ...
            ['%s = %.6g differs from the M2 nomenclature value %.6g. ' ...
             'Confirm this is intended and record it in research_log.md.'], ...
            fn{k}, p.(fn{k}), ref.(fn{k}));
    end
end
end


function [Fx, Fy, src] = local_get_force(out, t, opt)
%LOCAL_GET_FORCE  Applied force if logged, otherwise commanded (with warning).

hasF = false;
try
    hasF = any(strcmp('outF', out.who));
catch
    try
        hasF = isprop(out,'outF');
    catch
    end
end

if hasF
    F = local_sig(out,'outF');
    if numel(F) == 2*numel(t)
        F  = reshape(F, [], 2);
        Fx = F(:,1); Fy = F(:,2);
    else
        Fx = zeros(size(t));
        Fy = F(1:min(numel(F),numel(t)));
        Fy(end+1:numel(t)) = 0;
    end
    src = 'logged applied force (outF)';
else
    Fx = opt.Fx0       * sin(opt.Frequency * t);
    Fy = opt.Amplitude * sin(opt.Frequency * t);
    src = 'RECONSTRUCTED command (outF not logged) -- NOT a valid module-02 run';
    warning('recover_load:noAppliedForce', ...
        ['outF is not logged, so the COMMANDED force is being used as ' ...
         'ground truth. Any filtering or delay in the force input path ' ...
         'will appear as a phase error in the metrics. Log the applied ' ...
         'force before treating these numbers as a module-02 result.']);
end
end


function m = local_metrics(Qpred, ref, resid, tt, omega)
%LOCAL_METRICS  Regression, error and lag metrics for one channel.

pf          = polyfit(Qpred, resid, 1);
m.slope     = pf(1);
m.intercept = pf(2);
fit         = polyval(pf, Qpred);
ssRes       = sum((resid - fit).^2);
ssTot       = sum((resid - mean(resid)).^2);
m.R2        = 1 - ssRes/ssTot;
m.oneMinusR2 = ssRes/ssTot;
err         = resid - ref;
m.maxAbsErr = max(abs(err));
m.rmsErr    = sqrt(mean(err.^2));
m.refRange  = max(ref) - min(ref);
m.refPeak   = max(abs(ref));

m.NRMSE_pct = 100 * m.rmsErr / m.refRange;
m.relMaxErr = m.maxAbsErr / m.refPeak;

[pkRef, iRef] = max(abs(ref));
[pkRec, iRec] = max(abs(resid));
m.peakMagErr_pct = 100*abs(pkRec - pkRef)/pkRef;
m.peakTimeErr_s  = abs(tt(iRec) - tt(iRef));

rho = sqrt(max(min(m.R2,1),0));
m.impliedLag_s = acos(rho)/omega;
end


function prov = local_provenance(opt)
%LOCAL_PROVENANCE  Record enough to reproduce this result later.

prov.timestamp    = datestr(now,'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
prov.matlab       = version;
prov.host         = getenv('COMPUTERNAME');
prov.model        = opt.Model;
prov.solverRelTol = opt.RelTol;
prov.solverAbsTol = opt.AbsTol;
prov.script       = mfilename('fullpath');

prov.gitCommit = 'unknown';
prov.gitDirty  = 'unknown';
try
    [st, hash] = system('git rev-parse --short HEAD');
    if st == 0, prov.gitCommit = strtrim(hash); end
    [st, dirty] = system('git status --porcelain');
    if st == 0
        if isempty(strtrim(dirty))
            prov.gitDirty = 'clean';
        else
            prov.gitDirty = 'DIRTY - uncommitted changes present';
        end
    end
catch
end
end


function local_report(R)
%LOCAL_REPORT  Print the summary.

fprintf('\n============================================================\n');
fprintf('  %s\n', R.case);
fprintf('============================================================\n');
fprintf('Force source     : %s\n', R.forceSrc);
fprintf('Time base        : %s\n', R.alignNote);
fprintf('Station offset   : %+.2f m   (s = d_4 %+.2f)\n', ...
    R.opt.Station, R.opt.Station);
fprintf('Solver           : RelTol %g / AbsTol %g\n', ...
    R.opt.RelTol, R.opt.AbsTol);
fprintf('Samples          : %d total, %d after trim at %.1f s\n', ...
    R.nSamples, R.nTrimmed, R.opt.TrimStart);
fprintf('Git              : %s (%s)\n', R.prov.gitCommit, R.prov.gitDirty);

if R.isNull
    fprintf('\n--- MODULE 01: ZERO-FORCE FLOOR ---\n');
    fprintf('q3  max|residual| = %.6e N m   (%.3e of torque scale)\n', ...
        R.q3.maxAbsResidual, R.q3.relFloor);
    fprintf('q3  rms|residual| = %.6e N m\n', R.q3.rmsResidual);
    fprintf('d4  max|residual| = %.6e N     (%.3e of force scale)\n', ...
        R.d4.maxAbsResidual, R.d4.relFloor);
    fprintf('d4  rms|residual| = %.6e N\n', R.d4.rmsResidual);
    fprintf('\nThis is the floor. Module 02 cannot claim better.\n');
else
    fprintf('\n%-22s %14s %14s\n','','q3 channel','d4 channel');
    fprintf('%-22s %14.6f %14.6f\n','slope (expect -1)', ...
        R.q3.slope, R.d4.slope);
    fprintf('%-22s %14.3f %14.3f\n','intercept', ...
        R.q3.intercept, R.d4.intercept);
    fprintf('%-22s %14.6f %14.6f\n','R^2', R.q3.R2, R.d4.R2);
    fprintf('%-22s %14.6e %14.6e\n','1 - R^2 (direct)', ...
        R.q3.oneMinusR2, R.d4.oneMinusR2);
    fprintf('%-22s %14.4f %14.4f\n','NRMSE (%)', ...
        R.q3.NRMSE_pct, R.d4.NRMSE_pct);
    fprintf('%-22s %14.3e %14.3e\n','max err / peak ref', ...
        R.q3.relMaxErr, R.d4.relMaxErr);
    fprintf('%-22s %14.4f %14.4f\n','peak mag err (%)', ...
        R.q3.peakMagErr_pct, R.d4.peakMagErr_pct);
    fprintf('%-22s %14.6f %14.6f\n','peak time err (s)', ...
        R.q3.peakTimeErr_s, R.d4.peakTimeErr_s);
    fprintf('%-22s %14.6f %14.6f\n','implied lag (s)', ...
        R.q3.impliedLag_s, R.d4.impliedLag_s);

    lagGap = abs(R.q3.impliedLag_s - R.d4.impliedLag_s) / ...
             max(R.q3.impliedLag_s, eps);
    if R.q3.impliedLag_s > 1e-4 && lagGap < 0.05
        fprintf(['\nNOTE: both channels imply the same lag (~%.4f s). ' ...
                 'A COMMON lag cannot come from geometry, station or\n' ...
                 'Jacobian errors -- those corrupt the channels ' ...
                 'differently. Look at the shared force input path.\n'], ...
                 R.q3.impliedLag_s);
    end
end
fprintf('============================================================\n');
end