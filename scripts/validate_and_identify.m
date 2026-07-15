%% VALIDATE_AND_IDENTIFY.M   (v5.1 - function form)
%  Stage-1 Protocol: forward validation (BOTH equations) + linear-in-
%  parameters identification with uncertainty reporting.
%  Model: ShovelSimulator.slx (Revolute q_3 + Prismatic d_4)
%
%  FIXES vs validate_and_identify_4.m (this revision, H-series):
%   [H1] Declared run configuration block (T_TRIM, FORCE_GRADIENT),
%        printed at start of every run. Experimental modes are now
%        switches, not code edits.
%   [H2] TRIM BUG FIXED: v4 shipped with keep = true(size(t)) left over
%        from the 2026-07-14 ablation session -> the [G4] trim was
%        silently OFF in every run since. Comment claimed one thing,
%        code did another (the same disease [G1] fixed for parameters).
%   [H3] FORCE_GRADIENT switch: reproduces the C1 differentiation study
%        (Izz bias ~ -56%) on demand without touching the model. Both
%        acceleration branches honour it.
%   [H4] Run configuration (T_TRIM, FORCE_GRADIENT, script version) is
%        saved inside stage1_results.mat: every result now self-describes
%        the settings that produced it.
%   [H5] datestr(now) -> char(datetime("now")) (datestr is deprecated).
%   [H6] Covariance via (Y'*Y)\eye(p) instead of inv() (numerically
%        preferred idiom; identical result at cond(Y)~82).
%   [H7] Post-trim sanity guards: error if the trim empties the record
%        or if N <= p (covariance undefined).
%   [H8] (v5.1) Converted to a FUNCTION with name-value options via an
%        arguments block. No file edits needed to switch modes:
%        validate_and_identify(out, ForceGradient=true, T_TRIM=0).
%        Functions get a private workspace (clearvars obsolete) and the
%        signature enforces that out exists.
%
%  FIXES inherited from v4 (G-series):
%   [G1] Parameters from shovel_params.m (single source of truth).
%   [G2] Logged-signal detection uses out.who (isfield silently fails).
%   [G3] Forward validation includes the viscous term fv3*q_d.
%   [G4] All signals trimmed together (no mismatched lengths).
%   [G5] Section 7 validates the PRISMATIC (crowd) equation.
%   [G6] cond(Y) raw+scaled; standard errors / 95% CIs; delta method.
%   [G7] fv3 printout in native N*m*s/rad.
%   [G8] Unverified "Rasuli R^2 = 0.942" citation removed.
%   [G9] Results struct saved to stage1_results.mat.
%
%  DOCUMENTED SCOPE (deliberate, not an oversight):
%   - Identification uses the REVOLUTE equation only; the crowd equation
%     is validated forward but contributes no rows to Y. The revolute
%     carries all five Stage-1 parameters. Stacking both equations into
%     one regression is the planned upgrade when M6 multiplies the
%     parameter count.
%
%  MODEL-SIDE PREREQUISITES (done 2026-07-15, kept for provenance):
%   - Revolute joint: sensing ON; port "b" -> PS-Simulink -> To Workspace,
%     VariableName q_3_ddot.
%   - Prismatic joint: "Sense Acceleration" ON; port "a" -> PS-Simulink
%     -> To Workspace, VariableName d_4_ddot.
%   Without them this script falls back to gradient() and SAYS SO.

function results = validate_and_identify(out, opts)
%  USAGE:
%    validate_and_identify(out)                               % sensed, trim 0.5 s
%    validate_and_identify(out, ForceGradient=true)           % C1 reproduction
%    validate_and_identify(out, T_TRIM=0)                     % ablation run
%    r = validate_and_identify(out, T_TRIM=0, ForceGradient=true)  % 2x2 corner
arguments
    out                                     % Simulink.SimulationOutput
    opts.T_TRIM        (1,1) double  = 0.5  % s; 10*tau_filter startup trim
    opts.ForceGradient (1,1) logical = false % true = differentiate velocities
end
SCRIPT_VERSION = 'v5.1 (2026-07-15)';
close all
P = shovel_params();                                    % [G1]

%% 0b. Run configuration - declared, printed, saved with results   [H1]
T_TRIM         = opts.T_TRIM;
FORCE_GRADIENT = opts.ForceGradient;
fprintf('[i] %s | Config: T_TRIM = %.2f s | FORCE_GRADIENT = %d\n', ...
        SCRIPT_VERSION, T_TRIM, FORCE_GRADIENT);

%% 1. Grab data from the SimulationOutput (real solver time)
logged  = out.who;                                      % [G2]
haveVar = @(name) any(strcmp(logged, name));

getv = @(name) local_col(out, name);   % robust extractor (array/timeseries)

t    = out.tout(:);
q    = getv('q_3');       q_d = getv('q_3_dot');
d    = getv('d_4');       d_d = getv('d_4_dot');
tau  = getv('tau_3'); % NOTE: this is the SADDLE JOINT torque tau_3.
                          % Rename before the hoist-rope actuation map is
                          % added (M5), or the name will collide with the
                          % real hoist force.
F4   = getv('f_4');   % prismatic actuator force (crowd equation RHS)

%% 2. Accelerations: sensed if wired (and allowed), else gradient  [G2][H3]
if ~FORCE_GRADIENT && haveVar('q_3_ddot')
    q_dd = getv('q_3_ddot');
    accel_src_q = 'SENSED (Simscape b port)';
elseif FORCE_GRADIENT
    q_dd = gradient(q_d, t);
    accel_src_q = 'gradient() FORCED (C1 reproduction mode)';
else
    q_dd = gradient(q_d, t);
    accel_src_q = 'gradient() FALLBACK - wire the b port!';
end
if ~FORCE_GRADIENT && haveVar('d_4_ddot')
    d_dd = getv('d_4_ddot');
    accel_src_d = 'SENSED (Simscape a port)';
elseif FORCE_GRADIENT
    d_dd = gradient(d_d, t);
    accel_src_d = 'gradient() FORCED (C1 reproduction mode)';
else
    d_dd = gradient(d_d, t);
    accel_src_d = 'gradient() FALLBACK - enable prismatic accel sensing!';
end
fprintf('[i] q_3_ddot: %s\n[i] d_4_ddot: %s\n', accel_src_q, accel_src_d);

%% 3. Trim the input-filter startup transient (ALL signals)   [G4][H2]
keep = t > T_TRIM;
if ~any(keep)
    error('T_TRIM = %.2f s removed every sample. Check the time base.', T_TRIM);
end
t    = t(keep);
q    = q(keep);   q_d  = q_d(keep);   q_dd = q_dd(keep);
d    = d(keep);   d_d  = d_d(keep);   d_dd = d_dd(keep);
tau  = tau(keep); F4   = F4(keep);
N    = numel(t);
fprintf('[i] Trim: kept %d of %d samples (t > %.2f s)\n', N, numel(keep), T_TRIM);

%% 4. FORWARD VALIDATION - revolute (saddle) equation
%  tau_3 = (Izz + m r^2) qdd + 2 m r rdot qd + m g r cos(q) + fv3 qd
r = d + P.c;
tau_pred = (P.Izz + P.m.*r.^2).*q_dd ...        % inertia (parallel axis)
         + 2*P.m.*r.*d_d.*q_d ...               % Coriolis
         + P.m*P.g.*r.*cos(q) ...               % gravity
         + P.fv3.*q_d;                          % viscous [G3]
if P.model_has_coulomb
    tau_pred = tau_pred + P.fc3*sign(q_d);
end

[R2_f, RMSE_f, NRMSE_f] = local_fit_metrics(tau, tau_pred);

figure('Name','Forward Validation - Revolute','Color','w');
subplot(2,1,1);
plot(t, tau/1e6, 'k', 'LineWidth', 2.5); hold on;
plot(t, tau_pred/1e6, 'r--', 'LineWidth', 1.2);
ylabel('\tau_3 (MNm)'); grid on;
title(sprintf('Revolute eq.: Simscape vs Analytical   (R^2 = %.5f)', R2_f));
legend('Simscape (oracle)','Analytical','Location','best');
subplot(2,1,2);
plot(t, (tau - tau_pred)/1e3, 'b'); grid on;
xlabel('Time (s)'); ylabel('Residual (kNm)');
title('Forward residual (deterministic terms removed; future \tau_{load} channel)');

fprintf('\n=== FORWARD VALIDATION: REVOLUTE (saddle) equation ===\n');
fprintf('  R^2 = %.6f   RMSE = %.0f Nm   NRMSE = %.2f %%\n', R2_f, RMSE_f, NRMSE_f);

%% 5. IDENTIFICATION (Rasuli-style tau = Y*theta) + uncertainty   [G6]
%  theta1=m  theta2=m*c  theta3=m*c^2+Izz  theta4=fv3  theta5=fc3
Y = [ d.^2.*q_dd + 2*d.*d_d.*q_d + P.g*d.*cos(q), ...  % m
      2*d.*q_dd  + 2*d_d.*q_d    + P.g*cos(q),    ...  % m*c
      q_dd, ...                                        % m*c^2 + Izz
      q_d,  ...                                        % fv3 (viscous)
      sign(q_d) ];                                     % fc3 (Coulomb)
p = size(Y,2);
if N <= p                                              % [H7]
    error('N = %d samples <= p = %d parameters: covariance undefined.', N, p);
end

theta  = Y \ tau;
tau_id = Y*theta;
res2   = tau - tau_id;
R2_id  = 1 - sum(res2.^2)/sum((tau-mean(tau)).^2);

% --- conditioning: raw and column-scaled -------------------------------
condY_raw    = cond(Y);
colnorm      = sqrt(sum(Y.^2,1));
condY_scaled = cond(Y ./ colnorm);

% --- LS covariance and standard errors ---------------------------------
s2    = sum(res2.^2)/(N - p);          % residual variance estimate
CovT  = s2 * ((Y'*Y) \ eye(p));        % [H6] solve, not inv()
seT   = sqrt(diag(CovT));
ci95  = 1.96*seT;

% --- un-group + delta-method propagation --------------------------------
m_hat   = theta(1);
c_hat   = theta(2)/theta(1);
Izz_hat = theta(3) - theta(2)^2/theta(1);
fv3_hat = theta(4);
fc3_hat = theta(5);

Jc   = [-theta(2)/theta(1)^2,  1/theta(1)];
se_c = sqrt(Jc*CovT(1:2,1:2)*Jc');
Ji   = [ theta(2)^2/theta(1)^2, -2*theta(2)/theta(1), 1];
se_I = sqrt(Ji*CovT(1:3,1:3)*Ji');

fprintf('\n=== IDENTIFICATION (OLS) ===\n');
fprintf('  cond(Y) raw           : %.3g\n', condY_raw);
fprintf('  cond(Y) column-scaled : %.3g   (>1e3 => excitation-limited)\n', condY_scaled);
fprintf('  Fit R^2               : %.6f\n\n', R2_id);
fprintf('  %-12s %14s %14s %14s\n','param','estimate','+-95%% CI','truth (P.*)');
fprintf('  %-12s %14.1f %14.1f %14g\n','m [kg]',      m_hat,   1.96*seT(1), P.m);
fprintf('  %-12s %14.4f %14.4f %14g\n','c [m]',       c_hat,   1.96*se_c,   P.c);
fprintf('  %-12s %14.0f %14.0f %14g\n','Izz [kgm^2]', Izz_hat, 1.96*se_I,   P.Izz);
fprintf('  %-12s %14.1f %14.1f %14g\n','fv3 [Nms/rad]',fv3_hat, ci95(4),    P.fv3);   % [G7]
fprintf('  %-12s %14.1f %14.1f %14s\n','fc3 [Nm]',    fc3_hat, ci95(5), ...
        local_tern(P.model_has_coulomb, num2str(P.fc3), '0 (not in model)'));
fprintf(['\n  Read the CIs, not the point estimates: a CI spanning the truth\n' ...
         '  AND a CI comparable to the estimate itself both mean "this\n' ...
         '  trajectory cannot identify that parameter". Expect m and c to be\n' ...
         '  tight (gravity ~2.5 MNm excites them) and Izz/fv3 to be loose\n' ...
         '  under FORCE_GRADIENT or correlated excitation (same frequency\n' ...
         '  AND phase on both joints).\n']);

%% 6. Identification residual - the future home of tau_load(t)
figure('Name','Identification Residual','Color','w');
plot(t, res2/1e3, 'b'); grid on;
xlabel('Time (s)'); ylabel('Residual (kNm)');
title('Identification residual (= \tau_{load} once external forces exist)');

%% 7. FORWARD VALIDATION - prismatic (crowd) equation   [G5]
%  F_4 = m ddd - m r qd^2 + m g sin(q) + fv4 dd
F4_pred = P.m.*d_dd ...                         % linear inertia
        - P.m.*r.*q_d.^2 ...                    % centrifugal
        + P.m*P.g.*sin(q) ...                   % gravity along slide
        + P.fv4.*d_d;                           % viscous (1473 N*s/m, in model)
if P.model_has_coulomb
    F4_pred = F4_pred + P.fc4*sign(d_d);
end

% Sign-convention check: detect a flipped sensing convention explicitly.
cc = corrcoef(F4, F4_pred);              % base MATLAB, no toolbox needed
if cc(1,2) < 0
    warning(['f_crowd anti-correlates with the prediction: the prismatic ' ...
             'sensing sign convention is flipped. Reporting metrics on ' ...
             '-f_crowd; fix the sign at the source (PS-Simulink gain of -1).']);
    F4 = -F4;
end
[R2_c, RMSE_c, NRMSE_c] = local_fit_metrics(F4, F4_pred);

figure('Name','Forward Validation - Prismatic','Color','w');
subplot(2,1,1);
plot(t, F4/1e5, 'k', 'LineWidth', 2.5); hold on;
plot(t, F4_pred/1e5, 'r--', 'LineWidth', 1.2);
ylabel('F_4 (x10^5 N)'); grid on;
title(sprintf('Crowd eq.: Simscape vs Analytical   (R^2 = %.5f)', R2_c));
legend('Simscape (oracle)','Analytical','Location','best');
subplot(2,1,2);
plot(t, (F4 - F4_pred)/1e3, 'b'); grid on;
xlabel('Time (s)'); ylabel('Residual (kN)');

fprintf('\n=== FORWARD VALIDATION: PRISMATIC (crowd) equation ===\n');
fprintf('  R^2 = %.6f   RMSE = %.0f N   NRMSE = %.2f %%\n', R2_c, RMSE_c, NRMSE_c);

%% 8. Save the run record for the thesis   [G9][H4][H5]
results = struct('date',char(datetime("now")), ...
    'script_version',SCRIPT_VERSION, ...
    'T_TRIM',T_TRIM, 'FORCE_GRADIENT',FORCE_GRADIENT, ...
    'accel_source_q',accel_src_q, 'accel_source_d',accel_src_d, 'N',N, ...
    'fwd_revolute',struct('R2',R2_f,'RMSE',RMSE_f,'NRMSE_pct',NRMSE_f), ...
    'fwd_prismatic',struct('R2',R2_c,'RMSE',RMSE_c,'NRMSE_pct',NRMSE_c), ...
    'ident',struct('theta',theta,'se_theta',seT,'CovT',CovT, ...
        'm',m_hat,'c',c_hat,'Izz',Izz_hat,'fv3',fv3_hat,'fc3',fc3_hat, ...
        'se_c',se_c,'se_Izz',se_I,'R2',R2_id, ...
        'condY_raw',condY_raw,'condY_scaled',condY_scaled), ...
    'params_used',P);
save('stage1_results.mat','results');
fprintf('\n[i] Run record saved to stage1_results.mat (%s | T_TRIM=%.2f | FG=%d)\n', ...
        SCRIPT_VERSION, T_TRIM, FORCE_GRADIENT);
end   % ===== closes function validate_and_identify =====

%% ----------------------------------------------------------------------
function v = local_col(out, name)
% Extract a logged variable as a column, whether saved as Array/timeseries.
x = out.(name);
if isa(x,'timeseries'), v = x.Data(:);
elseif isnumeric(x),    v = x(:);
else, error('Unsupported To Workspace save format for %s.', name);
end
end

function [R2, RMSE, NRMSE] = local_fit_metrics(y, yhat)
res  = y - yhat;
R2   = 1 - sum(res.^2)/sum((y-mean(y)).^2);
RMSE = sqrt(mean(res.^2));
NRMSE= 100*RMSE/(max(y)-min(y));
end

function s = local_tern(cond_, a, b)
if cond_, s = a; else, s = b; end
end