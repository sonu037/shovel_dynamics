%% VALIDATE_AND_IDENTIFY_4.M
%  Stage-1 Protocol: forward validation (BOTH equations) + linear-in-
%  parameters identification with uncertainty reporting.
%  Model: ShovelSimulator_v4.slx (Revolute q_3 + Prismatic d_4)
%
%  FIXES vs validate_and_identify_3.m:
%   [G1] Parameters come from shovel_params.m (single source of truth).
%        v3 comments contradicted the .slx (Izz "839,000" vs actual 287,900;
%        fv3 "340" vs actual 3,800). Fixed by construction.
%   [G2] Logged-signal detection uses out.who. isfield/isprop on a
%        Simulink.SimulationOutput object silently return false, so v3
%        ALWAYS fell back to gradient() even with sensed accelerations.
%   [G3] Forward validation now includes the viscous term fv3*q_d
%        (3,800 N*m*s/rad genuinely exists in the joint; the sensed
%        actuator torque includes work done against it).
%   [G4] d_dd is now trimmed with everything else (v3 omitted it -> the
%        exact mismatched-length landmine its own header warned about).
%   [G5] Section 7 validates the PRISMATIC (crowd) equation against the
%        already-logged f_crowd. Previously logged but never used.
%   [G6] Identification reports cond(Y) (raw + column-scaled) and
%        parameter standard errors / 95% CIs via the LS covariance,
%        with delta-method propagation to the physical parameters.
%   [G7] fv3 printout fixed: theta(4) is ALREADY in N*m*s/rad (the
%        regressor uses rad/s). v3 multiplied by 180/pi -> nonsense value.
%   [G8] Unverified "Rasuli R^2 = 0.942" citation removed. The paper
%        reports payload errors and std devs (e.g. 8402 vs 8420 kg,
%        sigma = 104 kg, swing), not an R^2 for this comparison.
%   [G9] Results struct saved to stage1_results.mat for the thesis record.
%
%  MODEL-SIDE PREREQUISITES (10 minutes, do once):
%   - Revolute joint: sensing already ON; wire port "b" -> PS-Simulink
%     -> To Workspace, VariableName q_3_ddot.
%   - Prismatic joint: enable "Sense Acceleration"; wire port "a"
%     -> PS-Simulink -> To Workspace, VariableName d_4_ddot.
%   Until then this script falls back to gradient() and SAYS SO.

%% 0. Clean slate
clearvars -except out
close all
P = shovel_params();                                    % [G1]

%% 1. Grab data from workspace (real solver time)
if ~exist('out','var')
    error(['Run the sim with "Single simulation output" enabled ' ...
           'so out (SimulationOutput) exists.']);
end
logged = out.who;                                       % [G2]
haveVar = @(name) any(strcmp(logged, name));

getv = @(name) local_col(out, name);   % robust extractor (array/timeseries)

t    = out.tout(:);
q    = getv('q_3');       q_d = getv('q_3_dot');
d    = getv('d_4');       d_d = getv('d_4_dot');
tau  = getv('tau_3'); % NOTE: this is the SADDLE JOINT torque tau_3.
                          % Rename the To Workspace block to tau_3 before
                          % the hoist-rope actuation map is added, or the
                          % name will collide with the real hoist force.
F4   = getv('f_4');   % prismatic actuator force (crowd equation RHS)

%% 2. Accelerations: sensed if wired, else central differences   [G2]
if haveVar('q_3_ddot')
    q_dd = getv('q_3_ddot');
    accel_src_q = 'SENSED (Simscape b port)';
else
    q_dd = gradient(q_d, t);
    accel_src_q = 'gradient() FALLBACK - wire the b port!';
end
if haveVar('d_4_ddot')
    d_dd = getv('d_4_ddot');
    accel_src_d = 'SENSED (Simscape a port)';
else
    d_dd = gradient(d_d, t);
    accel_src_d = 'gradient() FALLBACK - enable prismatic accel sensing!';
end
fprintf('[i] q_3_ddot: %s\n[i] d_4_ddot: %s\n', accel_src_q, accel_src_d);

%% 3. Trim the input-filter startup transient (ALL signals)   [G4]
keep = t > 0.5;
t    = t(keep);
q    = q(keep);   q_d  = q_d(keep);   q_dd = q_dd(keep);
d    = d(keep);   d_d  = d_d(keep);   d_dd = d_dd(keep);
tau  = tau(keep); F4   = F4(keep);
N    = numel(t);

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
CovT  = s2 * inv(Y'*Y);                %#ok<MINV> parameter covariance
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
         '  until (a) sensed accelerations are wired and (b) the two sine\n' ...
         '  inputs are decorrelated (different frequency AND phase).\n']);

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

% Sign-convention check: v4 planar geometry needed a sign fix on tau;
% f_crowd may carry one too. Detect it explicitly rather than silently.
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

%% 8. Save the run record for the thesis   [G9]
results = struct('date',datestr(now), 'accel_source_q',accel_src_q, ...
    'accel_source_d',accel_src_d, 'N',N, ...
    'fwd_revolute',struct('R2',R2_f,'RMSE',RMSE_f,'NRMSE_pct',NRMSE_f), ...
    'fwd_prismatic',struct('R2',R2_c,'RMSE',RMSE_c,'NRMSE_pct',NRMSE_c), ...
    'ident',struct('theta',theta,'se_theta',seT,'CovT',CovT, ...
        'm',m_hat,'c',c_hat,'Izz',Izz_hat,'fv3',fv3_hat,'fc3',fc3_hat, ...
        'se_c',se_c,'se_Izz',se_I,'R2',R2_id, ...
        'condY_raw',condY_raw,'condY_scaled',condY_scaled), ...
    'params_used',P);
save('stage1_results.mat','results');
fprintf('\n[i] Run record saved to stage1_results.mat\n');

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

