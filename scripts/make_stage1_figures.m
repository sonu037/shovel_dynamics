%% MAKE_STAGE1_FIGURES.M
%  Regenerates ALL Stage-1 thesis figures as vector PDFs from archived data.
%  Protocol: every thesis figure = archived data + this script + vector PDF.
%  Never save figures manually via File > Save As again.
%
%  PREREQUISITE: run the simulation once so 'out' exists in the workspace
%  (the comparison figure C1 needs both gradient() and sensed accelerations,
%  which are recomputed here from the same 'out').
%
%  OUTPUT: ../experiments/2026-07-07_stage1_final/figures/*.pdf
%
%  Figures produced:
%   A3  fig_A3_excitation.pdf        input trajectories q3(t), d4(t)
%   B1  fig_B1_revolute_validation.pdf   Simscape vs analytical + residual
%   B2  fig_B2_prismatic_validation.pdf  crowd equation + residual
%   B3  fig_B3_identification_ci.pdf     identified vs true, 95% CIs
%   C1  fig_C1_gradient_vs_sensed.pdf    differentiation-noise comparison

clearvars -except out
close all
P = shovel_params();

here   = fileparts(mfilename('fullpath'));          % folder of THIS script
outdir = fullfile(here, '..', 'experiments', '2026-07-07_stage1_final', 'figures');
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% ---- SHARED THESIS STYLE (reuse this block in every future figure script)
S.font      = 'Liberation Serif';   % metrically = Times New Roman
S.fs        = 10;                   % pt, at FINAL printed size
S.fsSmall   = 9;
S.lwData    = 1.2;                  % data lines
S.lwRef     = 0.9;                  % reference/secondary lines
S.colMeas   = [0 0 0];              % measured: black solid
S.colPred   = [0.75 0.10 0.10];     % predicted: dark red dashed
S.colAux    = [0.25 0.25 0.60];     % residuals/aux: dark blue
S.wSingle   = 8.4;                  % cm, single-column width
S.wFull     = 17.0;                 % cm, full text width
applyStyle = @(ax) set(ax,'FontName',S.font,'FontSize',S.fs, ...
    'LineWidth',0.6,'Box','on','XGrid','on','YGrid','on', ...
    'GridAlpha',0.15,'TickDir','in');
newFig = @(w_cm,h_cm) figure('Units','centimeters','Position',[2 2 w_cm h_cm], ...
    'Color','w','PaperPositionMode','auto');
saveFig = @(fig,name) exportgraphics(fig, fullfile(outdir,name), ...
    'ContentType','vector');

%% ---- Load data (identical pipeline to validate_and_identify) ------------
if ~exist('out','var')
    error('Run the simulation first so ''out'' exists.');
end
logged  = out.who;
haveVar = @(n) any(strcmp(logged,n));
getv    = @(n) local_col(out,n);

t   = out.tout(:);
q   = getv('q_3');   q_d = getv('q_3_dot');
d   = getv('d_4');   d_d = getv('d_4_dot');
tau = getv('tau_3');
F4  = getv('f_4');

% Both acceleration sources, for the C1 comparison:
q_dd_grad = gradient(q_d, t);
d_dd_grad = gradient(d_d, t);
if haveVar('q_3_ddot'), q_dd_sens = getv('q_3_ddot'); else, q_dd_sens = []; end
if haveVar('d_4_ddot'), d_dd_sens = getv('d_4_ddot'); else, d_dd_sens = []; end

keep = t > 0.5;
t=t(keep); q=q(keep); q_d=q_d(keep); d=d(keep); d_d=d_d(keep);
tau=tau(keep); F4=F4(keep);
q_dd_grad=q_dd_grad(keep); d_dd_grad=d_dd_grad(keep);
if ~isempty(q_dd_sens), q_dd_sens=q_dd_sens(keep); end
if ~isempty(d_dd_sens), d_dd_sens=d_dd_sens(keep); end

r = d + P.c;
tau_model = @(qdd) (P.Izz + P.m.*r.^2).*qdd + 2*P.m.*r.*d_d.*q_d ...
                 + P.m*P.g.*r.*cos(q) + P.fv3.*q_d;
F4_model  = @(ddd) P.m.*ddd - P.m.*r.*q_d.^2 + P.m*P.g.*sin(q) + P.fv4.*d_d;

%% ---- A3: excitation signals ---------------------------------------------
f = newFig(S.wSingle, 6.5);
tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
ax1 = nexttile;
plot(t, q*180/pi, 'Color', S.colMeas, 'LineWidth', S.lwData);
ylabel('q_3 (deg)'); applyStyle(ax1);
title('Excitation trajectories','FontWeight','normal');
ax2 = nexttile;
plot(t, d, 'Color', S.colMeas, 'LineWidth', S.lwData);
ylabel('d_4 (m)'); xlabel('Time (s)'); applyStyle(ax2);
saveFig(f,'fig_A3_excitation.pdf');

%% ---- B1: revolute forward validation ------------------------------------
qdd_best = q_dd_grad; src='gradient';
if ~isempty(q_dd_sens), qdd_best = q_dd_sens; src='sensed'; end
tp = tau_model(qdd_best);
res = tau - tp;
R2  = 1 - sum(res.^2)/sum((tau-mean(tau)).^2);

f = newFig(S.wFull, 8.5);
tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
ax1 = nexttile;
plot(t, tau/1e6, '-', 'Color', S.colMeas, 'LineWidth', 1.8); hold on;
plot(t, tp/1e6, '--', 'Color', S.colPred, 'LineWidth', S.lwData);
ylabel('\tau_3 (MN m)'); applyStyle(ax1);
legend({'Multibody simulation','Analytical model'},'Location','best', ...
    'FontSize',S.fsSmall);
title(sprintf('Saddle-joint torque: forward validation (R^2 = %.6f, %s accel.)', ...
    R2, src),'FontWeight','normal');
ax2 = nexttile;
plot(t, res/1e3, '-', 'Color', S.colAux, 'LineWidth', S.lwRef);
ylabel('Residual (kN m)'); xlabel('Time (s)'); applyStyle(ax2);
saveFig(f,'fig_B1_revolute_validation.pdf');

%% ---- B2: prismatic forward validation -----------------------------------
ddd_best = d_dd_grad; if ~isempty(d_dd_sens), ddd_best = d_dd_sens; end
Fp  = F4_model(ddd_best);
cc  = corrcoef(F4,Fp); if cc(1,2)<0, F4 = -F4; end
resF = F4 - Fp;
R2F  = 1 - sum(resF.^2)/sum((F4-mean(F4)).^2);

f = newFig(S.wFull, 8.5);
tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
ax1 = nexttile;
plot(t, F4/1e5, '-', 'Color', S.colMeas, 'LineWidth', 1.8); hold on;
plot(t, Fp/1e5, '--', 'Color', S.colPred, 'LineWidth', S.lwData);
ylabel('F_4 (\times10^5 N)'); applyStyle(ax1);
legend({'Multibody simulation','Analytical model'},'Location','best', ...
    'FontSize',S.fsSmall);
title(sprintf('Crowd force: forward validation (R^2 = %.6f)',R2F), ...
    'FontWeight','normal');
ax2 = nexttile;
plot(t, resF/1e3, '-', 'Color', S.colAux, 'LineWidth', S.lwRef);
ylabel('Residual (kN)'); xlabel('Time (s)'); applyStyle(ax2);
saveFig(f,'fig_B2_prismatic_validation.pdf');

%% ---- B3: identification with 95% CIs (both acceleration sources) --------
[mG,cG,IG,fvG,ciG] = local_id(d,q,q_d,d_d,q_dd_grad,tau,P);
if ~isempty(q_dd_sens)
    [mS,cS,IS,fvS,ciS] = local_id(d,q,q_d,d_d,q_dd_sens,tau,P);
else
    mS=NaN; cS=NaN; IS=NaN; fvS=NaN; ciS=nan(1,4);
end

truth = [P.m, P.c, P.Izz, P.fv3];
names = {'m (kg)','c (m)','I_{zz} (kg m^2)','f_{v3} (N m s/rad)'};
estG  = [mG,cG,IG,fvG];  estS = [mS,cS,IS,fvS];

f = newFig(S.wFull, 6.0);
tiledlayout(f,1,4,'TileSpacing','compact','Padding','compact');
for k = 1:4
    ax = nexttile; hold on;
    % normalize to truth = 1 for a common visual scale
    e1 = estG(k)/truth(k); c1 = abs(ciG(k)/truth(k));
    e2 = estS(k)/truth(k); c2 = abs(ciS(k)/truth(k));
    errorbar(1, e1, c1, 'o', 'Color', S.colAux,  'LineWidth', S.lwData, ...
        'MarkerFaceColor','w','CapSize',6);
    errorbar(2, e2, c2, 's', 'Color', S.colPred, 'LineWidth', S.lwData, ...
        'MarkerFaceColor','w','CapSize',6);
    yline(1,'-','Color',[0 0 0],'LineWidth',0.7);
    xlim([0.4 2.6]); xticks([1 2]); xticklabels({'grad.','sensed'});
    title(names{k},'FontWeight','normal','FontSize',S.fsSmall);
    if k==1, ylabel('Estimate / truth'); end
    applyStyle(ax);
end
saveFig(f,'fig_B3_identification_ci.pdf');

%% ---- C1: gradient vs sensed differentiation-noise comparison ------------
if ~isempty(q_dd_sens)
    tpG = tau_model(q_dd_grad);   resG = tau - tpG;
    tpS = tau_model(q_dd_sens);   resS = tau - tpS;
    nrmseG = 100*sqrt(mean(resG.^2))/(max(tau)-min(tau));
    nrmseS = 100*sqrt(mean(resS.^2))/(max(tau)-min(tau));

    f = newFig(S.wFull, 8.5);
    tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
    ax1 = nexttile;
    plot(t, resG/1e3, '-', 'Color', S.colAux, 'LineWidth', S.lwRef);
    ylabel('Residual (kN m)'); applyStyle(ax1);
    title(sprintf('Numerical differentiation of velocity (NRMSE = %.2f%%)', ...
        nrmseG),'FontWeight','normal');
    yl = ylim;                          % lock a COMMON scale for fairness
    ax2 = nexttile;
    plot(t, resS/1e3, '-', 'Color', S.colAux, 'LineWidth', S.lwRef);
    ylim(yl);                           % identical axis -> honest comparison
    ylabel('Residual (kN m)'); xlabel('Time (s)'); applyStyle(ax2);
    title(sprintf('Directly sensed acceleration (NRMSE = %.2e%%)', ...
        nrmseS),'FontWeight','normal');
    saveFig(f,'fig_C1_gradient_vs_sensed.pdf');
else
    warning('q_3_ddot not logged: C1 comparison skipped.');
end

fprintf('\n[i] Figures written to %s\n', outdir);
fprintf('    Insert PDFs at native width: single-column %.1f cm, full %.1f cm.\n', ...
    S.wSingle, S.wFull);

%% ---- local functions -----------------------------------------------------
function v = local_col(out,name)
x = out.(name);
if isa(x,'timeseries'), v = x.Data(:);
elseif isnumeric(x),    v = x(:);
else, error('Unsupported save format for %s.',name);
end
end

function [m_hat,c_hat,Izz_hat,fv3_hat,ci] = local_id(d,q,q_d,d_d,q_dd,tau,P)
Y = [ d.^2.*q_dd + 2*d.*d_d.*q_d + P.g*d.*cos(q), ...
      2*d.*q_dd  + 2*d_d.*q_d    + P.g*cos(q),    ...
      q_dd, q_d ];
th   = Y\tau;
res  = tau - Y*th;
N    = numel(tau); p = size(Y,2);
CovT = (sum(res.^2)/(N-p)) * inv(Y'*Y); %#ok<MINV>
seT  = sqrt(diag(CovT));
m_hat   = th(1);
c_hat   = th(2)/th(1);
Izz_hat = th(3) - th(2)^2/th(1);
fv3_hat = th(4);
Jc = [-th(2)/th(1)^2, 1/th(1)];
Ji = [ th(2)^2/th(1)^2, -2*th(2)/th(1), 1];
ci = 1.96*[seT(1), sqrt(Jc*CovT(1:2,1:2)*Jc'), ...
           sqrt(Ji*CovT(1:3,1:3)*Ji'), seT(4)];
end
