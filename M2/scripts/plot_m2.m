function plot_m2(varargin)
%PLOT_M2  Publication-standard figure set for M2.
%
%   Produces the four M2 figures at Elsevier column widths, exported as
%   vector PDF (for submission) and PNG (for drafts and slides).
%
%   PLOT_M2('Result', R)
%       Figures 2 and 3 from a single loaded-case result struct.
%
%   PLOT_M2('Result', R, 'Null', Rnull)
%       Adds the module-01 floor line to figure 2.
%
%   PLOT_M2('Wide', Rwide, 'Narrow', Rnarrow)
%       Figures 1 and 4 -- the identifiability result. Both structs must
%       be wrong-station runs (Station = -1.32) differing only in crowd
%       stroke.
%
%   Name-value options:
%     'OutDir'   where figures are written. Default 'M2/figures'.
%     'Prefix'   filename prefix. Default 'M2'.
%     'Formats'  cell array from {'pdf','png','fig'}. Default all three.
%
%   FIGURE HIERARCHY
%     Fig 1  station confounding, wide vs narrow band     HEADLINE
%     Fig 2  log recovery error against the module-01 floor
%     Fig 3  measured vs model, and residual vs -J'F      method verification
%     Fig 4  rho(d4) analytical curve with measured slopes overlaid
%
%   Figure 3 is verification, not a result: the residual contains one
%   unmodelled term placed there by construction. Keep it single-column.
%
%   Repository:  M2/scripts/plot_m2.m
%
%   See also RECOVER_LOAD.

ip = inputParser;
ip.addParameter('Result', [], @(x)isstruct(x) || isempty(x));
ip.addParameter('Null',   [], @(x)isstruct(x) || isempty(x));
ip.addParameter('Wide',   [], @(x)isstruct(x) || isempty(x));
ip.addParameter('Narrow', [], @(x)isstruct(x) || isempty(x));
ip.addParameter('OutDir', fullfile('M2','figures'), @ischar);
ip.addParameter('Prefix', 'M2', @ischar);
ip.addParameter('Formats', {'pdf','png','fig'}, @iscell);
ip.parse(varargin{:});
o = ip.Results;

if ~exist(o.OutDir,'dir'), mkdir(o.OutDir); end

S = local_style();

if ~isempty(o.Wide) && ~isempty(o.Narrow)
    local_fig1_confounding(o.Wide, o.Narrow, S, o);
    local_fig4_rho(o.Wide, o.Narrow, S, o);
end

if ~isempty(o.Result)
    local_fig2_error(o.Result, o.Null, S, o);
    local_fig3_verification(o.Result, S, o);
end

if isempty(o.Result) && isempty(o.Wide)
    warning('plot_m2:nothingToDo', ...
        'Supply ''Result'' and/or both ''Wide'' and ''Narrow''.');
end
end


% =========================================================================
%  Style
% =========================================================================

function S = local_style()
%LOCAL_STYLE  Elsevier-compatible figure conventions.
%
%   Column widths (mm): 90 single, 140 one-and-a-half, 190 double.
%   Minimum type size in the printed figure is 7 pt; 8 pt is safer.

S.wSingle = 90;
S.wHalf   = 140;
S.wDouble = 190;

S.fsAxis   = 8;
S.fsLabel  = 8;
S.fsLegend = 7;
S.fsAnnot  = 7;

S.lwMain = 1.0;
S.lwRef  = 0.9;
S.lwAxes = 0.5;
S.lwGrid = 0.5;

% Restrained palette. Distinguishable in greyscale by lightness ordering,
% and safe for the common forms of colour vision deficiency.
S.cPrimary = [0.106 0.239 0.416];   % deep blue   - measured / primary
S.cAccent  = [0.698 0.235 0.106];   % rust        - reference / secondary
S.cTertiary= [0.290 0.400 0.243];   % olive       - third series
S.cGrey    = [0.450 0.450 0.450];   % annotation
S.cFloor   = [0.600 0.600 0.600];   % floor line

S.font = 'Helvetica';
end


function f = local_newfig(widthMM, heightMM, S, tag)
f = figure('Units','centimeters', ...
           'Position',[2 2 widthMM/10 heightMM/10], ...
           'Color','w','Name',tag,'NumberTitle','off', ...
           'PaperUnits','centimeters', ...
           'PaperSize',[widthMM/10 heightMM/10], ...
           'PaperPositionMode','auto');
set(f,'DefaultAxesFontName',S.font,'DefaultTextFontName',S.font);
end


function local_axstyle(ax, S)
set(ax,'FontSize',S.fsAxis,'FontName',S.font, ...
       'LineWidth',S.lwAxes,'Box','off','TickDir','out', ...
       'XGrid','on','YGrid','on','GridAlpha',0.12, ...
       'Layer','top');
end


function local_export(f, name, S, o) %#ok<INUSL>
base = fullfile(o.OutDir, sprintf('%s_%s', o.Prefix, name));
for k = 1:numel(o.Formats)
    switch lower(o.Formats{k})
        case 'pdf'
            try
                exportgraphics(f, [base '.pdf'], 'ContentType','vector', ...
                    'BackgroundColor','white');
            catch
                print(f, [base '.pdf'], '-dpdf', '-painters');
            end
        case 'png'
            try
                exportgraphics(f, [base '.png'], 'Resolution',600, ...
                    'BackgroundColor','white');
            catch
                print(f, [base '.png'], '-dpng', '-r600');
            end
        case 'fig'
            savefig(f, [base '.fig']);
    end
end
fprintf('  wrote %s.{%s}\n', base, strjoin(o.Formats,','));
end


% =========================================================================
%  Figure 1 -- HEADLINE: station confounding vs crowd stroke
% =========================================================================

function local_fig1_confounding(Rw, Rn, S, o)
fprintf('Figure 1 (headline): station confounding\n');

f = local_newfig(S.wHalf, 105, S, 'M2 Fig1 station confounding');
tl = tiledlayout(f, 2, 2, 'TileSpacing','compact','Padding','compact');

d4w = Rw.sig.d4;  d4n = Rn.sig.d4;
iw  = Rw.trimIdx; in_ = Rn.trimIdx;

% --- (a) the two crowd strokes -------------------------------------------
ax = nexttile(tl,1); hold(ax,'on');
plot(ax, Rw.t, d4w, 'Color',S.cPrimary,'LineWidth',S.lwMain);
plot(ax, Rn.t, d4n, 'Color',S.cAccent, 'LineWidth',S.lwMain);
yline(ax, 9.50,'--','Color',S.cGrey,'LineWidth',0.6);
yline(ax, 10.04,'--','Color',S.cGrey,'LineWidth',0.6);
text(ax, Rw.t(end)*0.02, 10.35, 'Bi et al. operational band', ...
    'FontSize',S.fsAnnot,'Color',S.cGrey,'FontName',S.font);
local_axstyle(ax,S);
xlabel(ax,'Time (s)','FontSize',S.fsLabel);
ylabel(ax,'Crowd extension {\itd}_4 (m)','FontSize',S.fsLabel);
title(ax,'(a) Excitation','FontSize',S.fsLabel,'FontWeight','normal');
lg = legend(ax, {'wide stroke','narrow stroke'}, 'Location','southeast');
set(lg,'FontSize',S.fsLegend,'Box','off');

% --- (b),(c) regression scatter ------------------------------------------
ax = nexttile(tl,2); hold(ax,'on');
local_scatterfit(ax, Rw.sig.Qq3_pred(iw)/1e6, Rw.sig.e_tau3(iw)/1e6, ...
                 S.cPrimary, S);
local_axstyle(ax,S);
xlabel(ax,'{\itQ}_{q_3}^{pred} (MN m)','FontSize',S.fsLabel);
ylabel(ax,'{\ite}_{\tau_3} (MN m)','FontSize',S.fsLabel);
title(ax, sprintf('(b) Wide:  {\\itR}^2 = %.6f', Rw.q3.R2), ...
    'FontSize',S.fsLabel,'FontWeight','normal');

ax = nexttile(tl,3); hold(ax,'on');
local_scatterfit(ax, Rn.sig.Qq3_pred(in_)/1e6, Rn.sig.e_tau3(in_)/1e6, ...
                 S.cAccent, S);
local_axstyle(ax,S);
xlabel(ax,'{\itQ}_{q_3}^{pred} (MN m)','FontSize',S.fsLabel);
ylabel(ax,'{\ite}_{\tau_3} (MN m)','FontSize',S.fsLabel);
title(ax, sprintf('(c) Narrow:  {\\itR}^2 = %.6f', Rn.q3.R2), ...
    'FontSize',S.fsLabel,'FontWeight','normal');

% --- (d) the collapse ----------------------------------------------------
ax = nexttile(tl,4); hold(ax,'on');
disc = [1-Rw.q3.R2, 1-Rn.q3.R2];
b = bar(ax, [1 2], disc, 0.55, 'FaceColor','flat','EdgeColor','none');
b.CData(1,:) = S.cPrimary;  b.CData(2,:) = S.cAccent;
set(ax,'YScale','log','XTick',[1 2], ...
       'XTickLabel',{'wide','narrow'});
local_axstyle(ax,S);
ylabel(ax,'Discriminating fraction  1 - {\itR}^2','FontSize',S.fsLabel);
title(ax, sprintf('(d) Collapse by factor %.0f', disc(1)/disc(2)), ...
    'FontSize',S.fsLabel,'FontWeight','normal');
for k = 1:2
    text(ax, k, disc(k)*1.6, sprintf('%.2e',disc(k)), ...
        'HorizontalAlignment','center','FontSize',S.fsAnnot, ...
        'FontName',S.font);
end
ylim(ax,[min(disc)/6 max(disc)*8]);

local_export(f,'fig1_station_confounding',S,o);
end


function local_scatterfit(ax, x, y, col, S)
scatter(ax, x, y, 3, col, 'filled', 'MarkerFaceAlpha',0.30);
pf = polyfit(x,y,1);
xl = [min(x) max(x)];
plot(ax, xl, polyval(pf,xl), '-', 'Color',[0 0 0], 'LineWidth',0.7);
text(ax, xl(1)+0.05*diff(xl), max(y)-0.08*range(y), ...
    sprintf('slope = %.4f', pf(1)), ...
    'FontSize',S.fsAnnot,'FontName',S.font);
end


% =========================================================================
%  Figure 2 -- recovery error against the numerical floor
% =========================================================================

function local_fig2_error(R, Rnull, S, o)
fprintf('Figure 2: recovery error vs floor\n');

f = local_newfig(S.wSingle, 65, S, 'M2 Fig2 recovery error');
ax = axes(f); hold(ax,'on');

i = R.trimIdx;
e3 = abs(R.sig.e_tau3(i) - R.sig.ref3(i));
e3(e3 < eps) = eps;

semilogy(ax, R.t(i), e3, 'Color',S.cPrimary,'LineWidth',S.lwMain);

if ~isempty(Rnull)
    fl = Rnull.q3.maxAbsResidual;
    yline(ax, fl, '--', 'Color',S.cFloor,'LineWidth',0.7);
    text(ax, R.t(i(find(i,1)))+1, fl*1.9, ...
        sprintf('zero-force floor  %.2e N m', fl), ...
        'FontSize',S.fsAnnot,'Color',S.cFloor,'FontName',S.font);
end

set(ax,'YScale','log');
local_axstyle(ax,S);
xlim(ax,[R.opt.TrimStart R.t(end)]);
xlabel(ax,'Time (s)','FontSize',S.fsLabel);
ylabel(ax,'| {\ite}_{\tau_3} + {\itQ}_{q_3} |  (N m)','FontSize',S.fsLabel);

local_export(f,'fig2_recovery_error',S,o);
end


% =========================================================================
%  Figure 3 -- method verification (keep small)
% =========================================================================

function local_fig3_verification(R, S, o)
fprintf('Figure 3: method verification\n');

f = local_newfig(S.wHalf, 90, S, 'M2 Fig3 verification');
tl = tiledlayout(f, 2, 2, 'TileSpacing','compact','Padding','compact');

t = R.t;

ax = nexttile(tl,1); hold(ax,'on');
plot(ax,t,R.sig.tau3_meas/1e6,'Color',S.cPrimary,'LineWidth',S.lwMain);
plot(ax,t,R.sig.tau3_model/1e6,'--','Color',S.cAccent,'LineWidth',S.lwRef);
local_axstyle(ax,S);
ylabel(ax,'\tau_3 (MN m)','FontSize',S.fsLabel);
title(ax,'(a) Saddle: measured vs model','FontSize',S.fsLabel,'FontWeight','normal');
lg = legend(ax,{'measured','M1 model'},'Location','best');
set(lg,'FontSize',S.fsLegend,'Box','off');

ax = nexttile(tl,2); hold(ax,'on');
plot(ax,t,R.sig.f4_meas/1e5,'Color',S.cPrimary,'LineWidth',S.lwMain);
plot(ax,t,R.sig.f4_model/1e5,'--','Color',S.cAccent,'LineWidth',S.lwRef);
local_axstyle(ax,S);
ylabel(ax,'{\itf}_4 (10^5 N)','FontSize',S.fsLabel);
title(ax,'(b) Crowd: measured vs model','FontSize',S.fsLabel,'FontWeight','normal');

ax = nexttile(tl,3); hold(ax,'on');
plot(ax,t,R.sig.e_tau3/1e5,'Color',S.cPrimary,'LineWidth',S.lwMain);
plot(ax,t,R.sig.ref3/1e5,'--','Color',S.cAccent,'LineWidth',S.lwRef);
local_axstyle(ax,S);
xlabel(ax,'Time (s)','FontSize',S.fsLabel);
ylabel(ax,'{\ite}_{\tau_3} (10^5 N m)','FontSize',S.fsLabel);
title(ax,'(c) Saddle residual vs -{\itJ}^T{\bfF}','FontSize',S.fsLabel,'FontWeight','normal');
lg = legend(ax,{'residual','-{\itJ}^T{\bfF}'},'Location','best');
set(lg,'FontSize',S.fsLegend,'Box','off');

ax = nexttile(tl,4); hold(ax,'on');
plot(ax,t,R.sig.e_f4/1e5,'Color',S.cPrimary,'LineWidth',S.lwMain);
plot(ax,t,R.sig.ref4/1e5,'--','Color',S.cAccent,'LineWidth',S.lwRef);
local_axstyle(ax,S);
xlabel(ax,'Time (s)','FontSize',S.fsLabel);
ylabel(ax,'{\ite}_{f_4} (10^5 N)','FontSize',S.fsLabel);
title(ax,'(d) Crowd residual vs -{\itJ}^T{\bfF}','FontSize',S.fsLabel,'FontWeight','normal');

local_export(f,'fig3_verification',S,o);
end


% =========================================================================
%  Figure 4 -- rho(d4): derivation meets measurement
% =========================================================================

function local_fig4_rho(Rw, Rn, S, o)
fprintf('Figure 4: rho(d4) prediction vs measurement\n');

f = local_newfig(S.wSingle, 70, S, 'M2 Fig4 rho');
ax = axes(f); hold(ax,'on');

d4 = linspace(6.0, 11.5, 600);
rho = (d4 + 1.5)./(d4 - 1.32);

% Operating ranges actually used
wRange = [min(Rw.sig.d4(Rw.trimIdx)) max(Rw.sig.d4(Rw.trimIdx))];
nRange = [min(Rn.sig.d4(Rn.trimIdx)) max(Rn.sig.d4(Rn.trimIdx))];

local_band(ax, wRange, S.cPrimary, 0.10);
local_band(ax, nRange, S.cAccent,  0.18);
local_band(ax, [9.50 10.04], S.cTertiary, 0.22);

plot(ax, d4, rho, 'Color',[0 0 0], 'LineWidth',S.lwMain);

% Measured slopes (magnitude of the wrong-station regression slope)
plot(ax, mean(wRange), abs(Rw.q3.slope), 'o', ...
    'MarkerSize',4.5,'MarkerFaceColor',S.cPrimary,'MarkerEdgeColor','none');
plot(ax, mean(nRange), abs(Rn.q3.slope), 's', ...
    'MarkerSize',4.5,'MarkerFaceColor',S.cAccent,'MarkerEdgeColor','none');

text(ax, mean(wRange), abs(Rw.q3.slope)-0.035, ...
    sprintf('%.4f', abs(Rw.q3.slope)), 'FontSize',S.fsAnnot, ...
    'HorizontalAlignment','center','FontName',S.font);
text(ax, mean(nRange), abs(Rn.q3.slope)+0.030, ...
    sprintf('%.4f', abs(Rn.q3.slope)), 'FontSize',S.fsAnnot, ...
    'HorizontalAlignment','center','FontName',S.font);

local_axstyle(ax,S);
xlabel(ax,'Crowd extension {\itd}_4 (m)','FontSize',S.fsLabel);
ylabel(ax,'\rho({\itd}_4) = ({\itd}_4+1.5)/({\itd}_4-1.32)','FontSize',S.fsLabel);
xlim(ax,[6 11.5]);

lg = legend(ax, {'wide stroke','narrow stroke','Bi et al. band', ...
                 '\rho({\itd}_4) analytical','measured (wide)','measured (narrow)'}, ...
            'Location','northeast');
set(lg,'FontSize',S.fsLegend,'Box','off');

local_export(f,'fig4_rho_prediction_vs_measurement',S,o);
end


function local_band(ax, xr, col, alpha)
yl = [1.0 2.2];
patch(ax, [xr(1) xr(2) xr(2) xr(1)], [yl(1) yl(1) yl(2) yl(2)], col, ...
    'FaceAlpha',alpha,'EdgeColor','none');
end
