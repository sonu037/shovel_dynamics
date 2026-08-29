function D = sensitivity_diagnostic()
%SENSITIVITY_DIAGNOSTIC  Load-vs-mass sensitivity shape comparison for M2.
%
%   Compares the external-load sensitivity against the full mass-error
%   sensitivity on two trajectories: the wide reference case and the
%   Bi-band case.
%
%   METRIC. Mean-removed Pearson correlation. The estimator fits a free
%   intercept, y = b0 + s_load*F + s_dm*dM + eps, so b0 is a nuisance
%   parameter and mean-removal is the projection that removes the constant
%   direction. Cosine similarity on raw vectors answers a question about a
%   model WITHOUT an intercept and gives a different ranking.
%
%   Repository:  M2/scripts/sensitivity_diagnostic.m
%   SIGN CONVENTION. Columns are d(tau3)/d(parameter) WITHOUT negation.
%   The residual carries the minus sign separately:
%   residual = tau_measured - tau_model = -J'F.
%   Pearson correlation is invariant under a sign flip applied to both
%   columns, so this does not affect the shape results. It DOES affect the
%   signs of estimates at the conditioning step.

cases = { ...
    'wide',   'M2/experiments/02_known_load/M2_100kN_0p1111Hz_sp1p50.mat'; ...
    'bi_band','M2/experiments/06_identifiability/M2_wrongstation_bi_band.mat' };

D = struct();
D.date   = datestr(now,'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
D.metric = ['mean-removed Pearson; equivalent to projecting out the ' ...
            'intercept direction, which the estimator fits freely'];
D.script = mfilename('fullpath');

for k = 1:size(cases,1)
    tag  = cases{k,1};
    S    = load(cases{k,2});
    R    = S.R;
    i    = R.trimIdx;

    g  = R.params.g;
    c  = R.params.c;
    q3 = R.sig.q3(i);   q3d = R.sig.q3d(i);   q3dd = R.sig.q3dd(i);
    d4 = R.sig.d4(i);   d4d = R.sig.d4d(i);
    r  = d4 + c;

    s_load = (d4 + 1.5).*cos(q3);              % [m]
    s_grav = g*r.*cos(q3);                     % [m^2/s^2]
    s_in   = r.^2.*q3dd;
    s_co   = 2*r.*d4d.*q3d;
    s_dyn  = s_in + s_co;
    s_full = s_grav + s_dyn;

    c_ = struct();
    c_.source      = cases{k,2};
    c_.case        = R.case;
    c_.nSamples    = numel(i(i));
    c_.trimStart   = R.opt.TrimStart;
    c_.d4_range    = [min(d4) max(d4)];
    c_.d4_span     = max(d4) - min(d4);
    c_.d4dot_max   = max(abs(d4d));
    c_.q3_range    = [min(q3) max(q3)];
    c_.q3dd_max    = max(abs(q3dd));

    c_.corr_matrix = corrcoef([s_load(:), s_grav(:), s_in(:), s_co(:)]);
    c_.corr_labels = {'load','gravity','inertia','coriolis'};
    c_.r_load_grav = corr(s_load(:), s_grav(:));
    c_.r_load_full = corr(s_load(:), s_full(:));
    c_.r_load_in   = corr(s_load(:), s_in(:));
    c_.r_load_co   = corr(s_load(:), s_co(:));

    c_.rms_in_over_grav  = rms(s_in)/rms(s_grav);
    c_.rms_co_over_grav  = rms(s_co)/rms(s_grav);
    c_.rms_dyn_over_grav = rms(s_dyn)/rms(s_grav);
    c_.grav_over_dyn     = rms(s_grav)/rms(s_dyn);

    % Offsets, recorded because raw cosine similarity is dominated by them
    c_.mean_std_load = [mean(s_load) std(s_load)];
    c_.mean_std_in   = [mean(s_in)   std(s_in)];
    c_.cos_load_in_raw = dot(s_load/norm(s_load), s_in/norm(s_in));

    D.(tag) = c_;
    local_report(tag, c_);
end

if ~exist('M2/experiments/06_identifiability','dir')
    mkdir('M2/experiments/06_identifiability');
end
save('M2/experiments/06_identifiability/M2_sensitivity_diagnostic.mat','D');
fprintf('\nSaved: M2/experiments/06_identifiability/M2_sensitivity_diagnostic.mat\n');
end


function local_report(tag, c)
fprintf('\n============================================================\n');
fprintf('  %s   (%s)\n', tag, c.case);
fprintf('============================================================\n');
fprintf('d4 range      : %.4f to %.4f m  (span %.4f)\n', ...
    c.d4_range(1), c.d4_range(2), c.d4_span);
fprintf('max |d4dot|   : %.6f m/s\n', c.d4dot_max);
fprintf('max |q3ddot|  : %.6f rad/s^2\n', c.q3dd_max);
fprintf('samples       : %d (trim at %.1f s)\n', c.nSamples, c.trimStart);

fprintf('\nPearson (mean-removed):\n');
disp(array2table(c.corr_matrix, ...
    'VariableNames', c.corr_labels, 'RowNames', c.corr_labels));

fprintf('r(load, gravity-only) = %.10f\n', c.r_load_grav);
fprintf('r(load, FULL mass)    = %.10f\n', c.r_load_full);
fprintf('1 - r(load, full)     = %.6e\n', 1 - c.r_load_full);

fprintf('\nMagnitudes relative to gravity (RMS):\n');
fprintf('  inertia  = %.6f %%\n', 100*c.rms_in_over_grav);
fprintf('  Coriolis = %.6f %%\n', 100*c.rms_co_over_grav);
fprintf('  dynamic  = %.6f %%   (gravity:dynamic = %.2f:1)\n', ...
    100*c.rms_dyn_over_grav, c.grav_over_dyn);

fprintf('\nOffsets (why raw cosine similarity misleads):\n');
fprintf('  s_load  mean %.4f  std %.4f\n', c.mean_std_load(1), c.mean_std_load(2));
fprintf('  s_in    mean %.4f  std %.4f\n', c.mean_std_in(1), c.mean_std_in(2));
fprintf('  raw cosine(load,inertia) = %.6f  vs Pearson %.6f\n', ...
    c.cos_load_in_raw, c.r_load_in);
fprintf('============================================================\n');
end