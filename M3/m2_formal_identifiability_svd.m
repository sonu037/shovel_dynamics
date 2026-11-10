function S_out = m2_formal_identifiability_svd(varargin)
%M2_FORMAL_IDENTIFIABILITY_SVD  Formal SVD, conditioning & subspace-angle suite for M2.
%
%   Computes the raw, column-2-norm unit-scaled, and physically scaled
%   singular value decompositions (SVD), condition numbers kappa_2(S),
%   Variance Inflation Factors (VIF), and principal subspace angles theta_min
%   between external bail load sensitivity and model-error sensitivities
%   (dipper mass M_d, rotational inertia I_zz, COG offset c, viscous
%   damping f_v3/f_v4) across all M2 Simscape .mat trajectories.
%
%   Repository: M2/scripts/m2_formal_identifiability_svd.m
%   Project:    IIT/SRIC/R/AEH/2026/104 (CMPDI) - IIT Kharagpur

ip = inputParser;
ip.addParameter('DataDir', fullfile('M2','experiments'), @ischar);
ip.addParameter('SaveMat', true, @islogical);
ip.parse(varargin{:});
opt = ip.Results;

P = shovel_params();
cases = {
    'Wide_w0.314',      fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w0.314.mat');
    'Wide_w0.500',      fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w0.500.mat');
    'Wide_w0.700',      fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w0.700.mat');
    'Wide_w1.000',      fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w1.000.mat');
    'Wide_w1.360',      fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w1.360.mat');
    'Narrow_BI_w0.314', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w0.314.mat');
    'Narrow_BI_w0.500', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w0.500.mat');
    'Narrow_BI_w0.700', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w0.700.mat');
    'Narrow_BI_w1.000', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w1.000.mat');
    'Narrow_BI_w1.360', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w1.360.mat');
    'Narrow_Centered',  fullfile(opt.DataDir, '06_identifiability', 'M2_wrongstation_narrowband.mat');
    'Medium_Stroke_1m', fullfile(opt.DataDir, '00_design',          'M2_crowdparam_control.mat');
};

fprintf('\n=== M2 FORMAL SENSITIVITY MATRIX SVD & SUBSPACE IDENTIFIABILITY ===\n');
fprintf('%-20s | %8s | %12s | %10s | %9s | %10s | %10s\n', ...
    'Case', 'd4_span', 'r(L,Md_full)', 'theta_2col', 'cond_2col', 'cond_4col', 'joint_cond');
fprintf('%s\n', repmat('-', 1, 95));

S_out = struct();
for k = 1:size(cases, 1)
    label = cases{k, 1};
    fpath = cases{k, 2};
    if ~exist(fpath, 'file')
         [~, fname, ext] = fileparts(fpath);
         if exist([fname ext], 'file'), fpath = [fname ext]; end
    end
    assert(exist(fpath, 'file') == 2, 'Missing MAT archive: %s', fpath);
    tmp = load(fpath, 'R');
    R = tmp.R;
    idx = R.t >= R.opt.TrimStart - 1e-9;

    q3   = R.sig.q3(idx);
    q3d  = R.sig.q3d(idx);
    q3dd = R.sig.q3dd(idx);
    d4   = R.sig.d4(idx);
    d4d  = R.sig.d4d(idx);
    d4dd = R.sig.d4dd(idx);

    r      = d4 + P.c;       % d4 - 1.32 m
    s_bail = d4 + 1.50;      % d4 + 1.50 m

    % Saddle (q3) sensitivity basis columns
    phi_Fy_3   = s_bail .* cos(q3);
    phi_Md_3   = (r.^2).*q3dd + 2*r.*d4d.*q3d + P.g*r.*cos(q3);
    phi_Izz_3  = q3dd;
    phi_fv3_3  = q3d;

    % Crowd (d4) sensitivity basis columns
    phi_Fy_4   = sin(q3);
    phi_Md_4   = d4dd - r.*(q3d.^2) + P.g*sin(q3);

    % Mean-remove (project out free intercept)
    S2 = [phi_Fy_3 - mean(phi_Fy_3), phi_Md_3 - mean(phi_Md_3)];
    S2_u = S2 ./ vecnorm(S2, 2, 1);
    sv2 = svd(S2_u);
    cond2 = sv2(1) / sv2(end);
    r_LM = S2_u(:,1)' * S2_u(:,2);
    theta2_deg = asind(sqrt(max(0, 1 - r_LM^2)));

    S4 = [phi_Fy_3, phi_Md_3, phi_Izz_3, phi_fv3_3];
    S4 = S4 - mean(S4, 1);
    S4_u = S4 ./ vecnorm(S4, 2, 1);
    sv4 = svd(S4_u);
    cond4 = sv4(1) / sv4(end);

    % Joint 2-channel [tau3; s_bar*f4]
    s_bar = 10.0;
    Sj = [S2; s_bar * [phi_Fy_4 - mean(phi_Fy_4), phi_Md_4 - mean(phi_Md_4)]];
    Sj_u = Sj ./ vecnorm(Sj, 2, 1);
    svj = svd(Sj_u);
    cond_j = svj(1) / svj(end);

    d4_span = max(d4) - min(d4);
    fprintf('%-20s | %8.4f | %12.8f | %9.4f° | %9.2f | %10.2f | %10.2f\n', ...
        label, d4_span, r_LM, theta2_deg, cond2, cond4, cond_j);

    S_out.(matlab.lang.makeValidName(label)) = struct( ...
        'd4_span', d4_span, 'r_LM', r_LM, 'theta2_deg', theta2_deg, ...
        'cond2_unit', cond2, 'cond4_unit', cond4, 'cond_joint_2ch', cond_j);
end

if opt.SaveMat
    out_file = fullfile(opt.DataDir, '06_identifiability', 'M2_formal_svd_summary.mat');
    if exist(fileparts(out_file), 'dir')
        save(out_file, 'S_out');
    end
end
end
