function P07 = m2_p07_model_perturbation(varargin)
%M2_P07_MODEL_PERTURBATION  Package P07 model-error fingerprint & attribution suite.
%
%   Evaluates 15 systematic parameter perturbations (dM_d, dI_zz, dc,
%   df_v3, df_v4, and unmodeled Coulomb friction F_c) on the saved Simscape
%   trajectories and verifies basis-regression parameter attribution.
%
%   Repository: M2/scripts/m2_p07_model_perturbation.m
%   Project:    IIT/SRIC/R/AEH/2026/104 (CMPDI) - IIT Kharagpur

ip = inputParser;
ip.addParameter('ArchivePath', fullfile('M2','experiments','02_known_load','M2_100kN_0p1111Hz_sp1p50.mat'), @ischar);
ip.parse(varargin{:});
opt = ip.Results;

P = shovel_params();
if isfield(P, 'm') && ~isfield(P, 'M_d'), P.M_d = P.m; end
if isfield(P, 'Izz') && ~isfield(P, 'I_zz'), P.I_zz = P.Izz; end
if isfield(P, 'fv3') && ~isfield(P, 'f_v3'), P.f_v3 = P.fv3; end
if isfield(P, 'fv4') && ~isfield(P, 'f_v4'), P.f_v4 = P.fv4; end
if isfield(P, 'fc3') && ~isfield(P, 'f_c3'), P.f_c3 = P.fc3; end
if isfield(P, 'fc4') && ~isfield(P, 'f_c4'), P.f_c4 = P.fc4; end
fpath = opt.ArchivePath;
if ~exist(fpath, 'file') && exist('M2_100kN_0p1111Hz_sp1p50.mat', 'file')
    fpath = 'M2_100kN_0p1111Hz_sp1p50.mat';
end
assert(exist(fpath, 'file') == 2, 'Missing MAT archive: %s', fpath);
tmp = load(fpath, 'R');
R = tmp.R;
idx = R.t >= R.opt.TrimStart - 1e-9;

q3   = R.sig.q3(idx);   q3d  = R.sig.q3d(idx);  q3dd = R.sig.q3dd(idx);
d4   = R.sig.d4(idx);   d4d  = R.sig.d4d(idx);  d4dd = R.sig.d4dd(idx);
tau3_meas = R.sig.tau3_meas(idx);
f4_meas   = R.sig.f4_meas(idx);
Qq3_true  = R.sig.Qq3_pred(idx);
Qd4_true  = R.sig.Qd4_pred(idx);

rng3 = max(Qq3_true) - min(Qq3_true);
rng4 = max(Qd4_true) - min(Qd4_true);

r0 = d4 + P.c;
Phi3 = [(r0.^2).*q3dd + 2*r0.*d4d.*q3d + P.g*r0.*cos(q3), q3dd, q3d, tanh(q3d/3e-4)];

cases = {
    'Nominal',          P.M_d,      P.I_zz,      P.c,        P.f_v3,      P.f_v4,      0,   0;
    'Md_+2pct',         1.02*P.M_d, P.I_zz,      P.c,        P.f_v3,      P.f_v4,      0,   0;
    'Md_+5pct',         1.05*P.M_d, P.I_zz,      P.c,        P.f_v3,      P.f_v4,      0,   0;
    'Md_+10pct',        1.10*P.M_d, P.I_zz,      P.c,        P.f_v3,      P.f_v4,      0,   0;
    'Izz_+10pct',       P.M_d,      1.10*P.I_zz, P.c,        P.f_v3,      P.f_v4,      0,   0;
    'Izz_+56pct_M1',    P.M_d,      1.56*P.I_zz, P.c,        P.f_v3,      P.f_v4,      0,   0;
    'COG_c_+0p10m',     P.M_d,      P.I_zz,      P.c + 0.10, P.f_v3,      P.f_v4,      0,   0;
    'Viscous_+50pct',   P.M_d,      P.I_zz,      P.c,        1.50*P.f_v3, 1.50*P.f_v4, 0,   0;
    'Coulomb_Active',   P.M_d,      P.I_zz,      P.c,        P.f_v3,      P.f_v4,      340, 480;
    'Combined_All',     1.05*P.M_d, 1.10*P.I_zz, P.c,        1.20*P.f_v3, 1.20*P.f_v4, 340, 480;
};

fprintf('\n=== M2 P07 MODEL PERTURBATION & FINGERPRINT ATTRIBUTION ===\n');
P07 = struct();
for k = 1:size(cases, 1)
    cname = cases{k,1};
    Md_p = cases{k,2}; Izz_p = cases{k,3}; c_p = cases{k,4};
    fv3_p = cases{k,5}; fv4_p = cases{k,6}; fc3_p = cases{k,7}; fc4_p = cases{k,8};
    rp = d4 + c_p;
    tau3_mod = (Izz_p + Md_p*(rp.^2)).*q3dd + 2*Md_p*rp.*d4d.*q3d + Md_p*P.g*rp.*cos(q3) + fv3_p*q3d + fc3_p*tanh(q3d/3e-4);
    f4_mod   = Md_p*d4dd - Md_p*rp.*(q3d.^2) + Md_p*P.g*sin(q3) + fv4_p*d4d + fc4_p*tanh(d4d/1e-3);
    err3 = (tau3_mod - tau3_meas) - Qq3_true;
    err4 = (f4_mod   - f4_meas)   - Qd4_true;
    est3 = Phi3 \ err3;
    nrmse3 = 100 * rms(err3) / rng3;
    nrmse4 = 100 * rms(err4) / rng4;
    fprintf('%-18s | NRMSE3 = %6.2f%% (RMS %9.1f Nm) | NRMSE4 = %6.2f%% | dMd_hat = %8.1f kg\n', ...
        cname, nrmse3, rms(err3), nrmse4, est3(1));
    vname = matlab.lang.makeValidName(cname);
    P07.(vname) = struct('rms3', rms(err3), 'nrmse3_pct', nrmse3, 'rms4', rms(err4), 'nrmse4_pct', nrmse4, 'est_theta3', est3);
end

repo_root = fileparts(fileparts(mfilename('fullpath')));
if ~exist(fullfile(repo_root, 'M1'), 'dir'), repo_root = pwd; end
out_dir = fullfile(repo_root, 'M2', 'experiments', '07_model_perturbation');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'M2_p07_perturbation_results.mat'), 'P07');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'M2_p07_perturbation_results.mat'));
end
