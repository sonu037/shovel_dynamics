function P05 = m2_p05_config_sweep(varargin)
%M2_P05_CONFIG_SWEEP  Package P05 configuration-space observability & sweep suite.
%
%   Evaluates the two-channel planar Jacobian observability maps across
%   (q3, d4), verifies configuration-quadrant recovery errors on saved
%   Simscape MAT archives, and optionally runs a 9-point pose grid on
%   M2_LoadInjection.slx when Simulink is available.
%
%   Repository: M2/scripts/m2_p05_config_sweep.m
%   Project:    IIT/SRIC/R/AEH/2026/104 (CMPDI) - IIT Kharagpur

ip = inputParser;
ip.addParameter('Mode', 'AnalyticalAndArchive', @(s)ischar(s)||isstring(s));
ip.addParameter('ArchivePath', fullfile('M2','experiments','02_known_load','M2_100kN_0p1111Hz_sp1p50.mat'), @ischar);
ip.parse(varargin{:});
opt = ip.Results;

P = shovel_params();
q3_deg_grid = [15, 30, 45, 60, 75, 85, 90];
d4_m_grid   = [6.75, 7.75, 8.75, 9.77, 10.75];

[Q3D, D4M] = meshgrid(q3_deg_grid, d4_m_grid);
Q3R = deg2rad(Q3D);
S_BAIL = D4M + 1.50;
R_COG  = D4M + P.c;
RHO    = S_BAIL ./ R_COG;
DRHO   = -2.82 ./ (R_COG .^ 2);

% Vertical unit force Fy = 1 N sensitivities
G_Fy_q3 = S_BAIL .* cos(Q3R);
G_Fy_d4 = sin(Q3R);
Obs_Norm = sqrt((G_Fy_q3 ./ S_BAIL).^2 + G_Fy_d4.^2); % Identically 1.0

P05 = struct();
P05.q3_deg_grid = q3_deg_grid;
P05.d4_m_grid   = d4_m_grid;
P05.S_BAIL      = S_BAIL;
P05.R_COG       = R_COG;
P05.RHO         = RHO;
P05.DRHO        = DRHO;
P05.G_Fy_q3     = G_Fy_q3;
P05.G_Fy_d4     = G_Fy_d4;
P05.Obs_Norm    = Obs_Norm;

% Quadrant verification on M2_100kN_0p1111Hz_sp1p50.mat
fpath = opt.ArchivePath;
if ~exist(fpath, 'file') && exist('M2_100kN_0p1111Hz_sp1p50.mat', 'file')
    fpath = 'M2_100kN_0p1111Hz_sp1p50.mat';
end
if exist(fpath, 'file') == 2
    tmp = load(fpath, 'R');
    R = tmp.R;
    idx = R.t >= R.opt.TrimStart - 1e-9;
    q3_deg = rad2deg(R.sig.q3(idx));
    d4_m   = R.sig.d4(idx);
    err3   = abs(R.sig.e_tau3(idx) - R.sig.ref3(idx));
    err4   = abs(R.sig.e_f4(idx)   - R.sig.ref4(idx));
    P05.archive_rms_err3 = rms(err3);
    P05.archive_rms_err4 = rms(err4);
    fprintf('P05 Archive Quadrant Check (%s): RMS_err3 = %.3e Nm, RMS_err4 = %.3e N\n', ...
        fpath, P05.archive_rms_err3, P05.archive_rms_err4);
end

if strcmpi(opt.Mode, 'RunSimulinkGrid')
    assert(exist('M2_LoadInjection.slx', 'file') == 4, ...
        'Simulink model M2_LoadInjection.slx not found on MATLAB path.');
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    if ~exist(fullfile(repo_root, 'M1'), 'dir')
        repo_root = pwd;
    end
    out_dir = fullfile(repo_root, 'M2', 'experiments', '05_configuration_sweep');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    for d_bias = [7.75, 8.75, 9.77]
        cname = sprintf('M2_05_d4bias_%dp%02dm', floor(d_bias), round(100*(d_bias-floor(d_bias))));
        recover_load('CaseName', cname, 'OutDir', out_dir, ...
            'Amplitude', 1e5, 'Frequency', 2*pi/9, 'Station', 1.50, ...
            'CrowdBias', d_bias, 'CrowdAmplitude', 0.50, 'Save', true);
    end
end
end
