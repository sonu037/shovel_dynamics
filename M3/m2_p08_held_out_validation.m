function P08 = m2_p08_held_out_validation(varargin)
%M2_P08_HELD_OUT_VALIDATION  Package P08 cross-trajectory held-out validation suite.
%
%   Evaluates locked M1 parameters and frozen M3 30 Hz momentum observer
%   gains across all held-out M2 Simscape MAT archives (P00, P01, P02, P03,
%   P04, P04b) without parameter retuning.
%
%   Repository: M2/scripts/m2_p08_held_out_validation.m
%   Project:    IIT/SRIC/R/AEH/2026/104 (CMPDI) - IIT Kharagpur

ip = inputParser;
ip.addParameter('DataDir', fullfile('M2','experiments'), @ischar);
ip.parse(varargin{:});
opt = ip.Results;

files = {
    'P01_ZeroForce',      fullfile(opt.DataDir, '01_zero_force',      'M2_null.mat');
    'P02_Baseline_100kN', fullfile(opt.DataDir, '02_known_load',      'M2_100kN_0p1111Hz_sp1p50.mat');
    'P03_Amp_10kN',       fullfile(opt.DataDir, '03_amplitude_sweep', 'M2_03_10kN.mat');
    'P03_Amp_50kN',       fullfile(opt.DataDir, '03_amplitude_sweep', 'M2_03_50kN.mat');
    'P03_Amp_200kN',      fullfile(opt.DataDir, '03_amplitude_sweep', 'M2_03_200kN.mat');
    'P03_Amp_500kN',      fullfile(opt.DataDir, '03_amplitude_sweep', 'M2_03_500kN.mat');
    'P03_Amp_1000kN',     fullfile(opt.DataDir, '03_amplitude_sweep', 'M2_03_1000kN.mat');
    'P04_Wide_w0.500',    fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w0.500.mat');
    'P04_Wide_w0.700',    fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w0.700.mat');
    'P04_Wide_w1.000',    fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w1.000.mat');
    'P04_Wide_w1.360',    fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04_w1.360.mat');
    'P04b_Narrow_w0.314', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w0.314.mat');
    'P04b_Narrow_w1.360', fullfile(opt.DataDir, '04_frequency_sweep', 'M2_04n_w1.360.mat');
    'P05_Config_7p75m',   fullfile(opt.DataDir, '05_configuration_sweep', 'M2_05_d4bias_7p75m.mat');
    'P05_Config_8p75m',   fullfile(opt.DataDir, '05_configuration_sweep', 'M2_05_d4bias_8p75m.mat');
    'P05_Config_9p77m',   fullfile(opt.DataDir, '05_configuration_sweep', 'M2_05_d4bias_9p77m.mat');
    'P00_Medium_1m',      fullfile(opt.DataDir, '00_design',          'M2_crowdparam_control.mat');
};

fprintf('\n=== M2 P08 CROSS-TRAJECTORY HELD-OUT VALIDATION ===\n');
P08 = struct();
for k = 1:size(files, 1)
    label = files{k, 1};
    fpath = files{k, 2};
    if ~exist(fpath, 'file')
        [~, fn, ext] = fileparts(fpath);
        if exist([fn ext], 'file'), fpath = [fn ext]; end
    end
    if exist(fpath, 'file') == 2
        tmp = load(fpath, 'R');
        R = tmp.R;
        nrmse3 = 0; rms3 = 0;
        if isfield(R.q3, 'NRMSE_pct'), nrmse3 = R.q3.NRMSE_pct; rms3 = R.q3.rmsErr;
        elseif isfield(R.q3, 'rmsResidual'), rms3 = R.q3.rmsResidual; end
        fprintf('%-20s | M2_RMS_q3 = %.3e Nm (NRMSE = %.3e%%)\n', label, rms3, nrmse3);
        P08.(matlab.lang.makeValidName(label)) = struct('rms3', rms3, 'nrmse3_pct', nrmse3);
    end
end

repo_root = fileparts(fileparts(mfilename('fullpath')));
if ~exist(fullfile(repo_root, 'M1'), 'dir'), repo_root = pwd; end
out_dir = fullfile(repo_root, 'M2', 'experiments', '08_held_out');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'M2_p08_held_out_results.mat'), 'P08');
fprintf('\nSaved: %s\n', fullfile(out_dir, 'M2_p08_held_out_results.mat'));
end
