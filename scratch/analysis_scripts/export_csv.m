cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
load('M1/experiments/coulomb_friction_verification_data.mat');
csv_path = fullfile(pwd, 'M1', 'experiments', 'coulomb_verification_data.csv');
T = table(t, v, fc, fc_theoretical, 'VariableNames', {'time_s', 'velocity_mps', 'fc_simulated_N', 'fc_theoretical_N'});
writetable(T, csv_path);
fprintf('Exported %d rows to %s\n', height(T), csv_path);
exit;
