% scratch/apply_translational_friction.m
cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
src_slx = fullfile(pwd, 'M1', 'model', 'ShovelSimulatorr2025b_M1C.slx');
[~, mdl, ~] = fileparts(src_slx);
load_system(src_slx);
fprintf('1. Loaded model: %s\n', mdl);

load_system('fl_lib');
fprintf('2. Loaded fl_lib\n');

tf_blk = [mdl, '/Translational Friction'];
% If block already exists (from earlier attempt), remove or update
if ~isempty(find_system(mdl, 'Name', 'Translational Friction'))
    delete_block(tf_blk);
    fprintf('Removed old block instance\n');
end

add_block('fl_lib/Mechanical/Translational Elements/Translational Friction', tf_blk);
fprintf('3. Block added: %s\n', tf_blk);

% Set parameters for 480 N Coulomb force (Rasuli 2014 Table II)
set_param(tf_blk, 'Col_frc', '480');
set_param(tf_blk, 'Col_frc_unit', 'N');
set_param(tf_blk, 'brkwy_frc', '480');
set_param(tf_blk, 'brkwy_frc_unit', 'N');
set_param(tf_blk, 'brkwy_vel', '0.003');
set_param(tf_blk, 'brkwy_vel_unit', 'm/s');
set_param(tf_blk, 'visc_coef', '0');
set_param(tf_blk, 'visc_coef_unit', 'N*s/m');
fprintf('4. Parameters set: Col_frc=480 N, brkwy_frc=480 N, brkwy_vel=0.003 m/s, visc_coef=0\n');

% Position it directly below Rotational Friction
rot_pos = get_param([mdl, '/Rotational Friction'], 'Position');
new_pos = [rot_pos(1), rot_pos(4) + 40, rot_pos(3), rot_pos(4) + 70];
set_param(tf_blk, 'Position', new_pos);
fprintf('5. Position set: [%d, %d, %d, %d]\n', new_pos(1), new_pos(2), new_pos(3), new_pos(4));

save_system(mdl);
fprintf('6. Model saved successfully!\n');

close_system(mdl);
close_system('fl_lib', 0);
fprintf('7. Closed systems. All done!\n');
exit;
