% scratch/test_add_block.m
cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
fprintf('1. Working directory: %s\n', pwd);

src_slx = fullfile(pwd, 'M1', 'model', 'ShovelSimulatorr2025b_M1C.slx');
fprintf('2. Source SLX: %s\n', src_slx);

backup_dir = fullfile(pwd, '_safety_backups');
if ~exist(backup_dir, 'dir'), mkdir(backup_dir); end
backup_slx = fullfile(backup_dir, 'ShovelSimulatorr2025b_M1C_before_adding_fc4.slx');
copyfile(src_slx, backup_slx);
fprintf('3. Safety backup created: %s\n', backup_slx);

[~, mdl, ~] = fileparts(src_slx);
load_system(src_slx);
fprintf('4. Model loaded: %s\n', mdl);

load_system('fl_lib');
fprintf('5. fl_lib loaded\n');

tf_blk = [mdl, '/Translational Friction'];
try
    add_block('fl_lib/Mechanical/Translational Elements/Translational Friction', tf_blk);
    fprintf('6. Block added: %s\n', tf_blk);
    
    set_param(tf_blk, 'Col_force', '480');
    set_param(tf_blk, 'Col_force_unit', 'N');
    set_param(tf_blk, 'brkwy_force', '480');
    set_param(tf_blk, 'brkwy_force_unit', 'N');
    set_param(tf_blk, 'brkwy_vel', '0.003');
    set_param(tf_blk, 'brkwy_vel_unit', 'm/s');
    fprintf('7. Parameters set (480 N)\n');
    
    rot_pos = get_param([mdl, '/Rotational Friction'], 'Position');
    new_pos = [rot_pos(1), rot_pos(4) + 40, rot_pos(3), rot_pos(4) + 70];
    set_param(tf_blk, 'Position', new_pos);
    fprintf('8. Position set: [%d, %d, %d, %d]\n', new_pos(1), new_pos(2), new_pos(3), new_pos(4));
    
    save_system(mdl);
    fprintf('9. Model saved successfully!\n');
catch ME
    fprintf('Error during add/set: %s\n', ME.message);
end

close_system(mdl, 0);
close_system('fl_lib', 0);
fprintf('10. Closed systems, all done!\n');
exit;
