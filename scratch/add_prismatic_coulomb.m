% scratch/add_prismatic_coulomb.m
diary('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics/scratch/add_prismatic_coulomb_log.txt');
diary on;
try
    cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
    src_slx = fullfile(pwd, 'M1', 'model', 'ShovelSimulatorr2025b_M1C.slx');
    backup_dir = fullfile(pwd, '_safety_backups');
    if ~exist(backup_dir, 'dir'), mkdir(backup_dir); end
    backup_slx = fullfile(backup_dir, 'ShovelSimulatorr2025b_M1C_before_adding_fc4.slx');
    copyfile(src_slx, backup_slx);
    fprintf('Created safety backup at: %s\n', backup_slx);
    
    [~, mdl, ~] = fileparts(src_slx);
    load_system(src_slx);
    fprintf('Loaded model: %s\n', mdl);
    
    % Check if Translational Friction already exists
    tf_blk = [mdl, '/Translational Friction'];
    if isempty(find_system(mdl, 'Name', 'Translational Friction'))
        fprintf('Adding Translational Friction block from fl_lib...\n');
        load_system('fl_lib');
        add_block('fl_lib/Mechanical/Translational Elements/Translational Friction', tf_blk);
        fprintf('Block added: %s\n', tf_blk);
    else
        fprintf('Block already exists: %s\n', tf_blk);
    end
    
    % Set parameters for 480 N Coulomb force (Rasuli 2014 Table II)
    set_param(tf_blk, 'Col_force', '480');
    set_param(tf_blk, 'Col_force_unit', 'N');
    set_param(tf_blk, 'brkwy_force', '480');
    set_param(tf_blk, 'brkwy_force_unit', 'N');
    set_param(tf_blk, 'brkwy_vel', '0.003');
    set_param(tf_blk, 'brkwy_vel_unit', 'm/s');
    
    % Position it neatly below Rotational Friction
    rot_pos = get_param([mdl, '/Rotational Friction'], 'Position');
    % rot_pos is [left, top, right, bottom]
    new_pos = [rot_pos(1), rot_pos(4) + 30, rot_pos(3), rot_pos(4) + 60];
    set_param(tf_blk, 'Position', new_pos);
    fprintf('Set Position to [%d, %d, %d, %d]\n', new_pos(1), new_pos(2), new_pos(3), new_pos(4));
    
    % Save model
    save_system(mdl);
    fprintf('Model %s successfully saved with Translational Friction block (480 N)!\n', mdl);
    close_system(mdl);
    close_system('fl_lib', 0);
catch ME
    fprintf('Error: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
diary off;
exit;
