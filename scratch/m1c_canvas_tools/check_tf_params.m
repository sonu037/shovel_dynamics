% scratch/check_tf_params.m
load_system('fl_lib');
b = 'fl_lib/Mechanical/Translational Elements/Translational Friction';
p = get_param(b, 'DialogParameters');
p_names = fieldnames(p);
for i = 1:length(p_names)
    fprintf('Param: %s\n', p_names{i});
end
close_system('fl_lib', 0);
exit;
