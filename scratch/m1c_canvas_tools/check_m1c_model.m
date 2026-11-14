% scratch/check_m1c_model.m
diary('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics/scratch/check_m1c_log.txt');
diary on;
try
    cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
    mdl_path = fullfile(pwd, 'M1', 'model', 'ShovelSimulatorr2025b_M1C.slx');
    fprintf('Loading model: %s\n', mdl_path);
    [~, mdl, ~] = fileparts(mdl_path);
    load_system(mdl_path);
    fprintf('Model loaded successfully: %s\n', mdl);
    
    blks = find_system(mdl, 'Type', 'Block');
    fprintf('Total blocks: %d\n', length(blks));
    for i = 1:length(blks)
        b = blks{i};
        if contains(lower(b), 'friction') || contains(lower(b), 'joint') || contains(lower(b), 'prismatic') || contains(lower(b), 'revolute')
            st = get_param(b, 'BlockType');
            fprintf('  Block: %s | Type: %s\n', b, st);
            try
                sb = get_param(b, 'SourceBlock');
                fprintf('    SourceBlock: %s\n', sb);
            catch
            end
            try
                params = get_param(b, 'DialogParameters');
                p_names = fieldnames(params);
                for p_idx = 1:length(p_names)
                    pn = p_names{p_idx};
                    if contains(lower(pn), 'fric') || contains(lower(pn), 'trq') || contains(lower(pn), 'force') || contains(lower(pn), 'damp')
                        val = get_param(b, pn);
                        fprintf('    %s: %s\n', pn, val);
                    end
                end
            catch
            end
        end
    end
    close_system(mdl, 0);
catch ME
    fprintf('Error: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
diary off;
exit;
