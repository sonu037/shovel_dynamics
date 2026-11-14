% M4/scripts/execute_MS04P06.m
% Driver script to execute MS04P06 cleanly without shell escaping issues
diary('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics/M4/experiments/06_heldout/MS04P06_output.txt');
diary on;
try
    cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
    addpath('M4/scripts');
    fprintf('Starting MS04P06 execution at %s...\n', datestr(now));
    RES = run_MS04P06_HeldOutValidation();
    fprintf('\nMS04P06 execution completed successfully at %s!\n', datestr(now));
catch ME
    fprintf('\nERROR in MS04P06: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
diary off;
exit;
