% M4/scripts/execute_MS04P04.m
% Driver script to execute MS04P04 cleanly without shell escaping issues
diary('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics/M4/experiments/04_harmonics/MS04P04_output.txt');
diary on;
try
    cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
    addpath('M4/scripts');
    fprintf('Starting MS04P04 execution at %s...\n', datestr(now));
    RES = run_MS04P04_InverterHarmonics();
    fprintf('\nMS04P04 execution completed successfully at %s!\n', datestr(now));
catch ME
    fprintf('\nERROR in MS04P04: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
diary off;
exit;
