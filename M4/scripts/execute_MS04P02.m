% M4/scripts/execute_MS04P02.m
% Driver script to execute MS04P02 cleanly without shell escaping issues
diary('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics/M4/experiments/02_awgn/MS04P02_output.txt');
diary on;
try
    cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
    addpath('M4/scripts');
    fprintf('Starting MS04P02 execution at %s...\n', datestr(now));
    RES = run_MS04P02_AWGNSweep();
    fprintf('\nMS04P02 execution completed successfully at %s!\n', datestr(now));
catch ME
    fprintf('\nERROR in MS04P02: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('  File: %s, Line: %d, Name: %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end
end
diary off;
exit;
