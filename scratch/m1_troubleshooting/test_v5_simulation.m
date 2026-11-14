cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
diary('M1/scripts/test_v5_simulation_log.txt');
diary on;
try
    mdl = 'ShovelSimulatorr2025b';
    fprintf('Loading authoritative model: %s...\n', mdl);
    load_system(fullfile(pwd, 'M1', 'model', [mdl, '.slx']));
    set_param(mdl, 'StopTime', '30.0');
    fprintf('Simulating for 30 seconds...\n');
    t_start = tic;
    out = sim(mdl);
    elapsed = toc(t_start);
    fprintf('SUCCESS: Simulation finished in %.2f seconds!\n', elapsed);
    close_system(mdl, 0);
catch ME
    fprintf('ERROR: %s\n', ME.message);
end
diary off;
exit;
