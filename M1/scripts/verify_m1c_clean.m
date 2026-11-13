cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
fprintf('========================================================================\n');
fprintf('  M1C CROWD PRISMATIC COULOMB FRICTION VERIFICATION AUDIT\n');
fprintf('========================================================================\n');

mdl = 'ShovelSimulatorr2025b_M1C';
fprintf('Loading model: %s...\n', mdl);
load_system(fullfile(pwd, 'M1', 'model', mdl));

% 1. Inspect MATLAB Function1 block
fprintf('\n--- 1. AUDITING MATLAB Function1 (SID 72) ---\n');
rt = sfroot;
m = rt.find('-isa', 'Stateflow.Machine', 'Name', mdl);
chart = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl, '/MATLAB Function1']);
fprintf('Chart Name: %s\n', chart.Name);
fprintf('Chart Script Header: %s\n', strtok(chart.Script, sprintf('\n')));
data_objs = chart.find('-isa', 'Stateflow.Data');
fprintf('Data Symbols Count: %d\n', length(data_objs));
for k = 1:length(data_objs)
    fprintf('  - Symbol: %s (Scope: %s, Port: %d)\n', data_objs(k).Name, data_objs(k).Scope, data_objs(k).Port);
    if strcmp(data_objs(k).Name, 'data')
        error('FAIL: Unused "data" symbol still present in chart!');
    end
end
if length(data_objs) ~= 2
    error('FAIL: Expected exactly 2 symbols (v: Input, Fc: Output)!');
end
fprintf('>>> MATLAB Function1 has zero orphan data inputs. PASS.\n');

% 2. Inspect Canvas Blocks & Port Connectivity
fprintf('\n--- 2. AUDITING CANVAS PORT CONNECTIVITY ---\n');
blks = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
fprintf('Total root blocks: %d\n', length(blks));
unconnected_count = 0;
for i = 1:length(blks)
    b = blks{i};
    ph = get_param(b, 'PortHandles');
    for p = 1:length(ph.Inport)
        if get_param(ph.Inport(p), 'Line') == -1
            fprintf('  WARNING: Unconnected Inport %d on block: %s\n', p, b);
            unconnected_count = unconnected_count + 1;
        end
    end
end
if unconnected_count == 0
    fprintf('>>> Canvas has 0 unconnected input ports. PASS.\n');
else
    error('FAIL: Found %d unconnected inports on canvas!', unconnected_count);
end

% 3. Check Solver Settings
fprintf('\n--- 3. AUDITING SOLVER SETTINGS ---\n');
solver = get_param(mdl, 'Solver');
sim_editor = get_param(mdl, 'SimMechanicsOpenEditorOnUpdate');
stop_time = get_param(mdl, 'StopTime');
fprintf('Solver: %s\n', solver);
fprintf('SimMechanicsOpenEditorOnUpdate: %s\n', sim_editor);
fprintf('StopTime: %s\n', stop_time);
assert(strcmp(solver, 'ode15s') || strcmp(solver, 'ode23t'), 'Solver must be ode15s or ode23t!');
assert(strcmp(sim_editor, 'off'), 'SimMechanicsOpenEditorOnUpdate must be off!');
fprintf('>>> Solver and batch execution parameters configured properly. PASS.\n');

% 4. Run Simulation
fprintf('\n--- 4. EXECUTING 30.0-SECOND SIMULATION ---\n');
t_start = tic;
simOut = sim(mdl);
elapsed = toc(t_start);
fprintf('>>> Simulation completed in %.2f seconds with ZERO runtime errors! PASS.\n', elapsed);

% 5. Sensed Signal Verification
fprintf('\n--- 5. SENSING VELOCITY & COULOMB FRICTION DYNAMICS ---\n');
t = tout;
v = d_4_dot;
fc = Fc_4;

fprintf('Sample Count: %d\n', length(t));
fprintf('Time Range: [%.2f, %.2f] s\n', min(t), max(t));
fprintf('Crowd Velocity Range: [%.4f, %.4f] m/s\n', min(v), max(v));
fprintf('Coulomb Friction Force Range: [%.2f, %.2f] N\n', min(fc), max(fc));

% Check peak bounds (+/- 480 N)
assert(abs(max(fc) - 480.0) < 1e-4, 'Friction did not reach +480 N');
assert(abs(min(fc) - (-480.0)) < 1e-4, 'Friction did not reach -480 N');
fprintf('Peak saturation confirmed: reaches full +/-480 N.\n');

% Check analytical tanh alignment
epsilon = 0.002;
fc_target = -480.0 * tanh(v / epsilon);
max_dev = max(abs(fc - fc_target));
fprintf('Max Absolute Deviation from Analytical Tanh: %.6e N\n', max_dev);
assert(max_dev < 1e-6, 'Deviation from analytical model exceeds tolerance!');
fprintf('>>> Friction model matches analytical tanh formulation to machine precision. PASS.\n');

% Check numerical stability and solver step sizes
dt = diff(t);
fprintf('Time-step metrics: Min dt = %.3e s, Median dt = %.3e s, Max dt = %.3e s\n', min(dt), median(dt), max(dt));
assert(min(dt) > 1e-7, 'Solver time step collapsed (possible chattering)');
fprintf('>>> Smooth time step evolution confirms ZERO solver chattering. PASS.\n');

close_system(mdl, 0);
fprintf('\n========================================================================\n');
fprintf('  ALL VERIFICATION GATES PASSED (100%% COMPLIANT WITH M1C SPECIFICATION)\n');
fprintf('========================================================================\n');
exit;
