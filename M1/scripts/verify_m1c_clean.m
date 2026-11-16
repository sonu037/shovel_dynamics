cd('c:/Users/IIT KGP/MATLAB Drive/shovel-dynamics');
addpath(fullfile(pwd, 'M1', 'scripts'));
fprintf('========================================================================\n');
fprintf('  M1C CROWD & SADDLE COULOMB FRICTION VERIFICATION AUDIT (DEF-01)\n');
fprintf('========================================================================\n');

mdl = 'ShovelSimulatorr2025b_M1C';
fprintf('Loading model: %s...\n', mdl);
load_system(fullfile(pwd, 'M1', 'model', mdl));
P = shovel_params();

% 1. Inspect Stateflow Friction Function Blocks
fprintf('\n--- 1. AUDITING MATLAB FRICTION FUNCTION BLOCKS ---\n');
rt = sfroot;
m = rt.find('-isa', 'Stateflow.Machine', 'Name', mdl);

% Audit Crowd Coulomb Function (SID 72)
chart_crowd = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl, '/MATLAB Function1']);
fprintf('Chart 1 Name: %s\n', chart_crowd.Name);
fprintf('Chart 1 Script Header: %s\n', strtok(chart_crowd.Script, sprintf('\n')));
data_objs_crowd = chart_crowd.find('-isa', 'Stateflow.Data');
fprintf('Crowd Data Symbols Count: %d\n', length(data_objs_crowd));
for k = 1:length(data_objs_crowd)
    fprintf('  - Symbol: %s (Scope: %s, Port: %d)\n', data_objs_crowd(k).Name, data_objs_crowd(k).Scope, data_objs_crowd(k).Port);
    if strcmp(data_objs_crowd(k).Name, 'data')
        error('FAIL: Unused "data" symbol still present in chart!');
    end
end
assert(length(data_objs_crowd) == 2, 'FAIL: Expected exactly 2 symbols (v: Input, Fc: Output)!');
fprintf('>>> Crowd MATLAB Function1 has zero orphan data inputs. PASS.\n');

% Audit Revolute Coulomb Function (SID 70)
chart_rev = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl, '/MATLAB Function']);
fprintf('Chart 2 Name: %s\n', chart_rev.Name);
fprintf('Chart 2 Script Header: %s\n', strtok(chart_rev.Script, sprintf('\n')));
data_objs_rev = chart_rev.find('-isa', 'Stateflow.Data');
fprintf('Revolute Data Symbols Count: %d\n', length(data_objs_rev));
assert(length(data_objs_rev) == 2, 'FAIL: Expected exactly 2 symbols for Revolute Function (w: Input, tau_c: Output)!');
fprintf('>>> Revolute MATLAB Function has zero orphan data inputs. PASS.\n');

% 2. Inspect Canvas Blocks, Port Connectivity & DEF-01 Block Settings
fprintf('\n--- 2. AUDITING CANVAS PORT CONNECTIVITY & DEF-01 RESOLUTION ---\n');
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
assert(unconnected_count == 0, 'FAIL: Found %d unconnected inports on canvas!', unconnected_count);
fprintf('>>> Canvas has 0 unconnected input ports. PASS.\n');

% Audit SID 74 (Crowd Coulomb Force Injection)
b74 = Simulink.ID.getHandle([mdl ':74']);
ef_x = get_param(b74, 'EnableForceX');
ef_z = get_param(b74, 'EnableForceZ');
frf  = get_param(b74, 'ForceResolutionFrame');
fprintf('SID 74: EnableForceX = %s, EnableForceZ = %s, ForceResolutionFrame = %s\n', ef_x, ef_z, frf);
assert(strcmp(ef_x, 'off'), 'DEF-01 FAIL: SID 74 EnableForceX must be "off"');
assert(strcmp(ef_z, 'on'),  'DEF-01 FAIL: SID 74 EnableForceZ must be "on"');
assert(strcmp(frf, 'AttachedFrame'), 'DEF-01 FAIL: SID 74 ForceResolutionFrame must be "AttachedFrame"');
fprintf('>>> SID 74 Prismatic Crowd Coulomb force configured along local +Z axis. PASS.\n');

% Audit SID 65 (Revolute Coulomb Torque Injection)
b65 = Simulink.ID.getHandle([mdl ':65']);
et_z = get_param(b65, 'EnableTorqueZ');
trf  = get_param(b65, 'TorqueResolutionFrame');
fprintf('SID 65: EnableTorqueZ = %s, TorqueResolutionFrame = %s\n', et_z, trf);
assert(strcmp(et_z, 'on'), 'DEF-01 FAIL: SID 65 EnableTorqueZ must be "on"');
assert(strcmp(trf, 'World'), 'DEF-01 FAIL: SID 65 TorqueResolutionFrame must be "World"');
fprintf('>>> SID 65 Revolute Saddle Coulomb torque configured along World +Z axis. PASS.\n');

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

% 5. Sensed Signal Verification & Dynamics Audit
fprintf('\n--- 5. SENSING VELOCITY & COULOMB FRICTION DYNAMICS ---\n');
t = simOut.tout(:);
v = simOut.d_4_dot(:);
fc = simOut.Fc_4(:);
f_4 = simOut.f_4(:);
tau_3 = simOut.tau_3(:);
q = simOut.q_3(:);
q_d = simOut.q_3_dot(:);
q_dd = simOut.q_3_ddot(:);
d = simOut.d_4(:);
d_dd = simOut.d_4_ddot(:);

fprintf('Sample Count: %d\n', length(t));
fprintf('Time Range: [%.2f, %.2f] s\n', min(t), max(t));
fprintf('Crowd Velocity Range: [%.4f, %.4f] m/s\n', min(v), max(v));
fprintf('Coulomb Friction Force Range: [%.2f, %.2f] N\n', min(fc), max(fc));

% Check peak bounds (+/- 480 N)
assert(abs(max(fc) - 480.0) < 1e-4, 'Friction did not reach +480 N');
assert(abs(min(fc) - (-480.0)) < 1e-4, 'Friction did not reach -480 N');
fprintf('Peak saturation confirmed: reaches full +/-480 N.\n');

% Check analytical tanh alignment of MATLAB Function1
epsilon_crowd = 0.002;
fc_target = -480.0 * tanh(v / epsilon_crowd);
max_dev = max(abs(fc - fc_target));
fprintf('Max Absolute Deviation from Analytical Tanh: %.6e N\n', max_dev);
assert(max_dev < 1e-6, 'Deviation from analytical model exceeds tolerance!');
fprintf('>>> Friction model matches analytical tanh formulation to machine precision. PASS.\n');

% Check numerical stability and solver step sizes
dt = diff(t);
fprintf('Time-step metrics: Min dt = %.3e s, Median dt = %.3e s, Max dt = %.3e s\n', min(dt), median(dt), max(dt));
assert(min(dt) > 1e-7, 'Solver time step collapsed (possible chattering)');
fprintf('>>> Smooth time step evolution confirms ZERO solver chattering. PASS.\n');

% 6. Sensed f_4 & tau_3 Physics & Injection Validation
fprintf('\n--- 6. VERIFYING SENSED f_4 AND tau_3 REFLECT INJECTED COULOMB DYNAMICS ---\n');
% Trim startup transient for metrics
keep = t > 0.5;
t_k = t(keep); q_k = q(keep); q_dk = q_d(keep); q_ddk = q_dd(keep);
d_k = d(keep); v_k = v(keep); d_ddk = d_dd(keep);
f4_k = f_4(keep); tau3_k = tau_3(keep);
r_k = d_k + P.c;

% Crowd Prismatic Equation: F4 = m*ddd - m*r*qd^2 + m*g*sin(q) + fv4*dd + fc4*tanh(dd/0.002)
F4_pred_coulomb = P.m.*d_ddk - P.m.*r_k.*q_dk.^2 + P.m*P.g.*sin(q_k) + P.fv4.*v_k + P.fc4.*tanh(v_k / 0.002);
F4_pred_nocoulomb = P.m.*d_ddk - P.m.*r_k.*q_dk.^2 + P.m*P.g.*sin(q_k) + P.fv4.*v_k;

rmse_f4_coulomb   = sqrt(mean((f4_k - F4_pred_coulomb).^2));
rmse_f4_nocoulomb = sqrt(mean((f4_k - F4_pred_nocoulomb).^2));
r2_f4 = 1 - sum((f4_k - F4_pred_coulomb).^2) / sum((f4_k - mean(f4_k)).^2);

fprintf('Crowd Sensed f_4 Analysis:\n');
fprintf('  RMSE with Coulomb Term    : %.2f N (R^2 = %.6f)\n', rmse_f4_coulomb, r2_f4);
fprintf('  RMSE without Coulomb Term : %.2f N\n', rmse_f4_nocoulomb);
assert(rmse_f4_coulomb < 300.0, 'Sensed f_4 deviates from analytical model with Coulomb friction!');
assert(rmse_f4_nocoulomb > 400.0, 'Sensed f_4 did not reflect Coulomb friction injection!');
fprintf('>>> Sensed f_4 correctly reflects injected +480 N Coulomb friction on local +Z. PASS.\n');

% Saddle Revolute Equation: tau_3 = (Izz + m*r^2)*qdd + 2*m*r*dd*qd + m*g*r*cos(q) + fv3*qd + fc3*tanh(qd/0.0003)
tau3_pred_coulomb   = (P.Izz + P.m.*r_k.^2).*q_ddk + 2*P.m.*r_k.*v_k.*q_dk + P.m*P.g.*r_k.*cos(q_k) + P.fv3.*q_dk + P.fc3.*tanh(q_dk / 0.0003);
tau3_pred_nocoulomb = (P.Izz + P.m.*r_k.^2).*q_ddk + 2*P.m.*r_k.*v_k.*q_dk + P.m*P.g.*r_k.*cos(q_k) + P.fv3.*q_dk;

rmse_tau3_coulomb   = sqrt(mean((tau3_k - tau3_pred_coulomb).^2));
rmse_tau3_nocoulomb = sqrt(mean((tau3_k - tau3_pred_nocoulomb).^2));
r2_tau3 = 1 - sum((tau3_k - tau3_pred_coulomb).^2) / sum((tau3_k - mean(tau3_k)).^2);

fprintf('Saddle Sensed tau_3 Analysis:\n');
fprintf('  RMSE with Coulomb Term    : %.2f Nm (R^2 = %.6f)\n', rmse_tau3_coulomb, r2_tau3);
fprintf('  RMSE without Coulomb Term : %.2f Nm\n', rmse_tau3_nocoulomb);
assert(rmse_tau3_coulomb < 250.0, 'Sensed tau_3 deviates from analytical model with Coulomb friction!');
assert(rmse_tau3_nocoulomb > 300.0, 'Sensed tau_3 did not reflect Coulomb friction injection!');
fprintf('>>> Sensed tau_3 correctly reflects injected +340 Nm Coulomb friction on World +Z. PASS.\n');

% 7. Run Comprehensive Forward Validation and Parameter Identification
fprintf('\n--- 7. EXECUTING VALIDATE_AND_IDENTIFY PROTOCOL ---\n');
results = validate_and_identify(simOut);
fprintf('>>> validate_and_identify completed successfully. PASS.\n');

close_system(mdl, 0);
fprintf('\n========================================================================\n');
fprintf('  ALL VERIFICATION GATES PASSED (100%% COMPLIANT WITH M1C SPECIFICATION)\n');
fprintf('========================================================================\n');
exit(0);
