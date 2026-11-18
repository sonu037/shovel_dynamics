function build_M2_TorqueDriven_Validation(varargin)
%BUILD_M2_TORQUEDRIVEN_VALIDATION  Build the torque-driven Simscape plant for MS02P08B.
%
%   Creates   M2/model/M2_TorqueDriven_Validation.slx
%   from      M2/model/M2_LoadInjection.slx   (verified motion-prescribed M2 model)
%   The source model file is never modified.
%
%   WHY: MS02P08B (torque-driven held-out case) was a code check only: the
%   9/16 Nov runs integrated the analytic equations with the estimator's own
%   parameters, and M2_TorqueDriven.slx was never simulated (its joint inputs
%   are unconnected and it has no controller). This builder produces a plant
%   that can actually be simulated with sim().
%
%   CHANGES relative to M2_LoadInjection.slx
%     1. Revolute (q3) and prismatic (d4) joints: motion input -> torque/force
%        input (MotionActuationMode = ComputedMotion, TorqueActuationMode = InputTorque).
%        Initial position/velocity targets come from q3_0_deg, q3d_0_degps, d4_0, d4d_0.
%     2. The two motion-input Sine Wave + Simulink-PS Converter pairs are removed.
%     3. One PID position controller per joint (basic Simulink blocks) tracks
%            q3_ref = q3_B + q3_A sin(omega_traj t)        [rad]
%            d4_ref = d4_B + d4_A sin(omega_traj t)        [m]
%        The controller output is the DRIVE torque/force: logged as tau3_cmd, f4_cmd.
%        This is the "measured torque" the estimator receives.
%     4. Plant Coulomb friction (Rasuli 2014 Table II: fc3 = 340 N m, fc4 = 480 N)
%        is subtracted from the drive torque before it reaches the joint:
%            tau_joint = tau_cmd - fc * tanh(qdot / 1e-3)
%        Viscous friction stays in the joint damping, unchanged
%        (66.322512 N m/(deg/s) = 3800 N m s/rad; 1473 N s/m).
%     5. Sensor converters get explicit SI units; every To Workspace block logs
%        at Ts_log in Timeseries format.
%     6. The model workspace is cleared, so values passed with setVariable cannot
%        be shadowed (defect of 2026-09-02).
%     7. Solver: ode15s, RelTol 1e-8, AbsTol 1e-9, MaxStep 1e-3.
%   UNCHANGED: body mass / COM / inertia (54,300 kg; COM 2.82 m behind the bail;
%   Izz 287,900 kg m^2), the 1.5 m datum transform, gravity, and the external
%   force at the bail frame (Fy, World frame, amplitude F0_y, frequency omega_F).
%
%   USAGE (MATLAB R2025b desktop, any current folder):
%     build_M2_TorqueDriven_Validation
%     build_M2_TorqueDriven_Validation('Overwrite', true)   % rebuild an existing file
%
%   OUTPUT: the .slx above plus M2_TorqueDriven_Validation_BUILD.txt (build receipt).
%   NEXT:   run_MS02P08B_simscape('SmokeTest', true), then run_MS02P08B_simscape.
%
%   Block SIDs below were read from M2_LoadInjection.slx (system_root.xml, 2026-11-18).
%   Each one is checked before use; the build stops if the source model differs.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18 (untested:
%   written without MATLAB; first run must be checked by the researcher).

    p = inputParser;
    addParameter(p, 'Overwrite', false, @islogical);
    parse(p, varargin{:});

    here    = fileparts(mfilename('fullpath'));          % .../M2/scripts
    root    = fileparts(fileparts(here));                % repo root
    srcName = 'M2_LoadInjection';
    dstName = 'M2_TorqueDriven_Validation';
    srcFile = fullfile(root, 'M2', 'model', [srcName '.slx']);
    dstFile = fullfile(root, 'M2', 'model', [dstName '.slx']);

    if ~isfile(srcFile)
        error('build:noSource', 'Source model not found: %s', srcFile);
    end
    if isfile(dstFile) && ~p.Results.Overwrite
        error('build:exists', ['%s already exists. Pass ''Overwrite'', true to rebuild ' ...
              '(the old file is replaced; git keeps its history).'], dstFile);
    end
    if bdIsLoaded(srcName), close_system(srcName, 0); end
    if bdIsLoaded(dstName), close_system(dstName, 0); end

    srcMD5 = local_md5(srcFile);
    fprintf('Source: %s (MD5 %s)\n', srcFile, srcMD5);

    % ---- 1. Copy: save the loaded source under the new name (source file untouched)
    load_system(srcFile);
    save_system(srcName, dstFile);          % in-memory model is now dstName
    mdl = dstName;
    if ~bdIsLoaded(mdl), load_system(dstFile); end

    % ---- 2. Locate and verify the blocks we rely on (by SID)
    hRev   = local_h(mdl, 29, 'Revolute Joint');
    hPri   = local_h(mdl, 37, 'Prismatic Joint');
    hBody  = local_h(mdl, 31, 'Brick Solid');
    hForce = local_h(mdl, 62, 'External Force');
    hS2psQ = local_h(mdl,  2, 'Simulink-PS');           % q3 motion input (to delete)
    hS2psD = local_h(mdl, 39, 'Simulink-PS');           % d4 motion input (to delete)
    hS2psF = local_h(mdl, 66, 'Simulink-PS');           % force input (kept; used as template)
    hSinQ  = local_h(mdl, 34, 'Sin');                   % q3 motion sine (to delete)
    hSinD  = local_h(mdl, 38, 'Sin');                   % d4 motion sine (to delete)
    hPsTau3 = local_h(mdl, 26, 'PS-Simulink');          % revolute actuator torque -> tau_3
    hPsF4   = local_h(mdl, 41, 'PS-Simulink');          % prismatic actuator force -> f_4
    hPsQ3   = local_h(mdl, 49, 'PS-Simulink');          % q3
    hPsW3   = local_h(mdl, 50, 'PS-Simulink');          % q3dot
    hPsA3   = local_h(mdl, 59, 'PS-Simulink');          % q3ddot
    hPsD4   = local_h(mdl, 52, 'PS-Simulink');          % d4
    hPsV4   = local_h(mdl, 51, 'PS-Simulink');          % d4dot
    hPsA4   = local_h(mdl, 60, 'PS-Simulink');          % d4ddot
    twSID   = [43 44 45 46 47 48 58 61 67];
    twVar   = {'q_3','q_3_dot','tau_3','d_4','d_4_dot','f_4','q_3_ddot','d_4_ddot','outF'};
    hTW = zeros(size(twSID));
    for k = 1:numel(twSID)
        hTW(k) = local_h(mdl, twSID(k), 'ToWorkspace');
        v = get_param(hTW(k), 'VariableName');
        if ~strcmp(v, twVar{k})
            error('build:map', 'SID %d logs "%s", expected "%s". Source model has changed.', ...
                  twSID(k), v, twVar{k});
        end
    end
    % the force must act on the bail frame (prismatic follower)
    local_assert_connected(hForce, hPri);

    % ---- 3. Remove the motion-input path
    local_delete(hS2psQ); local_delete(hS2psD);
    local_delete(hSinQ);  local_delete(hSinD);

    % ---- 4. Joints: torque/force input, initial state from variables
    local_set_actuation(hRev);
    local_set_actuation(hPri);
    set_param(hRev, 'PositionTargetSpecify','on', 'PositionTargetPriority','High', ...
        'PositionTargetValue','q3_0_deg', 'PositionTargetValueUnits','deg', ...
        'VelocityTargetSpecify','on', 'VelocityTargetPriority','High', ...
        'VelocityTargetValue','q3d_0_degps', 'VelocityTargetValueUnits','deg/s');
    set_param(hPri, 'PositionTargetSpecify','on', 'PositionTargetPriority','High', ...
        'PositionTargetValue','d4_0', 'PositionTargetValueUnits','m', ...
        'VelocityTargetSpecify','on', 'VelocityTargetPriority','High', ...
        'VelocityTargetValue','d4d_0', 'VelocityTargetValueUnits','m/s');

    % ---- 5. Explicit SI units on every sensor converter
    set_param(hPsQ3,'Unit','rad');   set_param(hPsW3,'Unit','rad/s');  set_param(hPsA3,'Unit','rad/s^2');
    set_param(hPsTau3,'Unit','N*m');
    set_param(hPsD4,'Unit','m');     set_param(hPsV4,'Unit','m/s');    set_param(hPsA4,'Unit','m/s^2');
    set_param(hPsF4,'Unit','N');
    for k = 1:numel(hTW)
        set_param(hTW(k), 'SampleTime','Ts_log', 'SaveFormat','Timeseries', 'MaxDataPoints','inf');
    end

    % ---- 6. Controllers + plant Coulomb friction
    twTemplate  = getfullname(hTW(1));
    s2psTemplate = getfullname(hS2psF);
    x0 = 1600; y3 = 100; y4 = 700;    % layout origin (arranged at the end)

    local_joint_loop(mdl, 'q3', 'rad', x0, y3, hPsQ3, hPsW3, hRev, ...
        'Kp3','Ki3','Kd3','I3_0', '340', 'fc3 plant (Rasuli 2014 Tab II)', ...
        'tau3_cmd','tau3_coul','q3_ref', 'N*m', twTemplate, s2psTemplate);
    local_joint_loop(mdl, 'd4', 'm',   x0, y4, hPsD4, hPsV4, hPri, ...
        'Kp4','Ki4','Kd4','I4_0', '480', 'fc4 plant (Rasuli 2014 Tab II)', ...
        'f4_cmd','f4_coul','d4_ref', 'N', twTemplate, s2psTemplate);

    % ---- 7. Model workspace, solver, description
    hws = get_param(mdl, 'ModelWorkspace');
    hws.clear;
    set_param(mdl, 'Solver','ode15s', 'RelTol','1e-8', 'AbsTol','1e-9', ...
                   'MaxStep','1e-3', 'StopTime','T_stop');
    set_param(mdl, 'Description', sprintf(['MS02P08B torque-driven plant. Built %s by ' ...
        'build_M2_TorqueDriven_Validation.m from M2_LoadInjection.slx (MD5 %s). ' ...
        'Plant parameters: Rasuli 2014 Table II (Machine A). Do not edit by hand; rebuild.'], ...
        datestr(now,'yyyy-mm-dd HH:MM'), srcMD5));

    try
        Simulink.BlockDiagram.arrangeSystem(mdl);
    catch
        % layout only; not needed for simulation
    end

    save_system(mdl);
    close_system(mdl, 0);
    dstMD5 = local_md5(dstFile);

    % ---- 8. Build receipt
    rcpt = fullfile(root, 'M2', 'model', [dstName '_BUILD.txt']);
    fid = fopen(rcpt, 'w');
    fprintf(fid, 'Build receipt: %s\n', dstName);
    fprintf(fid, 'Date:          %s\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, 'MATLAB:        %s\n', version);
    fprintf(fid, 'Builder:       M2/scripts/build_M2_TorqueDriven_Validation.m\n');
    fprintf(fid, 'Source:        M2/model/%s.slx  MD5 %s\n', srcName, srcMD5);
    fprintf(fid, 'Output:        M2/model/%s.slx  MD5 %s\n', dstName, dstMD5);
    fprintf(fid, 'Changes:       joints -> torque/force input; motion inputs removed; PID per joint;\n');
    fprintf(fid, '               plant Coulomb 340 N m / 480 N (tanh, eps 1e-3); SI sensor units;\n');
    fprintf(fid, '               logging at Ts_log; model workspace cleared; ode15s 1e-8/1e-9, MaxStep 1e-3.\n');
    fclose(fid);

    fprintf('\nBuilt %s\n  MD5 %s\n  receipt %s\n', dstFile, dstMD5, rcpt);
    fprintf('Next: commit the builder, the model and the spec, then run\n');
    fprintf('      run_MS02P08B_simscape(''SmokeTest'', true)\n');
end

% =====================================================================================
function local_joint_loop(mdl, tag, unit, x, y, hPos, hVel, hJoint, ...
        Kp, Ki, Kd, I0, fcVal, fcName, cmdVar, coulVar, refVar, torqueUnit, twT, s2psT)
% PID position loop and plant Coulomb friction for one joint.
%   e  = ref - pos ;  ed = refdot - vel
%   cmd = Kp*e + Ki*int(e) + Kd*ed          -> logged as cmdVar (drive torque/force)
%   joint input = cmd - fc*tanh(vel/1e-3)   -> Coulomb logged as coulVar
    B = 'simulink/Sources/Sine Wave';
    A = [tag '_B']; Amp = [tag '_A'];
    hRef  = local_add(mdl, B, sprintf('%s_ref [%s]', tag, unit), [x y x+40 y+30], ...
        'Amplitude',Amp, 'Bias',A, 'Frequency','omega_traj', 'Phase','0', 'SampleTime','0');
    hRefD = local_add(mdl, B, sprintf('%s_ref_dot [%s per s]', tag, unit), [x y+80 x+40 y+110], ...
        'Amplitude',[Amp '*omega_traj'], 'Bias','0', 'Frequency','omega_traj', ...
        'Phase','pi/2', 'SampleTime','0');
    hE    = local_add(mdl, 'simulink/Math Operations/Sum', [tag ' error'], [x+120 y x+150 y+30], 'Inputs','+-');
    hED   = local_add(mdl, 'simulink/Math Operations/Sum', [tag ' rate error'], [x+120 y+80 x+150 y+110], 'Inputs','+-');
    hInt  = local_add(mdl, 'simulink/Continuous/Integrator', [tag ' error integral'], [x+200 y+40 x+230 y+70], ...
        'InitialCondition', I0);
    hKp   = local_add(mdl, 'simulink/Math Operations/Gain', [Kp ' gain'], [x+280 y x+330 y+30], 'Gain',Kp);
    hKi   = local_add(mdl, 'simulink/Math Operations/Gain', [Ki ' gain'], [x+280 y+40 x+330 y+70], 'Gain',Ki);
    hKd   = local_add(mdl, 'simulink/Math Operations/Gain', [Kd ' gain'], [x+280 y+80 x+330 y+110], 'Gain',Kd);
    hPID  = local_add(mdl, 'simulink/Math Operations/Sum', [tag ' drive command'], [x+380 y+20 x+410 y+90], 'Inputs','+++');
    hInvE = local_add(mdl, 'simulink/Math Operations/Gain', [tag ' inv eps_c (plant)'], [x+120 y+160 x+170 y+190], 'Gain','1000');
    hTanh = local_add(mdl, 'simulink/Math Operations/Trigonometric Function', [tag ' tanh'], [x+200 y+160 x+240 y+190], 'Operator','tanh');
    hFc   = local_add(mdl, 'simulink/Math Operations/Gain', fcName, [x+280 y+160 x+330 y+190], 'Gain',fcVal);
    hNet  = local_add(mdl, 'simulink/Math Operations/Sum', [tag ' joint input'], [x+460 y+40 x+490 y+70], 'Inputs','+-');
    hS2ps = local_add(mdl, s2psT, [tag ' drive S-PS'], [x+540 y+40 x+580 y+70]);
    set_param(hS2ps, 'Unit',torqueUnit, 'FilteringAndDerivatives','zero');
    hTwC  = local_add(mdl, twT, [cmdVar ' log'],  [x+460 y+120 x+520 y+150], 'VariableName',cmdVar,  'SampleTime','Ts_log', 'SaveFormat','Timeseries');
    hTwF  = local_add(mdl, twT, [coulVar ' log'], [x+380 y+200 x+440 y+230], 'VariableName',coulVar, 'SampleTime','Ts_log', 'SaveFormat','Timeseries');
    hTwR  = local_add(mdl, twT, [refVar ' log'],  [x+120 y-60 x+180 y-30],   'VariableName',refVar,  'SampleTime','Ts_log', 'SaveFormat','Timeseries');

    local_sig(mdl, hRef,1, hE,1);    local_sig(mdl, hPos,1, hE,2);
    local_sig(mdl, hRefD,1, hED,1);  local_sig(mdl, hVel,1, hED,2);
    local_sig(mdl, hE,1, hKp,1);     local_sig(mdl, hE,1, hInt,1);   local_sig(mdl, hInt,1, hKi,1);
    local_sig(mdl, hED,1, hKd,1);
    local_sig(mdl, hKp,1, hPID,1);   local_sig(mdl, hKi,1, hPID,2);  local_sig(mdl, hKd,1, hPID,3);
    local_sig(mdl, hVel,1, hInvE,1); local_sig(mdl, hInvE,1, hTanh,1); local_sig(mdl, hTanh,1, hFc,1);
    local_sig(mdl, hPID,1, hNet,1);  local_sig(mdl, hFc,1, hNet,2);
    local_sig(mdl, hNet,1, hS2ps,1);
    local_sig(mdl, hPID,1, hTwC,1);  local_sig(mdl, hFc,1, hTwF,1);  local_sig(mdl, hRef,1, hTwR,1);
    % physical: converter -> joint actuation port (LConn 2)
    a = get_param(hS2ps,'PortHandles'); b = get_param(hJoint,'PortHandles');
    add_line(mdl, a.RConn(1), b.LConn(2), 'autorouting','on');
end

function h = local_add(mdl, src, name, pos, varargin)
    h = add_block(src, [mdl '/' name], 'MakeNameUnique','on', 'Position',pos, varargin{:});
end

function local_sig(mdl, hs, ps, hd, pd)
    a = get_param(hs,'PortHandles'); b = get_param(hd,'PortHandles');
    add_line(mdl, a.Outport(ps), b.Inport(pd), 'autorouting','on');
end

function h = local_h(mdl, sid, expect)
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    if iscell(h) || isempty(h), error('build:sid', 'SID %d not found in %s.', sid, mdl); end
    ref = '';
    try, ref = get_param(h, 'ReferenceBlock'); catch, end
    if isempty(ref), ref = get_param(h, 'BlockType'); end
    ref = strrep(ref, newline, ' ');
    if ~contains(ref, expect)
        error('build:sid', 'SID %d is "%s", expected "%s". Source model has changed.', sid, ref, expect);
    end
end

function local_delete(h)
    lh = get_param(h, 'LineHandles');
    f = fieldnames(lh);
    for i = 1:numel(f)
        v = lh.(f{i}); v = v(v ~= -1);
        for k = 1:numel(v)
            try, delete_line(v(k)); catch, end
        end
    end
    delete_block(h);
end

function local_set_actuation(h)
% Motion input -> torque input. Try both orders in case a transient combination is rejected.
    try
        set_param(h, 'MotionActuationMode','ComputedMotion', 'TorqueActuationMode','InputTorque');
    catch
        set_param(h, 'TorqueActuationMode','InputTorque');
        set_param(h, 'MotionActuationMode','ComputedMotion');
    end
    if ~strcmp(get_param(h,'TorqueActuationMode'),'InputTorque') || ...
       ~strcmp(get_param(h,'MotionActuationMode'),'ComputedMotion')
        error('build:actuation', 'Could not switch %s to torque input.', getfullname(h));
    end
end

function local_assert_connected(hForce, hPri)
% The External Force frame port must share a connection with the prismatic follower frame.
    pf = get_param(hForce, 'PortConnectivity');
    ok = false;
    for k = 1:numel(pf)
        if any(ismember([pf(k).DstBlock(:)', pf(k).SrcBlock(:)'], hPri)), ok = true; end
    end
    if ~ok
        warning('build:force', ['Could not confirm that the External Force block is attached ' ...
            'to the prismatic follower (bail) frame. Check the model visually.']);
    end
end

function h = local_md5(f)
    fid = fopen(f, 'r'); b = fread(fid, inf, '*uint8'); fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'))', 1, []));
end
