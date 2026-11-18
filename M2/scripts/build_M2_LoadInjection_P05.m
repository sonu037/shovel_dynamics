function build_M2_LoadInjection_P05(varargin)
%BUILD_M2_LOADINJECTION_P05  Make the MS02P05 copy of the verified M2 model.
%
%   Creates   M2/model/M2_LoadInjection_P05.slx
%   from      M2/model/M2_LoadInjection.slx   (verified motion-prescribed M2 model)
%   The source model file is never modified (MS02P05 amendment 1, 2026-11-18).
%
%   WHY: MS02P05 Part 2 moves the handle-angle trajectory to new centres.
%   In M2_LoadInjection.slx the handle-angle Sine Wave (SID 34) holds the
%   literals Amplitude = 15 and Bias = 45 [deg]. This builder makes a copy in
%   which they read the variables A_q3 and B_q3, exactly as the crowd Sine
%   Wave1 (SID 38) was parameterised with A_d4/B_d4 on 2026-09-07.
%
%   CHANGES relative to M2_LoadInjection.slx (nothing else):
%     1. Sine Wave (SID 34): Amplitude '15' -> 'A_q3', Bias '45' -> 'B_q3'.
%        Units stay degrees (the Simulink-PS Converter after it is in deg).
%     2. Model workspace: A_q3 = 15 and B_q3 = 45 added, so that the copy
%        reproduces the source exactly until a script changes them.
%     3. Model description records the provenance.
%
%   USAGE (MATLAB R2025b desktop, any current folder):
%     build_M2_LoadInjection_P05
%     build_M2_LoadInjection_P05('Overwrite', true)   % rebuild an existing copy
%
%   OUTPUT: the .slx above plus M2_LoadInjection_P05_BUILD.txt (build receipt).
%   NEXT:   commit both, then run_MS02P05_configuration.
%   The copy is proven equivalent by the MS02P05 read-back (acceptance (a)):
%   at A_q3 = 15, B_q3 = 45 it must reproduce M2_100kN_0p1111Hz_sp1p50 to every
%   printed digit, and a control at A_q3 = 10 must differ.
%
%   Project: IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189). Drafted 2026-11-18
%   (written without MATLAB; the first run must be checked by the researcher).

    p = inputParser;
    addParameter(p, 'Overwrite', false, @islogical);
    parse(p, varargin{:});

    root    = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    srcName = 'M2_LoadInjection';
    dstName = 'M2_LoadInjection_P05';
    srcFile = fullfile(root, 'M2', 'model', [srcName '.slx']);
    dstFile = fullfile(root, 'M2', 'model', [dstName '.slx']);

    if ~isfile(srcFile)
        error('buildP05:noSource', 'Source model not found: %s', srcFile);
    end
    if isfile(dstFile) && ~p.Results.Overwrite
        error('buildP05:exists', ['%s already exists. Pass ''Overwrite'', true ' ...
              'to rebuild it.'], dstFile);
    end

    srcMD5 = local_md5(srcFile);
    fprintf('Source: %s (MD5 %s)\n', srcFile, srcMD5);

    % Start from the file on disk, never from a model already open in memory.
    if bdIsLoaded(dstName), close_system(dstName, 0); end
    if bdIsLoaded(srcName), close_system(srcName, 0); end
    if isfile(dstFile), delete(dstFile); end
    load_system(srcFile);
    save_system(srcName, dstFile);          % the model in memory is now dstName
    mdl = dstName;

    % ---- 1. handle-angle Sine Wave: check it is the block we expect, then change it
    h = Simulink.ID.getHandle([mdl ':34']);
    if ~strcmp(get_param(h, 'BlockType'), 'Sin') || ~strcmp(get_param(h, 'Name'), 'Sine Wave')
        error('buildP05:sid34', 'SID 34 is not the handle-angle Sine Wave (found %s "%s").', ...
              get_param(h, 'BlockType'), get_param(h, 'Name'));
    end
    expect = {'Amplitude','15'; 'Bias','45'; 'Frequency','omega_traj'};
    for k = 1:size(expect, 1)
        v = get_param(h, expect{k,1});
        if ~strcmp(strtrim(v), expect{k,2})
            error('buildP05:sid34value', ['Sine Wave %s is "%s", expected "%s". The source ' ...
                  'model differs from the one this builder was written for.'], expect{k,1}, v, expect{k,2});
        end
    end
    % the crowd Sine Wave1 must already use A_d4/B_d4 (2026-09-07)
    h38 = Simulink.ID.getHandle([mdl ':38']);
    if ~strcmp(get_param(h38, 'Amplitude'), 'A_d4') || ~strcmp(get_param(h38, 'Bias'), 'B_d4')
        error('buildP05:sid38', 'Sine Wave1 (SID 38) is not parameterised with A_d4/B_d4.');
    end
    set_param(h, 'Amplitude', 'A_q3', 'Bias', 'B_q3');

    % ---- 2. model workspace defaults that reproduce the source
    mw = get_param(mdl, 'ModelWorkspace');
    if ~strcmp(mw.DataSource, 'Model File')
        error('buildP05:mwSource', 'Model workspace data source is "%s", expected "Model File".', mw.DataSource);
    end
    mw.assignin('A_q3', 15);
    mw.assignin('B_q3', 45);
    if mw.getVariable('A_q3') ~= 15 || mw.getVariable('B_q3') ~= 45
        error('buildP05:mwWrite', 'Model-workspace write of A_q3/B_q3 did not take.');
    end

    % ---- 3. description and save
    set_param(mdl, 'Description', sprintf(['MS02P05 copy built by build_M2_LoadInjection_P05.m ' ...
        'from M2_LoadInjection.slx (MD5 %s) on %s. Only change: handle-angle Sine Wave ' ...
        'Amplitude/Bias read A_q3/B_q3 [deg] (model-workspace defaults 15/45). ' ...
        'Do not edit by hand; rebuild with the builder.'], srcMD5, datestr(now, 'yyyy-mm-dd HH:MM')));
    save_system(mdl);
    close_system(mdl, 0);

    % ---- receipt
    dstMD5 = local_md5(dstFile);
    rcpt = fullfile(root, 'M2', 'model', [dstName '_BUILD.txt']);
    fid = fopen(rcpt, 'w');
    fprintf(fid, 'MS02P05 model build receipt\n');
    fprintf(fid, 'Built:         %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, 'MATLAB:        %s\n', version);
    fprintf(fid, 'Builder:       M2/scripts/build_M2_LoadInjection_P05.m\n');
    fprintf(fid, 'Source:        M2/model/%s.slx  MD5 %s\n', srcName, srcMD5);
    fprintf(fid, 'Output:        M2/model/%s.slx  MD5 %s\n', dstName, dstMD5);
    fprintf(fid, 'Change:        Sine Wave (SID 34) Amplitude 15 -> A_q3, Bias 45 -> B_q3 [deg];\n');
    fprintf(fid, '               model workspace A_q3 = 15, B_q3 = 45. Nothing else.\n');
    fclose(fid);

    fprintf('\nBuilt %s\n  MD5 %s\n  receipt %s\n', dstFile, dstMD5, rcpt);
    fprintf('Next: commit the builder, the copy, the receipt and the spec amendment, then run\n');
    fprintf('      run_MS02P05_configuration\n');
end

function h = local_md5(f)
    fid = fopen(f, 'r'); b = fread(fid, inf, '*uint8'); fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(b);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end
