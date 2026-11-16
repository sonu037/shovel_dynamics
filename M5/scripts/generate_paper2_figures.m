function generate_paper2_figures(save_png)
%GENERATE_PAPER2_FIGURES  Generates publication-quality figures for Milestone 5 (Paper 2).
%
%   Target Journal: IEEE Transactions on Mechatronics / IEEE Trans. Industrial Electronics
%   Produces 3 Figures in papers/paper2_electromechanical_drive/figures:
%     - fig1_foc_and_power_balance: FOC torque-speed envelope & 38 s cycle power balance.
%     - fig2_spong_observer_and_impact: Rank-1 Coriolis suppression & boulder impact step response.
%     - fig3_thermal_drift_detuning: Rotor thermal drift (tau_r, slip frequency, and torque error).
%
%   USAGE:
%     generate_paper2_figures();
%     generate_paper2_figures(true); % Export vector PDF and 300 DPI PNGs
%
%   See also RUN_MS05_VERIFICATION_SUITE, MOTOR_FOC_WK20, ELECTROMECHANICAL_GMO.

    if nargin < 1 || isempty(save_png), save_png = true; end

    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(repo_root, 'M1', 'scripts'));
    addpath(fullfile(repo_root, 'M5', 'scripts'));

    fig_dir = fullfile(repo_root, 'papers', 'paper2_electromechanical_drive', 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

    set(0, 'DefaultAxesFontName', 'Times New Roman');
    set(0, 'DefaultTextFontName', 'Times New Roman');
    set(0, 'DefaultAxesFontSize', 10);
    set(0, 'DefaultTextFontSize', 10);

    % Colors
    c_blue   = [0.00, 0.45, 0.74];
    c_orange = [0.85, 0.33, 0.10];
    c_green  = [0.47, 0.67, 0.19];
    c_purple = [0.49, 0.18, 0.56];

    fprintf('========================================================================================\n');
    fprintf(' GENERATING PUBLICATION FIGURES FOR PAPER 2 (MILESTONE 5)\n');
    fprintf(' Target Directory: %s\n', fig_dir);
    fprintf('========================================================================================\n\n');

    %% ------------------------------------------------------------------------
    %  FIGURE 1: FOC Torque-Speed Envelope & Full-Cycle Power Balance
    %  ------------------------------------------------------------------------
    fprintf('Generating Figure 1: FOC Envelope & 38 s Cycle Power Flow...\n');
    fig1 = figure('Visible', 'off', 'Color', 'w', 'Units', 'inches', 'Position', [1, 1, 7.2, 3.2]);

    % Subplot 1: Hoist Motor Torque-Speed Envelope
    subplot(1, 2, 1);
    w_vec = linspace(0, 105.0, 300); % [rad/s]
    w_base = 78.5; % [rad/s] (25.4 Hz synchronous base speed)
    tau_rated_tot = 15280.0; % [Nm] combined dual motor rated torque
    tau_peak_tot  = 18805.0; % [Nm] combined dual motor peak torque

    tau_rated_curve = zeros(size(w_vec));
    tau_peak_curve  = zeros(size(w_vec));
    for i = 1:numel(w_vec)
        wi = w_vec(i);
        if wi <= w_base
            tau_rated_curve(i) = tau_rated_tot;
            tau_peak_curve(i)  = tau_peak_tot;
        else
            tau_rated_curve(i) = tau_rated_tot * (w_base / wi);
            tau_peak_curve(i)  = tau_peak_tot * (w_base / wi);
        end
    end

    plot(w_vec, tau_peak_curve * 1e-3, 'LineWidth', 2.0, 'Color', c_orange); hold on;
    plot(w_vec, tau_rated_curve * 1e-3, 'LineWidth', 2.0, 'Color', c_blue);
    xline(w_base, 'k--', 'LineWidth', 1.2);
    text(w_base + 2, 18, 'Base Speed 25.4 Hz', 'FontSize', 8);
    grid on; box on;
    xlabel('Hoist Motor Shaft Speed \omega_m (rad/s)');
    ylabel('Shaft Torque \tau_m (kN\cdotm)');
    title('(a) Dual YJ56B1 Field-Weakening');
    legend('Peak Torque (1,500 kW)', 'Continuous Rated (1,200 kW)', 'Location', 'northeast');
    xlim([0, 105]); ylim([0, 22]);

    % Subplot 2: Full 38 s Cycle Power Flow (Motoring vs Regeneration)
    subplot(1, 2, 2);
    t_cyc = linspace(0, 38.0, 500);
    P_elec_sample = zeros(size(t_cyc));
    P_mech_sample = zeros(size(t_cyc));
    for i = 1:numel(t_cyc)
        ti = t_cyc(i);
        if ti <= 12.0
            P_mech_sample(i) = 600.0 * sin(pi * ti / 12.0);
            P_elec_sample(i) = P_mech_sample(i) / (0.93 * 0.94);
        elseif ti <= 20.0
            P_mech_sample(i) = 150.0;
            P_elec_sample(i) = P_mech_sample(i) / (0.93 * 0.94);
        elseif ti <= 25.0
            P_mech_sample(i) = 20.0;
            P_elec_sample(i) = P_mech_sample(i) / (0.93 * 0.94);
        else
            % Regenerative lowering
            tau_ret = (ti - 25.0) / 13.0;
            P_mech_sample(i) = -250.0 * sin(pi * tau_ret);
            P_elec_sample(i) = P_mech_sample(i) * (0.90 * 0.94);
        end
    end

    plot(t_cyc, P_elec_sample, 'LineWidth', 2.0, 'Color', c_blue); hold on;
    plot(t_cyc, P_mech_sample, '--', 'LineWidth', 1.8, 'Color', c_green);
    yline(0, 'k-');
    xline(12.0, 'k:', 'LineWidth', 1.0);
    xline(20.0, 'k:', 'LineWidth', 1.0);
    xline(25.0, 'k:', 'LineWidth', 1.0);
    text(3.5, 700, 'Dig', 'FontSize', 8);
    text(14.5, 700, 'Swing', 'FontSize', 8);
    text(21.0, 700, 'Dump', 'FontSize', 8);
    text(29.0, -250, 'Regen Lowering', 'FontSize', 8, 'Color', c_blue);
    grid on; box on;
    xlabel('Cycle Time t (s)');
    ylabel('Power (kW)');
    title('(b) 38 s Cycle Power & Regeneration');
    legend('DC Inverter Power P_{dc}', 'Joint Power P_{mech}', 'Location', 'northeast');
    xlim([0, 38]); ylim([-300, 800]);

    if save_png
        exportgraphics(fig1, fullfile(fig_dir, 'fig1_foc_and_power_balance.pdf'), 'ContentType', 'vector');
        exportgraphics(fig1, fullfile(fig_dir, 'fig1_foc_and_power_balance.png'), 'Resolution', 300);
        fprintf('  Saved: fig1_foc_and_power_balance.pdf / .png\n');
    end
    close(fig1);

    %% ------------------------------------------------------------------------
    %  FIGURE 2: Spong Observer, Rank-1 Coriolis & Boulder Impact
    %  ------------------------------------------------------------------------
    fprintf('Generating Figure 2: Spong Observer & Boulder Impact Response...\n');
    mat_m3 = fullfile(repo_root, 'M5', 'experiments', 'MS05P03_observer_results.mat');
    obs_data = load(mat_m3);

    fig2 = figure('Visible', 'off', 'Color', 'w', 'Units', 'inches', 'Position', [1, 1, 7.2, 3.2]);

    % Subplot 1: Rank-1 Coriolis Suppression on Unloaded Sweep
    subplot(1, 2, 1);
    t_obs = obs_data.res_comp.t;
    plot(t_obs, obs_data.res_naive.tau_L_hat(:, 1) * 1e-3, '--', 'LineWidth', 1.8, 'Color', c_orange); hold on;
    plot(t_obs, obs_data.res_comp.tau_L_hat(:, 1) * 1e-3, 'LineWidth', 2.0, 'Color', c_blue);
    grid on; box on;
    xlabel('Time t (s)');
    ylabel('False Disturbance Torque (kN\cdotm)');
    title('(a) Rank-1 Coriolis Cancellation');
    legend('Naive GMO (No Rank-1: 22.15 kN\cdotm)', 'Spong GMO with Rank-1 (< 0.06 kN\cdotm)', ...
           'Location', 'northwest');
    xlim([0, 15]); ylim([-2, 25]);

    % Subplot 2: Boulder Impact Step Response at t = 8.0 s
    subplot(1, 2, 2);
    plot(t_obs, obs_data.res_loaded.tau_L_true(:, 1) * 1e-3, 'k--', 'LineWidth', 1.8); hold on;
    plot(t_obs, obs_data.res_loaded.tau_L_hat(:, 1) * 1e-3, 'LineWidth', 2.0, 'Color', c_blue);
    xline(8.0, 'r--', 'LineWidth', 1.2);
    text(8.2, 3800, 'Boulder Step (+300 kN)', 'Color', 'r', 'FontSize', 8);
    grid on; box on;
    xlabel('Time t (s)');
    ylabel('Saddle Load (kN\cdotm)');
    title('(b) Innovation Gating (33.3 ms Response)');
    legend('True Load \tau_L', 'Estimated \tau_{L,hat}', 'Location', 'northwest');
    xlim([6, 12]); ylim([500, 4500]);

    if save_png
        exportgraphics(fig2, fullfile(fig_dir, 'fig2_spong_observer_and_impact.pdf'), 'ContentType', 'vector');
        exportgraphics(fig2, fullfile(fig_dir, 'fig2_spong_observer_and_impact.png'), 'Resolution', 300);
        fprintf('  Saved: fig2_spong_observer_and_impact.pdf / .png\n');
    end
    close(fig2);

    %% ------------------------------------------------------------------------
    %  FIGURE 3: Thermal Detuning & Parameter Drift (Delta T = 0 to +95 C)
    %  ------------------------------------------------------------------------
    fprintf('Generating Figure 3: Summer Thermal Detuning & VFD Slip Drift...\n');
    mat_m4 = fullfile(repo_root, 'M5', 'experiments', 'MS05P04_benchmark_summary.mat');
    m4_data = load(mat_m4);
    c4 = m4_data.report.case4;

    fig3 = figure('Visible', 'off', 'Color', 'w', 'Units', 'inches', 'Position', [1, 1, 7.2, 3.2]);

    % Subplot 1: Rotor Time Constant Decay
    subplot(1, 2, 1);
    plot(c4.T_sweep, c4.tau_r_sweep, 'o-', 'LineWidth', 2.0, 'Color', c_purple, 'MarkerSize', 5);
    grid on; box on;
    xlabel('Rotor Winding Temperature T (^\circC)');
    ylabel('Rotor Time Constant \tau_r (s)');
    title('(a) Rotor Time Constant Decay \tau_r(T)');
    ylim([0.80, 1.25]);

    % Subplot 2: VFD Slip Divergence & Torque Error
    subplot(1, 2, 2);
    yyaxis left;
    plot(c4.T_sweep, c4.slip_freq_sweep, 's-', 'LineWidth', 2.0, 'Color', c_blue, 'MarkerSize', 5);
    ylabel('VFD Slip Frequency f_{slip} (Hz)');
    ylim([1.3, 2.1]);

    yyaxis right;
    plot(c4.T_sweep, c4.detuned_tau_err, '^-', 'LineWidth', 2.0, 'Color', c_orange, 'MarkerSize', 5);
    ylabel('Torque Detuning Error (%)');
    ylim([0, 8]);

    grid on; box on;
    xlabel('Rotor Winding Temperature T (^\circC)');
    title('(b) Slip Divergence & Parameter Detuning');

    if save_png
        exportgraphics(fig3, fullfile(fig_dir, 'fig3_thermal_drift_detuning.pdf'), 'ContentType', 'vector');
        exportgraphics(fig3, fullfile(fig_dir, 'fig3_thermal_drift_detuning.png'), 'Resolution', 300);
        fprintf('  Saved: fig3_thermal_drift_detuning.pdf / .png\n');
    end
    close(fig3);

    fprintf('\nAll Paper 2 figures successfully generated and exported to:\n  %s\n\n', fig_dir);
end
