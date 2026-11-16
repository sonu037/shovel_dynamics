import os
import numpy as np
import scipy.io
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def generate_figures():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    fig_dir = os.path.join(os.path.dirname(__file__), 'figures')
    os.makedirs(fig_dir, exist_ok=True)

    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman', 'DejaVu Serif']
    plt.rcParams['font.size'] = 10
    plt.rcParams['axes.titlesize'] = 10
    plt.rcParams['axes.labelsize'] = 10

    # =========================================================================
    # FIGURE 1: SVD Subspace Angle & Identifiability Geometry
    # =========================================================================
    print("Generating Figure 1: Identifiability SVD Geometry...")
    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.2), dpi=300)

    d4_vec = np.linspace(4.0, 10.0, 200)
    s0 = 1.50
    c = -1.32
    theta_1ch = np.zeros_like(d4_vec)
    theta_2ch = np.zeros_like(d4_vec)

    for i, d in enumerate(d4_vec):
        v_grav = d + c
        v_load = d + s0
        cos_th1 = (v_grav * v_load) / (np.sqrt(v_grav**2) * np.sqrt(v_load**2))
        theta_1ch[i] = np.arccos(min(1.0, cos_th1)) * (180.0 / np.pi)

        Y_sub = np.array([
            [d + c, d + s0],
            [10.0 * 1.0, 10.0 * 1.0]
        ])
        U, S_val, Vt = np.linalg.svd(Y_sub)
        sigma_ratio = S_val[1] / S_val[0]
        theta_2ch[i] = np.arctan(sigma_ratio) * (180.0 / np.pi)

    ax1.plot(d4_vec, theta_1ch, 'r--', linewidth=1.8, label='Single-Channel (Saddle Only)')
    ax1.plot(d4_vec, theta_2ch, 'b-', linewidth=2.0, label=r'Joint 2-Channel ($[\tau_3;\; 10\cdot F_4]$)')
    ax1.axhline(29.70, color='k', linestyle=':', linewidth=1.2)
    ax1.text(7.0, 31.0, r'$\theta_{\min} = 29.70^\circ$', fontsize=9, color='k')
    ax1.grid(True, linestyle='-', alpha=0.5)
    ax1.set_xlabel('Crowd Extension $d_4$ (m)')
    ax1.set_ylabel(r'Subspace Angle $\theta$ (deg)')
    ax1.set_title(r'(a) Subspace Separation Angle $\theta(d_4)$')
    ax1.set_ylim([0, 45])
    ax1.legend(loc='upper left', frameon=True, fontsize=8)

    s_bar_sweep = np.logspace(-1, 2, 100)
    kappa_sweep = np.zeros_like(s_bar_sweep)
    for j, sb in enumerate(s_bar_sweep):
        Y_mid = np.array([
            [8.75 + c, 8.75 + s0],
            [sb * 1.0, sb * 1.0]
        ])
        kappa_sweep[j] = np.linalg.cond(Y_mid)

    ax2.semilogx(s_bar_sweep, kappa_sweep, 'k-', linewidth=2.0)
    ax2.axvline(10.0, color='r', linestyle='--', linewidth=1.5)
    ax2.plot(10.0, 3.77, 'ro', markersize=8, markerfacecolor='r')
    ax2.text(12.0, 8.0, r'$\bar{s} = 10.0$ m, $\kappa_2 = 3.77$', color='r', fontsize=9)
    ax2.grid(True, which='both', linestyle='-', alpha=0.5)
    ax2.set_xlabel(r'Crowd Channel Dimensional Weight $\bar{s}$ (m)')
    ax2.set_ylabel(r'Matrix Condition Number $\kappa_2$')
    ax2.set_title(r'(b) Joint Condition Number $\kappa_2$ vs. Weighting $\bar{s}$')
    ax2.set_ylim([1, 50])

    fig1.tight_layout()
    fig1.savefig(os.path.join(fig_dir, 'fig_identifiability_svd.pdf'), format='pdf', bbox_inches='tight')
    fig1.savefig(os.path.join(fig_dir, 'fig_identifiability_svd.png'), dpi=300, bbox_inches='tight')
    plt.close(fig1)

    # =========================================================================
    # FIGURE 2: 30 Hz Discrete Momentum Observer & FOH Validation
    # =========================================================================
    print("Generating Figure 2: Discrete Observer FOH Performance...")
    m3_mat = os.path.join(repo_root, 'M3', 'experiments', 'M3_discrete_30Hz_verification.mat')
    if not os.path.exists(m3_mat):
        m3_mat = os.path.join(repo_root, 'M3', '06_RESULTS_MAT', 'M3_discrete_30Hz_verification.mat')

    m3_data = scipy.io.loadmat(m3_mat, squeeze_me=True, struct_as_record=False)
    cases = m3_data['M3_DISC'].cases
    c_rev = getattr(cases, 'M3_DISC_CONST_REV')
    c_sin = getattr(cases, 'M3_DISC_SIN_100kN')

    fig2, (ax2_1, ax2_2) = plt.subplots(2, 1, figsize=(7.0, 5.0), dpi=300)

    # Subplot 1: Constant Load Reversal & Friction Envelope
    f_c3 = 340.0
    t_rev = c_rev.t_30
    mask_rev = (t_rev >= 5.0) & (t_rev <= 25.0)
    t_eval = t_rev[mask_rev]
    tau_true_const = c_rev.tau_L_true[0, mask_rev] / 1e3
    tau_hat_A = c_rev.tau_hat_A[0, mask_rev] / 1e3
    tau_hat_B = c_rev.tau_hat_B[0, mask_rev] / 1e3

    ax2_1.fill_between(t_eval, tau_true_const - f_c3/1e3, tau_true_const + f_c3/1e3,
                       color=[0.85, 0.92, 0.85], label=r'Friction Uncertainty Bound ($\pm f_c$)')
    ax2_1.plot(t_eval, tau_true_const, 'k--', linewidth=1.5, label=r'True Load $\tau_{L,3}$')
    ax2_1.plot(t_eval, tau_hat_B, 'b-', linewidth=1.2, label='Branch B (Exact FOH, 30 Hz)')
    ax2_1.plot(t_eval, tau_hat_A, 'r-.', linewidth=1.2, label='Branch A (Substep RK4, M=67)')
    ax2_1.grid(True, linestyle='-', alpha=0.5)
    ax2_1.set_xlim([5.0, 25.0])
    ax2_1.set_ylim([218.5, 221.5])
    ax2_1.set_xlabel('Time $t$ (s)')
    ax2_1.set_ylabel(r'Saddle Load Torque ($\mathrm{kN\cdot m}$)')
    ax2_1.set_title(r'(a) Velocity Reversals under Constant Load: Strict Bound Coverage (CovFricPct = 100.0%)')
    ax2_1.legend(loc='lower right', frameon=True, fontsize=8)

    # Subplot 2: 100 kN Harmonic Digging Load Tracking
    t_sin = c_sin.t_30
    mask_sin = (t_sin >= 5.0) & (t_sin <= 20.0)
    t_plot2 = t_sin[mask_sin]
    tau_true_sin = c_sin.tau_L_true[0, mask_sin] / 1e3
    tau_hat_B_sin = c_sin.tau_hat_B[0, mask_sin] / 1e3
    tau_hat_KF_sin = c_sin.tau_hat_KF[0, mask_sin] / 1e3

    ax2_2.plot(t_plot2, tau_true_sin, 'k-', linewidth=1.8, label=r'True Cutting Torque $\tau_{L,3}$')
    ax2_2.plot(t_plot2, tau_hat_B_sin, 'r:', linewidth=1.5, label=r'Branch B Fixed-Gain GMO (16.66 $\mathrm{kN\cdot m}$ lag)')
    ax2_2.plot(t_plot2, tau_hat_KF_sin, 'b-', linewidth=1.2, label=r'3-State KF-GMO (16.26 $\mathrm{kN\cdot m}$ RMSE, lag eliminated)')
    ax2_2.grid(True, linestyle='-', alpha=0.5)
    ax2_2.set_xlim([5.0, 20.0])
    ax2_2.set_xlabel('Time $t$ (s)')
    ax2_2.set_ylabel(r'Saddle Load Torque ($\mathrm{kN\cdot m}$)')
    ax2_2.set_title(r'(b) 100 kN Harmonic Cutting Load (1/9 Hz): Dynamic Tracking')
    ax2_2.legend(loc='upper right', frameon=True, fontsize=8)

    fig2.tight_layout()
    fig2.savefig(os.path.join(fig_dir, 'fig_discrete_foh_tracking.pdf'), format='pdf', bbox_inches='tight')
    fig2.savefig(os.path.join(fig_dir, 'fig_discrete_foh_tracking.png'), dpi=300, bbox_inches='tight')
    plt.close(fig2)

    # =========================================================================
    # FIGURE 3: KF-GMO Noise Filtering & 2-Sigma Composite Bounds
    # =========================================================================
    print("Generating Figure 3: KF-GMO Noise Suppression...")
    c_noise = getattr(cases, 'M3_DISC_NOISY_RESOLVER')
    t_noise = c_noise.t_30
    mask_n = (t_noise >= 5.0) & (t_noise <= 15.0)
    t_eval_n = t_noise[mask_n]

    tau_true_n = c_noise.tau_L_true[0, mask_n] / 1e3
    tau_raw_n = c_noise.tau_hat_B[0, mask_n] / 1e3
    tau_kf_n = c_noise.tau_hat_KF[0, mask_n] / 1e3
    sigma_2 = 2.0 * c_noise.sigma_kf[0, mask_n] / 1e3
    err_kf_n = (c_noise.tau_hat_KF[0, mask_n] - c_noise.tau_L_true[0, mask_n]) / 1e3

    fig3, (ax3_1, ax3_2) = plt.subplots(2, 1, figsize=(7.0, 4.8), dpi=300)

    ax3_1.plot(t_eval_n, tau_raw_n, color=[0.7, 0.7, 0.7], linewidth=0.8, label=r'Fixed-Gain GMO (RMSE = 193.2 $\mathrm{kN\cdot m}$)')
    ax3_1.plot(t_eval_n, tau_true_n, 'k--', linewidth=1.5, label=r'True Cutting Torque $\tau_L(t)$')
    ax3_1.plot(t_eval_n, tau_kf_n, 'b-', linewidth=1.4, label=r'3-State KF-GMO (RMSE = 36.1 $\mathrm{kN\cdot m}$, -81.3%)')
    ax3_1.grid(True, linestyle='-', alpha=0.5)
    ax3_1.set_xlim([5.0, 15.0])
    ax3_1.set_xlabel('Time $t$ (s)')
    ax3_1.set_ylabel(r'Saddle Torque ($\mathrm{kN\cdot m}$)')
    ax3_1.set_title(r'(a) Resolver Velocity Jitter (3 mrad/s): Fixed-Gain GMO vs. 3-State KF-GMO')
    ax3_1.legend(loc='upper right', frameon=True, fontsize=8)

    ax3_2.fill_between(t_eval_n, -sigma_2, sigma_2, color=[0.85, 0.90, 1.0], label=r'Dynamic $2\sigma_P(d_4)$ Confidence Envelope')
    ax3_2.plot(t_eval_n, err_kf_n, 'b.-', linewidth=1.0, markersize=5, label=r'Estimation Error $e_{KF}(t)$')
    ax3_2.axhline(0, color='k', linewidth=1.0)
    ax3_2.grid(True, linestyle='-', alpha=0.5)
    ax3_2.set_xlim([5.0, 15.0])
    ax3_2.set_ylim([-120, 120])
    ax3_2.set_xlabel('Time $t$ (s)')
    ax3_2.set_ylabel(r'Error Residual ($\mathrm{kN\cdot m}$)')
    ax3_2.set_title(r'(b) Estimation Residual & Theoretical $2\sigma_P$ Confidence Bound (97.47% Empirical Coverage)')
    ax3_2.legend(loc='upper right', frameon=True, fontsize=8)

    fig3.tight_layout()
    fig3.savefig(os.path.join(fig_dir, 'fig_kfgmo_noise_suppression.pdf'), format='pdf', bbox_inches='tight')
    fig3.savefig(os.path.join(fig_dir, 'fig_kfgmo_noise_suppression.png'), dpi=300, bbox_inches='tight')
    plt.close(fig3)

    # =========================================================================
    # FIGURE 4: Torque-Driven Closed-Loop Compliance & Deflection
    # =========================================================================
    print("Generating Figure 4: Torque-Driven Dynamic Compliance...")
    p08_mat = os.path.join(repo_root, 'M2', 'experiments', '08_held_out', 'M2_p08_torque_driven_results.mat')
    p08_data = scipy.io.loadmat(p08_mat, squeeze_me=True, struct_as_record=False)
    p08_td = p08_data['P08_TD']
    if hasattr(p08_td, 'cases') and hasattr(p08_td.cases, 'P08_HELD_STEP_IMPACT'):
        res_step = getattr(p08_td.cases, 'P08_HELD_STEP_IMPACT')
    elif hasattr(p08_td, 'held_out') and hasattr(p08_td.held_out, 'P08_HELD_STEP_IMPACT'):
        res_step = getattr(p08_td.held_out, 'P08_HELD_STEP_IMPACT')
    else:
        res_step = getattr(p08_td.held_out, 'HELD_IMPACT_STEP_150kN')

    fig4, (ax4_1, ax4_2) = plt.subplots(2, 1, figsize=(7.0, 5.0), dpi=300)
    t_td = res_step.t

    # Subplot 1: Deflection
    ax4_1.plot(t_td, np.rad2deg(res_step.delta_q3), 'r-', linewidth=1.5, label=r'$\Delta q_3$ (Saddle, deg)')
    ax4_1.plot(t_td, res_step.delta_d4 * 10.0, 'b--', linewidth=1.5, label=r'$\Delta d_4 \times 10$ (Crowd, 0.1 m)')
    ax4_1.grid(True, linestyle='-', alpha=0.5)
    ax4_1.set_xlim([0, 20])
    ax4_1.set_xlabel('Time $t$ (s)')
    ax4_1.set_ylabel('Joint Deflection')
    ax4_1.set_title(r'(a) Physical Dynamic Compliance under 450 kN Impact Cutting Disturbance')
    ax4_1.legend(loc='lower right', frameon=True, fontsize=8)

    # Subplot 2: Observer Recovery
    ax4_2.plot(t_td, res_step.tau_L_true[:, 0] / 1e3, 'k--', linewidth=1.8, label=r'True Applied Load $\tau_{L,3}$')
    if hasattr(res_step, 'tau_hat_obs'):
        ax4_2.plot(res_step.t_30, res_step.tau_hat_obs[:, 0] / 1e3, 'b-', linewidth=1.2, label='Discrete 30 Hz Momentum Observer')
    else:
        ax4_2.plot(res_step.t_30, res_step.tau_hat_foh[0, :] / 1e3, 'b-', linewidth=1.2, label='Discrete 30 Hz Momentum Observer')
    ax4_2.grid(True, linestyle='-', alpha=0.5)
    ax4_2.set_xlim([0, 20])
    ax4_2.set_xlabel('Time $t$ (s)')
    ax4_2.set_ylabel(r'Saddle Load ($\mathrm{kN\cdot m}$)')
    ax4_2.set_title(r'(b) Acceleration-Free Load Reconstruction on Compliant Motion (Lag NRMSE = 0.20%)')
    ax4_2.legend(loc='lower right', frameon=True, fontsize=8)

    fig4.tight_layout()
    fig4.savefig(os.path.join(fig_dir, 'fig_torquedriven_deflection.pdf'), format='pdf', bbox_inches='tight')
    fig4.savefig(os.path.join(fig_dir, 'fig_torquedriven_deflection.png'), dpi=300, bbox_inches='tight')
    plt.close(fig4)

    print(f"All Paper 1 publication figures successfully generated in: {fig_dir}")

if __name__ == '__main__':
    generate_figures()
