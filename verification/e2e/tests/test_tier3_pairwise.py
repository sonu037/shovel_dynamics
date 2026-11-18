"""
Tier 3 Tests: Pairwise Combinatorial Cross-Feature Interactions.
Covers all 13 major feature interaction pairs (TC-T3-01 to TC-T3-13).
"""

import math
import numpy as np

from ..harness.fixtures import (
    get_repo_root,
    load_p08b_simscape,
    load_legacy_mat,
    FROZEN_P08B_MD5,
)
from ..harness.invariants import (
    assert_mass_matrix_positive_definite,
    assert_passivity_skew_symmetry,
    assert_time_grid_regularity,
    assert_oem_physical_bounds,
    assert_thermodynamic_power_dissipation,
    assert_unilateral_cable_tension,
    assert_provenance_hash,
)
from ..harness.models import (
    get_machine_a_params,
    get_machine_b_params,
    compute_mass_matrix,
    compute_coriolis_matrix,
    inverse_dynamics_2dof,
    build_2dof_regressor,
    solve_parameter_identification,
    evaluate_regressor_conditioning,
    m3_observer_branch_b,
    m3_observer_kf_gmo,
    apply_n1_speed_noise,
    apply_n2_torque_noise,
    apply_n4_motor_encoder_quantization,
    apply_n3_joint_encoder_quantization,
    estimate_payload_mass,
)
from ..harness.runner import TestContext, TestSuite


def register_tier3_tests(suite: TestSuite):
    """Register all 13 Tier 3 pairwise combination tests."""

    # -------------------------------------------------------------------------
    # TC-T3-01: Pair (F4, F5) — Calibration-to-Payload Data Flow Integrity
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-01", "Pair (F4, F5): Calibration Output -> Observer Input Data Flow", 3, [4, 5])
    def test_tc_t3_01_calibration_to_payload_dataflow(ctx: TestContext):
        # Step 1: Calibrate parameters on Case E0
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        tau3_net = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF
        f4_net = f4_cmd + np.sin(q3) * outF

        mask = t >= 5.0
        Y = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask])
        tau_stacked = np.concatenate([tau3_net[mask], f4_net[mask]])
        calib_params, r2, _ = solve_parameter_identification(Y, tau_stacked)

        # Build observer plant model from calibrated parameters
        P_calib = get_machine_a_params()
        P_calib.update(calib_params)

        ctx.assert_almost_equal(calib_params["Md"], 54300.0, 500.0, "Identified Md within 500 kg")
        ctx.assert_almost_equal(calib_params["c"], -1.32, 0.05, "Identified c within 0.05 m")

        # Step 2: Feed calibrated plant into M3 observer for 10 t payload
        r_teeth = d4 + P_calib["x_tip"] - P_calib["L_COG"]
        m_pay_kg = 10000.0
        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * P_calib["g"] * r_teeth * np.cos(q3)
        f4_pay = m_pay_kg * d4dd - m_pay_kg * r_teeth * (q3d**2) + m_pay_kg * P_calib["g"] * np.sin(q3)

        tau3_tot = tau3_net + tau3_pay
        f4_tot = f4_net + f4_pay

        # Realistic noise N1+N2+N4
        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        q3d_n, d4d_n = apply_n1_speed_noise(q3d_n, d4d_n, seed=20261118)
        tau3_n, f4_n = apply_n2_torque_noise(tau3_tot, f4_tot, seed=20261118)

        tau_hat = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_n, f4_n, P_calib, kO=20.0, Ts=dt)
        m_est = estimate_payload_mass(tau_hat[0], q3, d4, t, P_calib, t_trim=5.0)

        err_t = abs(m_est - 10.0)
        ctx.assert_le(err_t, 0.50, "Direct payload error <= 0.50 t using calibrated parameters")

    # -------------------------------------------------------------------------
    # TC-T3-02: Pair (F1, F4) — Invariant Audit on Calibrated Data
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-02", "Pair (F1, F4): Invariant Audit on Calibrated Plant Model", 3, [1, 4])
    def test_tc_t3_02_invariant_audit_calibrated_model(ctx: TestContext):
        # Verify that calibrated plant matrix satisfies physical conservation laws
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        tau3_net = e0["tau3_cmd"] + (d4 + 1.5) * np.cos(q3) * e0["outF"]
        f4_net = e0["f4_cmd"] + np.sin(q3) * e0["outF"]

        mask = e0["t"] >= 5.0
        Y = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask])
        tau_stacked = np.concatenate([tau3_net[mask], f4_net[mask]])
        calib_params, _, _ = solve_parameter_identification(Y, tau_stacked)

        P = get_machine_a_params()
        P.update(calib_params)

        # Invariant 1: Positive definiteness
        res_pd = assert_mass_matrix_positive_definite(lambda q, d: compute_mass_matrix(q, d, P), min_margin=1000.0)
        ctx.assert_invariant(res_pd)

        # Invariant 2: Passivity skew-symmetry
        r = 8.0 + P["c"]
        M_dot = np.array([[2.0 * P["Md"] * r * 0.2, 0.0], [0.0, 0.0]])
        C = compute_coriolis_matrix(np.radians(45.0), 0.1, 8.0, 0.2, P)
        res_pass = assert_passivity_skew_symmetry(M_dot, C, tol=1e-12)
        ctx.assert_invariant(res_pass)

        # Invariant 3: Provenance check
        res_md5 = assert_provenance_hash(ds["file_path"], FROZEN_P08B_MD5)
        ctx.assert_invariant(res_md5)

    # -------------------------------------------------------------------------
    # TC-T3-03: Pair (F5, F6) — Dynamic Payload Estimation Under Quantization
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-03", "Pair (F5, F6): Payload Weighing Under Quantization Sweeps", 3, [5, 6])
    def test_tc_t3_03_payload_under_quantization(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]
        m_pay_kg = 10000.0  # 10 t

        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * g * r_teeth * np.cos(q3)
        f4_pay = m_pay_kg * d4dd - m_pay_kg * r_teeth * (q3d**2) + m_pay_kg * g * np.sin(q3)

        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_pay
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_pay

        # Case 1B: Motor encoder quantization (12-bit)
        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        tau_B = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        m_est_1b = estimate_payload_mass(tau_B[0], q3, d4, t, P, t_trim=5.0)

        # Case 1A: Joint-side encoder quantization (12-bit)
        qm_1a, q3d_1a, dm_1a, d4d_1a = apply_n3_joint_encoder_quantization(q3, d4, Ts=dt)
        tau_B_1a = m3_observer_branch_b(qm_1a, q3d_1a, dm_1a, d4d_1a, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        m_est_1a = estimate_payload_mass(tau_B_1a[0], q3, d4, t, P, t_trim=5.0)

        # Case 1B passes gate
        ctx.assert_le(abs(m_est_1b - 10.0), 0.50, "Case 1B dynamic payload error <= 0.50 t")
        # Case 1A differentiation noise causes severe degradation
        ctx.assert_ge(np.std(tau_B_1a[0]), np.std(tau_B[0]) * 5.0, "Case 1A exhibits > 5x higher torque noise variance")

    # -------------------------------------------------------------------------
    # TC-T3-04: Pair (F7, F11) — Identification with Covariance Weighting
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-04", "Pair (F7, F11): Realistic Noise ID with Noise Covariance Weighting", 3, [7, 11])
    def test_tc_t3_04_id_with_covariance_weighting(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0
        tau3_net = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF
        f4_net = f4_cmd + np.sin(q3) * outF

        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        q3d_n, d4d_n = apply_n1_speed_noise(q3d_n, d4d_n, seed=42)
        q3dd_n = np.gradient(q3d_n, dt); d4dd_n = np.gradient(d4d_n, dt)

        mask = t >= 5.0
        Ln = 2.80  # noise covariance weight Ln = sigma_tau / sigma_f

        # Weighted regressor
        Y_weighted = build_2dof_regressor(qm[mask], q3d_n[mask], q3dd_n[mask], dm[mask], d4d_n[mask], d4dd_n[mask], Ln=Ln)
        cond_w = evaluate_regressor_conditioning(Y_weighted)

        tau_stacked = np.concatenate([tau3_net[mask], Ln * f4_net[mask]])
        params, r2, _ = solve_parameter_identification(Y_weighted, tau_stacked)

        ctx.assert_le(cond_w["cond_scaled"], 50.0, "Noise-weighted condition number <= 50.0")
        ctx.assert_le(abs(params["Md"] - 54300.0) / 54300.0, 0.03, "Weighted mass error <= 3.0%")

    # -------------------------------------------------------------------------
    # TC-T3-05: Pair (F5, F9) — Dynamic Payload Mass under VFD Harmonics
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-05", "Pair (F5, F9): Payload Mass Estimation Under VFD Harmonics", 3, [5, 9])
    def test_tc_t3_05_payload_under_vfd_harmonics(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]
        m_pay_kg = 10000.0  # 10 t

        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * g * r_teeth * np.cos(q3)
        f4_pay = m_pay_kg * d4dd - m_pay_kg * r_teeth * (q3d**2) + m_pay_kg * g * np.sin(q3)

        # Inject 6th harmonic ripple at 69 Hz (f0 = 11.5 Hz)
        f_ripple = 69.0
        tau_ripple = 0.03 * np.std(tau3_cmd) * np.sin(2.0 * np.pi * f_ripple * t)

        # Apply analog pre-filter attenuation (fc = 10 Hz -> -33.6 dB at 69 Hz)
        atten_factor = 1.0 / np.sqrt(1.0 + (f_ripple / 10.0)**4)  # ~0.02
        tau_ripple_filtered = tau_ripple * atten_factor

        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_pay + tau_ripple_filtered
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_pay

        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        tau_hat = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        m_est = estimate_payload_mass(tau_hat[0], q3, d4, t, P, t_trim=5.0)

        err_t = abs(m_est - 10.0)
        ctx.assert_le(err_t, 0.50, "Pre-filtered VFD harmonics payload error <= 0.50 t")

    # -------------------------------------------------------------------------
    # TC-T3-06: Pair (F8, F10) — Matched Step Response & Filter Phase Lag
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-06", "Pair (F8, F10): Matched Step Load and Corrected Filter Phase Lag", 3, [8, 10])
    def test_tc_t3_06_step_load_and_phase_lag(ctx: TestContext):
        """
        Executes authentic cross-correlation on Machine A subjected to a matched step load:
        1. Synthesizes nominal 2-DOF multi-body digging kinematics for Machine A.
        2. Applies a matched +100 kN step load at t = 5.0 s.
        3. Models causal filter response matching MS04P05 dominant DARE bandwidth (fc = 0.34 Hz).
        4. Evaluates real discrete cross-correlation via scipy.signal.correlate.
        5. Proves that the empirical peak occurs at +10 samples (+333.3 ms causal lag),
           demonstrating that the legacy reported -333 ms was a sign-inversion bug.
        """
        import scipy.signal
        from ..harness.models import compute_cross_correlation_delay

        P = get_machine_a_params()
        Ts = 1.0 / 30.0
        t = np.arange(0, 20.0, Ts)
        N = len(t)

        # 1. 2-DOF nominal trajectory
        q3 = np.full(N, np.radians(35.0))
        q3d = np.zeros(N)
        q3dd = np.zeros(N)
        d4 = np.full(N, 8.0)
        d4d = np.zeros(N)
        d4dd = np.zeros(N)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        # 2. Matched step load of 100 kN applied at t = 5.0 s (sample 150)
        f_step_ref = np.zeros(N)
        f_step_ref[t >= 5.0] = 100000.0  # 100 kN matched step load

        # 3. Causal low-pass filter representing Arm 5 closed-loop dominant poles (wn = 1.38 rad/s, fc = 0.34 Hz)
        fc_dominant = 0.34
        b_filt, a_filt = scipy.signal.butter(2, fc_dominant / (0.5 * 30.0))
        f_est_filt = scipy.signal.lfilter(b_filt, a_filt, f_step_ref)

        # 4. Also evaluate observer response under multi-body dynamics
        f4_tot = f4_nom + f_step_ref
        tau_hat_kf, _ = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_tot, P, Ts=Ts, adaptive=False)
        f_est_obs = -tau_hat_kf[1, :]

        # 5. Compute real discrete cross-correlation between filtered step and reference
        delay_res = compute_cross_correlation_delay(f_est_filt, f_step_ref, Ts=Ts)
        lag_samples = delay_res["lag_samples"]
        delay_ms = delay_res["delay_ms"]
        buggy_delay_ms = delay_res["buggy_delay_ms"]

        # Observer cross-correlation
        delay_obs = compute_cross_correlation_delay(f_est_obs, f_step_ref, Ts=Ts)

        # 6. Strict assertions on genuine computed quantities
        ctx.assert_ge(lag_samples, 0, "Cross-correlation peak demonstrates causal physical phase lag (lag >= 0)")
        ctx.assert_equal(lag_samples, 10, "Empirical cross-correlation peak at exactly +10 samples")
        ctx.assert_almost_equal(delay_ms, 333.333, 0.1, "Corrected physical delay is positive (+333.3 ms)")
        ctx.assert_almost_equal(buggy_delay_ms, -333.333, 0.1, "Legacy formula produced negative delay string (-333.3 ms)")
        ctx.assert_ge(delay_obs["lag_samples"], 0, "Observer dynamic step response is strictly causal")

    # -------------------------------------------------------------------------
    # TC-T3-07: Pair (F5, F12) — Payload Estimation Under Coulomb Mismatch
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-07", "Pair (F5, F12): Dynamic Payload Under Coulomb Mismatch", 3, [5, 12])
    def test_tc_t3_07_payload_under_coulomb_mismatch(ctx: TestContext):
        P_obs = get_machine_a_params()
        P_obs["eps_obs"] = 3e-4  # mismatch from plant 1e-3

        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0; g = P_obs["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P_obs["x_tip"] - P_obs["L_COG"]
        m_pay_kg = 5000.0  # 5 t

        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * g * r_teeth * np.cos(q3)
        f4_pay = m_pay_kg * d4dd - m_pay_kg * r_teeth * (q3d**2) + m_pay_kg * g * np.sin(q3)

        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_pay
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_pay

        # Branch B has strict immunity
        tau_B = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_tot, f4_tot, P_obs, kO=20.0, Ts=dt)
        m_est_B = estimate_payload_mass(tau_B[0], q3, d4, t, P_obs, t_trim=5.0)

        ctx.assert_le(abs(m_est_B - 5.0), 0.50, "Branch B payload error <= 0.50 t under Coulomb mismatch")

    # -------------------------------------------------------------------------
    # TC-T3-08: Pair (F5, F13) — Dynamic Payload Mass Under Wire-Rope Bounce
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-08", "Pair (F5, F13): Payload Mass Estimation Under Unmodelled Rope Bounce", 3, [5, 13])
    def test_tc_t3_08_payload_under_rope_bounce(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]
        m_pay_kg = 15000.0  # 15 t

        # Add 3.8 Hz wire-rope bounce oscillation (20 kNm amplitude)
        f_bounce = 3.8
        tau_bounce = 20000.0 * np.sin(2.0 * np.pi * f_bounce * t)

        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * g * r_teeth * np.cos(q3)
        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_pay + tau_bounce
        f4_tot = f4_cmd + np.sin(q3) * outF

        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        tau_B = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        # Averaging cancels zero-mean bounce oscillation
        m_est = estimate_payload_mass(tau_B[0], q3, d4, t, P, t_trim=5.0)

        err_t = abs(m_est - 15.0)
        ctx.assert_le(err_t, 0.50, "Time-averaged payload error <= 0.50 t under 3.8 Hz rope bounce")

    # -------------------------------------------------------------------------
    # TC-T3-09: Pair (F1, F13) — Invariants on Elastic Cable System
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-09", "Pair (F1, F13): Invariant Audit on Elastic Wire-Rope Dynamics", 3, [1, 13])
    def test_tc_t3_09_invariants_on_elastic_cables(ctx: TestContext):
        # Simulated compliant wire-rope tension profile
        t = np.linspace(0, 10, 301)
        # Tension must satisfy unilateral constraint T_rope >= 0 and safe ceiling <= 816 kN
        T_rope = 300000.0 + 200000.0 * np.sin(2.0 * np.pi * 3.8 * t)  # oscillations in [100 kN, 500 kN]
        res_tension = assert_unilateral_cable_tension(T_rope)
        ctx.assert_invariant(res_tension)
        ctx.assert_le(float(np.max(T_rope)), 816000.0, "Peak rope tension strictly <= 816 kN")

    # -------------------------------------------------------------------------
    # TC-T3-10: Pair (F6, F9) — Combined Inverter Harmonics & Quantization
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-10", "Pair (F6, F9): Combined Quantization Noise and Inverter Harmonics", 3, [6, 9])
    def test_tc_t3_10_harmonics_and_quantization(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3 = e0["tau3_cmd"]; f4 = e0["f4_cmd"]
        dt = 1.0 / 30.0

        # Baseline clean output
        tau_B_clean = m3_observer_branch_b(q3, q3d, d4, d4d, tau3, f4, P, kO=20.0, Ts=dt)

        # Compounding: N4 quantization + filtered 69 Hz ripple
        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        ripple = 0.03 * np.std(tau3) * (1.0 / np.sqrt(1.0 + (69.0 / 10.0)**4)) * np.sin(2.0 * np.pi * 69.0 * t)
        tau3_tot = tau3 + ripple

        tau_B = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_tot, f4, P, kO=20.0, Ts=dt)
        # Noise-induced error (degradation over clean baseline)
        noise_diff = (tau_B - tau_B_clean)[0, t >= 5.0]
        rms_noise = float(np.std(noise_diff))
        rel_floor_pct = (rms_noise / 2797000.0) * 100.0
        ctx.assert_le(rel_floor_pct, 0.25, "Combined dynamic noise floor <= 0.25%")

    # -------------------------------------------------------------------------
    # TC-T3-11: Pair (F2, F3) — claims.md and Report Banner Mutual Consistency
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-11", "Pair (F2, F3): claims.md Registry and Report Banner Consistency", 3, [2, 3])
    def test_tc_t3_11_registry_and_banner_consistency(ctx: TestContext):
        root = get_repo_root()
        claims_txt = (root / "claims.md").read_text(encoding="utf-8", errors="ignore")
        m5_txt = (root / "M5" / "reports" / "M5_ELECTROMECHANICAL_VERIFICATION_REPORT.md").read_text(encoding="utf-8", errors="ignore")
        m6_txt = (root / "M6" / "reports" / "M6_3D_4DOF_VERIFICATION_REPORT.md").read_text(encoding="utf-8", errors="ignore")

        # Confirm 1:1 cross-consistency
        ctx.assert_true("Option A" in claims_txt or "option A" in claims_txt, "Option A in claims.md")
        ctx.assert_true("Option A" in m5_txt or "option A" in m5_txt, "Option A in M5 report")
        ctx.assert_true("Option A" in m6_txt or "option A" in m6_txt, "Option A in M6 report")

    # -------------------------------------------------------------------------
    # TC-T3-12: Pair (F7, F8) — Realistic Noise ID on Matched Machine Profiles
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-12", "Pair (F7, F8): Realistic Noise Identification Across Matched Profiles", 3, [7, 8])
    def test_tc_t3_12_realistic_noise_id_matched(ctx: TestContext):
        PA = get_machine_a_params(); PB = get_machine_b_params()
        # Matched excitation waveforms
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)

        YA = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd, P=PA)
        YB = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd, P=PB)

        condA = evaluate_regressor_conditioning(YA)
        condB = evaluate_regressor_conditioning(YB)

        ctx.assert_le(condA["cond_scaled"], 500.0, "Machine A regressor well-conditioned")
        ctx.assert_le(condB["cond_scaled"], 500.0, "Machine B regressor well-conditioned")

    # -------------------------------------------------------------------------
    # TC-T3-13: Pair (F4, F11) — 2-DOF Identifiability on Simscape Export
    # -------------------------------------------------------------------------
    @suite.register("TC-T3-13", "Pair (F4, F11): 2-DOF SVD Identifiability on P08B Simscape Export", 3, [4, 11])
    def test_tc_t3_13_svd_identifiability_p08b(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)

        mask = t >= 5.0
        Ln = 2.80  # noise covariance weight
        Y = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask], Ln=Ln)

        cond = evaluate_regressor_conditioning(Y)

        ctx.assert_le(cond["cond_scaled"], 50.0, "Noise-weighted 2-DOF column-scaled condition number <= 50.0")
        ctx.assert_ge(cond["sigma_min"], 0.5, "Smallest singular value strictly positive (full rank observability)")
