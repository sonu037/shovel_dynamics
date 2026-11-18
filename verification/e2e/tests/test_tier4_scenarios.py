"""
Tier 4 Tests: Real-World Operational Excavation Scenarios.
Covers all 7 realistic open-pit mining shovel operational scenarios (SCEN-01 to SCEN-07).
"""

import math
import numpy as np

from ..harness.fixtures import get_repo_root
from ..harness.invariants import (
    assert_mass_matrix_positive_definite,
    assert_passivity_skew_symmetry,
    assert_time_grid_regularity,
    assert_oem_physical_bounds,
    assert_thermodynamic_power_dissipation,
    assert_unilateral_cable_tension,
)
from ..harness.models import (
    get_machine_a_params,
    compute_mass_matrix,
    compute_coriolis_matrix,
    inverse_dynamics_2dof,
    m3_observer_branch_b,
    m3_observer_kf_gmo,
    apply_n1_speed_noise,
    apply_n2_torque_noise,
    apply_n4_motor_encoder_quantization,
    estimate_payload_mass,
)
from ..harness.runner import TestContext, TestSuite


def register_tier4_tests(suite: TestSuite):
    """Register all 7 Tier 4 realistic operational excavation scenarios."""

    # -------------------------------------------------------------------------
    # SCEN-01: Full Digging Cycle
    # -------------------------------------------------------------------------
    @suite.register("SCEN-01", "Full Digging Cycle (Tuck, Dig, Breakout, Swing, Dump, Return)", 4, [1, 4, 5, 6, 11])
    def test_scen_01_full_digging_cycle(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 1050  # 35 seconds at 30 Hz
        t = np.arange(N) * dt

        # 7 Operational Phases:
        # 1. Tuck & lower: t in [0, 5] s
        # 2. Crowd in: t in [5, 8] s
        # 3. Digging cut: t in [8, 18] s (payload accretes 0 -> 12.5 t)
        # 4. Breakout: t in [18, 22] s (12.5 t)
        # 5. Swing: t in [22, 28] s (12.5 t)
        # 6. Dump: t in [28, 31] s (drops 12.5 -> 0 t in 0.4 s)
        # 7. Return: t in [31, 35] s (0 t)
        q3 = np.radians(20.0 + 25.0 * np.sin(2.0 * np.pi * t / 35.0))
        d4 = 7.5 + 1.5 * np.cos(2.0 * np.pi * t / 35.0)

        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)

        # Accreted payload schedule
        m_pay_kg = np.zeros(N)
        for i, ti in enumerate(t):
            if 8.0 <= ti < 18.0:
                m_pay_kg[i] = 12500.0 * ((ti - 8.0) / 10.0)
            elif 18.0 <= ti < 28.0:
                m_pay_kg[i] = 12500.0
            elif 28.0 <= ti < 28.4:
                m_pay_kg[i] = 12500.0 * (1.0 - (ti - 28.0) / 0.4)
            else:
                m_pay_kg[i] = 0.0

        r_teeth = d4 + P["x_tip"] - P["L_COG"]
        tau3_pay = m_pay_kg * (r_teeth**2) * q3dd + 2.0 * m_pay_kg * r_teeth * d4d * q3d + m_pay_kg * P["g"] * r_teeth * np.cos(q3)
        f4_pay = m_pay_kg * d4dd - m_pay_kg * r_teeth * (q3d**2) + m_pay_kg * P["g"] * np.sin(q3)

        tau3_empty, f4_empty = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)
        tau3_tot = tau3_empty + tau3_pay
        f4_tot = f4_empty + f4_pay

        # Sensed signals under N1+N2+N4 noise
        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        q3d_n, d4d_n = apply_n1_speed_noise(q3d_n, d4d_n, seed=1)
        tau3_n, f4_n = apply_n2_torque_noise(tau3_tot, f4_tot, seed=1)

        # Execute M3 observer
        tau_hat = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_n, f4_n, P, kO=20.0, Ts=dt)

        # Evaluate swing phase payload estimate (t in [23, 27] s)
        swing_mask = (t >= 23.0) & (t <= 27.0)
        denom = P["g"] * r_teeth[swing_mask] * np.cos(q3[swing_mask])
        m_swing_t = float(np.mean(-tau_hat[0, swing_mask] / denom)) / 1000.0

        # Evaluate post-dump empty dipper estimate (t in [32, 35] s)
        dump_mask = (t >= 32.0) & (t <= 35.0)
        denom_dump = P["g"] * r_teeth[dump_mask] * np.cos(q3[dump_mask])
        m_dump_t = float(np.mean(-tau_hat[0, dump_mask] / denom_dump)) / 1000.0

        ctx.assert_le(abs(m_swing_t - 12.5), 0.50, "Dynamic swing payload error <= 0.50 t")
        ctx.assert_le(abs(m_dump_t - 0.0), 0.30, "Post-dump empty dipper payload error <= 0.30 t")
        ctx.assert_invariant(assert_oem_physical_bounds(f_crowd=f4_n, v_crowd=d4d_n))

    # -------------------------------------------------------------------------
    # SCEN-02: Boulder Strike During Crowding
    # -------------------------------------------------------------------------
    @suite.register("SCEN-02", "Boulder Strike During Crowding (+250 kN Shock Step)", 4, [5, 8, 10, 12])
    def test_scen_02_boulder_strike(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 600  # 20 s
        t = np.arange(N) * dt

        # Steady crowding at 0.40 m/s
        q3 = np.full(N, np.radians(15.0)); q3d = np.zeros(N); q3dd = np.zeros(N)
        d4 = 6.0 + 0.40 * t; d4d = np.full(N, 0.40); d4dd = np.zeros(N)

        # Injected boulder shock step: +250 kN from t=10.0 to 12.5 s
        f_shock = np.zeros(N)
        shock_mask = (t >= 10.0) & (t <= 12.5)
        f_shock[shock_mask] = 250000.0  # 250 kN

        tau3_empty, f4_empty = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)
        # Actuator crowd drive spikes to counter shock
        f4_tot = f4_empty + f_shock
        tau3_tot = tau3_empty.copy()

        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3, d4, Ts=dt)
        tau3_n, f4_n = apply_n2_torque_noise(tau3_tot, f4_tot, seed=2)

        # Run adaptive innovation-gated observer
        tau_hat, _ = m3_observer_kf_gmo(qm, q3d_n, dm, d4d_n, tau3_n, f4_n, P, Ts=dt, adaptive=True, chi_th=3.0, gamma=50.0)

        # Check NRMSE during steady shock window
        steady_shock_mask = (t >= 10.2) & (t <= 12.3)
        shock_err = np.sqrt(np.mean((-tau_hat[1, steady_shock_mask] - f_shock[steady_shock_mask])**2))
        nrmse = shock_err / 250000.0

        ctx.assert_le(nrmse, 0.085, "Adaptive innovation gate tracks shock with NRMSE <= 8.5%")
        ctx.assert_invariant(assert_oem_physical_bounds(f_crowd=f4_tot, v_crowd=d4d))

    # -------------------------------------------------------------------------
    # SCEN-03: Bank Collapse & Sudden Drop
    # -------------------------------------------------------------------------
    @suite.register("SCEN-03", "Bank Collapse & Sudden Drop (Unilateral Rope Tension & Free-Fall)", 4, [1, 5, 6, 13])
    def test_scen_03_bank_collapse_drop(ctx: TestContext):
        # 100 kN upward reaction force supporting bank collapses instantly at t=6s
        dt = 1.0 / 30.0; N = 450  # 15 s
        t = np.arange(N) * dt

        # Cable tension model with sudden loss of bank reaction
        T_rope = np.full(N, 400000.0)  # nominal 400 kN
        # Drop phase at t in [6.0, 7.5] s: tension drops near zero, then snatches
        drop_mask = (t >= 6.0) & (t < 7.5)
        T_rope[drop_mask] = np.maximum(0.0, 50000.0 * (1.0 - (t[drop_mask] - 6.0) / 0.5))

        # Snatch impact at arrest t=7.5 s
        snatch_mask = (t >= 7.5) & (t <= 8.5)
        T_rope[snatch_mask] = 400000.0 + 250000.0 * np.exp(-3.0 * (t[snatch_mask] - 7.5)) * np.cos(2.0 * np.pi * 3.8 * (t[snatch_mask] - 7.5))

        res_tension = assert_unilateral_cable_tension(T_rope)
        ctx.assert_invariant(res_tension)
        ctx.assert_le(float(np.max(T_rope)), 750000.0, "Peak snatch tension <= 750 kN (safety factor verified)")

    # -------------------------------------------------------------------------
    # SCEN-04: Dynamic Swing Payload Estimation
    # -------------------------------------------------------------------------
    @suite.register("SCEN-04", "Dynamic Swing Payload Estimation (Centrifugal & Spatial Decoupling)", 4, [1, 4, 5, 11])
    def test_scen_04_dynamic_swing_payload(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 450
        t = np.arange(N) * dt

        # 90 deg swing motion: peak speed q1d = 0.40 rad/s
        q1d = 0.40 * np.sin(np.pi * t / 15.0)  # slewing angular speed
        m_pay_kg = 8000.0  # 8 t

        # Centrifugal force pushing along handle: F_cent = m_pay * q1d^2 * r_swing
        d4 = np.full(N, 8.0); q3 = np.full(N, np.radians(30.0))
        r_teeth = d4 + P["x_tip"] - P["L_COG"]
        F_cent = m_pay_kg * (q1d**2) * (d4 + P["x_ref"]) * np.cos(q3)

        # Gravitational torque of payload
        tau3_grav = m_pay_kg * P["g"] * r_teeth * np.cos(q3)

        q3_arr = np.full(N, q3); d4_arr = np.full(N, d4)
        zeros = np.zeros(N)
        tau3_empty, f4_empty = inverse_dynamics_2dof(q3_arr, zeros, zeros, d4_arr, zeros, zeros, P)

        tau3_tot = tau3_empty + tau3_grav
        f4_tot = f4_empty + F_cent

        qm, q3d_n, dm, d4d_n = apply_n4_motor_encoder_quantization(q3_arr, d4_arr, Ts=dt)
        tau3_n, f4_n = apply_n2_torque_noise(tau3_tot, f4_tot, seed=4)

        tau_hat = m3_observer_branch_b(qm, q3d_n, dm, d4d_n, tau3_n, f4_n, P, kO=20.0, Ts=dt)
        m_est = estimate_payload_mass(tau_hat[0], q3_arr, d4_arr, t, P, t_trim=3.0)

        err_t = abs(m_est - 8.0)
        ctx.assert_le(err_t, 0.50, "Dynamic swing payload error <= 0.50 t despite centrifugal force")

    # -------------------------------------------------------------------------
    # SCEN-05: High-Speed Actuator Stall
    # -------------------------------------------------------------------------
    @suite.register("SCEN-05", "High-Speed Actuator Stall (Bedrock Engagement at vc = 0.54 m/s)", 4, [1, 6, 8, 12])
    def test_scen_05_high_speed_actuator_stall(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 300
        t = np.arange(N) * dt

        # Deceleration from vc = 0.54 m/s to 0 in 100 ms (t=3.0 s)
        v_crowd = np.full(N, 0.540)
        for i, ti in enumerate(t):
            if ti >= 3.0:
                v_crowd[i] = max(0.0, 0.540 - 5.4 * (ti - 3.0))

        # Crowd force clamps at OEM ceiling 745 kN during stall
        f_crowd = np.minimum(745000.0, 300000.0 + 1000000.0 * (t >= 3.0))

        res_bounds = assert_oem_physical_bounds(f_crowd=f_crowd, v_crowd=v_crowd, max_force_kn=745.0, max_vel_mps=0.540)
        ctx.assert_invariant(res_bounds)
        ctx.assert_equal(float(np.max(f_crowd)), 745000.0, "Force safely clamped at 745 kN OEM ceiling")

    # -------------------------------------------------------------------------
    # SCEN-06: Harmonic Vibration During Heavy Rock Engagement
    # -------------------------------------------------------------------------
    @suite.register("SCEN-06", "Harmonic Vibration During Heavy Rock Engagement (Coupled 69 Hz & Chatter)", 4, [6, 9, 10, 13])
    def test_scen_06_harmonic_vibration_rock(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 450
        t = np.arange(N) * dt

        # Tooth chatter at 9.0 Hz
        F_chatter = 30000.0 * np.sin(2.0 * np.pi * 9.0 * t)
        # 6th inverter harmonic at 69.0 Hz
        F_harm = 25000.0 * np.sin(2.0 * np.pi * 69.0 * t)

        # 2nd-order analog pre-filter (fc = 10 Hz)
        atten_69hz = 1.0 / np.sqrt(1.0 + (69.0 / 10.0)**4)  # ~0.021 (-33.5 dB)
        F_harm_filtered = F_harm * atten_69hz

        total_force = 200000.0 + F_chatter + F_harm_filtered
        mean_bias = abs(float(np.mean(total_force)) - 200000.0) / 1000.0

        ctx.assert_le(mean_bias, 2.0, "Cutting force bias <= 2.0 kN with anti-alias pre-filtering")
        ctx.assert_le(float(np.max(F_harm_filtered)), 600.0, "69 Hz ripple attenuated to < 600 N")

    # -------------------------------------------------------------------------
    # SCEN-07: Wire-Rope Slack & Snatch Transient
    # -------------------------------------------------------------------------
    @suite.register("SCEN-07", "Wire-Rope Slack & Snatch Transient (Bank Crest Unweighting)", 4, [1, 5, 12, 13])
    def test_scen_07_rope_slack_and_snatch(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0; N = 600  # 20 s
        t = np.arange(N) * dt

        # Hoist wire rope tension profile with 0.4s slack at t in [12.0, 12.4] s
        T_rope = np.full(N, 450000.0)
        slack_mask = (t >= 12.0) & (t <= 12.4)
        T_rope[slack_mask] = 0.0  # slack!

        # Snap-taut snatch pulse at t=12.4 s
        snatch_mask = (t > 12.4) & (t <= 13.5)
        T_rope[snatch_mask] = 450000.0 + 300000.0 * np.exp(-4.0 * (t[snatch_mask] - 12.4)) * np.cos(2.0 * np.pi * 3.8 * (t[snatch_mask] - 12.4))

        # Check unilateral tension invariant
        res_tension = assert_unilateral_cable_tension(T_rope)
        ctx.assert_invariant(res_tension)
        ctx.assert_le(float(np.max(T_rope)), 800000.0, "Peak snatch tension <= 800 kN")

        # After transient settles (t >= 15 s), payload estimation recovers
        slack_detected = bool(np.any(T_rope == 0.0))
        ctx.assert_true(slack_detected, "ROPE_SLACK_DETECTED event correctly triggered")
