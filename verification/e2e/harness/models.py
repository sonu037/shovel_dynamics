"""
Planar multi-body dynamics, discrete observers, parameter regressors, and noise models.
Pure-Python analytical implementations corresponding to Machine A and Machine B.
"""

from typing import Any, Dict, Optional, Tuple
import numpy as np


def get_machine_a_params() -> Dict[str, Any]:
    """
    Machine A (P&H 2100BL class) physical ground truth parameters.
    Traceable to Rasuli (IEEE 2014) Table II and shovel_params.m.
    """
    return {
        "Md": 54300.0,  # dipper+handle mass (kg)
        "Izz": 287900.0,  # inertia about COM (kg*m^2)
        "L_COG": 2.82,  # COM offset along handle (m)
        "x_ref": 1.50,  # bail reference station along slide axis (m)
        "c": -1.32,  # net in-plane lever offset: x_ref - L_COG = 1.5 - 2.82 = -1.32 m
        "x_tip": 2.00,  # tooth-tip station offset along slide axis (m)
        "fv3": 3800.0,  # revolute viscous friction (Nm*s/rad)
        "fc3": 340.0,  # revolute Coulomb friction (Nm)
        "fv4": 1473.0,  # prismatic crowd viscous friction (N*s/m)
        "fc4": 480.0,  # prismatic crowd Coulomb friction (N)
        "g": 9.80665,  # gravitational acceleration (m/s^2)
        "eps_c": 1e-3,  # plant regularized Coulomb velocity threshold
        "eps_obs": 3e-4,  # observer regularized Coulomb velocity threshold
    }


def get_machine_b_params() -> Dict[str, Any]:
    """
    Machine B (TZ WK-20 class) physical ground truth parameters.
    Traceable to shovel_params_wk20.m.
    """
    return {
        "Md": 56500.0,  # kg
        "Izz": 310000.0,  # kg*m^2
        "L_COG": 2.95,  # m
        "x_ref": 1.50,  # m
        "c": -1.45,  # m
        "x_tip": 2.10,  # m
        "fv3": 4200.0,  # Nm*s/rad
        "fc3": 380.0,  # Nm
        "fv4": 1600.0,  # N*s/m
        "fc4": 520.0,  # N
        "g": 9.80665,
        "eps_c": 1e-3,
        "eps_obs": 3e-4,
    }


def compute_mass_matrix(q3: float, d4: float, P: Dict[str, Any]) -> np.ndarray:
    """Compute 2-DOF mass/inertia matrix M(q)."""
    r = d4 + P["c"]
    D11 = P["Izz"] + P["Md"] * (r**2)
    D22 = P["Md"]
    return np.array([[D11, 0.0], [0.0, D22]], dtype=np.float64)


def compute_coriolis_matrix(
    q3: float, q3d: float, d4: float, d4d: float, P: Dict[str, Any]
) -> np.ndarray:
    """Compute 2-DOF Coriolis/centrifugal matrix C(q, qd) in skew-symmetric passivity form."""
    r = d4 + P["c"]
    # Passivity parameterization:
    # C11 = Md*r*d4d, C12 = Md*r*q3d, C21 = -Md*r*q3d, C22 = 0
    C11 = P["Md"] * r * d4d
    C12 = P["Md"] * r * q3d
    C21 = -P["Md"] * r * q3d
    C22 = 0.0
    return np.array([[C11, C12], [C21, C22]], dtype=np.float64)


def compute_gravity_vector(q3: float, d4: float, P: Dict[str, Any]) -> np.ndarray:
    """Compute 2-DOF gravitational torque/force vector G(q)."""
    r = d4 + P["c"]
    G1 = P["Md"] * P["g"] * r * np.cos(q3)
    G2 = P["Md"] * P["g"] * np.sin(q3)
    return np.array([G1, G2], dtype=np.float64)


def compute_friction_forces(
    q3d: float, d4d: float, P: Dict[str, Any], eps: Optional[float] = None
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute viscous (Fv) and Coulomb (Fc) friction vectors."""
    eps3 = eps if eps is not None else P.get("eps_c", 1e-3)
    eps4 = eps if eps is not None else P.get("eps_c", 1e-3)

    Fv = np.array([P["fv3"] * q3d, P["fv4"] * d4d], dtype=np.float64)
    Fc = np.array(
        [
            P["fc3"] * np.tanh(q3d / eps3),
            P["fc4"] * np.tanh(d4d / eps4),
        ],
        dtype=np.float64,
    )
    return Fv, Fc


def inverse_dynamics_2dof(
    q3: np.ndarray,
    q3d: np.ndarray,
    q3dd: np.ndarray,
    d4: np.ndarray,
    d4d: np.ndarray,
    d4dd: np.ndarray,
    P: Dict[str, Any],
    eps: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute inverse dynamics generalized efforts: tau3 and f4."""
    r = d4 + P["c"]
    eps_val = eps if eps is not None else P.get("eps_c", 1e-3)

    # Saddle (revolute pitch)
    tau3 = (
        (P["Izz"] + P["Md"] * (r**2)) * q3dd
        + 2.0 * P["Md"] * r * d4d * q3d
        + P["Md"] * P["g"] * r * np.cos(q3)
        + P["fv3"] * q3d
        + P["fc3"] * np.tanh(q3d / eps_val)
    )

    # Crowd (prismatic extension)
    f4 = (
        P["Md"] * d4dd
        - P["Md"] * r * (q3d**2)
        + P["Md"] * P["g"] * np.sin(q3)
        + P["fv4"] * d4d
        + P["fc4"] * np.tanh(d4d / eps_val)
    )

    return tau3, f4


def build_2dof_regressor(
    q3: np.ndarray,
    q3d: np.ndarray,
    q3dd: np.ndarray,
    d4: np.ndarray,
    d4d: np.ndarray,
    d4dd: np.ndarray,
    P: Optional[Dict[str, Any]] = None,
    Ln: Optional[float] = None,
) -> np.ndarray:
    """
    Construct stacked 2-DOF linear regressor matrix Y:
    [Y3; Ln * Y4] identifying 7 base parameters:
    theta = [Md, Md*c, Izz + Md*c^2, fv3, fc3, fv4, fc4]^T.
    """
    if P is None:
        P = get_machine_a_params()
    g = P["g"]
    eps_c = P.get("eps_c", 1e-3)
    N = len(q3)

    Y3 = np.zeros((N, 7), dtype=np.float64)
    Y4 = np.zeros((N, 7), dtype=np.float64)

    # Channel 3 (revolute pitch)
    Y3[:, 0] = (d4**2) * q3dd + 2.0 * d4 * d4d * q3d + g * d4 * np.cos(q3)
    Y3[:, 1] = 2.0 * d4 * q3dd + 2.0 * d4d * q3d + g * np.cos(q3)
    Y3[:, 2] = q3dd
    Y3[:, 3] = q3d
    Y3[:, 4] = np.tanh(q3d / eps_c)

    # Channel 4 (prismatic crowd)
    Y4[:, 0] = d4dd - d4 * (q3d**2) + g * np.sin(q3)
    Y4[:, 1] = -(q3d**2)
    Y4[:, 5] = d4d
    Y4[:, 6] = np.tanh(d4d / eps_c)

    weight = 1.0 if Ln is None else float(Ln)
    return np.vstack([Y3, weight * Y4])


def solve_parameter_identification(
    Y: np.ndarray, tau_net: np.ndarray
) -> Tuple[Dict[str, float], float, np.ndarray]:
    """
    Solve linear least-squares parameter identification:
    theta = Y \\ tau_net, un-group physical parameters, and evaluate R^2.
    """
    theta, residuals, rank, s = np.linalg.lstsq(Y, tau_net, rcond=None)

    Md = float(theta[0])
    c = float(theta[1] / Md) if abs(Md) > 1e-6 else 0.0
    Izz = float(theta[2] - Md * (c**2))
    fv3 = float(theta[3])
    fc3 = float(theta[4])
    fv4 = float(theta[5])
    fc4 = float(theta[6])

    tau_pred = Y @ theta
    ss_tot = np.sum((tau_net - np.mean(tau_net)) ** 2)
    ss_res = np.sum((tau_net - tau_pred) ** 2)
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 1e-12 else 0.0

    params = {
        "Md": Md,
        "c": c,
        "Izz": Izz,
        "fv3": fv3,
        "fc3": fc3,
        "fv4": fv4,
        "fc4": fc4,
    }
    return params, float(r2), theta


def evaluate_regressor_conditioning(Y: np.ndarray) -> Dict[str, float]:
    """Compute raw and column-scaled condition numbers and singular values of regressor Y."""
    s = np.linalg.svd(Y, compute_uv=False)
    cond_raw = float(s[0] / s[-1]) if s[-1] > 0 else float("inf")

    col_norms = np.linalg.norm(Y, axis=0)
    col_norms[col_norms == 0.0] = 1.0
    Y_scaled = Y / col_norms
    s_scaled = np.linalg.svd(Y_scaled, compute_uv=False)
    cond_scaled = float(s_scaled[0] / s_scaled[-1]) if s_scaled[-1] > 0 else float("inf")

    return {
        "cond_raw": cond_raw,
        "cond_scaled": cond_scaled,
        "sigma_max": float(s[0]),
        "sigma_min": float(s[-1]),
    }


def m3_observer_branch_b(
    q3: np.ndarray,
    q3d: np.ndarray,
    d4: np.ndarray,
    d4d: np.ndarray,
    tau3: np.ndarray,
    f4: np.ndarray,
    P: Dict[str, Any],
    kO: float = 20.0,
    Ts: float = 1.0 / 30.0,
) -> np.ndarray:
    """
    Branch B exact discrete First-Order Hold (FOH) momentum observer.
    Line-for-line equivalent to M3/scripts/m3_observer_30Hz.m.
    Friction-uncertain: does not include Coulomb friction in state equations.
    """
    N = len(q3)
    phi_d = np.exp(-kO * Ts)
    beta1 = (1.0 - (1.0 - phi_d) / (kO * Ts)) / kO
    beta0 = (1.0 - phi_d) / kO - beta1

    z_B = np.zeros((2, N), dtype=np.float64)
    tau_hat_B = np.zeros((2, N), dtype=np.float64)
    p_prev = np.zeros(2, dtype=np.float64)
    h_B_prev = np.zeros(2, dtype=np.float64)

    for k in range(N):
        r_k = d4[k] + P["c"]
        D11 = P["Izz"] + P["Md"] * (r_k**2)
        D22 = P["Md"]
        p_k = np.array([D11 * q3d[k], D22 * d4d[k]], dtype=np.float64)
        G_k = np.array(
            [P["Md"] * P["g"] * r_k * np.cos(q3[k]), P["Md"] * P["g"] * np.sin(q3[k])],
            dtype=np.float64,
        )
        CT_k = np.array([0.0, P["Md"] * r_k * (q3d[k] ** 2)], dtype=np.float64)
        Fv_k = np.array([P["fv3"] * q3d[k], P["fv4"] * d4d[k]], dtype=np.float64)

        h_B_k = np.array([tau3[k], f4[k]], dtype=np.float64) - G_k - Fv_k + CT_k

        if k == 0:
            z_B[:, k] = p_k
        else:
            uB_prev = h_B_prev + kO * p_prev
            uB_k = h_B_k + kO * p_k
            z_B[:, k] = phi_d * z_B[:, k - 1] + beta0 * uB_prev + beta1 * uB_k

        tau_hat_B[:, k] = kO * (p_k - z_B[:, k])
        p_prev = p_k
        h_B_prev = h_B_k

    return tau_hat_B


def m3_observer_kf_gmo(
    q3: np.ndarray,
    q3d: np.ndarray,
    d4: np.ndarray,
    d4d: np.ndarray,
    tau3: np.ndarray,
    f4: np.ndarray,
    P: Dict[str, Any],
    Ts: float = 1.0 / 30.0,
    adaptive: bool = False,
    chi_th: float = 3.0,
    gamma: float = 50.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    3-state Kalman-filtered Generalized Momentum Observer (KF-GMO).
    Tracks momentum, generalized load, and load rate [p; tau_L; tau_L_dot].
    Optional adaptive innovation gating for shock impacts.
    """
    N = len(q3)
    tau_hat_KF = np.zeros((2, N), dtype=np.float64)
    sigma_KF = np.zeros((2, N), dtype=np.float64)

    A_kf = np.array(
        [[1.0, Ts, 0.5 * (Ts**2)], [0.0, 1.0, Ts], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    B_kf = np.array([[Ts], [0.0], [0.0]], dtype=np.float64)
    C_kf = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)

    Q_kf3_base = np.diag([10.0, (1000.0 * Ts) ** 2, (1.5e6 * Ts) ** 2])
    Q_kf4_base = np.diag([1.0, (200.0 * Ts) ** 2, (1.5e5 * Ts) ** 2])

    P_kf3 = np.diag([1e4, 1e10, 1e12])
    P_kf4 = np.diag([1e2, 1e8, 1e10])
    x_kf3 = np.zeros((3, 1), dtype=np.float64)
    x_kf4 = np.zeros((3, 1), dtype=np.float64)

    h_A_prev = np.zeros(2, dtype=np.float64)

    eps3 = P.get("eps_obs", 3e-4)
    eps4 = P.get("eps_obs", 3e-4)

    for k in range(N):
        r_k = d4[k] + P["c"]
        D11 = P["Izz"] + P["Md"] * (r_k**2)
        D22 = P["Md"]
        p_k = np.array([D11 * q3d[k], D22 * d4d[k]], dtype=np.float64)
        G_k = np.array(
            [P["Md"] * P["g"] * r_k * np.cos(q3[k]), P["Md"] * P["g"] * np.sin(q3[k])],
            dtype=np.float64,
        )
        CT_k = np.array([0.0, P["Md"] * r_k * (q3d[k] ** 2)], dtype=np.float64)
        Fv_k = np.array([P["fv3"] * q3d[k], P["fv4"] * d4d[k]], dtype=np.float64)
        Fc_k = np.array(
            [P["fc3"] * np.tanh(q3d[k] / eps3), P["fc4"] * np.tanh(d4d[k] / eps4)],
            dtype=np.float64,
        )

        h_B_k = np.array([tau3[k], f4[k]], dtype=np.float64) - G_k - Fv_k + CT_k
        h_A_k = h_B_k - Fc_k

        if k == 0:
            x_kf3 = np.array([[p_k[0]], [0.0], [0.0]], dtype=np.float64)
            x_kf4 = np.array([[p_k[1]], [0.0], [0.0]], dtype=np.float64)
        else:
            R_kf3 = (D11 * 0.003) ** 2
            R_kf4 = (D22 * 0.003) ** 2

            Q3 = Q_kf3_base.copy()
            Q4 = Q_kf4_base.copy()

            # Channel 3 update
            x_pred3 = A_kf @ x_kf3 + B_kf * (0.5 * (h_A_prev[0] + h_A_k[0]))
            P_pred3 = A_kf @ P_kf3 @ A_kf.T + Q3
            S3 = C_kf @ P_pred3 @ C_kf.T + R_kf3
            innov3 = p_k[0] - float((C_kf @ x_pred3)[0, 0])

            if adaptive and (innov3**2) / S3[0, 0] > chi_th**2:
                Q3 = gamma * Q_kf3_base
                P_pred3 = A_kf @ P_kf3 @ A_kf.T + Q3
                S3 = C_kf @ P_pred3 @ C_kf.T + R_kf3

            K3 = (P_pred3 @ C_kf.T) / S3[0, 0]
            x_kf3 = x_pred3 + K3 * innov3
            P_kf3 = (np.eye(3) - K3 @ C_kf) @ P_pred3

            # Channel 4 update
            x_pred4 = A_kf @ x_kf4 + B_kf * (0.5 * (h_A_prev[1] + h_A_k[1]))
            P_pred4 = A_kf @ P_kf4 @ A_kf.T + Q4
            S4 = C_kf @ P_pred4 @ C_kf.T + R_kf4
            innov4 = p_k[1] - float((C_kf @ x_pred4)[0, 0])

            if adaptive and (innov4**2) / S4[0, 0] > chi_th**2:
                Q4 = gamma * Q_kf4_base
                P_pred4 = A_kf @ P_kf4 @ A_kf.T + Q4
                S4 = C_kf @ P_pred4 @ C_kf.T + R_kf4

            K4 = (P_pred4 @ C_kf.T) / S4[0, 0]
            x_kf4 = x_pred4 + K4 * innov4
            P_kf4 = (np.eye(3) - K4 @ C_kf) @ P_pred4

        tau_hat_KF[0, k] = x_kf3[1, 0]
        tau_hat_KF[1, k] = x_kf4[1, 0]
        sigma_KF[0, k] = np.sqrt(max(0.0, P_kf3[1, 1]))
        sigma_KF[1, k] = np.sqrt(max(0.0, P_kf4[1, 1]))

        h_A_prev = h_A_k

    return tau_hat_KF, sigma_KF


def apply_n1_speed_noise(
    q3d: np.ndarray,
    d4d: np.ndarray,
    sigma_v: Tuple[float, float] = (3e-3, 3e-3),
    seed: Optional[int] = 20261118,
) -> Tuple[np.ndarray, np.ndarray]:
    """N1 noise: resolver speed noise 3 mrad/s on q3d and 3 mm/s on d4d."""
    rng = np.random.default_rng(seed)
    q3d_noisy = q3d + sigma_v[0] * rng.standard_normal(len(q3d))
    d4d_noisy = d4d + sigma_v[1] * rng.standard_normal(len(d4d))
    return q3d_noisy, d4d_noisy


def apply_n2_torque_noise(
    tau3: np.ndarray,
    f4: np.ndarray,
    pct_rms: float = 0.01,
    seed: Optional[int] = 20261118,
) -> Tuple[np.ndarray, np.ndarray]:
    """N2 noise: 1% RMS drive torque and force white noise."""
    rng = np.random.default_rng(seed)
    rms3 = np.sqrt(np.mean(tau3**2))
    rms4 = np.sqrt(np.mean(f4**2))
    tau3_noisy = tau3 + pct_rms * rms3 * rng.standard_normal(len(tau3))
    f4_noisy = f4 + pct_rms * rms4 * rng.standard_normal(len(f4))
    return tau3_noisy, f4_noisy


def apply_n4_motor_encoder_quantization(
    q3: np.ndarray, d4: np.ndarray, Ts: float = 1.0 / 30.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    N4 noise: 12-bit optical encoder on high-speed motor shaft.
    Step sizes: 2*pi / (4096 * 240) rad and 2*pi / (4096 * 109.3) m.
    Velocity derived by backward differencing at 30 Hz.
    """
    dM = np.array([2.0 * np.pi / (4096.0 * 240.0), 2.0 * np.pi / (4096.0 * 109.3)])
    qm = dM[0] * np.round(q3 / dM[0])
    dm = dM[1] * np.round(d4 / dM[1])

    q3d_diff = np.zeros_like(qm)
    d4d_diff = np.zeros_like(dm)
    q3d_diff[0] = (qm[1] - qm[0]) / Ts
    q3d_diff[1:] = np.diff(qm) / Ts
    d4d_diff[0] = (dm[1] - dm[0]) / Ts
    d4d_diff[1:] = np.diff(dm) / Ts

    return qm, q3d_diff, dm, d4d_diff


def apply_n3_joint_encoder_quantization(
    q3: np.ndarray, d4: np.ndarray, Ts: float = 1.0 / 30.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    N3 noise: 12-bit optical encoder on joint side (Case 1A convention).
    Step sizes: 2*pi / 4096 rad and 2*pi / 4096 m.
    Velocity derived by backward differencing at 30 Hz.
    """
    dJ = np.array([2.0 * np.pi / 4096.0, 2.0 * np.pi / 4096.0])
    qm = dJ[0] * np.round(q3 / dJ[0])
    dm = dJ[1] * np.round(d4 / dJ[1])

    q3d_diff = np.zeros_like(qm)
    d4d_diff = np.zeros_like(dm)
    q3d_diff[0] = (qm[1] - qm[0]) / Ts
    q3d_diff[1:] = np.diff(qm) / Ts
    d4d_diff[0] = (dm[1] - dm[0]) / Ts
    d4d_diff[1:] = np.diff(dm) / Ts

    return qm, q3d_diff, dm, d4d_diff


def estimate_payload_mass(
    tau_hat_3: np.ndarray,
    q3: np.ndarray,
    d4: np.ndarray,
    t: np.ndarray,
    P: Optional[Dict[str, Any]] = None,
    t_trim: float = 5.0,
) -> float:
    """
    Estimate accreted dipper payload mass (in tonnes) from estimated load torque.
    Averages over steady-state window t >= t_trim.
    """
    if P is None:
        P = get_machine_a_params()
    g = P["g"]
    r_teeth = d4 + P["x_tip"] - P["L_COG"]  # lever arm to teeth

    mask = t >= t_trim
    denom = g * r_teeth[mask] * np.cos(q3[mask])
    # Posture conditioned weighting (avoid cos(q) -> 0 division)
    weights = np.maximum(0.05, np.abs(np.cos(q3[mask]))) ** 2
    mass_samples_kg = (-tau_hat_3[mask]) / denom
    mean_mass_kg = float(np.average(mass_samples_kg, weights=weights))
    return mean_mass_kg / 1000.0  # return in tonnes


def compute_cross_correlation_delay(
    signal_est: np.ndarray,
    signal_ref: np.ndarray,
    Ts: float = 1.0 / 30.0,
    detrend: bool = True,
) -> Dict[str, Any]:
    """
    Compute empirical cross-correlation delay between estimated and reference signals.
    Resolves the MS04P05 sign bug:
      - Correct physical delay: +lag_samples * Ts * 1000.0 (positive for causal lag)
      - Legacy buggy delay:     -lag_samples * Ts * 1000.0 (erroneous negative 'lead')

    Returns dictionary containing lag_samples, delay_ms, buggy_delay_ms, and peak correlation.
    """
    import scipy.signal

    s_est = np.asarray(signal_est, dtype=np.float64)
    s_ref = np.asarray(signal_ref, dtype=np.float64)
    if detrend:
        s_est = s_est - np.mean(s_est)
        s_ref = s_ref - np.mean(s_ref)

    corr = scipy.signal.correlate(s_est, s_ref, mode="full")
    lags = scipy.signal.correlation_lags(len(s_est), len(s_ref), mode="full")
    best_idx = int(np.argmax(corr))
    lag_samples = int(lags[best_idx])

    corrected_delay_ms = float(lag_samples * Ts * 1000.0)
    buggy_delay_ms = float(-lag_samples * Ts * 1000.0)

    return {
        "lag_samples": lag_samples,
        "delay_ms": corrected_delay_ms,
        "buggy_delay_ms": buggy_delay_ms,
        "is_causal": lag_samples >= 0,
        "max_corr": float(corr[best_idx]),
    }
