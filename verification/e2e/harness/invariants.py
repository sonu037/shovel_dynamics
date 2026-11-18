"""
Physical and mathematical invariant assertion engine for shovel-dynamics.
Strictly verifies conservation laws, stability bounds, timing regularity, and provenance.
"""

from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union
import numpy as np

from .fixtures import compute_file_md5, FROZEN_P08B_MD5


def assert_mass_matrix_positive_definite(
    M_input: Union[np.ndarray, Callable[[float, float], np.ndarray]],
    q_grid: Optional[List[Tuple[float, float]]] = None,
    min_margin: float = 1400.0,
    sym_tol: float = 1e-10,
) -> Dict[str, Any]:
    """
    Assert that mass matrix M(q) is strictly symmetric and positive definite:
    1. Strict symmetry: ||M - M^T||_F < sym_tol (Lagrangian multi-body invariance)
    2. Strict positive definiteness: min_q lambda_min(M(q)) >= min_margin > 0.
    """
    if callable(M_input):
        if q_grid is None:
            # Default operational mesh: q3 in [-30 deg, 60 deg], d4 in [5.0 m, 11.0 m]
            q3_vals = np.linspace(np.radians(-30.0), np.radians(60.0), 20)
            d4_vals = np.linspace(5.0, 11.0, 20)
            q_grid = [(q3, d4) for q3 in q3_vals for d4 in d4_vals]

        min_eig = float("inf")
        worst_pose = None
        for q3, d4 in q_grid:
            M = np.asarray(M_input(q3, d4), dtype=np.float64)
            if M.ndim != 2 or M.shape[0] != M.shape[1]:
                return {
                    "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                    "passed": False,
                    "min_eigenvalue": float("-inf"),
                    "required_margin": float(min_margin),
                    "worst_pose": (q3, d4),
                    "message": f"INVARIANT_VIOLATION: Mass matrix at pose ({q3:.3f}, {d4:.3f}) is not square: shape {M.shape}",
                }
            asymm_norm = float(np.linalg.norm(M - M.T, "fro"))
            if asymm_norm >= sym_tol:
                return {
                    "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                    "passed": False,
                    "min_eigenvalue": float("-inf"),
                    "required_margin": float(min_margin),
                    "worst_pose": (q3, d4),
                    "message": f"INVARIANT_VIOLATION: Mass matrix is not symmetric at pose ({q3:.3f}, {d4:.3f}): ||M - M^T||_F = {asymm_norm:.2e} >= {sym_tol:.2e}",
                }
            eigs = np.linalg.eigvalsh(M)
            local_min = float(np.min(eigs))
            if local_min < min_eig:
                min_eig = local_min
                worst_pose = (q3, d4)
    else:
        M_arr = np.asarray(M_input, dtype=np.float64)
        if M_arr.ndim == 2:
            if M_arr.shape[0] != M_arr.shape[1]:
                return {
                    "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                    "passed": False,
                    "min_eigenvalue": float("-inf"),
                    "required_margin": float(min_margin),
                    "worst_pose": None,
                    "message": f"INVARIANT_VIOLATION: Mass matrix is not square: shape {M_arr.shape}",
                }
            asymm_norm = float(np.linalg.norm(M_arr - M_arr.T, "fro"))
            if asymm_norm >= sym_tol:
                return {
                    "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                    "passed": False,
                    "min_eigenvalue": float("-inf"),
                    "required_margin": float(min_margin),
                    "worst_pose": None,
                    "message": f"INVARIANT_VIOLATION: Mass matrix is not symmetric: ||M - M^T||_F = {asymm_norm:.2e} >= {sym_tol:.2e}",
                }
            eigs = np.linalg.eigvalsh(M_arr)
            min_eig = float(np.min(eigs))
            worst_pose = None
        elif M_arr.ndim == 3:
            if M_arr.shape[1] != M_arr.shape[2]:
                return {
                    "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                    "passed": False,
                    "min_eigenvalue": float("-inf"),
                    "required_margin": float(min_margin),
                    "worst_pose": None,
                    "message": f"INVARIANT_VIOLATION: 3D mass matrix slices are not square: shape {M_arr.shape}",
                }
            for idx, M_slice in enumerate(M_arr):
                asymm_norm = float(np.linalg.norm(M_slice - M_slice.T, "fro"))
                if asymm_norm >= sym_tol:
                    return {
                        "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
                        "passed": False,
                        "min_eigenvalue": float("-inf"),
                        "required_margin": float(min_margin),
                        "worst_pose": idx,
                        "message": f"INVARIANT_VIOLATION: Mass matrix slice {idx} is not symmetric: ||M - M^T||_F = {asymm_norm:.2e} >= {sym_tol:.2e}",
                    }
            min_eig = float(np.min([np.min(np.linalg.eigvalsh(M)) for M in M_arr]))
            worst_pose = None
        else:
            raise ValueError(f"Unsupported mass matrix shape {M_arr.shape}")

    passed = min_eig >= min_margin
    return {
        "invariant": "MASS_MATRIX_POSITIVE_DEFINITE",
        "passed": bool(passed),
        "min_eigenvalue": float(min_eig),
        "required_margin": float(min_margin),
        "worst_pose": worst_pose,
        "message": (
            f"Mass matrix strictly positive definite (min eig: {min_eig:.2e} >= {min_margin:.2e})"
            if passed
            else f"INVARIANT_VIOLATION: Mass matrix non-positive definite or margin violated ({min_eig:.2e} < {min_margin:.2e})"
        ),
    }


def assert_passivity_skew_symmetry(
    M_dot: np.ndarray, C: np.ndarray, tol: float = 1e-12
) -> Dict[str, Any]:
    """
    Assert that S = M_dot - 2*C is skew-symmetric, i.e., ||S + S^T||_F < tol.
    Guarantees strict passivity and mechanical energy conservation.
    """
    S = M_dot - 2.0 * C
    skew_residual = S + S.T
    norm_fro = float(np.linalg.norm(skew_residual, "fro"))
    passed = norm_fro < tol
    return {
        "invariant": "PASSIVITY_SKEW_SYMMETRY",
        "passed": bool(passed),
        "frobenius_norm": norm_fro,
        "tolerance": tol,
        "message": (
            f"Passivity skew-symmetry satisfied (norm: {norm_fro:.2e} < {tol:.2e})"
            if passed
            else f"INVARIANT_VIOLATION: Skew-symmetry passivity violated (norm: {norm_fro:.2e} >= {tol:.2e})"
        ),
    }


def assert_time_grid_regularity(
    t: np.ndarray, expected_dt: float = 1.0 / 30.0, tol: float = 1e-9
) -> Dict[str, Any]:
    """
    Assert strict 30 Hz uniform time grid with zero clock jitter: max |dt - expected_dt| < tol.
    Handles short arrays (N < 2) gracefully by returning passed=False without raising unhandled ValueError.
    """
    t_arr = np.asarray(t, dtype=np.float64) if t is not None else np.array([])
    if t_arr.size < 2:
        return {
            "invariant": "TIME_GRID_REGULARITY",
            "passed": False,
            "sample_count": int(t_arr.size),
            "max_jitter_sec": float("inf"),
            "tolerance_sec": tol,
            "message": f"INVARIANT_VIOLATION: Time grid must contain at least 2 points (got {t_arr.size})",
        }

    if not np.all(np.isfinite(t_arr)):
        return {
            "invariant": "TIME_GRID_REGULARITY",
            "passed": False,
            "sample_count": int(t_arr.size),
            "max_jitter_sec": float("inf"),
            "tolerance_sec": tol,
            "message": "INVARIANT_VIOLATION: Non-finite timestamps (NaN/Inf) detected in time grid",
        }

    diffs = np.diff(t_arr)
    max_dev = float(np.max(np.abs(diffs - expected_dt)))
    passed = max_dev < tol
    return {
        "invariant": "TIME_GRID_REGULARITY",
        "passed": bool(passed),
        "sample_count": int(len(t_arr)),
        "max_jitter_sec": max_dev,
        "tolerance_sec": tol,
        "message": (
            f"Time grid strictly uniform at {1.0/expected_dt:.1f} Hz (jitter: {max_dev:.2e} s < {tol:.2e} s)"
            if passed
            else f"INVARIANT_VIOLATION: 30 Hz time grid regularity failure (jitter: {max_dev:.2e} s >= {tol:.2e} s)"
        ),
    }


def assert_oem_physical_bounds(
    f_crowd: Optional[np.ndarray] = None,
    v_crowd: Optional[np.ndarray] = None,
    max_force_kn: float = 745.0,
    max_vel_mps: float = 0.540,
    unit: str = "auto",
) -> Dict[str, Any]:
    """
    Assert that actuator dynamics remain strictly within OEM physical envelopes:
    |F_crowd| <= 745 kN, |v_crowd| <= 0.540 m/s.

    Unit Handling for f_crowd:
      - 'auto': If max(|f_crowd|) > 2000.0, input is in Newtons (N), convert once to kN (/ 1000.0).
                If max(|f_crowd|) <= 2000.0, input is already in kilonewtons (kN).
                Eliminates the double-divide bug where forces > 1,000,000 N were divided twice.
      - 'N': Force values are strictly in Newtons (converted to kN via / 1000.0).
      - 'kN': Force values are strictly in kilonewtons (no conversion).
    """
    force_passed = True
    peak_force_kn = 0.0
    if f_crowd is not None:
        f_arr = np.asarray(f_crowd, dtype=np.float64)
        if f_arr.size == 0:
            return {
                "invariant": "OEM_PHYSICAL_BOUNDS",
                "passed": False,
                "peak_force_kn": 0.0,
                "max_force_kn": max_force_kn,
                "peak_vel_mps": 0.0,
                "max_vel_mps": max_vel_mps,
                "message": "INVARIANT_VIOLATION: Empty crowd force array provided",
            }
        if not np.all(np.isfinite(f_arr)):
            return {
                "invariant": "OEM_PHYSICAL_BOUNDS",
                "passed": False,
                "peak_force_kn": float("inf"),
                "max_force_kn": max_force_kn,
                "peak_vel_mps": 0.0,
                "max_vel_mps": max_vel_mps,
                "message": "INVARIANT_VIOLATION: Non-finite values (NaN/Inf) detected in crowd force",
            }

        max_val = float(np.max(np.abs(f_arr)))
        if unit == "N":
            peak_force_kn = max_val / 1000.0
        elif unit == "kN":
            peak_force_kn = max_val
        elif unit == "auto":
            if max_val > 2000.0:
                peak_force_kn = max_val / 1000.0
            else:
                peak_force_kn = max_val
        else:
            raise ValueError(f"Unsupported force unit '{unit}'. Must be 'auto', 'N', or 'kN'.")

        force_passed = peak_force_kn <= max_force_kn

    vel_passed = True
    peak_vel_mps = 0.0
    if v_crowd is not None:
        v_arr = np.asarray(v_crowd, dtype=np.float64)
        if v_arr.size == 0:
            return {
                "invariant": "OEM_PHYSICAL_BOUNDS",
                "passed": False,
                "peak_force_kn": peak_force_kn,
                "max_force_kn": max_force_kn,
                "peak_vel_mps": 0.0,
                "max_vel_mps": max_vel_mps,
                "message": "INVARIANT_VIOLATION: Empty crowd velocity array provided",
            }
        if not np.all(np.isfinite(v_arr)):
            return {
                "invariant": "OEM_PHYSICAL_BOUNDS",
                "passed": False,
                "peak_force_kn": peak_force_kn,
                "max_force_kn": max_force_kn,
                "peak_vel_mps": float("inf"),
                "max_vel_mps": max_vel_mps,
                "message": "INVARIANT_VIOLATION: Non-finite values (NaN/Inf) detected in crowd velocity",
            }

        peak_vel_mps = float(np.max(np.abs(v_arr)))
        vel_passed = peak_vel_mps <= max_vel_mps

    passed = force_passed and vel_passed
    return {
        "invariant": "OEM_PHYSICAL_BOUNDS",
        "passed": bool(passed),
        "peak_force_kn": peak_force_kn,
        "max_force_kn": max_force_kn,
        "peak_vel_mps": peak_vel_mps,
        "max_vel_mps": max_vel_mps,
        "message": (
            f"Actuator states within OEM envelopes (Force: {peak_force_kn:.1f}/{max_force_kn} kN, Speed: {peak_vel_mps:.3f}/{max_vel_mps} m/s)"
            if passed
            else f"INVARIANT_VIOLATION: OEM physical envelope exceeded (Force: {peak_force_kn:.1f} kN, Speed: {peak_vel_mps:.3f} m/s)"
        ),
    }


def assert_thermodynamic_power_dissipation(
    P_diss: np.ndarray, tol: float = 0.0
) -> Dict[str, Any]:
    """
    Assert that mechanical energy dissipation is strictly non-negative: P_diss >= 0.
    Ensures second law of thermodynamics (no unphysical energy generation).
    """
    min_diss = float(np.min(P_diss))
    passed = min_diss >= -tol
    return {
        "invariant": "THERMODYNAMIC_POWER_DISSIPATION",
        "passed": bool(passed),
        "min_dissipation_watts": min_diss,
        "message": (
            f"Thermodynamic dissipation non-negative (min: {min_diss:.2e} W)"
            if passed
            else f"INVARIANT_VIOLATION: Negative dissipation detected (spurious energy creation: {min_diss:.2e} W)"
        ),
    }


def assert_unilateral_cable_tension(
    T_rope: np.ndarray, tol: float = 0.0
) -> Dict[str, Any]:
    """
    Assert that wire-rope tension is strictly unilateral: T_rope(t) >= 0 (zero compressive resistance).
    """
    min_tension = float(np.min(T_rope))
    passed = min_tension >= -tol
    return {
        "invariant": "UNILATERAL_CABLE_TENSION",
        "passed": bool(passed),
        "min_tension_n": min_tension,
        "message": (
            f"Unilateral cable constraint strictly satisfied (min tension: {min_tension:.2f} N >= 0)"
            if passed
            else f"INVARIANT_VIOLATION: Compressive cable force detected (tension: {min_tension:.2f} N < 0)"
        ),
    }


def assert_provenance_hash(
    file_path: Union[str, Path], expected_md5: str = FROZEN_P08B_MD5
) -> Dict[str, Any]:
    """
    Assert exact cryptographic provenance MD5 match on raw binary archives.
    """
    actual_md5 = compute_file_md5(file_path)
    passed = actual_md5.lower() == expected_md5.lower()
    return {
        "invariant": "DATASET_PROVENANCE_MD5",
        "passed": bool(passed),
        "actual_md5": actual_md5,
        "expected_md5": expected_md5,
        "file_path": str(file_path),
        "message": (
            f"Cryptographic provenance verified (MD5: {actual_md5})"
            if passed
            else f"PROVENANCE_MISMATCH: Computed MD5 {actual_md5} != expected {expected_md5}"
        ),
    }


def assert_finite_telemetry(signals: Dict[str, np.ndarray]) -> Dict[str, Any]:
    """
    Assert that all signals in telemetry contain strictly finite values (no NaN or Inf).
    """
    non_finite_fields = []
    for name, arr in signals.items():
        if isinstance(arr, np.ndarray) and not np.all(np.isfinite(arr)):
            non_finite_fields.append(name)
    passed = len(non_finite_fields) == 0
    return {
        "invariant": "NUMERICAL_FINITENESS",
        "passed": bool(passed),
        "non_finite_fields": non_finite_fields,
        "message": (
            "All telemetry signals finite (zero NaN/Inf)"
            if passed
            else f"INVARIANT_VIOLATION: Non-finite values (NaN/Inf) detected in fields: {non_finite_fields}"
        ),
    }
