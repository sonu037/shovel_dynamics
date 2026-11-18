"""
End-to-End Opaque-Box Test Harness for shovel-dynamics.
Zero-dependency Python 3.14 verification infrastructure.
"""

from .fixtures import (
    get_repo_root,
    verify_provenance_md5,
    load_p08b_simscape,
    load_legacy_mat,
)
from .invariants import (
    assert_mass_matrix_positive_definite,
    assert_passivity_skew_symmetry,
    assert_time_grid_regularity,
    assert_oem_physical_bounds,
    assert_thermodynamic_power_dissipation,
    assert_unilateral_cable_tension,
    assert_provenance_hash,
    assert_finite_telemetry,
)
from .models import (
    get_machine_a_params,
    get_machine_b_params,
    compute_mass_matrix,
    compute_coriolis_matrix,
    compute_gravity_vector,
    compute_friction_forces,
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
    compute_cross_correlation_delay,
)
from .runner import TestCase, TestSuite, TestResult, AssertionResult
from .reporter import generate_json_receipt, generate_markdown_report

__all__ = [
    "get_repo_root",
    "verify_provenance_md5",
    "load_p08b_simscape",
    "load_legacy_mat",
    "assert_mass_matrix_positive_definite",
    "assert_passivity_skew_symmetry",
    "assert_time_grid_regularity",
    "assert_oem_physical_bounds",
    "assert_thermodynamic_power_dissipation",
    "assert_unilateral_cable_tension",
    "assert_provenance_hash",
    "assert_finite_telemetry",
    "get_machine_a_params",
    "get_machine_b_params",
    "compute_mass_matrix",
    "compute_coriolis_matrix",
    "compute_gravity_vector",
    "compute_friction_forces",
    "inverse_dynamics_2dof",
    "build_2dof_regressor",
    "solve_parameter_identification",
    "evaluate_regressor_conditioning",
    "m3_observer_branch_b",
    "m3_observer_kf_gmo",
    "apply_n1_speed_noise",
    "apply_n2_torque_noise",
    "apply_n4_motor_encoder_quantization",
    "apply_n3_joint_encoder_quantization",
    "estimate_payload_mass",
    "compute_cross_correlation_delay",
    "TestCase",
    "TestSuite",
    "TestResult",
    "AssertionResult",
    "generate_json_receipt",
    "generate_markdown_report",
]
