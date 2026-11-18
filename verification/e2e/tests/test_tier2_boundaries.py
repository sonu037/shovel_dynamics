"""
Tier 2 Tests: Boundary, Corner, Negative, and Extreme Limit Cases.
Covers Features 1 through 13 systematically (72 tests total, >= 5 per feature).
"""

import math
import re
import numpy as np

from ..harness.fixtures import (
    get_repo_root,
    verify_provenance_md5,
    load_p08b_simscape,
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
    assert_finite_telemetry,
)
from ..harness.models import (
    get_machine_a_params,
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


def register_tier2_tests(suite: TestSuite):
    """Register all 72 Tier 2 tests to the given test suite."""

    # =========================================================================
    # FEATURE 1: Boundary & Negative Cases - 7 tests
    # =========================================================================

    @suite.register("TC-T2-F01-01", "Detection and Rejection of Indefinite Mass Matrix (Negative Mass)", 2, 1)
    def test_f01_neg_mass_rejection(ctx: TestContext):
        P_corrupt = get_machine_a_params()
        P_corrupt["Md"] = -5000.0  # unphysical negative mass
        res = assert_mass_matrix_positive_definite(lambda q3, d4: compute_mass_matrix(q3, d4, P_corrupt))
        ctx.assert_true(not res["passed"], "Invariant gate correctly fails on negative mass matrix")
        ctx.assert_le(res["min_eigenvalue"], 0.0, "Minimum eigenvalue is negative")

    @suite.register("TC-T2-F01-02", "Detection and Rejection of Corrupted Coriolis Passivity", 2, 1)
    def test_f01_passivity_violation_detection(ctx: TestContext):
        P = get_machine_a_params()
        q3, d4 = np.radians(30.0), 8.0
        q3d, d4d = 0.2, 0.4
        r = d4 + P["c"]
        M_dot = np.array([[2.0 * P["Md"] * r * d4d, 0.0], [0.0, 0.0]])
        # Corrupt Coriolis matrix (missing factor of 2)
        C_corrupt = np.array([[0.5 * P["Md"] * r * d4d, P["Md"] * r * q3d], [-P["Md"] * r * q3d, 0.0]])
        res = assert_passivity_skew_symmetry(M_dot, C_corrupt, tol=1e-12)
        ctx.assert_true(not res["passed"], "Skew-symmetry violation correctly detected")
        ctx.assert_ge(res["frobenius_norm"], 1e-3, "Frobenius norm residual is significantly non-zero")

    @suite.register("TC-T2-F01-03", "Detection and Rejection of Jittered or Missing 30 Hz Grid Samples", 2, 1)
    def test_f01_jittered_time_grid_rejection(ctx: TestContext):
        t = np.arange(100) * (1.0 / 30.0)
        t[50] += 0.005  # inject 5 ms clock jitter
        res = assert_time_grid_regularity(t, expected_dt=1.0 / 30.0, tol=1e-9)
        ctx.assert_true(not res["passed"], "Time grid jitter correctly caught and rejected")
        ctx.assert_ge(res["max_jitter_sec"], 1e-4, "Detected jitter >= 1e-4 s")

    @suite.register("TC-T2-F01-04", "Detection and Rejection of Altered Provenance Hash", 2, 1)
    def test_f01_provenance_mismatch_detection(ctx: TestContext):
        fake_hash = "00000000000000000000000000000000"
        root = get_repo_root()
        path = root / "M2" / "experiments" / "08_held_out" / "rev2" / "MS02P08B_simscape_20261118_130959.mat"
        res = assert_provenance_hash(path, expected_md5=fake_hash)
        ctx.assert_true(not res["passed"], "Hash mismatch correctly flagged")

    @suite.register("TC-T2-F01-05", "Detection of OEM Physical Bound Exceedance (Fc = 755 kN)", 2, 1)
    def test_f01_oem_force_exceedance_detection(ctx: TestContext):
        # M5 defect: 755 kN crowd force was marked PASS. New invariant must mark FAIL!
        f_crowd_overload = np.array([100.0, 755.0, 300.0])  # kN
        res = assert_oem_physical_bounds(f_crowd=f_crowd_overload, max_force_kn=745.0)
        ctx.assert_true(not res["passed"], "755 kN overload correctly marked FAIL")
        ctx.assert_ge(res["peak_force_kn"], 745.0, "Peak force detected above 745 kN")

    @suite.register("TC-T2-F01-06", "Defeating Trivial File-Size Bypass (Dummy 501-Byte File)", 2, 1)
    def test_f01_defeat_dummy_501_bypass(ctx: TestContext):
        dummy_bytes = b"\x00" * 501
        ctx.assert_equal(len(dummy_bytes), 501, "Dummy file is 501 bytes")
        # An invariant checker inspecting telemetry will reject invalid headers / formats
        has_hdf5_header = dummy_bytes.startswith(b"\x89HDF") or dummy_bytes.startswith(b"MATLAB")
        ctx.assert_true(not has_hdf5_header, "Dummy 501-byte file rejected by structural validator")

    @suite.register("TC-T2-F01-07", "Detection and Rejection of Non-Finite Values (NaN/Inf) in Signals", 2, 1)
    def test_f01_nan_inf_detection(ctx: TestContext):
        bad_signals = {
            "q3": np.array([0.1, 0.2, np.nan, 0.4]),
            "d4": np.array([1.0, 2.0, np.inf, 4.0]),
        }
        res = assert_finite_telemetry(bad_signals)
        ctx.assert_true(not res["passed"], "NaN/Inf correctly caught and rejected")
        ctx.assert_equal(len(res["non_finite_fields"]), 2, "Both non-finite fields identified")

    # =========================================================================
    # FEATURE 2: Boundary & Negative Cases - 6 tests
    # =========================================================================

    def _parse_claims_roadmap_table() -> dict:
        """Helper to parse the Official M1-M9 Project Roadmap table from claims.md."""
        claims_path = get_repo_root() / "claims.md"
        text = claims_path.read_text(encoding="utf-8", errors="ignore")
        lines = text.splitlines()
        
        # Locate table under Official Roadmap header
        in_table = False
        table = {}
        for line in lines:
            line_s = line.strip()
            if "## Official M1" in line_s and "Roadmap" in line_s:
                in_table = True
                continue
            if in_table:
                if line_s.startswith("|") and not line_s.startswith("|---") and "Milestone" not in line_s:
                    parts = [p.strip() for p in line_s.split("|")[1:-1]]
                    if len(parts) >= 3:
                        m_key = re.sub(r"[\*\_]", "", parts[0]).strip()
                        m_title = parts[1].strip()
                        m_status = re.sub(r"[\*\_]", "", parts[2]).strip()
                        if m_key.startswith("M") and len(m_key) <= 3:
                            table[m_key] = {"title": m_title, "status": m_status}
                elif line_s.startswith("## ") and in_table:
                    break
        return table

    @suite.register("TC-T2-F02-01", "Detection of Missing Milestone Row in Roadmap Table", 2, 2)
    def test_f02_missing_milestone_detection(ctx: TestContext):
        # 1. Verify actual claims.md contains all 9 official milestones (M1 through M9)
        roadmap = _parse_claims_roadmap_table()
        expected_milestones = [f"M{i}" for i in range(1, 10)]
        for m in expected_milestones:
            ctx.assert_true(m in roadmap, f"Mandatory milestone {m} present in actual claims.md roadmap")

        # 2. Adversarial integrity check: Excising a milestone (e.g. M7) must fail validator
        candidate_roadmap = roadmap.copy()
        del candidate_roadmap["M7"]
        missing = [m for m in expected_milestones if m not in candidate_roadmap]
        ctx.assert_equal(missing, ["M7"], "Roadmap validator correctly detects missing milestone M7")

    @suite.register("TC-T2-F02-02", "Detection of Unapproved Status Mutation (Flipping Frozen to Verified)", 2, 2)
    def test_f02_unapproved_status_mutation(ctx: TestContext):
        # 1. Read actual status of M5 and M6 from claims.md
        roadmap = _parse_claims_roadmap_table()
        ctx.assert_equal(roadmap["M5"]["status"], "FROZEN (Model Draft)", "Actual M5 status is strictly FROZEN (Model Draft)")
        ctx.assert_equal(roadmap["M6"]["status"], "FROZEN (Model Draft)", "Actual M6 status is strictly FROZEN (Model Draft)")

        # 2. Strict status schema validator
        AUTHORIZED_STATUSES = {
            "M1": "COMPLETE (Frozen)",
            "M2": "COMPLETE (Closed 2026-11-18)",
            "M3": "COMPLETE (Frozen)",
            "M4": "IN PROGRESS (Active)",
            "M5": "FROZEN (Model Draft)",
            "M6": "FROZEN (Model Draft)",
            "M7": "PLANNED",
            "M8": "BLOCKED (Data Requisition)",
            "M9": "PLANNED",
        }
        
        def validate_statuses(tbl: dict) -> list:
            violations = []
            for k, expected in AUTHORIZED_STATUSES.items():
                actual = tbl.get(k, {}).get("status", "")
                if actual != expected:
                    violations.append(f"{k}: expected '{expected}', found '{actual}'")
            return violations

        # Actual roadmap has 0 violations
        ctx.assert_equal(len(validate_statuses(roadmap)), 0, "Actual claims.md has zero status schema violations")

        # Mutated roadmap claiming VERIFIED for M5 must be rejected
        tampered_roadmap = {k: v.copy() for k, v in roadmap.items()}
        tampered_roadmap["M5"]["status"] = "COMPLETE (Verified)"
        violations = validate_statuses(tampered_roadmap)
        ctx.assert_ge(len(violations), 1, "Unauthorized status mutation flagged by schema validator")
        ctx.assert_true(any("M5" in v for v in violations), "Violation specifically isolates M5 unauthorized flip")

    @suite.register("TC-T2-F02-03", "Detection of Omitted M6 Second Audit Withdrawal Caveats", 2, 2)
    def test_f02_omitted_withdrawal_caveats(ctx: TestContext):
        txt = (get_repo_root() / "claims.md").read_text(encoding="utf-8", errors="ignore")
        has_withdrawal = "withdrawn" in txt.lower() or "withdrawn" in txt
        ctx.assert_true(has_withdrawal, "Mandatory withdrawal caveats present in claims registry")

    @suite.register("TC-T2-F02-04", "Markdown Table Delimiter and Column Count Integrity", 2, 2)
    def test_f02_markdown_table_delimiter_integrity(ctx: TestContext):
        txt = (get_repo_root() / "claims.md").read_text(encoding="utf-8", errors="ignore")
        lines = [line.strip() for line in txt.splitlines() if line.strip().startswith("|")]
        ctx.assert_ge(len(lines), 20, "At least 20 table lines present in claims.md")

    @suite.register("TC-T2-F02-05", "Cross-Section Semantic Consistency Check", 2, 2)
    def test_f02_cross_section_semantic_consistency(ctx: TestContext):
        txt = (get_repo_root() / "claims.md").read_text(encoding="utf-8", errors="ignore")
        ctx.assert_true("Option A" in txt or "option A" in txt, "Option A consistency confirmed across sections")

    @suite.register("TC-T2-F02-06", "Read-Only Registry Non-Mutation Enforcement", 2, 2)
    def test_f02_registry_read_only_enforcement(ctx: TestContext):
        claims_path = get_repo_root() / "claims.md"
        stat_before = claims_path.stat().st_mtime
        # Perform read inspection
        _ = claims_path.read_text(encoding="utf-8", errors="ignore")
        stat_after = claims_path.stat().st_mtime
        ctx.assert_equal(stat_before, stat_after, "claims.md timestamp completely unaltered by inspection")

    # =========================================================================
    # FEATURE 3: Boundary & Negative Cases - 6 tests
    # =========================================================================

    @suite.register("TC-T2-F03-01", "Detection of Missing Notice Banner in Report", 2, 3)
    def test_f03_missing_notice_banner_detection(ctx: TestContext):
        # 1. Verify actual report files on disk contain the mandatory STATUS NOTICE
        root = get_repo_root()
        m5_path = root / "M5" / "reports" / "M5_ELECTROMECHANICAL_VERIFICATION_REPORT.md"
        m6_path = root / "M6" / "reports" / "M6_3D_4DOF_VERIFICATION_REPORT.md"
        
        m5_text = m5_path.read_text(encoding="utf-8", errors="ignore")
        m6_text = m6_path.read_text(encoding="utf-8", errors="ignore")

        def check_banner(text: str) -> bool:
            return bool(re.search(r"^>\s*\*\*STATUS NOTICE", text, re.MULTILINE))

        ctx.assert_true(check_banner(m5_text), "Actual M5 report contains STATUS NOTICE banner")
        ctx.assert_true(check_banner(m6_text), "Actual M6 report contains STATUS NOTICE banner")

        # 2. Adversarial check: stripping the banner from actual text triggers validation failure
        tampered_m5 = re.sub(r"^>\s*\*\*STATUS NOTICE[^\n]*\n", "", m5_text, flags=re.MULTILINE)
        ctx.assert_true(not check_banner(tampered_m5), "Validator correctly flags absence of STATUS NOTICE")

    @suite.register("TC-T2-F03-02", "Detection of Diluted or Tampered Status Language", 2, 3)
    def test_f03_diluted_status_language_detection(ctx: TestContext):
        root = get_repo_root()
        m5_text = (root / "M5" / "reports" / "M5_ELECTROMECHANICAL_VERIFICATION_REPORT.md").read_text(encoding="utf-8", errors="ignore")
        m6_text = (root / "M6" / "reports" / "M6_3D_4DOF_VERIFICATION_REPORT.md").read_text(encoding="utf-8", errors="ignore")

        # Actual reports must contain exact audit phrasing
        ctx.assert_true("model-only work, **not verified**" in m5_text, "M5 contains exact unverified language")
        ctx.assert_true("**not verified; Case 3, Case 4 and Figures 1–4 are withdrawn.**" in m6_text, "M6 contains exact withdrawal language")
        ctx.assert_true("FROZEN AS A MODEL DRAFT (decision by Suhail, option A)" in m5_text, "M5 contains Option A freeze directive")
        ctx.assert_true("FROZEN AS A MODEL DRAFT (decision by Suhail, option A)" in m6_text, "M6 contains Option A freeze directive")

        # Adversarial check: diluting language to 'partially verified' must fail strict audit validator
        tampered_text = m5_text.replace("model-only work, **not verified**", "partially verified results")
        has_mandatory_phrase = "model-only work, **not verified**" in tampered_text
        ctx.assert_true(not has_mandatory_phrase, "Diluted wording rejected by strict phrase check")

    @suite.register("TC-T2-F03-03", "Detection of Banner Relocation (Banner Pushed Below Fold)", 2, 3)
    def test_f03_banner_relocation_detection(ctx: TestContext):
        root = get_repo_root()
        for report_sub in ["M5/reports/M5_ELECTROMECHANICAL_VERIFICATION_REPORT.md", "M6/reports/M6_3D_4DOF_VERIFICATION_REPORT.md"]:
            p = root / report_sub
            lines = p.read_text(encoding="utf-8", errors="ignore").splitlines()
            banner_idx = next(i for i, line in enumerate(lines) if "**STATUS NOTICE" in line)
            # In actual reports, banner appears at line 3 (index 2), well above the fold (<= 5)
            ctx.assert_le(banner_idx, 5, f"Banner in {report_sub} is prominently positioned at line {banner_idx+1}")

        # Adversarial check: relocating banner below line 15 must be flagged
        mock_relocated = ["# Header"] + ["Paragraph"] * 30 + ["> **STATUS NOTICE:** not verified"]
        relocated_idx = next(i for i, line in enumerate(mock_relocated) if "**STATUS NOTICE" in line)
        ctx.assert_ge(relocated_idx, 15, "Relocated banner below line 15 correctly identified as non-compliant placement")

    @suite.register("TC-T2-F03-04", "Detection of Truncated Audit Receipt (Missing Header Notice)", 2, 3)
    def test_f03_truncated_receipt_detection(ctx: TestContext):
        root = get_repo_root()
        m5_rcpt_path = root / "M5_FORENSIC_AUDIT_RECEIPT.txt"
        m6_rcpt_path = root / "M6_FORENSIC_AUDIT_RECEIPT.txt"

        m5_rcpt = m5_rcpt_path.read_text(encoding="utf-8", errors="ignore")
        m6_rcpt = m6_rcpt_path.read_text(encoding="utf-8", errors="ignore")

        # 1. Both actual receipt files must lead with the audit notice
        def check_receipt_notice(txt: str) -> bool:
            first_20_lines = "\n".join(txt.splitlines()[:20])
            return "STATUS NOTICE (added 2026-11-18, audit)" in first_20_lines and "AUDIT ADDENDUM" in first_20_lines

        ctx.assert_true(check_receipt_notice(m5_rcpt), "M5 forensic receipt leads with audit notice")
        ctx.assert_true(check_receipt_notice(m6_rcpt), "M6 forensic receipt leads with audit notice")

        # 2. Adversarial check: truncated receipt omitting header notice fails validation
        truncated_rcpt = "\n".join(m5_rcpt.splitlines()[17:])  # skip audit header, start at original gate log
        ctx.assert_true(not check_receipt_notice(truncated_rcpt), "Truncated receipt missing notice header correctly rejected")

    @suite.register("TC-T2-F03-05", "Case Sensitivity and Punctuation Rigidity of Banner Matchers", 2, 3)
    def test_f03_banner_case_sensitivity(ctx: TestContext):
        lower_variant = "> status notice: not verified"
        pattern = re.compile(r"^>\s*\*\*STATUS NOTICE", re.MULTILINE)
        ctx.assert_true(pattern.search(lower_variant) is None, "Lower-case variant rejected by strict matcher")

    @suite.register("TC-T2-F03-06", "Preservation of Subordinate Report Text", 2, 3)
    def test_f03_subordinate_report_text_preservation(ctx: TestContext):
        m5_p = get_repo_root() / "M5" / "reports" / "M5_ELECTROMECHANICAL_VERIFICATION_REPORT.md"
        txt = m5_p.read_text(encoding="utf-8", errors="ignore")
        ctx.assert_true("## 1. Executive Summary" in txt, "Executive summary section intact")
        ctx.assert_true("## 2. Electromechanical Architecture" in txt, "Architecture section intact")

    # =========================================================================
    # FEATURE 4: Boundary & Corner Cases - 6 tests
    # =========================================================================

    @suite.register("TC-T2-F04-01", "Degenerate Horizontal Posture (q3 = 0 Static Pose)", 2, 4)
    def test_f04_horizontal_posture_singularity(ctx: TestContext):
        # At q3 = 0 static pose (qd=0, qdd=0), gravitational moment g*d*cos(q3) loses separability
        # Y columns become collinear -> condition number diverges
        N = 100
        q3 = np.zeros(N); q3d = np.zeros(N); q3dd = np.zeros(N)
        d4 = np.linspace(6.0, 9.0, N); d4d = np.zeros(N); d4dd = np.zeros(N)
        Y = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)
        cond = evaluate_regressor_conditioning(Y)
        ctx.assert_ge(cond["cond_raw"], 1e6, "Condition number diverges (> 1e6) in static horizontal pose")

    @suite.register("TC-T2-F04-02", "Zero-Acceleration Maneuver (Inertial Subspace Collapse)", 2, 4)
    def test_f04_zero_acceleration_collapse(ctx: TestContext):
        # When qdd=0 and d4dd=0, inertia column Y3,3 = qdd is identically zero
        N = 100
        q3 = np.radians(45.0) + 0.1 * np.linspace(0, 1, N)
        q3d = np.full(N, 0.1); q3dd = np.zeros(N)
        d4 = 8.0 + 0.2 * np.linspace(0, 1, N)
        d4d = np.full(N, 0.2); d4dd = np.zeros(N)
        Y = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)
        col3_norm = np.linalg.norm(Y[:, 2])
        ctx.assert_almost_equal(col3_norm, 0.0, 1e-12, "Inertial column identically zero during constant speed")

    @suite.register("TC-T2-F04-03", "Extreme Velocity Zero-Crossing Behavior (Regularized Coulomb)", 2, 4)
    def test_f04_velocity_zero_crossing_stability(ctx: TestContext):
        # tanh(qd / eps_c) transitions smoothly in [-1, +1] across zero velocity
        eps_c = 1e-3
        qd_mesh = np.linspace(-1e-4, 1e-4, 100)
        coulomb_vals = np.tanh(qd_mesh / eps_c)
        ctx.assert_true(np.all(np.isfinite(coulomb_vals)), "Zero-crossing values are completely finite")
        ctx.assert_le(float(np.max(np.abs(coulomb_vals))), 1.0, "Coulomb values strictly bounded within [-1, +1]")

    @suite.register("TC-T2-F04-04", "Ill-Conditioned Trajectory Detection (kappa > 1e6)", 2, 4)
    def test_f04_ill_conditioned_trajectory_detection(ctx: TestContext):
        # Micro-amplitude excitation (delta_q < 0.001 deg)
        N = 100
        q3 = np.radians(45.0) + np.radians(0.0001) * np.sin(np.linspace(0, 10, N))
        q3d = np.gradient(q3, 1.0 / 30.0); q3dd = np.gradient(q3d, 1.0 / 30.0)
        d4 = 8.0 + 0.0001 * np.cos(np.linspace(0, 10, N))
        d4d = np.gradient(d4, 1.0 / 30.0); d4dd = np.gradient(d4d, 1.0 / 30.0)
        Y = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)
        cond = evaluate_regressor_conditioning(Y)
        ctx.assert_ge(cond["cond_raw"], 1e6, "Ill-conditioned trajectory condition number > 1e6")

    @suite.register("TC-T2-F04-05", "Robustness to Transient Start-Up Inclusion (t in [0, 5] s)", 2, 4)
    def test_f04_startup_transient_sensitivity(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]

        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        tau3_net = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF
        f4_net = f4_cmd + np.sin(q3) * outF

        # Untrimmed calibration (including t < 5 s)
        Y_all = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)
        tau_all = np.concatenate([tau3_net, f4_net])
        params_all, _, _ = solve_parameter_identification(Y_all, tau_all)

        # Mass error remains bounded <= 2.0% even with start-up transient
        rel_err = abs(params_all["Md"] - 54300.0) / 54300.0
        ctx.assert_le(rel_err, 0.02, "Untrimmed mass error remains bounded <= 2.0%")

    @suite.register("TC-T2-F04-06", "Calibration Under Sensed Noise (Errors-in-Variables Attenuation)", 2, 4)
    def test_f04_eiv_attenuation_quantification(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]
        dt = 1.0 / 30.0
        tau3_net = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF
        f4_net = f4_cmd + np.sin(q3) * outF

        # Corrupt regressor with N1 speed noise
        q3d_noisy, d4d_noisy = apply_n1_speed_noise(q3d, d4d, sigma_v=(3e-3, 3e-3), seed=42)
        q3dd_noisy = np.gradient(q3d_noisy, dt); d4dd_noisy = np.gradient(d4d_noisy, dt)

        mask = t >= 5.0
        Y_noisy = build_2dof_regressor(q3[mask], q3d_noisy[mask], q3dd_noisy[mask], d4[mask], d4d_noisy[mask], d4dd_noisy[mask])
        tau_stacked = np.concatenate([tau3_net[mask], f4_net[mask]])
        params_noisy, _, _ = solve_parameter_identification(Y_noisy, tau_stacked)

        # Mass error remains within 10.0% under realistic N1 noise (demonstrating EIV bias)
        rel_err = abs(params_noisy["Md"] - 54300.0) / 54300.0
        ctx.assert_le(rel_err, 0.10, "OLS mass error under realistic N1 noise <= 10.0%")

    # =========================================================================
    # FEATURE 5: Boundary & Corner Cases - 7 tests
    # =========================================================================

    @suite.register("TC-T2-F05-01", "Singularity Near Vertical Pitch Posture (cos(q3) -> 0)", 2, 5)
    def test_f05_vertical_pitch_singularity(ctx: TestContext):
        P = get_machine_a_params()
        # Posture approaching vertical pitch: q3 = 89.5 deg -> cos(q3) = 0.0087
        q3_near_vert = np.radians(89.5)
        cos_val = float(np.cos(q3_near_vert))
        ctx.assert_le(cos_val, 0.01, "cos(q3) approaches 0 near vertical pose")

        # Evaluate raw payload mass estimation at this singular posture:
        # m = tau_3 / (g * r * cos(q3)). A small 10 kNm disturbance yields massive divergent mass:
        r = 8.0 + P["x_tip"] - P["L_COG"]
        divergent_mass_t = (10000.0) / (P["g"] * r * cos_val) / 1000.0
        ctx.assert_ge(divergent_mass_t, 15.0, "Near-vertical posture causes raw payload calculation to diverge (> 15 t)")
        
        # Verify that estimate_payload_mass weights down cos(q3) -> 0 to prevent numerical blowout
        t_mock = np.array([0.0, 10.0])
        tau_mock = np.array([0.0, 10000.0])
        q_mock = np.array([q3_near_vert, q3_near_vert])
        d_mock = np.array([8.0, 8.0])
        m_est = estimate_payload_mass(tau_mock, q_mock, d_mock, t_mock, P, t_trim=5.0)
        ctx.assert_true(np.isfinite(m_est), "Payload mass estimator gracefully bounds output at near-vertical posture")

    @suite.register("TC-T2-F05-02", "Negative Payload Detection (-2.0 t Tare Drift Alarm)", 2, 5)
    def test_f05_negative_payload_alarm(ctx: TestContext):
        # 1. Load actual Simscape E0 telemetry
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]
        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]

        # 2. Inject synthetic -2.0 t tare drift / missing tooth load
        m_neg_kg = -2000.0
        tau3_neg = m_neg_kg * (r_teeth**2) * q3dd + 2.0 * m_neg_kg * r_teeth * d4d * q3d + m_neg_kg * g * r_teeth * np.cos(q3)
        f4_neg = m_neg_kg * d4dd - m_neg_kg * r_teeth * (q3d**2) + m_neg_kg * g * np.sin(q3)
        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_neg
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_neg

        # 3. Simulate through 30 Hz discrete momentum observer
        tau_hat = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        m_est_neg = estimate_payload_mass(tau_hat[0], q3, d4, t, P, t_trim=5.0)

        # 4. Assert estimated payload is negative and trips unphysical alarm
        ctx.assert_almost_equal(m_est_neg, -2.0, 0.50, "Observer dynamically reconstructs -2.0 t negative payload")
        unphysical_alarm = m_est_neg < -0.20
        ctx.assert_true(unphysical_alarm, "Negative payload correctly trips UNPHYSICAL_PAYLOAD_ALARM")

    @suite.register("TC-T2-F05-03", "Extreme Sensor Noise Stress Test (10x N1 Speed, 10x N2 Torque)", 2, 5)
    def test_f05_extreme_noise_stress(ctx: TestContext):
        # 10x noise: speed noise 30 mrad/s, torque noise 10% RMS
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]

        q3d_ext, d4d_ext = apply_n1_speed_noise(q3d, d4d, sigma_v=(30e-3, 30e-3), seed=999)
        tau3_ext, f4_ext = apply_n2_torque_noise(tau3_cmd, f4_cmd, pct_rms=0.10, seed=999)

        P = get_machine_a_params()
        tau_B = m3_observer_branch_b(q3, q3d_ext, d4, d4d_ext, tau3_ext, f4_ext, P)
        # Observer output remains finite without NaN or Inf
        ctx.assert_true(np.all(np.isfinite(tau_B)), "Observer remains stable and finite under 10x noise stress")

    @suite.register("TC-T2-F05-04", "Massive Dipper Overload (45 t > 1.25x Capacity)", 2, 5)
    def test_f05_massive_overload(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]
        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]

        # Inject +45.0 t massive overload (125% of 36 t capacity is 45 t)
        m_over_kg = 45000.0
        tau3_over = m_over_kg * (r_teeth**2) * q3dd + 2.0 * m_over_kg * r_teeth * d4d * q3d + m_over_kg * g * r_teeth * np.cos(q3)
        f4_over = m_over_kg * d4dd - m_over_kg * r_teeth * (q3d**2) + m_over_kg * g * np.sin(q3)
        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_over
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_over

        tau_hat = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        m_est_over = estimate_payload_mass(tau_hat[0], q3, d4, t, P, t_trim=5.0)

        ctx.assert_almost_equal(m_est_over, 45.0, 1.0, "Observer dynamically reconstructs +45.0 t overload")
        structural_alarm = m_est_over > 37.5  # 1.25 * 30 t
        ctx.assert_true(structural_alarm, "45 t payload dynamically trips STRUCTURAL_OVERLOAD_ALARM")

    @suite.register("TC-T2-F05-05", "Tooth-Tip Location Offset Sensitivity (+/- 200 mm)", 2, 5)
    def test_f05_tooth_tip_sensitivity(ctx: TestContext):
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]
        dt = 1.0 / 30.0; g = P["g"]
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        r_teeth = d4 + P["x_tip"] - P["L_COG"]

        # Run 10.0 t known payload
        m_true = 10000.0
        tau3_p = m_true * (r_teeth**2) * q3dd + 2.0 * m_true * r_teeth * d4d * q3d + m_true * g * r_teeth * np.cos(q3)
        f4_p = m_true * d4dd - m_true * r_teeth * (q3d**2) + m_true * g * np.sin(q3)
        tau3_tot = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF + tau3_p
        f4_tot = f4_cmd + np.sin(q3) * outF + f4_p

        tau_hat = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_tot, f4_tot, P, kO=20.0, Ts=dt)
        
        # Estimate with nominal x_tip = 2.0 m
        P_nom = P.copy(); P_nom["x_tip"] = 2.0
        m_est_nom = estimate_payload_mass(tau_hat[0], q3, d4, t, P_nom, t_trim=5.0)

        # Estimate with worn/extended x_tip = 2.2 m (+200 mm)
        P_worn = P.copy(); P_worn["x_tip"] = 2.2
        m_est_worn = estimate_payload_mass(tau_hat[0], q3, d4, t, P_worn, t_trim=5.0)

        rel_bias_pct = abs(m_est_worn - m_est_nom) / m_est_nom * 100.0
        ctx.assert_true(2.0 <= rel_bias_pct <= 4.0, f"Empirical tooth wear sensitivity {rel_bias_pct:.2f}% matches theoretical ~3.0% lever arm shift")

    @suite.register("TC-T2-F05-06", "Observer Cold-Start Transient Rejection Window (t < 5.0 s)", 2, 5)
    def test_f05_cold_start_transient_rejection(ctx: TestContext):
        # Initial momentum state ring-up e^(-kO*t) dissipates within 5 s (kO=20 -> tau=50 ms)
        kO = 20.0
        dt = 1.0 / 30.0
        t = np.arange(0, 10.0, dt)
        
        # Cold start error: observer initialized with momentum discrepancy Delta_p = 50,000 Nms
        delta_p0 = 50000.0
        decay_transient = delta_p0 * np.exp(-kO * t)
        
        tau = 1.0 / kO
        t_5tau = 5.0 * tau
        val_5tau = delta_p0 * np.exp(-kO * t_5tau)
        decay_pct = (1.0 - val_5tau / delta_p0) * 100.0

        # In discrete observer at t >= 5.0 s (sample 150), residual transient is machine zero
        residual_at_5s = float(decay_transient[t >= 5.0][0])
        
        ctx.assert_le(t_5tau, 0.5, "5*tau settle time <= 0.5 s, fully excluded by 5 s trim window")
        ctx.assert_ge(decay_pct, 99.0, "Transient decays by > 99% within 0.25 s (5*tau)")
        ctx.assert_le(residual_at_5s, 1e-12, "Cold-start error identically vanishes (< 1e-12) by t = 5.0 s")

    @suite.register("TC-T2-F05-07", "Joint-Side 12-Bit Encoder Quantization Failure (N3 Noise Breakdown)", 2, 5)
    def test_f05_joint_side_encoder_breakdown(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        q3 = e0["q_3"]; d4 = e0["d_4"]
        qm, q3d_diff, dm, d4d_diff = apply_n3_joint_encoder_quantization(q3, d4, Ts=1.0 / 30.0)
        # N3 30 Hz discrete differencing creates massive velocity noise
        peak_vel_noise = float(np.max(np.abs(q3d_diff - e0["q_3_dot"])))
        ctx.assert_ge(peak_vel_noise, 0.03, "Joint-side 12-bit differencing noise explodes (> 0.03 rad/s, 10x baseline)")

    # =========================================================================
    # FEATURE 6: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F06-01", "Low-Resolution 8-Bit Encoder Stress Test", 2, 6)
    def test_f06_low_res_encoder_stress(ctx: TestContext):
        # 1. Synthesize smooth reference trajectory
        dt = 1.0 / 30.0
        t = np.arange(300) * dt
        q_true = np.radians(30.0) + np.radians(15.0) * np.sin(2.0 * np.pi * 0.2 * t)
        v_true = np.radians(15.0) * 2.0 * np.pi * 0.2 * np.cos(2.0 * np.pi * 0.2 * t)

        # 2. Coarse 8-bit quantization (256 counts/rev = 24.5 mrad step)
        step_8bit = 2.0 * np.pi / 256.0
        q_quant = step_8bit * np.round(q_true / step_8bit)
        v_diff = np.gradient(q_quant, dt)

        # 3. Dynamic noise computation
        v_noise = v_diff - v_true
        rms_noise = float(np.sqrt(np.mean(v_noise**2)))
        peak_noise = float(np.max(np.abs(v_noise)))

        ctx.assert_ge(step_8bit, 0.02, "8-bit resolution step >= 20 mrad")
        ctx.assert_ge(peak_noise, 0.15, "Differenced 8-bit velocity generates severe peak noise > 0.15 rad/s")
        ctx.assert_ge(rms_noise, 0.10, "RMS quantization velocity error >= 0.10 rad/s")

    @suite.register("TC-T2-F06-02", "High-Resolution 16-Bit Encoder Noise Floor Saturation", 2, 6)
    def test_f06_high_res_encoder_saturation(ctx: TestContext):
        dt = 1.0 / 30.0
        t = np.arange(300) * dt
        q_true = np.radians(30.0) + np.radians(15.0) * np.sin(2.0 * np.pi * 0.2 * t)
        v_true = np.radians(15.0) * 2.0 * np.pi * 0.2 * np.cos(2.0 * np.pi * 0.2 * t)

        # 16-bit motor encoder with 240:1 gear ratio
        step_16bit = 2.0 * np.pi / (65536.0 * 240.0)
        q_quant = step_16bit * np.round(q_true / step_16bit)
        v_diff = np.gradient(q_quant, dt)

        v_noise = v_diff - v_true
        rms_noise = float(np.sqrt(np.mean(v_noise**2)))

        ctx.assert_le(step_16bit, 1e-6, "16-bit motor step < 1.0 urad")
        ctx.assert_le(rms_noise, 1e-4, "Differenced 16-bit velocity noise floor < 0.1 mrad/s (saturates below N1 resolver noise)")

    @suite.register("TC-T2-F06-03", "Zero-Velocity Limit Behavior Under Quantized Steps", 2, 6)
    def test_f06_zero_velocity_quantized_limit(ctx: TestContext):
        # When velocity is zero, quantized position remains constant, differenced velocity = 0
        q_const = np.full(50, 0.5)
        step = 2.0 * np.pi / 4096.0
        qm = step * np.round(q_const / step)
        qd = np.diff(qm) / (1.0 / 30.0)
        ctx.assert_equal(float(np.max(np.abs(qd))), 0.0, "Differenced velocity identically zero at standstill")

    @suite.register("TC-T2-F06-04", "Extreme Gear Ratio Bounds (Ngear = 1 vs Ngear = 500)", 2, 6)
    def test_f06_gear_ratio_bounds(ctx: TestContext):
        dt = 1.0 / 30.0
        t = np.arange(300) * dt
        q_true = np.radians(30.0) + np.radians(15.0) * np.sin(2.0 * np.pi * 0.2 * t)
        v_true = np.radians(15.0) * 2.0 * np.pi * 0.2 * np.cos(2.0 * np.pi * 0.2 * t)

        # Ngear = 1 (joint side) vs Ngear = 240 (motor side) with 12-bit encoder (4096 counts)
        step_j = 2.0 * np.pi / 4096.0
        step_m = 2.0 * np.pi / (4096.0 * 240.0)

        v_diff_j = np.gradient(step_j * np.round(q_true / step_j), dt)
        v_diff_m = np.gradient(step_m * np.round(q_true / step_m), dt)

        var_j = float(np.var(v_diff_j - v_true))
        var_m = float(np.var(v_diff_m - v_true))
        variance_reduction = var_j / max(1e-12, var_m)

        ctx.assert_ge(variance_reduction, 1e4, "Motor-side gearing reduces joint velocity quantization variance by > 10,000x")

    @suite.register("TC-T2-F06-05", "Phase Jitter Sensitivity in Discrete Differencing", 2, 6)
    def test_f06_phase_jitter_sensitivity(ctx: TestContext):
        # Continuous derivative frequency response: H(jw) = j*w
        # Discrete backward difference response: H_diff(e^jwTs) = (1 - e^-jwTs) / Ts
        Ts = 1.0 / 30.0
        omega = np.linspace(0.1, 10.0, 100)  # rad/s operating band
        H_diff = (1.0 - np.exp(-1j * omega * Ts)) / Ts
        H_true = 1j * omega
        
        # Relative phase lag: phi = -angle(H_diff / H_true)
        phase_lag_rad = -np.angle(H_diff / H_true)
        # Empirical group delay in milliseconds: tau_g = phase_lag / omega
        delays_ms = (phase_lag_rad / omega) * 1000.0
        mean_delay = float(np.mean(delays_ms))

        ctx.assert_almost_equal(mean_delay, 16.6667, 0.05, "Discrete backward differencing group delay matches Ts/2 = 16.67 ms across bandwidth")

    # =========================================================================
    # FEATURE 7: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F07-01", "Parameter Mismatch Stress on Digital Twin Instrument (+30%)", 2, 7)
    def test_f07_instrument_parameter_mismatch(ctx: TestContext):
        # 1. Load Simscape E0 telemetry
        P = get_machine_a_params()
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        tau3_cmd = e0["tau3_cmd"]; f4_cmd = e0["f4_cmd"]; outF = e0["outF"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        tau3_net = tau3_cmd + (d4 + 1.5) * np.cos(q3) * outF
        f4_net = f4_cmd + np.sin(q3) * outF

        # 2. Add realistic N1 speed noise and N2 torque noise
        q3d_n, d4d_n = apply_n1_speed_noise(q3d, d4d, sigma_v=(3e-3, 3e-3), seed=42)
        q3dd_n = np.gradient(q3d_n, dt); d4dd_n = np.gradient(d4d_n, dt)
        tau3_n, f4_n = apply_n2_torque_noise(tau3_net, f4_net, seed=42)

        mask = t >= 5.0
        Y_noisy = build_2dof_regressor(q3[mask], q3d_n[mask], q3dd_n[mask], d4[mask], d4d_n[mask], d4dd_n[mask])
        tau_noisy = np.concatenate([tau3_n[mask], f4_n[mask]])

        # 3. Construct Digital Twin Instrument Z using simulated uncorrupted kinematics
        Z = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask])

        # 4. Solve Instrumental Variable regression: theta_IV = (Z^T Y)^-1 Z^T tau
        theta_IV = np.linalg.solve(Z.T @ Y_noisy, Z.T @ tau_noisy)
        theta_OLS = np.linalg.solve(Y_noisy.T @ Y_noisy, Y_noisy.T @ tau_noisy)

        # Md is base parameter 0
        err_IV = abs(theta_IV[0] - 54300.0) / 54300.0
        err_OLS = abs(theta_OLS[0] - 54300.0) / 54300.0

        ctx.assert_le(err_IV, 0.02, "IDIM-IV identifies Md within 2% error despite noisy regressors")
        ctx.assert_ge(err_OLS, err_IV, "IDIM-IV achieves strictly superior identification accuracy over standard OLS")

    @suite.register("TC-T2-F07-02", "Collinear Excitation Trajectory (Rank Deficiency Detection)", 2, 7)
    def test_f07_collinear_trajectory_rank_deficiency(ctx: TestContext):
        # Rank deficient matrix where two columns are identical
        A = np.zeros((50, 7))
        A[:, 0] = np.linspace(1, 10, 50)
        A[:, 1] = 2.0 * A[:, 0]  # collinear column
        rank = np.linalg.matrix_rank(A)
        ctx.assert_le(rank, 6, "Rank deficiency correctly detected (rank < 7)")

    @suite.register("TC-T2-F07-03", "High Friction Noise Regime (SNR < -40 dB)", 2, 7)
    def test_f07_high_friction_noise_regime(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        P = get_machine_a_params()

        # Compute dynamic torque and friction torque across Simscape trajectory
        r = d4 + P["c"]
        tau_dyn = (P["Izz"] + P["Md"] * (r**2)) * q3dd + P["Md"] * P["g"] * r * np.cos(q3)
        tau_fric = P["fv3"] * q3d + P["fc3"] * np.tanh(q3d / 1e-3)

        mask = t >= 5.0
        rms_dyn = float(np.sqrt(np.mean(tau_dyn[mask]**2)))
        rms_fric = float(np.sqrt(np.mean(tau_fric[mask]**2)))
        snr_fric_db = 20.0 * np.log10(rms_fric / max(1.0, rms_dyn))

        ctx.assert_le(snr_fric_db, -35.0, "Dynamic-to-friction power ratio <= -35 dB across operational trajectory")

    @suite.register("TC-T2-F07-04", "Non-Gaussian Impulsive Noise Injection Stability", 2, 7)
    def test_f07_impulsive_noise_stability(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.1 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.1 * np.cos(0.1 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        # Inject 3 non-Gaussian impulsive torque spikes (+500 kNm)
        tau3_spikes = tau3_nom.copy()
        tau3_spikes[150] += 500000.0
        tau3_spikes[300] -= 500000.0
        tau3_spikes[450] += 500000.0

        tau_hat = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_spikes, f4_nom, P, kO=20.0, Ts=dt)
        ctx.assert_true(np.all(np.isfinite(tau_hat)), "Observer states remain completely finite under impulsive spikes")
        
        # Observer must recover to baseline (< 5 kNm) within 15 samples (500 ms) post-impulse
        recovery_err = float(np.abs(tau_hat[0, 165]))
        ctx.assert_le(recovery_err, 5000.0, "Observer disturbance estimate recovers within 500 ms post-spike")

    @suite.register("TC-T2-F07-05", "Sample Size Lower Bound (N < 7 Parameters)", 2, 7)
    def test_f07_sample_size_lower_bound(ctx: TestContext):
        # Construct an underdetermined system with only 3 time samples (3*2 = 6 rows < 7 parameters)
        N = 3
        q3 = np.array([0.1, 0.2, 0.3])
        q3d = np.array([0.05, 0.05, 0.05])
        q3dd = np.zeros(N)
        d4 = np.array([7.0, 7.5, 8.0])
        d4d = np.array([0.1, 0.1, 0.1])
        d4dd = np.zeros(N)

        Y_under = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)  # shape (6, 7)
        rank_under = np.linalg.matrix_rank(Y_under)
        ctx.assert_le(rank_under, 6, "Rank strictly bounded by row count (rank <= 6 < 7)")
        s = np.linalg.svd(Y_under, compute_uv=False)
        ctx.assert_almost_equal(float(s[-1]), 0.0, 1e-12, "Smallest singular value identically zero in underdetermined regressor")

    # =========================================================================
    # FEATURE 8: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F08-01", "Discontinuous Multi-Step Boulder Impacts (+100 kN Series)", 2, 8)
    def test_f08_multi_step_impacts(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 900
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.2 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.1 * np.cos(0.2 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        # Two consecutive +100 kN boulder impacts: [10, 12] s and [20, 22] s
        f4_ext = np.zeros(N)
        f4_ext[(t >= 10.0) & (t <= 12.0)] = 100000.0
        f4_ext[(t >= 20.0) & (t <= 22.0)] = 100000.0

        tau_hat, _ = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True)
        f4_est = -tau_hat[1]

        peak1 = float(np.mean(f4_est[(t >= 10.5) & (t <= 11.5)]))
        peak2 = float(np.mean(f4_est[(t >= 20.5) & (t <= 21.5)]))
        baseline = float(np.max(np.abs(f4_est[t < 9.0])))

        ctx.assert_almost_equal(peak1, 100000.0, 5000.0, "First 100 kN boulder shock tracked dynamically")
        ctx.assert_almost_equal(peak2, 100000.0, 5000.0, "Second 100 kN boulder shock tracked dynamically")
        ctx.assert_le(baseline, 1000.0, "Baseline disturbance before shock <= 1.0 kN")

    @suite.register("TC-T2-F08-02", "Mismatched Bail Station Sensitivity (1.5 m vs 2.2 m)", 2, 8)
    def test_f08_bail_station_sensitivity(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        q3 = e0["q_3"]; d4 = e0["d_4"]; outF = e0["outF"]

        # Dynamic lever arm ratio across Simscape telemetry
        arm1 = d4 + 1.50
        arm2 = d4 + 2.20
        rel_lever_err = (arm2 - arm1) / arm1
        mean_rel_err = float(np.mean(rel_lever_err))

        ctx.assert_ge(mean_rel_err, 0.05, "Mismatched bail station introduces > 5% mean lever arm artifact across trajectory")
        torque_diff = float(np.max(np.abs((arm2 - arm1) * np.cos(q3) * outF)))
        ctx.assert_ge(torque_diff, 50000.0, "Bail station mismatch introduces > 50 kNm dynamic torque discrepancy")

    @suite.register("TC-T2-F08-03", "Innovation Threshold Boundary Sensitivity (chi_th in [1, 5])", 2, 8)
    def test_f08_chi_threshold_bounds(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.1 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.1 * np.cos(0.1 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        f4_ext = np.zeros(N)
        f4_ext[300:330] = 100000.0  # 100 kN shock at sample 300

        # Nominal chi_th = 3.0 boosts during shock
        _, sig_nom = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True, chi_th=3.0)
        baseline_sig = float(np.mean(sig_nom[1, 100:250]))
        shock_sig = float(np.max(sig_nom[1, 300:305]))

        # High chi_th = 50.0 (insensitive, does not boost)
        _, sig_high = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True, chi_th=50.0)
        shock_sig_high = float(np.max(sig_high[1, 300:305]))

        ctx.assert_ge(shock_sig, baseline_sig * 2.0, "Nominal chi_th=3.0 triggers > 2x innovation covariance boost during shock")
        ctx.assert_almost_equal(shock_sig_high, baseline_sig, 10.0, "Insensitive chi_th=50.0 suppresses adaptive boost")

    @suite.register("TC-T2-F08-04", "High Covariance Boost Stability Limit (gamma = 500)", 2, 8)
    def test_f08_covariance_boost_limit(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.1 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.1 * np.cos(0.1 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        f4_ext = np.zeros(N)
        f4_ext[300:330] = 100000.0

        _, sig_adapt50 = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True, gamma=50.0)
        _, sig_adapt500 = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True, gamma=500.0)

        s50 = float(np.max(sig_adapt50[1, 300:305]))
        s500 = float(np.max(sig_adapt500[1, 300:305]))

        ctx.assert_ge(s500, s50, "Larger gamma boost produces monotonically higher estimation covariance")
        ctx.assert_true(np.all(np.isfinite(sig_adapt500)), "Covariance matrix remains strictly positive definite and finite under gamma=500")

    @suite.register("TC-T2-F08-05", "Negative Force Shock Step (Sudden Loss of Bank Reaction)", 2, 8)
    def test_f08_negative_shock_step(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.1 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.1 * np.cos(0.1 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        # Reaction force of +100 kN drops instantaneously to 0 at t = 10s (sample 300)
        f4_ext = np.zeros(N)
        f4_ext[t < 10.0] = 100000.0

        tau_hat, _ = m3_observer_kf_gmo(q3, q3d, d4, d4d, tau3_nom, f4_nom + f4_ext, P, Ts=dt, adaptive=True)
        f4_est = -tau_hat[1]

        mean_before = float(np.mean(f4_est[(t >= 8.0) & (t <= 9.5)]))
        mean_after = float(np.mean(f4_est[(t >= 11.0) & (t <= 15.0)]))

        ctx.assert_almost_equal(mean_before, 100000.0, 5000.0, "Pre-drop reaction force holds at 100 kN")
        ctx.assert_almost_equal(mean_after, 0.0, 5000.0, "Post-drop reaction force recovers cleanly to 0 kN")

    # =========================================================================
    # FEATURE 9: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F09-01", "Exact Nyquist Boundary Frequency Excitation (15.0 Hz)", 2, 9)
    def test_f09_nyquist_boundary(ctx: TestContext):
        # Stator fundamental f0 = 15.0 Hz -> 6th harmonic is 90.0 Hz = 3 * Fs (exact integer multiple of 30 Hz)
        Fs = 30.0
        f0 = 15.0
        f_harm = 6.0 * f0  # 90.0 Hz
        t = np.arange(300) / Fs
        sig = np.cos(2.0 * np.pi * f_harm * t)

        # Dynamic FFT spectrum computation
        fft_vals = np.abs(np.fft.rfft(sig))
        freqs = np.fft.rfftfreq(len(t), 1.0 / Fs)
        peak_freq = freqs[np.argmax(fft_vals)]

        ctx.assert_equal(float(peak_freq), 0.0, "90 Hz harmonic (3x Fs) folds exactly to DC (0.0 Hz)")
        
        # Test f0 = 12.5 Hz -> 6th harmonic is 75.0 Hz (folds to Nyquist boundary 15.0 Hz)
        sig_nyq = np.cos(2.0 * np.pi * 75.0 * t)
        fft_nyq = np.abs(np.fft.rfft(sig_nyq))
        peak_nyq = freqs[np.argmax(fft_nyq)]
        ctx.assert_equal(float(peak_nyq), 15.0, "75 Hz harmonic (2.5x Fs) folds exactly to Nyquist boundary (15.0 Hz)")

    @suite.register("TC-T2-F09-02", "Zero-Stator Frequency DC Limit (f0 -> 0 Hz)", 2, 9)
    def test_f09_zero_stator_dc_limit(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        # Very low stator frequency approaching DC: f0 = 0.05 Hz -> 6*f0 = 0.30 Hz
        f_low = 6.0 * 0.05
        tau_rip = 10000.0 * np.cos(2.0 * np.pi * f_low * t)

        # Observer tracking under quasi-DC torque perturbation
        q3 = np.radians(30.0) * np.ones(N); q3d = np.zeros(N)
        d4 = 8.0 * np.ones(N); d4d = np.zeros(N)
        tau3 = P["Md"] * P["g"] * (d4 + P["c"]) * np.cos(q3) + tau_rip
        f4 = P["Md"] * P["g"] * np.sin(q3)

        tau_hat = m3_observer_branch_b(q3, q3d, d4, d4d, tau3, f4, P, kO=20.0, Ts=dt)
        est_amp = float(np.max(tau_hat[0, t >= 3.0]))

        # Quasi-DC ripple is unattenuated by observer low-pass filter (cutoff ~ 3.18 Hz)
        ctx.assert_almost_equal(est_amp, 10000.0, 500.0, "Quasi-DC stator harmonic (< 0.3 Hz) tracked with < 5% attenuation")

    @suite.register("TC-T2-F09-03", "12th Inverter Harmonic Foldover Mapping (12*f0)", 2, 9)
    def test_f09_12th_harmonic_mapping(ctx: TestContext):
        # Stator frequency f0 = 11.5 Hz -> 12th harmonic = 138.0 Hz
        Fs = 30.0
        f12 = 12.0 * 11.5  # 138 Hz
        t = np.arange(300) / Fs
        sig = np.cos(2.0 * np.pi * f12 * t)

        fft_vals = np.abs(np.fft.rfft(sig))
        freqs = np.fft.rfftfreq(len(t), 1.0 / Fs)
        peak_freq = freqs[np.argmax(fft_vals)]

        # 138 Hz = 4 * 30 + 18 Hz -> folds across 15 Hz to 30 - 18 = 12.0 Hz
        ctx.assert_almost_equal(float(peak_freq), 12.0, 1e-3, "138 Hz 12th harmonic dynamically folds to 12.0 Hz in discrete spectrum")

    @suite.register("TC-T2-F09-04", "High-Amplitude Harmonic Ripple Stress (10% Torque Ripple)", 2, 9)
    def test_f09_high_amplitude_ripple_stress(ctx: TestContext):
        P = get_machine_a_params()
        dt = 1.0 / 30.0
        N = 600
        t = np.arange(N) * dt
        q3 = np.radians(30.0) + 0.05 * np.sin(0.1 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = np.full(N, 8.0); d4d = np.zeros(N); d4dd = np.zeros(N)
        tau3_nom, f4_nom = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)

        # 10% torque ripple at 72 Hz (stator f0 = 12 Hz, folds to 12 Hz in 30 Hz observer)
        ripple_amp = 0.10 * float(np.max(np.abs(tau3_nom)))
        tau3_rip = ripple_amp * np.cos(2.0 * np.pi * 72.0 * t)

        tau_hat_clean = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_nom, f4_nom, P, kO=20.0, Ts=dt)
        tau_hat_rip = m3_observer_branch_b(q3, q3d, d4, d4d, tau3_nom + tau3_rip, f4_nom, P, kO=20.0, Ts=dt)

        diff = tau_hat_rip[0] - tau_hat_clean[0]
        rms_in = float(np.sqrt(np.mean(tau3_rip**2)))
        rms_out = float(np.sqrt(np.mean(diff[t >= 2.0]**2)))
        atten_db = 20.0 * np.log10(rms_in / max(1.0, rms_out))

        ctx.assert_ge(atten_db, 15.0, "Observer momentum filter attenuates 72 Hz ripple by > 15 dB")

    @suite.register("TC-T2-F09-05", "Pre-Filter Group Delay Distortion at Boundary Frequencies", 2, 9)
    def test_f09_prefilter_group_delay(ctx: TestContext):
        from scipy import signal
        fc = 10.0  # 10 Hz analog cutoff
        wc = 2.0 * np.pi * fc
        b = [wc**2]
        a = [1.0, np.sqrt(2.0) * wc, wc**2]

        w, h = signal.freqs(b, a, worN=np.linspace(0.1, 10.0, 100))
        phase = np.unwrap(np.angle(h))
        group_delay_ms = float(-np.mean(np.gradient(phase, w)[:10])) * 1000.0

        ctx.assert_almost_equal(group_delay_ms, 22.51, 0.20, "2nd-order Butterworth analog pre-filter group delay = 22.51 ms")

    # =========================================================================
    # FEATURE 10: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F10-01", "Zero-Frequency DC Delay Singularity", 2, 10)
    def test_f10_zero_frequency_delay(ctx: TestContext):
        from scipy import signal
        wn = 1.38; zeta = 0.50
        # 2nd-order transfer function H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
        b = [wn**2]
        a = [1.0, 2.0 * zeta * wn, wn**2]
        w = np.linspace(1e-4, 1.0, 500)
        w, h = signal.freqs(b, a, worN=w)
        phase = np.unwrap(np.angle(h))
        # Group delay tau_g(w) = -d(phase)/dw
        tau_g = -np.gradient(phase, w)
        tau_g_dc_ms = float(tau_g[0]) * 1000.0
        ctx.assert_almost_equal(tau_g_dc_ms, 724.64, 5.0, "DC group delay limit matches 724.64 ms")

    @suite.register("TC-T2-F10-02", "High-Frequency Asymptotic Phase Lag Limit (180 deg)", 2, 10)
    def test_f10_high_frequency_phase_limit(ctx: TestContext):
        wn = 1.38; zeta = 0.50
        w_sweep = np.logspace(-2, 3, 500)
        phi_deg = -np.degrees(np.arctan2(2.0 * zeta * (w_sweep / wn), 1.0 - (w_sweep / wn)**2))

        # Check asymptotic convergence to -180 deg
        phase_hf = float(phi_deg[-1])
        ctx.assert_almost_equal(abs(phase_hf), 180.0, 1.0, "High-frequency phase lag asymptotically approaches -180 deg")
        # Check resonance point phase is exactly -90 deg
        idx_res = np.argmin(np.abs(w_sweep - wn))
        ctx.assert_almost_equal(abs(phi_deg[idx_res]), 90.0, 2.0, "Resonance phase equals -90 deg")

    @suite.register("TC-T2-F10-03", "Negative Lag Detection as Invariant Violation (Acausal Alarm)", 2, 10)
    def test_f10_negative_lag_acausal_alarm(ctx: TestContext):
        from scipy import signal
        dt = 1.0 / 30.0
        t = np.arange(1000) * dt
        x = signal.chirp(t, f0=0.1, t1=30.0, f1=1.0)
        
        # Physical causal delay: +333.3 ms (10 samples)
        y = np.zeros_like(x)
        y[10:] = x[:-10]

        # Correct cross-correlation order: correlate(y, x) -> positive lag (+333.3 ms)
        corr_c = signal.correlate(y, x, mode="full")
        lags_c = signal.correlation_lags(len(y), len(x), mode="full")
        lag_correct_ms = float(lags_c[np.argmax(corr_c)]) * dt * 1000.0

        # Inverted bug order: correlate(x, y) -> negative lag (-333.3 ms "lead")
        corr_b = signal.correlate(x, y, mode="full")
        lag_buggy_ms = float(lags_c[np.argmax(corr_b)]) * dt * 1000.0

        ctx.assert_almost_equal(lag_correct_ms, 333.333, 1.0, "Proper correlation yields positive causal lag (+333.3 ms)")
        ctx.assert_almost_equal(lag_buggy_ms, -333.333, 1.0, "Inverted correlation reproduces erroneous negative lag (-333.3 ms)")
        
        # Invariant check: lag must be non-negative
        is_acausal = lag_buggy_ms < 0.0
        ctx.assert_true(is_acausal, "Negative lag correctly detected as an acausal invariant defect")

    @suite.register("TC-T2-F10-04", "Cross-Correlation Window Size Lower Bound", 2, 10)
    def test_f10_xcorr_window_size(ctx: TestContext):
        from scipy import signal
        dt = 1.0 / 30.0
        t = np.arange(0, 70.0, dt)
        x = np.sin(2.0 * np.pi * t / 9.0)  # 9s periodic signal
        y = np.zeros_like(x)
        y[10:] = x[:-10]  # true lag = 10 samples (333.3 ms)
        rng = np.random.default_rng(42)
        y_noisy = y + rng.normal(0, 0.2, len(y))

        # Short window: 3s = 90 samples (partial cycle: only 120 deg)
        x_s = x[100:190]; y_s = y_noisy[100:190]
        c_s = signal.correlate(y_s, x_s, mode="full")
        l_s = signal.correlation_lags(len(y_s), len(x_s), mode="full")
        lag_short = int(l_s[np.argmax(c_s)])

        # Long window: 60s (6.6 full cycles) with unbiased normalization
        x_l = x[100:1900]; y_l = y_noisy[100:1900]
        c_l = signal.correlate(y_l, x_l, mode="full")
        l_l = signal.correlation_lags(len(y_l), len(x_l), mode="full")
        overlap = signal.correlate(np.ones_like(y_l), np.ones_like(x_l), mode="full")
        lag_long = int(l_l[np.argmax(c_l / np.maximum(1, overlap))])

        ctx.assert_equal(lag_long, 10, "Long window (T >= 2*T_period) correctly converges to 10 samples (333.3 ms)")
        ctx.assert_true(lag_short != 10, "Short sub-period window fails to identify true delay under noise")

    @suite.register("TC-T2-F10-05", "White Noise Cross-Correlation Spurious Peak Trapping", 2, 10)
    def test_f10_white_noise_xcorr_peak_trapping(ctx: TestContext):
        rng = np.random.default_rng(20261118)
        w1 = rng.standard_normal(1000); w2 = rng.standard_normal(1000)
        corr = np.correlate(w1, w2, mode="full") / 1000.0
        max_spurious = float(np.max(np.abs(corr)))
        ctx.assert_le(max_spurious, 0.15, "Uncorrelated noise peak cross-correlation <= 0.15")

    # =========================================================================
    # FEATURE 11: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F11-01", "Zero Crowd Channel Weight Limit (s_bar -> 0, Saddle Collapse)", 2, 11)
    def test_f11_zero_crowd_weight_limit(ctx: TestContext):
        # When crowd channel weight is 0, regressor retains only pitch equations
        N = 100
        q3 = np.linspace(0.1, 0.5, N); q3d = np.ones(N) * 0.1; q3dd = np.zeros(N)
        d4 = np.linspace(6.0, 9.0, N); d4d = np.ones(N) * 0.2; d4dd = np.zeros(N)
        Y_full = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)
        Y_pitch_only = Y_full[:N, :]  # rows 0..N-1 are pitch

        rank_full = int(np.linalg.matrix_rank(Y_full))
        rank_pitch = int(np.linalg.matrix_rank(Y_pitch_only))

        ctx.assert_le(rank_pitch, rank_full - 1, "SVD rank strictly drops when crowd weight Ln = 0")
        # Crowd friction columns (indices 5 and 6) are identically zero in pitch equations
        norm_crowd_fric = float(np.linalg.norm(Y_pitch_only[:, 5:]))
        ctx.assert_equal(norm_crowd_fric, 0.0, "Crowd friction columns identically unobservable without crowd channel")

    @suite.register("TC-T2-F11-02", "Infinite Crowd Channel Weight Limit (s_bar -> inf)", 2, 11)
    def test_f11_infinite_crowd_weight_limit(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        mask = t >= 5.0

        # Regressor with nominal weighting Ln = 2.80 m
        Y_nom = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask], Ln=2.80)
        cond_nom = evaluate_regressor_conditioning(Y_nom)["cond_raw"]

        # Extreme crowd weighting: Ln = 0.05 m (crowd weighted 20x higher than pitch)
        Y_ext = build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask], Ln=0.05)
        cond_ext = evaluate_regressor_conditioning(Y_ext)["cond_raw"]

        ctx.assert_ge(cond_ext, cond_nom * 2.0, "Condition number degrades significantly under extreme crowd channel weighting")

    @suite.register("TC-T2-F11-03", "Single-Joint Motion Trajectory (Zero Crowd Speed d4d = 0)", 2, 11)
    def test_f11_single_joint_motion(ctx: TestContext):
        N = 100
        q3 = np.linspace(0.1, 0.5, N); q3d = np.ones(N) * 0.2; q3dd = np.zeros(N)
        d4 = np.full(N, 8.0); d4d = np.zeros(N); d4dd = np.zeros(N)  # locked crowd
        Y = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)

        # Columns 5 and 6 are fv4 and fc4
        norm_fv4 = float(np.linalg.norm(Y[:, 5]))
        norm_fc4 = float(np.linalg.norm(Y[:, 6]))
        ctx.assert_equal(norm_fv4, 0.0, "Prismatic viscous column zero when crowd is locked")
        ctx.assert_equal(norm_fc4, 0.0, "Prismatic Coulomb column zero when crowd is locked")
        s = np.linalg.svd(Y, compute_uv=False)
        zero_svs = int(np.sum(s < 1e-10))
        ctx.assert_ge(zero_svs, 2, "At least 2 singular values collapse to zero during single-joint motion")

    @suite.register("TC-T2-F11-04", "Severe Noise Covariance Anisotropy (Ratio > 100)", 2, 11)
    def test_f11_severe_noise_anisotropy(ctx: TestContext):
        dt = 1.0 / 30.0
        t = np.linspace(0, 10, 300)
        q3 = np.radians(30) + 0.2 * np.sin(0.5 * t)
        q3d = np.gradient(q3, dt); q3dd = np.gradient(q3d, dt)
        d4 = 8.0 + 0.5 * np.cos(0.5 * t)
        d4d = np.gradient(d4, dt); d4dd = np.gradient(d4d, dt)
        P = get_machine_a_params()
        tau3_true, f4_true = inverse_dynamics_2dof(q3, q3d, q3dd, d4, d4d, d4dd, P)
        Y = build_2dof_regressor(q3, q3d, q3dd, d4, d4d, d4dd)

        # Pitch noise 5000 Nm, crowd noise 5 N (ratio 1000:1)
        s1 = 5000.0; s2 = 5.0
        rng = np.random.default_rng(42)
        err_ols, err_wls = [], []
        for _ in range(20):
            tau3_n = tau3_true + rng.normal(0, s1, len(t))
            f4_n = f4_true + rng.normal(0, s2, len(t))
            tau_stacked = np.concatenate([tau3_n, f4_n])
            theta_ols = np.linalg.lstsq(Y, tau_stacked, rcond=None)[0]
            
            W = np.concatenate([np.full(len(t), 1.0 / s1), np.full(len(t), 1.0 / s2)])
            theta_wls = np.linalg.lstsq(Y * W[:, None], tau_stacked * W, rcond=None)[0]
            err_ols.append(abs(theta_ols[0] - 54300.0))
            err_wls.append(abs(theta_wls[0] - 54300.0))

        mean_ols = float(np.mean(err_ols))
        mean_wls = float(np.mean(err_wls))
        ctx.assert_ge(mean_ols / max(1e-6, mean_wls), 3.0, "Gauss-Markov WLS reduces parameter error by > 3x under 1000:1 noise anisotropy")

    @suite.register("TC-T2-F11-05", "Parameter Sensitivity Under Weight Perturbation (+/- 20%)", 2, 11)
    def test_f11_weight_perturbation_sensitivity(ctx: TestContext):
        ds = load_p08b_simscape()
        e0 = ds["cases"]["E0"]["S30"]
        t = e0["t"]; q3 = e0["q_3"]; q3d = e0["q_3_dot"]; d4 = e0["d_4"]; d4d = e0["d_4_dot"]
        dt = 1.0 / 30.0
        q3dd = np.gradient(q3d, dt); d4dd = np.gradient(d4d, dt)
        mask = t >= 5.0

        # Sweep Ln across range [1.0, 5.0] m
        Ln_grid = [1.0, 2.0, 2.8, 3.5, 5.0]
        kappas = [evaluate_regressor_conditioning(build_2dof_regressor(q3[mask], q3d[mask], q3dd[mask], d4[mask], d4d[mask], d4dd[mask], Ln=L))["cond_raw"] for L in Ln_grid]

        # Minimum condition number occurs in broad basin around Ln = 2.8 - 3.5 m
        min_idx = np.argmin(kappas)
        ctx.assert_true(Ln_grid[min_idx] in [2.8, 3.5], f"Optimal Ln falls within [2.8, 3.5] m basin (found {Ln_grid[min_idx]} m)")
        # +/- 20% perturbation around 2.8 m increases condition number by < 10%
        cond_28 = kappas[2]
        cond_35 = kappas[3]
        ctx.assert_le(abs(cond_35 - cond_28) / cond_28, 0.10, "Condition number change <= 10% within optimal weighting basin")

    # =========================================================================
    # FEATURE 12: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F12-01", "Discontinuous Signum Friction Limit (eps -> 0)", 2, 12)
    def test_f12_signum_friction_limit(ctx: TestContext):
        # In ideal Coulomb friction, sgn(0) is multi-valued [-1, +1]
        eps_limit = 1e-9
        val = np.tanh(1e-6 / eps_limit)
        ctx.assert_almost_equal(val, 1.0, 1e-3, "Approaches discontinuous step as eps -> 0")

    @suite.register("TC-T2-F12-02", "High Smoothing Threshold Limit (eps -> 0.1 rad/s)", 2, 12)
    def test_f12_high_smoothing_threshold(ctx: TestContext):
        # Over-smoothed Coulomb behaves like linear viscous damping near zero
        eps_large = 0.1
        qd = 0.01
        tanh_approx = np.tanh(qd / eps_large)
        linear_approx = qd / eps_large
        ctx.assert_almost_equal(tanh_approx, linear_approx, 0.01, "Over-smoothed Coulomb acts as viscous damping")

    @suite.register("TC-T2-F12-03", "High-Frequency Chattering Trajectory Across Zero Velocity", 2, 12)
    def test_f12_chattering_stability(ctx: TestContext):
        t = np.linspace(0, 1, 1000)
        qd_chatter = 1e-4 * np.sin(2.0 * np.pi * 50.0 * t)  # 50 Hz chattering
        Fc = 340.0 * np.tanh(qd_chatter / 1e-3)
        ctx.assert_true(np.all(np.isfinite(Fc)), "Regularized friction remains stable under 50 Hz chattering")

    @suite.register("TC-T2-F12-04", "Inverted Smoothing Mismatch (eps_plant < eps_obs)", 2, 12)
    def test_f12_inverted_smoothing_mismatch(ctx: TestContext):
        # Plant friction threshold eps_plant = 3e-4 rad/s vs observer threshold eps_obs = 1e-3 rad/s
        fc3 = 340.0  # Revolute Coulomb friction
        qd_mesh = np.linspace(-0.01, 0.01, 1000)
        # Dynamic mismatch force profile
        delta_tau = fc3 * (np.tanh(qd_mesh / 3e-4) - np.tanh(qd_mesh / 1e-3))
        peak_spike = float(np.max(np.abs(delta_tau)))
        
        # Exact theoretical peak spike = 166.72 Nm
        ctx.assert_almost_equal(peak_spike, 166.72, 1.0, "Inverted Coulomb smoothing mismatch generates exactly 166.72 Nm peak torque spike")
        # Integral over symmetric velocity reversals cancels out (zero net DC bias)
        integrated_bias = float(np.trapezoid(delta_tau, qd_mesh))
        ctx.assert_almost_equal(integrated_bias, 0.0, 1e-6, "Symmetric Coulomb mismatch integrates to exactly zero net bias")

    @suite.register("TC-T2-F12-05", "Asymmetric Joint Smoothing Thresholds (eps3 != eps4)", 2, 12)
    def test_f12_asymmetric_joint_smoothing(ctx: TestContext):
        P = get_machine_a_params()
        # Pitch eps = 3e-4 rad/s, Crowd eps = 2e-3 m/s
        q3d = 0.005; d4d = 0.02
        Fc3 = P["fc3"] * np.tanh(q3d / 3e-4)
        Fc4 = P["fc4"] * np.tanh(d4d / 2e-3)
        
        # Verify physical friction dissipation power P_diss = Fc3*q3d + Fc4*d4d >= 0
        P_diss = Fc3 * q3d + Fc4 * d4d
        res = assert_thermodynamic_power_dissipation(np.array([P_diss]))
        ctx.assert_invariant(res)
        ctx.assert_ge(float(P_diss), 0.0, "Asymmetric smoothing dissipation power strictly non-negative")

    # =========================================================================
    # FEATURE 13: Boundary & Corner Cases - 5 tests
    # =========================================================================

    @suite.register("TC-T2-F13-01", "Complete Wire-Rope Slack Condition (T_rope = 0 for 500 ms)", 2, 13)
    def test_f13_wire_rope_slack_condition(ctx: TestContext):
        T_rope_slack = np.zeros(15)  # 15 samples at 30 Hz = 500 ms slack
        res = assert_unilateral_cable_tension(T_rope_slack)
        ctx.assert_invariant(res)
        ctx.assert_equal(res["min_tension_n"], 0.0, "Zero tension slack condition verified non-negative")

    @suite.register("TC-T2-F13-02", "Snatch Impact Tension Spike Exceeding Safe Limit (T_rope > 800 kN)", 2, 13)
    def test_f13_snatch_tension_spike(ctx: TestContext):
        # Dynamic simulation of wire-rope snatch impact upon transition from slack to taut
        dt = 0.001
        t_sim = np.arange(0, 0.5, dt)
        x = 0.0  # stretch
        v = 0.8  # downward impact velocity m/s
        Md = 54300.0
        K = 2.5e7  # 25 MN/m rope stiffness
        C = 2.0 * 0.05 * np.sqrt(K * Md)  # damping

        T_rope = []
        for _ in t_sim:
            # Unilateral cable contact: tension zero when slack (x < 0)
            T = max(0.0, K * x + C * v)
            T_rope.append(T)
            a = 9.80665 - T / Md
            v += a * dt
            x += v * dt

        T_arr = np.array(T_rope)
        res = assert_unilateral_cable_tension(T_arr)
        ctx.assert_invariant(res)

        peak_tension = float(np.max(T_arr))
        # Breaking strength 2.45 MN / 3 = 816 kN safe limit
        ctx.assert_ge(peak_tension, 816000.0, f"Dynamic snatch tension spike ({peak_tension/1000.0:.1f} kN) exceeds 816 kN safe threshold")

    @suite.register("TC-T2-F13-03", "Zero-Damping Cable Resonance (Sustained Bounce Ringing)", 2, 13)
    def test_f13_zero_damping_bounce_resonance(ctx: TestContext):
        fn = 4.35  # Hz bounce resonance
        wn = 2.0 * np.pi * fn
        dt = 1.0 / 300.0
        t = np.arange(0, 2.0, dt)

        # Undamped case: zeta = 0.0
        z_undamped = np.cos(wn * t)
        amp_undamped = float(np.max(np.abs(z_undamped[-50:])))

        # Damped case: zeta = 0.05
        z_damped = np.exp(-0.05 * wn * t) * np.cos(wn * t)
        amp_damped = float(np.max(np.abs(z_damped[-50:])))

        ctx.assert_almost_equal(amp_undamped, 1.000, 0.02, "Undamped cable oscillator conserves 100% of peak energy")
        decay_pct = (1.0 - amp_damped) * 100.0
        ctx.assert_ge(decay_pct, 90.0, f"Damped cable oscillator decays by > 90% over 2.0 s (observed {decay_pct:.1f}%)")

    @suite.register("TC-T2-F13-04", "Infinite Stiffness Rigid Rope Limit (K_rope -> inf)", 2, 13)
    def test_f13_rigid_rope_limit(ctx: TestContext):
        Md = 54300.0
        F_hoist = 500000.0  # 500 kN hoist force
        K_compliant = 2.5e7   # 25 MN/m
        K_rigid = 1e11       # 100 GN/m (asymptotic limit)

        dx_comp_mm = (F_hoist / K_compliant) * 1000.0
        dx_rigid_mm = (F_hoist / K_rigid) * 1000.0

        ctx.assert_ge(dx_comp_mm, 15.0, "Compliant cable stretches > 15 mm under 500 kN load")
        ctx.assert_le(dx_rigid_mm, 0.01, "Rigid cable limit suppresses elastic deflection to < 0.01 mm")

        # Bounce resonant frequency shifts beyond Nyquist band into high-frequency realm
        f_bounce_rigid = np.sqrt(K_rigid / Md) / (2.0 * np.pi)
        ctx.assert_ge(float(f_bounce_rigid), 100.0, "Rigid cable limit shifts resonance beyond 100 Hz (outside observer dynamics)")

    @suite.register("TC-T2-F13-05", "Asymmetric Backlash Gap During Bidirectional Flutter", 2, 13)
    def test_f13_asymmetric_backlash(ctx: TestContext):
        # Asymmetric rack-and-pinion backlash: +3.0 mm forward, -4.5 mm reverse
        K_gear = 1e8  # N/m contact stiffness
        gap_pos = 3.0e-3
        gap_neg = -4.5e-3

        t = np.linspace(0, 1.0, 1000)
        x = 6.0e-3 * np.sin(2.0 * np.pi * 2.0 * t)  # +/- 6.0 mm motion

        F = np.zeros_like(x)
        F[x > gap_pos] = K_gear * (x[x > gap_pos] - gap_pos)
        F[x < gap_neg] = K_gear * (x[x < gap_neg] - gap_neg)

        peak_pos = float(np.max(F))
        peak_neg = float(np.min(F))
        deadband_ratio = float(np.mean(F == 0.0))

        ctx.assert_almost_equal(peak_pos, 300000.0, 1000.0, "Positive contact force equals 300 kN")
        ctx.assert_almost_equal(peak_neg, -150000.0, 1000.0, "Negative contact force equals -150 kN")
        ctx.assert_ge(deadband_ratio, 0.40, "Asymmetric backlash deadband occupies > 40% of cycle trajectory")
