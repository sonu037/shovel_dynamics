"""
Report and execution receipt generator for E2E testing.
Produces machine-readable JSON receipt (e2e_test_receipt.json) and Markdown audit report (e2e_test_report.md).
"""

import json
import os
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List
import numpy as np
import scipy

from .runner import TestResult


def generate_json_receipt(
    results: List[TestResult],
    summary: Dict[str, Any],
    output_path: str | Path,
) -> Path:
    """Generate structured machine-readable JSON execution receipt."""
    out_p = Path(output_path).resolve()
    out_p.parent.mkdir(parents=True, exist_ok=True)

    now_iso = datetime.now(timezone.utc).isoformat()

    receipt_data = {
        "suite": "shovel-dynamics-e2e",
        "version": "1.0.0",
        "timestamp": now_iso,
        "environment": {
            "python_version": sys.version.split()[0],
            "platform": platform.platform(),
            "numpy_version": np.__version__,
            "scipy_version": scipy.__version__,
        },
        "summary": summary,
        "results": [tr.to_dict() for tr in results],
    }

    with open(out_p, "w", encoding="utf-8") as f:
        json.dump(receipt_data, f, indent=2)

    return out_p


def generate_markdown_report(
    results: List[TestResult],
    summary: Dict[str, Any],
    output_path: str | Path,
) -> Path:
    """Generate comprehensive Markdown verification report."""
    out_p = Path(output_path).resolve()
    out_p.parent.mkdir(parents=True, exist_ok=True)

    now_iso = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

    lines = [
        "# End-to-End Test Suite Execution Report",
        "",
        f"**Date:** {now_iso}  ",
        f"**Environment:** Python {sys.version.split()[0]}, NumPy {np.__version__}, SciPy {scipy.__version__}  ",
        f"**Platform:** {platform.platform()}  ",
        f"**Overall Verdict:** {'✅ **PASS** (100% Verified)' if summary.get('all_passed') else '❌ **FAIL**'}  ",
        "",
        "---",
        "",
        "## 1. Executive Summary",
        "",
        "| Metric | Value |",
        "|---|---|",
        f"| Total Tests Executed | `{summary.get('total_executed', 0)}` |",
        f"| Passed | `{summary.get('passed', 0)}` |",
        f"| Failed | `{summary.get('failed', 0)}` |",
        f"| Errors | `{summary.get('errors', 0)}` |",
        f"| Pass Rate | `{summary.get('pass_rate_pct', 0.0):.2f}%` |",
        f"| Total Execution Duration | `{summary.get('duration_sec', 0.0):.2f} s` |",
        "",
        "### Tier Breakdown",
        "",
        "| Tier | Name | Total | Passed | Failed | Errors | Status |",
        "|:---:|---|:---:|:---:|:---:|:---:|:---:|",
    ]

    tier_names = {
        "tier_1": "Tier 1: Feature Coverage (Nominal)",
        "tier_2": "Tier 2: Boundary, Corner & Negative Cases",
        "tier_3": "Tier 3: Pairwise Combinatorial Interactions",
        "tier_4": "Tier 4: Real-World Excavation Scenarios",
    }

    tb = summary.get("tier_breakdown", {})
    for t_key in sorted(tb.keys()):
        stat = tb[t_key]
        name = tier_names.get(t_key, t_key.upper())
        status_tag = "✅ PASS" if stat["passed"] == stat["total"] and stat["total"] > 0 else "❌ FAIL"
        lines.append(
            f"| `{t_key}` | {name} | {stat['total']} | {stat['passed']} | {stat['failed']} | {stat['error']} | {status_tag} |"
        )

    lines.extend([
        "",
        "---",
        "",
        "## 2. Invariants Enforced Across Test Tiers",
        "",
        "1. **Mass Matrix Positive Definiteness**: $\\min_q \\lambda_{\\min}(M(q)) \\ge 1.4\\times 10^3 > 0$.",
        "2. **Passivity Skew-Symmetry**: $\\|\\dot{M} - 2C + (\\dot{M}-2C)^T\\|_F < 10^{-12}$.",
        "3. **Uniform Time Grid Regularity**: $|\\Delta t - 1/30| < 10^{-9}$ s over 1951 samples.",
        "4. **Actuator OEM Physical Envelopes**: $|F_{crowd}| \\le 745.0$ kN, $|v_{crowd}| \\le 0.540$ m/s.",
        "5. **Thermodynamic Energy Dissipation**: Mechanical power dissipation $P_{diss} \\ge 0$.",
        "6. **Unilateral Cable Tension**: Hoist wire-rope tension $T_{rope} \\ge 0$.",
        "7. **Dataset Provenance**: Exact MD5 hash match (`5eaa1131f51257e72b24b2596a83598d`).",
        "",
        "---",
        "",
        "## 3. Detailed Test Results Matrix",
        "",
        "| # | Test ID | Tier | Features | Test Name | Duration | Status |",
        "|---|---|:---:|:---:|---|:---:|:---:|",
    ])

    for idx, tr in enumerate(results, 1):
        status_icon = "PASS" if tr.status == "PASS" else f"**{tr.status}**"
        feat_str = ",".join(str(f) for f in tr.features)
        lines.append(
            f"| {idx} | `{tr.test_id}` | T{tr.tier} | F{feat_str} | {tr.name} | {tr.duration_sec:.3f}s | {status_icon} |"
        )

    if summary.get("failed", 0) > 0 or summary.get("errors", 0) > 0:
        lines.extend([
            "",
            "---",
            "",
            "## 4. Failure & Error Telemetry",
            "",
        ])
        for tr in results:
            if tr.status != "PASS":
                lines.append(f"### {tr.test_id}: {tr.name}")
                lines.append(f"- **Status**: `{tr.status}`")
                lines.append(f"- **Error Message**: `{tr.error_message}`")
                if tr.stack_trace:
                    lines.append("```text")
                    lines.append(tr.stack_trace.strip())
                    lines.append("```")
                lines.append("")

    content = "\n".join(lines) + "\n"
    with open(out_p, "w", encoding="utf-8") as f:
        f.write(content)

    return out_p
