"""
Unified CLI Test Runner for shovel-dynamics End-to-End Opaque-Box Test Suite.
Usage:
    python verification/e2e/run_all_e2e_tests.py --tier all --json verification/e2e/results/e2e_test_receipt.json --report verification/e2e/results/e2e_test_report.md
"""

import argparse
import os
import sys
from pathlib import Path

# Ensure repo root and e2e package are on sys.path
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from verification.e2e.harness.runner import TestSuite
from verification.e2e.harness.reporter import generate_json_receipt, generate_markdown_report
from verification.e2e.tests.test_tier1_features import register_tier1_tests
from verification.e2e.tests.test_tier2_boundaries import register_tier2_tests
from verification.e2e.tests.test_tier3_pairwise import register_tier3_tests
from verification.e2e.tests.test_tier4_scenarios import register_tier4_tests


def build_suite() -> TestSuite:
    """Instantiate and register all test cases across all 4 tiers."""
    suite = TestSuite()
    register_tier1_tests(suite)
    register_tier2_tests(suite)
    register_tier3_tests(suite)
    register_tier4_tests(suite)
    return suite


def parse_args():
    parser = argparse.ArgumentParser(
        description="End-to-End Opaque-Box Test Runner for Shovel Dynamics Research Roadmap"
    )
    parser.add_argument(
        "--tier",
        default="all",
        choices=["1", "2", "3", "4", "all"],
        help="Filter tests by execution tier (default: all)",
    )
    parser.add_argument(
        "--feature",
        default="all",
        help="Filter tests by Project Feature ID (1..13 or all, default: all)",
    )
    parser.add_argument(
        "--test-id",
        dest="test_id",
        default=None,
        help="Filter tests by exact ID or regex pattern (e.g. TC-T3-01, SCEN)",
    )
    parser.add_argument(
        "--json",
        dest="json_path",
        default=str(SCRIPT_DIR / "results" / "e2e_test_receipt.json"),
        help="Path to write machine-readable JSON execution receipt",
    )
    parser.add_argument(
        "--report",
        dest="report_path",
        default=str(SCRIPT_DIR / "results" / "e2e_test_report.md"),
        help="Path to write Markdown summary report",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail fast: terminate test execution on the first assertion failure",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable detailed assertion trace output for each test",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all discovered test cases without executing",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    suite = build_suite()

    if args.list:
        tests = suite.filter_tests(
            tier=args.tier, feature=args.feature, test_id_pattern=args.test_id
        )
        print(f"\nDiscovered {len(tests)} test cases matching criteria:")
        print(f"{'#':<4} {'Test ID':<16} {'Tier':<6} {'Features':<10} {'Name'}")
        print("-" * 75)
        for i, tc in enumerate(tests, 1):
            feats = ",".join(str(f) for f in tc.features)
            print(f"{i:<4} {tc.test_id:<16} {tc.tier:<6} {feats:<10} {tc.name}")
        sys.exit(0)

    print("\n" + "=" * 78)
    print("SHOVEL-DYNAMICS E2E VERIFICATION TEST SUITE (Python 3.14)")
    print(f"Filter: Tier={args.tier}, Feature={args.feature}, ID={args.test_id or '*'}")
    print("=" * 78 + "\n")

    results, summary = suite.run(
        tier=args.tier,
        feature=args.feature,
        test_id=args.test_id,
        strict=args.strict,
        verbose=args.verbose,
    )

    # Generate Reports
    json_path = Path(args.json_path)
    report_path = Path(args.report_path)
    generate_json_receipt(results, summary, json_path)
    generate_markdown_report(results, summary, report_path)

    print("\n" + "-" * 78)
    print("EXECUTION SUMMARY")
    print("-" * 78)
    print(f"Total Tests Executed : {summary['total_executed']}")
    print(f"Passed               : {summary['passed']}")
    print(f"Failed               : {summary['failed']}")
    print(f"Errors               : {summary['errors']}")
    print(f"Pass Rate            : {summary['pass_rate_pct']:.2f}%")
    print(f"Duration             : {summary['duration_sec']:.2f} s")
    print(f"JSON Receipt Saved   : {json_path}")
    print(f"Markdown Report Saved: {report_path}")

    tb = summary.get("tier_breakdown", {})
    if tb:
        print("\nTier Breakdown:")
        for t_key in sorted(tb.keys()):
            stat = tb[t_key]
            tag = "PASS" if stat["passed"] == stat["total"] and stat["total"] > 0 else "FAIL"
            print(f"  {t_key:<8}: {stat['passed']}/{stat['total']} passed ({tag})")

    if summary["all_passed"]:
        print("\n✅ OVERALL VERDICT: ALL TESTS PASSED (100% Verified)")
        sys.exit(0)
    else:
        print("\n❌ OVERALL VERDICT: ONE OR MORE TESTS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nFATAL RUNTIME ERROR: {e}", file=sys.stderr)
        sys.exit(2)
