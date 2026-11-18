"""
Test registration, filtering, execution engine, and assertion framework.
Supports strict mode, detailed assertion logging, and timing metrics.
"""

import re
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Union


@dataclass
class AssertionResult:
    name: str
    value: Any
    threshold: Any
    operator: str  # "<=", ">=", "==", "!=", "in", "<", ">", "is"
    status: str  # "PASS" or "FAIL"
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "value": float(self.value) if isinstance(self.value, (int, float, bool)) else str(self.value),
            "threshold": float(self.threshold) if isinstance(self.threshold, (int, float, bool)) else str(self.threshold),
            "operator": self.operator,
            "status": self.status,
            "message": self.message,
        }


@dataclass
class TestResult:
    test_id: str
    name: str
    tier: int
    features: List[int]
    status: str  # "PASS", "FAIL", "ERROR", "SKIPPED"
    duration_sec: float
    assertions: List[AssertionResult] = field(default_factory=list)
    error_message: Optional[str] = None
    stack_trace: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "test_id": self.test_id,
            "name": self.name,
            "tier": self.tier,
            "features": self.features,
            "status": self.status,
            "duration_sec": round(self.duration_sec, 4),
            "assertions": [a.to_dict() for a in self.assertions],
            "error_message": self.error_message,
        }


class TestContext:
    """Context object passed into each test function to collect assertions."""

    def __init__(self, test_id: str, name: str, verbose: bool = False):
        self.test_id = test_id
        self.name = name
        self.verbose = verbose
        self.assertions: List[AssertionResult] = []

    def assert_true(self, condition: bool, name: str, message: str = ""):
        passed = bool(condition)
        res = AssertionResult(
            name=name,
            value=passed,
            threshold=True,
            operator="==",
            status="PASS" if passed else "FAIL",
            message=message or ("Condition is True" if passed else "Condition is False"),
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {name}: {res.message}")
        if not passed:
            raise AssertionError(f"Assertion failed: {name} - {res.message}")

    def assert_equal(self, actual: Any, expected: Any, name: str, message: str = ""):
        passed = actual == expected
        res = AssertionResult(
            name=name,
            value=actual,
            threshold=expected,
            operator="==",
            status="PASS" if passed else "FAIL",
            message=message or f"Actual: {actual}, Expected: {expected}",
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {name}: {res.message}")
        if not passed:
            raise AssertionError(f"Assertion failed: {name} - {res.message}")

    def assert_le(self, actual: float, ceiling: float, name: str, message: str = ""):
        passed = actual <= ceiling
        res = AssertionResult(
            name=name,
            value=actual,
            threshold=ceiling,
            operator="<=",
            status="PASS" if passed else "FAIL",
            message=message or f"Value {actual:.4e} <= ceiling {ceiling:.4e}",
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {name}: {res.message}")
        if not passed:
            raise AssertionError(f"Assertion failed: {name} - {res.message}")

    def assert_ge(self, actual: float, floor: float, name: str, message: str = ""):
        passed = actual >= floor
        res = AssertionResult(
            name=name,
            value=actual,
            threshold=floor,
            operator=">=",
            status="PASS" if passed else "FAIL",
            message=message or f"Value {actual:.4e} >= floor {floor:.4e}",
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {name}: {res.message}")
        if not passed:
            raise AssertionError(f"Assertion failed: {name} - {res.message}")

    def assert_almost_equal(
        self, actual: float, expected: float, tol: float, name: str, message: str = ""
    ):
        diff = abs(actual - expected)
        passed = diff <= tol
        res = AssertionResult(
            name=name,
            value=actual,
            threshold=expected,
            operator=f"approx(+/-{tol:.2e})",
            status="PASS" if passed else "FAIL",
            message=message or f"|{actual:.4e} - {expected:.4e}| = {diff:.4e} <= {tol:.2e}",
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {name}: {res.message}")
        if not passed:
            raise AssertionError(f"Assertion failed: {name} - {res.message}")

    def assert_invariant(self, invariant_result: Dict[str, Any], name: Optional[str] = None):
        inv_name = name or invariant_result.get("invariant", "INVARIANT")
        passed = bool(invariant_result.get("passed", False))
        msg = invariant_result.get("message", "")
        res = AssertionResult(
            name=inv_name,
            value="PASS" if passed else "FAIL",
            threshold="PASS",
            operator="==",
            status="PASS" if passed else "FAIL",
            message=msg,
        )
        self.assertions.append(res)
        if self.verbose:
            print(f"      [{res.status}] {inv_name}: {msg}")
        if not passed:
            raise AssertionError(f"Invariant check failed: {inv_name} - {msg}")


@dataclass
class TestCase:
    test_id: str
    name: str
    tier: int
    features: List[int]
    func: Callable[[TestContext], None]
    description: str = ""


class TestSuite:
    """Central registry and execution coordinator for test cases."""

    def __init__(self):
        self.tests: List[TestCase] = []

    def register(
        self,
        test_id: str,
        name: str,
        tier: int,
        features: Union[int, List[int]],
        description: str = "",
    ):
        """Decorator to register test functions."""
        feature_list = [features] if isinstance(features, int) else list(features)

        def decorator(fn: Callable[[TestContext], None]):
            tc = TestCase(
                test_id=test_id,
                name=name,
                tier=tier,
                features=feature_list,
                func=fn,
                description=description,
            )
            self.tests.append(tc)
            return fn

        return decorator

    def filter_tests(
        self,
        tier: Optional[Union[int, str]] = None,
        feature: Optional[Union[int, str]] = None,
        test_id_pattern: Optional[str] = None,
    ) -> List[TestCase]:
        """Filter registered tests by tier, feature, or regex pattern."""
        filtered = list(self.tests)

        if tier is not None and str(tier).lower() != "all":
            t_int = int(tier)
            filtered = [tc for tc in filtered if tc.tier == t_int]

        if feature is not None and str(feature).lower() != "all":
            f_int = int(feature)
            filtered = [tc for tc in filtered if f_int in tc.features]

        if test_id_pattern:
            regex = re.compile(test_id_pattern, re.IGNORECASE)
            filtered = [tc for tc in filtered if regex.search(tc.test_id) or regex.search(tc.name)]

        return filtered

    def run(
        self,
        tier: Optional[Union[int, str]] = None,
        feature: Optional[Union[int, str]] = None,
        test_id: Optional[str] = None,
        strict: bool = False,
        verbose: bool = False,
    ) -> Tuple[List[TestResult], Dict[str, Any]]:
        """Execute selected test cases and return structured results and summary metrics."""
        selected_tests = self.filter_tests(tier=tier, feature=feature, test_id_pattern=test_id)
        results: List[TestResult] = []

        total = len(selected_tests)
        passed_count = 0
        failed_count = 0
        error_count = 0

        t_suite_start = time.perf_counter()

        for idx, tc in enumerate(selected_tests, 1):
            ctx = TestContext(test_id=tc.test_id, name=tc.name, verbose=verbose)
            t_start = time.perf_counter()
            status = "PASS"
            err_msg = None
            tb_str = None

            if verbose:
                print(f"\n[{idx}/{total}] RUNNING {tc.test_id} (Tier {tc.tier}): {tc.name}")

            try:
                tc.func(ctx)
                passed_count += 1
            except AssertionError as ae:
                status = "FAIL"
                failed_count += 1
                err_msg = str(ae)
                tb_str = traceback.format_exc()
                if verbose:
                    print(f"    --> FAILED: {err_msg}")
            except Exception as e:
                status = "ERROR"
                error_count += 1
                err_msg = f"{type(e).__name__}: {str(e)}"
                tb_str = traceback.format_exc()
                if verbose:
                    print(f"    --> ERROR: {err_msg}")
            finally:
                duration = time.perf_counter() - t_start

            tr = TestResult(
                test_id=tc.test_id,
                name=tc.name,
                tier=tc.tier,
                features=tc.features,
                status=status,
                duration_sec=duration,
                assertions=ctx.assertions,
                error_message=err_msg,
                stack_trace=tb_str,
            )
            results.append(tr)

            if not verbose:
                # Compact output indicator
                mark = "." if status == "PASS" else ("F" if status == "FAIL" else "E")
                sys.stdout.write(mark)
                sys.stdout.flush()
                if idx % 50 == 0 or idx == total:
                    sys.stdout.write(f" [{idx}/{total}]\n")
                    sys.stdout.flush()

            if strict and status != "PASS":
                if verbose:
                    print(f"\nStrict mode enabled: terminating on first failure ({tc.test_id})")
                break

        t_suite_duration = time.perf_counter() - t_suite_start

        # Aggregate tier breakdown
        tier_breakdown: Dict[str, Dict[str, int]] = {}
        for tr in results:
            t_key = f"tier_{tr.tier}"
            if t_key not in tier_breakdown:
                tier_breakdown[t_key] = {"total": 0, "passed": 0, "failed": 0, "error": 0}
            tier_breakdown[t_key]["total"] += 1
            if tr.status == "PASS":
                tier_breakdown[t_key]["passed"] += 1
            elif tr.status == "FAIL":
                tier_breakdown[t_key]["failed"] += 1
            else:
                tier_breakdown[t_key]["error"] += 1

        summary = {
            "total_executed": len(results),
            "passed": passed_count,
            "failed": failed_count,
            "errors": error_count,
            "pass_rate_pct": (passed_count / len(results) * 100.0) if results else 0.0,
            "duration_sec": round(t_suite_duration, 4),
            "all_passed": (passed_count == len(results) and len(results) > 0),
            "tier_breakdown": tier_breakdown,
        }

        return results, summary
