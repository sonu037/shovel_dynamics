# M2 Experiment Design Specification

STATUS: template. Fill each module before running it. Once filled, FREEZE.
The purpose of freezing is to stop the design drifting while building starts.

Per-module format:
  Question | Hypothesis | Setup | Variables | Equations | Metric |
  Acceptance criterion | Failure interpretation | Repository location

---

## OPEN GATE BEFORE ANY CODE — the excitation trilemma

Three requirements pull against each other and none is soft:

  1. DYNAMIC RICHNESS      quasi-static testing does not exercise the residual
                           (Frimpong 2005 shows kinematic/dynamic response varies
                           strongly through the excavation trajectory)
  2. PHYSICAL ADMISSIBILITY trajectories must respect real limits on q, qdot, qddot
                           (Fu 2024, Bi 2020, Zhang 2022)
  3. IDENTIFIABILITY        separability needs large d4 variation, and
                           |drho/dd4| = 2.82 / (d4 - 1.32)^2 is largest at SHORT
                           extension - exactly where a real crowd is most constrained

RESOLUTION REQUIRED: state the P&H 2100 crowd stroke and velocity limits, then
choose the d4 range and excitation that maximise separability subject to those
limits. Report the resulting conditioning. This is a constrained design problem,
not three separate constraints to list.

This is the gate. Nothing downstream is meaningful until it is answered.

---

## 01_zero_force
Question:
Hypothesis:
Setup:
Variables:
Equations:
Metric:                 zero-load residual, absolute AND normalised by torque scale
Acceptance criterion:   at or below the independently established numerical floor
                        (do not freeze 1e-12 as a universal constant - it depends
                        on solver, tolerances, signal magnitude, sampling)
Failure interpretation: injection point, reference frame, sign convention,
                        attachment body, or time alignment is wrong. STOP.
Repository location:    M2/experiments/01_zero_force/

## 02_known_load
Question:
Hypothesis:
Setup:
Variables:
Equations:
Metric:                 NRMSE + peak-magnitude error + peak-time error
Acceptance criterion:
Failure interpretation:
Repository location:    M2/experiments/02_known_load/

## 03_amplitude_sweep
[same fields]

## 04_frequency_sweep
[same fields]
Note: results are LOCAL to the operating trajectory. Do not claim broadband
performance from single-frequency tests.

## 05_configuration_sweep
[same fields]
Note: produce TWO maps, one per generalized coordinate. Observability
redistributes between coordinates rather than vanishing - at q3 -> 90 deg,
Q_q3 -> 0 but Q_d4 -> -F.

## 06_identifiability
[same fields]
Core result to test:
  s_load  = (d4 + 1.5) cos q3
  s_dm    = g (d4 - 1.32) cos q3
  ratio   = (1/g) * (d4 + 1.5)/(d4 - 1.32)     <- cos q3 CANCELS
Therefore, FOR A VERTICAL LOAD, separability comes from d4 variation alone; q3
variation contributes nothing. This is conditional on Fx = 0 and must be stated
as such - for a general force the q3 dependence returns.

Distinguish structural identifiability (d4 must take at least two distinct values)
from practical identifiability (d4 must vary enough for acceptable conditioning).

## 07_model_perturbation
[same fields]
Build the fingerprint library: deliberately set dm, dIzz, dc, dfv and record each
residual signature. Reusable at M9.

## 08_held_out
[same fields]
Naming: "held-out validation" is the honest label. "Blind" applies only if a third
party generates the load file and the truth is revealed afterwards.

## Follow-on: torque-driven variant
Repeat 02 with the joints torque-driven so the load alters the motion. This tests
whether tau_model generalises to an unseen trajectory - a stronger claim than the
motion-prescribed baseline.

## Follow-on: two-station injection
Inject the same force at two different stations along the handle. Consistent
recovery demonstrates the Jacobian handles application-point variation. Motivated
by Raza and Frimpong, who show the real payload centroid MIGRATES as the dipper
fills, so the application point is genuinely time-varying at M7.
