# claims.md — Every number, with its source, written down at first use

RULE: no number enters a script, slide, paper, or conversation-with-PI unless
it has a row here. "I remember it was about X" is not a source.
Status: V = verified against the source document by me · U = unverified/inherited.

## Machine parameters (current model, ShovelSimulator_v4)

| Quantity | Value | Source | Status |
|---|---|---|---|
| Dipper+handle mass Md | 54,300 kg | Rasuli et al. 2014, Table II (dynamic identification case) | V (read from rendered table, 2026-07-06) |
| COG offset L_COG | 2.82 m | Rasuli 2014, Table II | V |
| Rotational inertia Izz3 | 287,900 kg·m² | Rasuli 2014, Table II. OPEN ITEM: about-COM vs grouped (Izz+Md·L²) — resolve against their Eq. 23/26 before 4-DOF work | V (value), U (interpretation) |
| Revolute viscous fv3 | 3,800 N·m·s/rad = 66.322512 N·m/(deg/s) | Rasuli 2014, Table II. NOTE: initially misread as 340 (column swap); corrected 2026-07-06 | V |
| Revolute Coulomb fc3 | 340 N·m | Rasuli 2014, Table II. NOT yet in the Simscape model | V |
| Prismatic viscous fv4 | 1,473 N·s/m | Rasuli 2014, Table II | V |
| Prismatic Coulomb fc4 | 480 N | Rasuli 2014, Table II. NOT yet in model | V |
| Hoist motor const K_H | 1,940 N/A (dynamic) / 1,715 N/A (static) | Rasuli 2014, Tables II / I. Varies with field current (30 A vs 70 A regimes) — field log required for real data | V |
| Crowd motor const K_C | 3,950 N/A (dynamic) / 4,418 N/A (static) | Rasuli 2014, Tables II / I | V |
| Static-case Md, L | 56,900 kg, 3.27 m | Rasuli 2014, Table I (static postures, Eq. 29). Different from Table II because separate experiment — NOT an inconsistency | V |
| Geometric offset x_ref | 1.5 m | v4 model construction (Rigid Transform translation) — a modelling choice, not a literature value | V |
| Net lever constant c | −1.32 m (r = d4 − 1.32) | Derived: 1.5 − 2.82. Confirmed by blind identification: ĉ = −1.3200 | V |
| Gravity g | 9.80665 m/s² | Standard | V |

## Rasuli 2014 — results actually reported (for comparisons)

| Claim | Value | Where | Status |
|---|---|---|---|
| Swing-phase payload estimate | 8,402 kg vs 8,420 kg actual, σ = 104 kg | Rasuli 2014, results/Table 3 discussion | V |
| Dipper-lifting payload estimate | 8,340 kg, σ = 579 kg | Rasuli 2014 | V |
| "Rasuli reports R² = 0.942" | **FALSE — number does not exist in the paper.** Was fabricated in an AI session; struck 2026-07-07. Full-text search: zero hits for 0.942 / R² / R-squared | — | V (verified absent) |
| Sensor sampling rate used | 30 Hz | Rasuli 2014, Sec. on field experiment (9,000 points / 5 min) | V |
| Known-load calibration weight | 8,420 kg | Rasuli 2014 | V |
| Max measurable load in their field test | 10 tonnes; shovel downtime ~US$100k/hr limited experiments to ~1 hr | Rasuli 2014, Sec. on experiment limitations | V |

## Our results (Stage 1)

| Claim | Value | Evidence artifact | Status |
|---|---|---|---|
| Forward validation, revolute eq. (sensed accel) | R² = 1.000000, RMSE = 0 Nm | experiments/2026-07-07_stage1_final/stage1_results.mat | V |
| Forward validation, crowd eq. (first-ever check) | R² = 1.000000, RMSE = 0 N | same | V |
| Blind identification (sensed accel) | m=54,300.0; c=−1.3200; Izz=287,900; fv3=3,800.0; fc3=0.0 — all exact, CIs ~0 | same | V |
| With gradient() accelerations instead | NRMSE 0.79%; Izz biased to 125,804 (truth outside CI) | previous run, same script | V |
| cond(Y) column-scaled after d4 amplitude fix | 82–87 (healthy; >1e3 = excitation-limited) | script printout | V |

## Metrics precedent (for the methodology chapter)

| Claim | Source | Status |
|---|---|---|
| R², MAE, RMSE are the standard evaluation metrics for ECS load prediction | Fu et al. 2022 (HPINN), Sec. 4.2, Eq. 18 | V |
| Hybrid physics loss J = α·J_d + η·J_e (Lagrangian + energy conservation) | Fu et al. 2022, Eqs. 11–15 | V |
| Fu validated on synthetic WK-55 data with injected noise ±5–25% before field data | Fu et al. 2022, Sec. 4 | V |
