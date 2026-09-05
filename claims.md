# claims.md — Every number, with its source, written down at first use

RULE: no number enters a script, slide, paper, or conversation-with-PI unless
it has a row here. "I remember it was about X" is not a source.
Status: V = verified against the source document by me · U = unverified/inherited.

## Machine parameters (current model, ShovelSimulator\_v4)

|Quantity|Value|Source|Status|
|-|-|-|-|
|Dipper+handle mass Md|54,300 kg|Rasuli et al. 2014, Table II (dynamic identification case)|V (read from rendered table, 2026-07-06)|
|COG offset L\_COG\_Offset (thesis: L\_COGO)|2.82 m|Rasuli 2014 Table II / thesis Table 6. Thesis nomenclature verbatim: "the fixed perpendicular distance from the center of the gravity of the dipper and the dipper handle to the dipper bail" — used along-handle: eq. 56 gives r\_cd = d4 − L\_COGO, identical in form to our r = d4 + c|V|
|Rotational inertia Izz3|287,900 kg·m²|Rasuli 2014 Table II / thesis Table 6. OPEN ITEM CLOSED 2026-07-23: table value is about-COM Izz3; the parameter vector Φ carries the composite (Izz3 + Md·L²\_COG\_Offset) separately — no double-counting in our parallel-axis usage|V|
|Revolute viscous fv3|3,800 N·m·s/rad = 66.322512 N·m/(deg/s)|Rasuli 2014, Table II. NOTE: initially misread as 340 (column swap); corrected 2026-07-06|V|
|Revolute Coulomb fc3|340 N·m|Rasuli 2014, Table II. NOT yet in the Simscape model|V|
|Prismatic viscous fv4|1,473 N·s/m|Rasuli 2014, Table II|V|
|Prismatic Coulomb fc4|480 N|Rasuli 2014, Table II. NOT yet in model|V|
|Hoist motor const K\_H|1,940 N/A (dynamic) / 1,715 N/A (static)|Rasuli 2014, Tables II / I. Varies with field current (30 A vs 70 A regimes) — field log required for real data|V|
|Crowd motor const K\_C|3,950 N/A (dynamic) / 4,418 N/A (static)|Rasuli 2014, Tables II / I|V|
|Static-case Md, L|56,900 kg (thesis Table 5: 56,960 kg), 3.27 m|Rasuli 2014 Table I vs thesis Table 5 — DISCREPANCY between the two documents, logged. Separate static experiment; we use the dynamic set, identical in both|V (with discrepancy)|
|Geometric offset x\_ref|1.5 m|v4 model construction (Rigid Transform translation) — a modelling choice, not a literature value|V|
|Net lever constant c|−1.32 m (r = d4 − 1.32)|Derived: 1.5 − 2.82. Confirmed by blind identification: ĉ = −1.3200|V|
|Gravity g|9.80665 m/s²|Standard|V|
|Rasuli 2012 UBC thesis|adopted as report reference \[2]|Open access, UBC cIRcle item 1.0073407; fuller definitions than the 2014 paper; filed in papers\_library/|V|
|Slide-through-pivot shared with source|a3 = 0, frames o2 = o3|Thesis verbatim: saddle and crowd frames "are located in the center of the saddle block"; inherited and re-declared as report assumption A10|V|
|Datum convention vs source|his d4 measured from the pivot; ours from a datum 1.5 m out|Only r is physical; the offset is absorbed exactly into identified c = −1.32|V|
|Payload COG placement rule|m\_Load at distance d4 (bail station)|Thesis assumption; adopted as M2 injection point and M7 structure|V|
|Md excludes payload — structural|experiments used "0Kg and 8420Kg" payloads, m\_Load carried separately|Thesis quote; no row says "empty" — exclusion is by model structure|V|
|Effective-parameter caveat|Table II values are parameters of the a3 = 0 model of that machine|May partially absorb any real eccentricity; not tape-measure geometry|V|

## Rasuli 2014 — results actually reported (for comparisons)

|Claim|Value|Where|Status|
|-|-|-|-|
|Swing-phase payload estimate|8,402 kg vs 8,420 kg actual, σ = 104 kg|Rasuli 2014, results/Table 3 discussion|V|
|Dipper-lifting payload estimate|8,340 kg, σ = 579 kg|Rasuli 2014|V|
|"Rasuli reports R² = 0.942"|**FALSE — number does not exist in the paper.** Was fabricated in an AI session; struck 2026-07-07. Full-text search: zero hits for 0.942 / R² / R-squared|—|V (verified absent)|
|Sensor sampling rate used|30 Hz|Rasuli 2014, Sec. on field experiment (9,000 points / 5 min)|V|
|Known-load calibration weight|8,420 kg|Rasuli 2014|V|
|Field-test limits (M8 input)|10 tonnes payload cap (mine weigh scale + truck capacity); \~8 h production interruption over 4 days; >US$100,000 per hour of normal operation|Rasuli thesis, limitations passage, verbatim: "each hour of the normal shovel operation is worth more than US$100,000… Our experiments interrupted the normal operation of the shovel for approximately eight hour." Corrects the earlier "\~1 hr" entry|V|

## Our results (Stage 1)

|Claim|Value|Evidence artifact|Status|
|-|-|-|-|
|Forward validation, revolute eq. (sensed accel)|R² = 1.000000, RMSE = 0 Nm|experiments/2026-07-07\_stage1\_final/stage1\_results.mat|V|
|Forward validation, crowd eq. (first-ever check)|R² = 1.000000, RMSE = 0 N|same|V|
|Blind identification (sensed accel)|m=54,300.0; c=−1.3200; Izz=287,900; fv3=3,800.0; fc3=0.0 — all exact, CIs \~0|same|V|
|With gradient() accelerations instead|NRMSE 0.79%; Izz biased to 125,804 (truth outside CI)|previous run, same script|V|
|cond(Y) column-scaled after d4 amplitude fix|82–87 (healthy; >1e3 = excitation-limited)|script printout|V|
|Trim ablation (T\_TRIM 0 vs 0.5 s, sensed accel)|No effect at Stage 1: both recover all parameters exactly; untrimmed keeps 373/374 and improves scaled cond 82.2 → 13.4|experiments/2026-07-15\_trim\_ablation/ (README + both run records)|V|

## Metrics precedent (for the methodology chapter)

|Claim|Source|Status|
|-|-|-|
|R², MAE, RMSE are the standard evaluation metrics for ECS load prediction|Fu et al. 2022 (HPINN), Sec. 4.2, Eq. 18|V|
|Hybrid physics loss J = α·J\_d + η·J\_e (Lagrangian + energy conservation)|Fu et al. 2022, Eqs. 11–15|V|
|Fu validated on synthetic WK-55 data with injected noise ±5–25% before field data|Fu et al. 2022, Sec. 4|V|
| Claim | Source | Status | Note |
|---|---|---|---|
| Lagrange model, high-order polynomial excitation trajectory, hybrid mean+wavelet filtering, adaptive ridge regression; model is simplified and **linearised**; objective is trajectory planning and tracking control | Hu, Z., Lin, S., Gong, Z. et al., *Struct Multidisc Optim* **68**, 101 (2025), DOI 10.1007/s00158-025-04048-1 | **Unverified — abstract only, PDF not yet read** | Closest concurrent work. Must read the "different disturbances" section: if disturbances include payload or excavation resistance, this touches the load–parameter confounding claim. Linearisation is a structural differentiator — it removes the inertia/Coriolis columns that carry separability. |
| Physics-informed CNN-BiLSTM-multi-head attention model for dynamic digging force prediction of electric cable shovel | *J. Mech. Sci. Technol.*, DOI 10.1007/s12206-025-0502-0 | **Unverified — author attribution and reported metrics unconfirmed** | Cites Rasul(i) 2014 and Frimpong & Hu 2008 — same lineage. Reported improvement figures seen second-hand were internally inconsistent (an "R² of 1.74%") and must be read from the paper before use. |
| ρ(d₄) = (d₄+1.5)/(d₄−1.32) governs bail-vs-COG station separability | Own derivation, M2 module 06 | **Verified numerically 2026-08-25** | Measured slope 1.378454 (wide) and 1.380943 (narrow) both fall inside the predicted ρ band. |

## M2 — literature and derivation claims

| Claim | Source | Status | Note |
|---|---|---|---|
| Bi et al. 2020 WK-55 crowd limits | Bi et al. 2020, *Energies* 13(3):3118, Table 2, crowd system, Anjialing field test | **V — verified against the PDF 2026-09-04** | Crowd velocity <= 0.76 m/s, crowd acceleration <= 0.5 m/s^2. Verbatim. The HOIST system differs (1.76 m/s, 0.8 m/s^2) — always specify which. WK-55, not P&H 2100; the gate still asks for P&H-specific limits. |
| "Bi operational extension band 9.50-10.04 m" is NOT a crowd extension band | Bi et al. 2020, Tables 6 and 7 | **CORRECTED 2026-09-04** | The numeric range is real: rho_0 spans 9.50 (Table 7, Curves 3 and 4) to 10.04 (Table 7, Curve 2 at d_m = 4.5 m). But rho_0 is the INITIAL POLAR DIAMETER — pivot-to-tooth distance at the START of each optimised dig, across five digging conditions and four pile profiles. It is NOT a stroke a single dig traverses, and NOT our d4. Our narrow-band trajectory oscillates d4 sinusoidally within these bounds: a different motion, on a different machine, from a different datum. Use the WIDTH (0.54 m) as a physically-motivated narrow-stroke case; do not claim the absolute position transfers. Supersedes the earlier rows that recorded this as a WK-55 operating envelope. |
| s_dm = d(tau3)/d(M_d) has three terms (inertia, Coriolis, gravity) | Own derivation from the M1 saddle equation | **V 2026-08-28** | Derived independently three times. Supersedes the gravity-only form previously in the spec. |
| Wrong-station regression slope = x^2-weighted mean of rho_station over the trajectory | Own derivation + measurement | **V** | 7e-06 (wide), 6e-06 (narrow), and slope 1.334832 vs rho_station midpoint 1.334 (narrow-band case). |

## M2 — measured results

| Claim | Value | Source | Status | Note |
|---|---|---|---|---|
| Narrow-band r(s_load, s_full) | 0.9999919261 | sensitivity_diagnostic.m | **D — diagnostic** | Correlation only. Confounded with 7.4x lower crowd velocity. Two trajectories, one frequency. Not a validated identifiability claim. |
| Narrow-band r(s_load, s_grav) | 0.9999729034 | same | D | |
| Wide r(s_load, s_full) | 0.8222157044 | same | D | Greater shape separation than the narrow band. Consistent with geometric separation through d4, but excitation differences NOT isolated. |
| Wide r(s_load, s_grav) | 0.5388857718 | same | D | Mean-removed Pearson. Compare narrow-band 0.9999729034. |
| Narrow-band RMS(s_co)/RMS(s_grav) | 0.127604 % | same | D | Most decorrelated column from the load in the narrow family (r = 0.161). Practical usefulness requires assessment against measurement and model uncertainty. |
| Gravity:dynamic RMS ratio | 40.25:1 wide, 42.71:1 narrow band | same | **Measured 2026-08-28** | Supersedes the unmeasured ~250:1 estimate. Trajectory-dependent. |
| M2 recovery linearity, 10-1000 kN | slope -1.000000, both channels, all six amplitudes | M2/experiments/03_amplitude_sweep/, MS02P03 closed 2026-09-02 | **V** | Verified from the saved .mat files: applied Fy equals requested at every level (10.0, 50.0, 100.0, 200.0, 500.0, 1000.0 kN), max abs Fx = 0.0 at every level. |
| M2 absolute recovery error | approx 2.00e-06 N m, varying 0.1% over a 100x amplitude range | same | **V** | max abs(residual - reference) on the q3 channel: 1.999448e-06 (10 kN), 1.999433e-06 (50), 1.999782e-06 (100), 1.999317e-06 (200), 2.000015e-06 (500), 2.001412e-06 (1000). NOT "constant" — the small variation is consistent with finite-precision round-off and is negligible relative to the reference load. Relative error therefore scales as 1/F: 2.723e-11 to 2.726e-13. |
| Station confounding, narrow band | 1-R^2 = 2.998519e-05, slope -1.334832 | M2/experiments/06_identifiability/M2_wrongstation_bi_band.mat | **V** | Wrong station assumed (d4-1.32 against true d4+1.5, 2.82 m apart), d4 in [9.5001, 10.0399] m. Slope matches the rho_station midpoint 1.334 over the band. |
| Station confounding collapse, wide to narrow band | 89.77x | 2.691764e-03 / 2.998519e-05, both read from the saved .mat files 2026-09-02 | **V** | Previously quoted as 89.8x without the division being performed. |
| M2 trim window | 0 s adequate; the 5 s used in MS02P01-03 made no difference | Extends the M1 trim-ablation row above (V 2026-07-15) + measurement 2026-09-02 | **V** | M1 recorded that trimming had no effect on Stage-1 identification with SENSED accelerations, and that the untrimmed case had better scaled conditioning (82.2 -> 13.4). M2 evaluated the same question in the external-load residual framework and found no material change: max abs recovery error identical to 7 s.f. from t=0 in M2_null (1.999550e-06 N m) and M2_03_100kN (1.999782e-06 N m) at trims of 0, 0.5, 1, 2 and 5 s. The startup transient IS present — motion converters still filter at tau=0.05 s — but it cancels, because tau_model is evaluated on the sensed motion including the transient. It mattered in M1 only when accelerations were numerically differentiated. Re-measure if frequency-domain analysis or torque-driven actuation is introduced. |
| MS02P04 common-frequency excitation | s_in/s_co ratio invariant at 18.1-19.0 across omega_traj = 0.314 to 1.36 rad/s | M2/experiments/04_frequency_sweep/M2_04_sweep_narrow.mat | **V 2026-09-04** | Under synchronized excitation (omega_q3 = omega_d4), s_co is a product of two velocities and s_in an acceleration, so both scale as omega^2. Common frequency changes their magnitude relative to gravity but not their relative weighting. Decoupling omega_d4 from omega_q3 remains untested. |
| MS02P04 stroke-dependent frequency effect | wide 1-r: 1.78e-01 -> 7.69e-04 (231x degradation); narrow 1-r: 8.07e-06 -> 8.53e-05 (~10x improvement) | M2_04_sweep_wide.mat, M2_04_sweep_narrow.mat | **V 2026-09-04** | Same five frequencies, opposite trends. r(s_load,s_grav) is the discriminator: 0.47-0.54 wide against 0.999973 narrow. The narrow improvement remains small in absolute terms: the distinguishable fraction reaches only 8.53e-05 at omega_traj = 1.36 rad/s. REFUTES the pre-registered prediction that r(s_load,s_in) would be stroke-independent at ~0.96; measured 0.9986 in the narrow stroke. |