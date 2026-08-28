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
| Claim | Source | Status | Note |
|---|---|---|---|
| WK-55 operational crowd extension band 9.50-10.04 m; crowd velocity <= 0.76 m/s; crowd acceleration <= 0.5 m/s^2, Anjialing mine | Bi et al. 2020 | **Verify against the PDF** | Used directly to define the Bi-band trajectory. These numbers now carry a result, so confirm them verbatim from the source. Also record that they are WK-55, not P&H 2100. |
| s_dm = d(tau3)/d(M_d) has three terms (inertia, Coriolis, gravity) | Own derivation from the M1 saddle equation | **Verified 2026-08-28** | Derived independently three times. Supersedes the gravity-only form previously in the spec. |
| Wrong-station regression slope = x^2-weighted mean of rho_station over the trajectory | Own derivation + measurement | **Verified** | 7e-06 (wide), 6e-06 (narrow), and slope 1.334832 vs rho_station midpoint 1.334 (Bi band). |