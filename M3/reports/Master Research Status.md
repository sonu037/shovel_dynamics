# Master Research Status: Electric Cable Shovel Digital Twin (`M1`, `M2`, and `M3`)

**Project:** `IIT/SRIC/R/AEH/2026/104` (CMPDI) — *AI-Enabled Diagnosis, Prognosis and Health Management of Mining Electrical Systems*
**Research Topic:** *Dynamic Modelling and Digital Twin of Electric Cable Shovels: Physics-Based External-Load Estimation, Payload Estimation and Machine Health Monitoring*
**Researcher:** Suhail Majeed Sheikh, Junior Research Fellow, Department of Mining Engineering, IIT Kharagpur
**Supervisor:** Prof. Sunita Mishra
**Master Status Date:** `2026-11-06`

## 1. Milestone Readiness Scorecard (`M1` – `M3`)

| Milestone | Scope & Technical Objective | Primary Scripts & Models | Verified `.mat` Archives | Current Status | Key Quantitative Benchmark |
| :--- | :--- | :--- | :---: | :---: | :--- |
| **Milestone `M1`** | 2-DOF Euler--Lagrange dipper--handle forward validation against Simscape Multibody & blind 5-parameter OLS identification | [shovel_params.m](https://drive.google.com/file/d/1wSthr6FhN89qiYM_WpuCWQzTnGdTye68/view?usp=drivesdk), [validate_and_identify.m](https://drive.google.com/file/d/1q4GkfvptOasG0OpBboKnpe_YM7sf_eeK/view?usp=drivesdk), [ShovelSimulatorr2025b.slx](https://drive.google.com/file/d/1gXqIx_AY9DKwvqiXwKCIJ_jq7o6NxRMn/view?usp=drivesdk) | 3 ([stage1_results.mat](https://drive.google.com/file/d/1n8CQ7cGmwlBWuYTk-bLxosWSVzeKPMoi/view?usp=drivesdk), [run_trim05.mat](https://drive.google.com/file/d/1itlUsQwhex0iAhySDcwilo0zTo3vavc5/view?usp=drivesdk), [run_notrim.mat](https://drive.google.com/file/d/1a184KMKzGgFJhlRblwVVFrNf2guKc8Gg/view?usp=drivesdk)) | **FROZEN & COMPLETE** | Forward $R^2 = 1.000000$ ($1.35\times 10^{-6}\ \mathrm{N\cdot m}$, $8.23\times 10^{-11}\ \mathrm{N}$); all 5 parameters recovered to $0.000\%$ error with sensed $\ddot{q}$; $+56.1\%$ $I_{zz}$ bias under `gradient` velocity differentiation |
| **Milestone `M2`** | Known external point-force injection $Q_{\mathrm{ext}} = J_s(q)^T F$, linearity/frequency/configuration sweeps, station confounding, formal SVD identifiability, perturbation fingerprints, & held-out validation (`P00`–`P08`) | [recover_load.m](https://drive.google.com/file/d/1ehp6vMN3XhxFLzd9OtakC2M7b8DyupA1/view?usp=drivesdk), [sensitivity_diagnostic.m](https://drive.google.com/file/d/1w1XN9udZpiWqmovkKkYe4GHIr6c5NHbO/view?usp=drivesdk), [plot_m2.m](https://drive.google.com/file/d/11IDWHBm1a7XaNcrJ_xtI-_b5baloUwGC/view?usp=drivesdk), `m2_formal_identifiability_svd.m`, `m2_p05_config_sweep.m`, `m2_p07_model_perturbation.m`, `m2_p08_held_out_validation.m`, [M2_LoadInjection.slx](https://drive.google.com/file/d/1H2y82vELZqY4r12xpzCbfWbv-QWwkYgB/view?usp=drivesdk) | 25 active (`P00`: 2, `P01`: 1, `P02`: 2, `P03`: 6, `P04`: 7, `P04b`: 6, `P06`: 3) | **ANALYSIS COMPLETE** (Local Simulink `.slx` runs optional for `P05` grid & torque-driven `P08`) | Bail load recovery $R^2 = 1.000000$, $\mathrm{NRMSE}_{q_3} \le 4.28\times 10^{-10}\%$; station discriminating fraction $1-R^2$ collapses $89.8\times$ on Bi et al. narrow band ($\kappa_2(\tilde{S}) = 497.71$, $\theta_{\min} = 0.2302^\circ$), resolved via joint 2-channel $(\tau_3, f_4)$ estimation ($\kappa_2 = 3.77$, $\theta_{\min} = 29.70^\circ$) |
| **Milestone `M3` (`Lane B`)** | Two-branch $30\ \mathrm{Hz}$ generalized-momentum external-load observer eliminating joint acceleration $\ddot{q}$ (Branch A: friction-aware RK4 $M=67$; Branch B: friction-uncertainty exact FOH) | `m3_observer_suite.m` (`v2.0.0-PRELIMINARY`), `run_m3_experiments_v2.py` (`v2.0.0-PRELIMINARY`) | `M3_observer_audit_results.mat` + verified across all 20 M2 `.mat` archives | **`PRELIMINARY` (Code-to-Report Audited)** | Zero-load $\mathrm{RMSE}_3 = 53.47\ \mathrm{N\cdot m}$ (Br. A) vs. $300.43\ \mathrm{N\cdot m}$ (Br. B); 100 kN bail sinusoidal tracking $\mathrm{NRMSE} = 1.151\%$ (peak-to-peak) / $2.27\%$ (peak) with $50.05\ \mathrm{ms}$ lag; 100% friction-discrepancy bound coverage |

## 2. Complete Index of Generated Deliverables and Documentation

1. **`MASTER_RESEARCH_STATUS.md`**: Executive status across `M1`, `M2`, and `M3`.
2. **`LITERATURE_TO_M2_M3_MAPPING.md`** (`research_documentation/literature_to_m2_m3_mapping.md`): Detailed mapping of key papers from [papers_library.bib](https://drive.google.com/file/d/1a0hVaxwweLyX3F7HOZYepSoOWgIZcFYc/view?usp=drivesdk) and [papers.md](https://drive.google.com/file/d/1KWLpLb73GjfKu73YnFzH7MZmQsYS7sMf/view?usp=drivesdk) to `M1`–`M3`.
3. **`M2_CURRENT_STATE_AUDIT.md`** (`M2/research_documentation/M2_current_state_audit.md`): Forensic audit of all 28 `.mat` files and `MS02P00`–`MS02P08` packages.
4. **`M2_EXPERIMENT_REGISTER.xlsx`**: Multi-sheet Excel workbook registering all M2 packages, all 28 `.mat` archives with SHA-256 hashes, and the 16-trajectory SVD conditioning table.
5. **`M2_FORMAL_IDENTIFIABILITY_REPORT.md`**: Formal sensitivity matrix SVD, unit-norm vs. physically scaled conditioning $\kappa_2(S)$, VIF, and subspace-angle analysis.
6. **`M2_P05_CONFIGURATION_SWEEP_REPORT.md`**: Two-channel Jacobian observability maps across $(q_3, d_4)$ and quadrant-binned verification.
7. **`M2_P06_GEOMETRY_STATION_REPORT.md`**: Station reconciliation ($r_{\mathrm{COG}} = d_4 - 1.32\ \mathrm{m}$ vs. $s_{\mathrm{bail}} = d_4 + 1.50\ \mathrm{m}$) and wrong-station regression verification.
8. **`M2_P07_MODEL_PERTURBATION_REPORT.md`**: 15-case parameter perturbation fingerprint library and least-squares basis attribution.
9. **`M2_P08_HELD_OUT_VALIDATION_REPORT.md`**: 20-archive cross-trajectory held-out validation for both `M2` algebraic recovery and `M3` $30\ \mathrm{Hz}$ momentum observers.
10. **`M3_FILE_MIGRATION_REGISTER.xlsx`**: Excel workbook tracking all 19 M3 artifacts across `M3/00_README` through `M3/10_ARCHIVE`.
11. **`M3_CODE_AUDIT_REPORT.md`**: Complete code-to-report audit (`AUD-01` through `AUD-07`) and Euler--Lagrange $\dot{D} = C + C^T$ verification.
12. **`M3_VALIDATION_PROTOCOL.md`**: Formal 7-gate data ingestion check, 8-test verification matrix (`Tests A`–`G`), and promotion criteria.
13. **`CHANGELOG.md`**: Complete chronological record of all `M1`, `M2`, and `M3` additions and audits.
14. **Four New M2 MATLAB Scripts**: `m2_formal_identifiability_svd.m`, `m2_p05_config_sweep.m`, `m2_p07_model_perturbation.m`, and `m2_p08_held_out_validation.m`.
