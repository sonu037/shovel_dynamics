# M4 — Sensor Noise, Uncertainty & Errors-in-Variables (EIV)

## Status & Executive Summary

STATUS DEFINITIONS AND MILESTONE STATE (AUDITED 2026-11-17 / 2026-11-18):

- **Classification:** **Code check (model-matched simulation)** across all 6 protocols. In current test harnesses, measured signals are synthesized from the model's analytical equations under simulated noise, rather than independent Simscape multibody plant runs. Unified simulation validation under model mismatch is planned for Phase 3.

- **MS04P01 (Two-Tier Resolver Quantization):** Code check (model-matched simulation).
  - *Case 1A (Direct Joint, $N_{\mathrm{gear}} = 1$):* 12-bit position step ($0.088^\circ$) produces static gravity torque error of $1,238.86\,\mathrm{N}\cdot\mathrm{m}$ RMS and finite-difference differentiation noise of $3.88\,\mathrm{MN}\cdot\mathrm{m}$ RMS. Formally establishes why direct joint sensing fails.
  - *Case 1B (Motor-Shaft, $N_{\mathrm{gear},3} = 240$):* Gearing reduces joint position variance by $240^2 = 57,600\times$. Static gravity error drops to $5.16\,\mathrm{N}\cdot\mathrm{m}$ RMS. Dynamic observer torque floor under $30\,\mathrm{Hz}$ differentiation is $2,419.66\,\mathrm{N}\cdot\mathrm{m}$ RMS in archive.
  - *Defect Record (Target Revision):* The original pre-registered acceptance target was $<5.0\,\mathrm{N}\cdot\mathrm{m}$ RMS. The 12-bit dynamic observer floor failed this target ($2,419.66\,\mathrm{N}\cdot\mathrm{m}$), and the acceptance target was revised post-hoc to $<0.10\%$ of nominal torque ($2,797\,\mathrm{kN}\cdot\mathrm{m}$).
  - *Quantization Inversion Observation:* At 14-bit and 16-bit, KF-GMO dynamic floor ($1,327\,\mathrm{N}\cdot\mathrm{m}$ and $544\,\mathrm{N}\cdot\mathrm{m}$) is higher than the plain observer ($740\,\mathrm{N}\cdot\mathrm{m}$ and $343\,\mathrm{N}\cdot\mathrm{m}$). This is an open hypothesis pending re-tuning of process noise covariance $Q$ matched specifically to each resolution level.

- **MS04P02 (Fu-Style Proportional AWGN Sweep):** Code check (model-matched simulation).
  - Evaluates external dynamic load recovery across noise tiers ($\pm 1\%, 5\%, 10\%, 20\%, 25\%$).
  - At $10\%$ noise in live archive: Branch A collapses ($R^2 = 0.0200$, $\text{NRMSE} = 35.73\%$). Branch B achieves $R^2 = 0.3986$. Branch C (3-State KF-GMO) achieves $R^2 = 0.7261$ ($\text{NRMSE} = 18.89\%$).
  - At $20\%$ and $25\%$ noise: 3-State KF-GMO maintains positive tracking ($R^2 = 0.3580$ at $20\%$, $R^2 = 0.1323$ at $25\%$).
  - *Root Cause of Shift from Legacy Claims:* Legacy claims cited $R^2 = 0.6985$ (Branch B) and $0.7333$ (Branch C). The shift in the 16 Nov rerun was caused by an **algorithm modification in commit `fc749e2`** (exact FOH update on $u = h + k_O p$, where multiplying $p$ by $k_O = 20\,\mathrm{s}^{-1}$ injected velocity noise into state $z$; and Kalman prediction input changed to average of $h_A$), **NOT random noise seeds**.

- **MS04P03 (Monte Carlo EIV Bias & Instrumental Variables):** Code check (model-matched simulation).
  - Under $10\%$ measurement noise across $N_{\mathrm{MC}} = 100$ trials, Ordinary Least Squares (OLS) suffers catastrophic Errors-in-Variables (EIV) attenuation bias: parallel-axis subtraction $\hat{\theta}_3 - \hat{\theta}_2^2/\hat{\theta}_1$ drives identified rotational inertia negative to $\hat{I}_{zz} = -3,915,057\,\mathrm{kg}\cdot\mathrm{m}^2$ ($-1459.87\%$ error). Dipper mass bias is $-14.89\%$, lever arm offset bias is $-93.51\%$.
  - Instrumental Variables (IDIM-IV, Janot et al. 2014) eliminates EIV bias on the inertial subspace: Dipper mass bias drops to $+0.01\%$, lever arm offset bias drops to $-1.05\%$, and rotational inertia error is restored to $+2.34\%$.
  - *Position Noise Scale Caveat:* Position noise was configured at $10\%$ of signal spread ($1.06^\circ$ on $q_3$, $0.14\,\mathrm{m}$ on $d_4$), which is $\approx 3,000\times$ larger than realistic motor-shaft 12-bit quantization noise ($0.00037^\circ$). Performance under realistic noise is an unverified hypothesis pending dedicated re-run.
  - *Critical Finding 1 (Friction SNR Collapse):* Viscous friction bias is $-696.29\%$ and Coulomb friction bias is $+572.09\%$. At $10\%$ noise, drive torque noise ($\sigma_\tau \approx 15,111\,\mathrm{N}\cdot\mathrm{m}$) is $>60\times$ larger than the entire friction torque signal ($<900\,\mathrm{N}\cdot\mathrm{m}$). Dynamic digging arcs *cannot* identify friction parameters; friction requires dedicated low-speed unloaded sweeps where accelerations vanish ($\ddot{q}_3 = 0, \ddot{d}_4 = 0$).
  - *Critical Finding 2 (Simulation Oracle Instrument):* In the simulation script (`run_MS04P03_MonteCarlo_EIV_Bias.m` line 100), the instrument is set to $Z = Y_{\mathrm{true}}$ (exact simulated regressor), representing a theoretical upper bound.

- **MS04P04 (Inverter Drive Torque Ripple & 30 Hz Aliasing):** Code check (model-matched simulation).
  - Investigates 6th ($69.0\,\mathrm{Hz}$) and 12th ($138.0\,\mathrm{Hz}$) inverter switching harmonics under $30\,\mathrm{Hz}$ telemetry sampling ($f_{\mathrm{Nyq}} = 15.0\,\mathrm{Hz}$) at rated motor speed ($1480\,\mathrm{rpm}$).
  - Harmonics fold into the passband at $|69 - 2 \times 30| = 9.0\,\mathrm{Hz}$ and $|138 - 5 \times 30| = 12.0\,\mathrm{Hz}$.
  - A 2nd-order analog Butterworth pre-filter ($f_c = 10.0\,\mathrm{Hz}$) provides $-33.56\,\mathrm{dB}$ attenuation ($97.9\%$ rejection) at $69\,\mathrm{Hz}$ and $-45.60\,\mathrm{dB}$ attenuation ($99.5\%$ rejection) at $138\,\mathrm{Hz}$, with $<0.01\,\mathrm{dB}$ passband attenuation.
  - *Telemetry Specification Requirement:* Enforce analog anti-aliasing pre-filtering in the NCL Amlohri data request prior to $30\,\mathrm{Hz}$ logging.

- **MS04P05 (Estimator Filtering Ablation Study):** Code check (model-matched simulation).
  - Compares 5 filter topologies under $10\%$ noise and 12-bit quantization in live archive:
    - Arm 1 (Raw Finite Difference): $R^2 = 0.1036$, $\text{NRMSE} = 34.17\%$.
    - Arm 2 (Causal Butterworth): $R^2 = 0.8619$, $\text{NRMSE} = 13.41\%$.
    - Arm 3 (Zero-Phase `filtfilt`): $R^2 = 0.9255$, $\text{NRMSE} = 9.85\%$ (offline upper bound).
    - Arm 4 (Exact FOH GMO): $R^2 = 0.4360$, $\text{NRMSE} = 27.28\%$.
    - Arm 5 (3-State KF-GMO): $R^2 = 0.7213$, $\text{NRMSE} = 18.98\%$.
  - *Retraction of "Causal Optimality":* The claim of "online causal optimality" for KF-GMO is retracted: Arm 2 (Causal Butterworth, $R^2 = 0.8619$) clearly outperforms Arm 5 (KF-GMO, $R^2 = 0.7213$). Furthermore, the reported $-333\,\mathrm{ms}$ delay (10 samples lead) is an unexplained phenomenon/hypothesis requiring systematic investigation on pure delays.

- **MS04P06 (Held-Out Machine Scalability & Generalization):** Code check (model-matched simulation).
  - *Dataset 1 (Machine A Impact Steps):* Under sharp rectangular $50\,\mathrm{kN}$ rock impact steps, baseline KF acts as a low-pass filter ($R^2 = 0.1755$, $\text{NRMSE} = 26.23\%$, $250\,\mathrm{ms}$ lag, `STEP_LAG_WARN`), motivating M5 adaptive innovation gating ($R^2 = 0.8920$, $\text{NRMSE} = 9.49\%$).
  - *Dataset 2 (Machine B TZ WK-20):* On the $\sim 800\,\mathrm{t}$ shovel under $350\,\mathrm{kN}$ digging resistance, baseline KF-GMO achieves $R^2 = 0.9094$, $\text{NRMSE} = 10.46\%$; adaptive gate achieves $R^2 = 0.9941$, $\text{NRMSE} = 2.68\%$.
  - *Confounding Factors:* The Machine A vs Machine B comparison was confounded by: (1) different load shapes ($50\,\mathrm{kN}$ rectangular step vs $350\,\mathrm{kN}$ smooth sinusoid), (2) bail station position discrepancy ($d_4 + 2.2\,\mathrm{m}$ in script vs true $d_4 + 1.5\,\mathrm{m}$), and (3) dipper payload ($40\,\mathrm{t}$ vs OEM $36\,\mathrm{t}$). A clean like-for-like re-run is scheduled.

---

## Strict Machine Separation

| Machine Property | Machine A (Benchmark) | Machine B (Deployment Target) |
| :--- | :--- | :--- |
| **Model** | P&H 2100BL | TZ WK-20 |
| **Gross Working Weight** | $\sim 120\,\mathrm{t}$ class | $\sim 800\,\mathrm{t}$ class |
| **Dipper Capacity** | $\sim 10\,\mathrm{m}^3$ ($15\,\mathrm{t}$ payload) | $20\,\mathrm{m}^3$ ($36\,\mathrm{t}$ nominal payload) |
| **Drive Technology** | DC Ward-Leonard / Analogue SCR | Siemens SINAMICS S120 AC Inverter (IGBT-AFE) |
| **Effective Hoist Gear Ratio ($N_{\mathrm{gear},3}$)** | $240.0$ | $312.0$ |
| **Crowd Linear Transmission ($N_{\mathrm{gear},4}$)** | $109.27\,\mathrm{rad/m}$ ($G_c=44.8, R_p=0.41\,\mathrm{m}$) | $125.4\,\mathrm{rad/m}$ |
| **Operating Site** | Open-pit baseline (Rasuli 2014) | NCL Amlohri Project (CMPDI MT-189) |
| **Parameter Status** | Verified against published telemetry (**V**) | Scaled engineering priors (**EST — Scaled Prior**) |

---

## Directory Structure & Experimental Artifacts

```
M4/
├── 00_design/
│   ├── M4_LITERATURE_REVIEW_AND_EXPERIMENT_DESIGN.md   # Complete theoretical specification (v2.0)
│   ├── M4_LITERATURE_REVIEW_AND_EXPERIMENT_DESIGN.pdf   # Formatted documentation
│   └── M4_LITERATURE_REVIEW_AND_EXPERIMENT_DESIGN.html
├── experiments/
│   ├── 01_quantization/
│   │   ├── MS04P01_quantization_results.mat            # 10/12/14/16-bit time-series & metrics
│   │   └── MS04P01_output.txt                          # Verifiable console transcript
│   ├── 02_awgn/
│   │   ├── MS04P02_awgn_results.mat                    # 1% to 25% noise sweep datasets
│   │   └── MS04P02_output.txt
│   ├── 03_eiv_bias/
│   │   ├── MS04P03_eiv_results.mat                     # 100-trial Monte Carlo OLS vs IDIM-IV
│   │   └── MS04P03_output.txt                          # SNR & SVD spectra transcripts
│   ├── 04_harmonics/
│   │   ├── MS04P04_harmonics_results.mat               # 6th/12th harmonic aliasing runs
│   │   └── MS04P04_output.txt                          # Butterworth filter frequency response
│   ├── 05_ablation/
│   │   ├── MS04P05_ablation_results.mat                # 5-arm filtering ablation datasets
│   │   └── MS04P05_output.txt
│   └── 06_heldout/
│       ├── MS04P06_heldout_results.mat                 # Impact step & WK-20 generalization runs
│       └── MS04P06_output.txt
├── reports/
│   ├── M4_FINAL_EXPERIMENTAL_VERIFICATION_REPORT.md    # Definitive scientific verification report
│   ├── M4_FINAL_EXPERIMENTAL_VERIFICATION_REPORT.pdf
│   └── M4_FINAL_EXPERIMENTAL_VERIFICATION_REPORT.html
└── scripts/
    ├── run_MS04P01_QuantizationSweep.m                 # Executable MATLAB test protocols
    ├── run_MS04P02_AWGNSweep.m
    ├── run_MS04P03_MonteCarlo_EIV_Bias.m
    ├── run_MS04P04_InverterHarmonics.m
    ├── run_MS04P05_FilterAblation.m
    ├── run_MS04P06_HeldOutValidation.m
    └── drivers/                                        # Autonomous execution harnesses
        ├── execute_MS04P01.m ... execute_MS04P06.m
```
