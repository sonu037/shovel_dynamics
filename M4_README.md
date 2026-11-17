# M4 — Sensor Noise, Uncertainty & Errors-in-Variables (EIV)

## Status & Executive Summary

STATUS DEFINITIONS AND MILESTONE STATE (AUDITED 2026-11-17):

- **MS04P01 (Two-Tier Resolver Quantization):** CLOSED (Simulation-Verified, 2026-11-10).
  - *Case 1A (Direct Joint, $N_{\mathrm{gear}} = 1$):* 12-bit position step ($0.088^\circ$) produces static gravity torque error of $1,238.86\,\mathrm{N}\cdot\mathrm{m}$ RMS and finite-difference differentiation noise of $3.88\,\mathrm{MN}\cdot\mathrm{m}$ RMS. Formally establishes why direct joint sensing fails.
  - *Case 1B (Motor-Shaft, $N_{\mathrm{gear},3} = 240$):* Gearing reduces joint position variance by $240^2 = 57,600\times$. Static gravity error drops to $5.16\,\mathrm{N}\cdot\mathrm{m}$ RMS. Dynamic observer torque floor under $30\,\mathrm{Hz}$ differentiation is $2,442.68\,\mathrm{N}\cdot\mathrm{m}$ RMS ($0.0873\%$ of nominal torque $2,797\,\mathrm{kN}\cdot\mathrm{m}$, satisfying the $<0.10\%$ full-scale specification). Resolves **DEF-01**.

- **MS04P02 (Fu-Style Proportional AWGN Sweep):** CLOSED (Simulation-Verified, 2026-11-10).
  - Evaluates external dynamic load recovery across noise tiers ($\pm 1\%, 5\%, 10\%, 20\%, 25\%$).
  - At $10\%$ noise: Branch A (raw finite-difference inverse dynamics) collapses ($R^2 = 0.0200$, $\text{NRMSE} = 35.73\%$). Branch B (fixed-gain FOH GMO) achieves $R^2 = 0.6985$. Branch C (3-State KF-GMO) achieves $R^2 = 0.7333$.
  - At $20\%$ and $25\%$ noise: 3-State KF-GMO is the *only estimator capable of maintaining positive tracking* ($R^2 = 0.3580$ at $20\%$, $R^2 = 0.1323$ at $25\%$).
  - *Fu et al. (2022) Benchmark Note:* Fu achieved $R^2 \approx 0.97$ using an offline deep-learning Physics-Informed Neural Network (PINN). An analytical causal linear Kalman filter operating at $30\,\mathrm{Hz}$ hits a realistic physical boundary at $R^2 = 0.7333$. Resolves **DEF-05**.

- **MS04P03 (Monte Carlo EIV Bias & Instrumental Variables):** CLOSED (Simulation-Verified, 2026-11-10).
  - Under $10\%$ measurement noise across $N_{\mathrm{MC}} = 100$ trials, Ordinary Least Squares (OLS) suffers catastrophic Errors-in-Variables (EIV) attenuation bias: parallel-axis subtraction $\hat{\theta}_3 - \hat{\theta}_2^2/\hat{\theta}_1$ drives identified rotational inertia negative to $\hat{I}_{zz} = -3,915,057\,\mathrm{kg}\cdot\mathrm{m}^2$ ($-1459.87\%$ error). Dipper mass bias is $-14.89\%$, lever arm offset bias is $-93.51\%$.
  - Instrumental Variables (IDIM-IV, Janot et al. 2014) eliminates EIV bias on the inertial subspace: Dipper mass bias drops to $+0.01\%$, lever arm offset bias drops to $-1.05\%$, and rotational inertia error is restored to $+2.34\%$.
  - *Critical Finding 1 (Friction SNR Collapse):* Viscous friction bias is $-696.29\%$ and Coulomb friction bias is $+572.09\%$. At $10\%$ noise, drive torque noise ($\sigma_\tau \approx 15,111\,\mathrm{N}\cdot\mathrm{m}$) is $>60\times$ larger than the entire friction torque signal ($<900\,\mathrm{N}\cdot\mathrm{m}$). Sensed friction SNR is $-36.70\,\mathrm{dB}$ (viscous) and $-32.97\,\mathrm{dB}$ (Coulomb). In accordance with robotic identification literature (Swevers et al. 1997, Gautier & Janot 2014), friction *cannot* be identified from dynamic digging arcs; it requires dedicated low-speed unloaded sweeps where accelerations vanish ($\ddot{q}_3 = 0, \ddot{d}_4 = 0$).
  - *Critical Finding 2 (Simulation Oracle Instrument):* In the simulation script (`run_MS04P03_MonteCarlo_EIV_Bias.m` line 100), the instrument is set to $Z = Y_{\mathrm{true}}$ (exact simulated regressor), providing the theoretical upper bound. In field deployment, an auxiliary simulated digital twin model must generate $Z$. Resolves **DEF-02**.

- **MS04P04 (Inverter Drive Torque Ripple & 30 Hz Aliasing):** CLOSED (Simulation-Verified, 2026-11-11).
  - Investigates 6th ($69.0\,\mathrm{Hz}$) and 12th ($138.0\,\mathrm{Hz}$) inverter switching harmonics under $30\,\mathrm{Hz}$ telemetry sampling ($f_{\mathrm{Nyq}} = 15.0\,\mathrm{Hz}$).
  - Harmonics fold into the digital twin mechanical passband at $|69 - 2 \times 30| = 9.0\,\mathrm{Hz}$ and $|138 - 5 \times 30| = 12.0\,\mathrm{Hz}$.
  - A 2nd-order analog Butterworth pre-filter ($f_c = 10.0\,\mathrm{Hz}$) provides $-33.56\,\mathrm{dB}$ attenuation ($97.9\%$ rejection) at $69\,\mathrm{Hz}$ and $-45.60\,\mathrm{dB}$ attenuation ($99.5\%$ rejection) at $138\,\mathrm{Hz}$, with $<0.01\,\mathrm{dB}$ passband attenuation ($0\text{--}1.5\,\mathrm{Hz}$).
  - *Telemetry Specification Requirement:* Supervisory PLC telemetry must enforce anti-aliasing pre-filtering (or activate Siemens SINAMICS S120 internal smoothing `p1441`/`p1442`) prior to $30\,\mathrm{Hz}$ logging. Resolves **DEF-04**.

- **MS04P05 (Estimator Filtering Ablation Study):** CLOSED (Simulation-Verified, 2026-11-10).
  - Compares 5 filter topologies under $10\%$ noise and 12-bit quantization.
  - Arm 1 (Raw Finite Difference): $R^2 = 0.1036$, $\text{NRMSE} = 34.17\%$ (unusable).
  - Arm 2 (Causal Butterworth): $R^2 = 0.8619$, $\text{NRMSE} = 13.41\%$.
  - Arm 3 (Zero-Phase `filtfilt`): $R^2 = 0.9255$, $\text{NRMSE} = 9.85\%$ (offline upper bound).
  - Arm 4 (Exact FOH GMO): $R^2 = 0.7185$, $\text{NRMSE} = 19.15\%$.
  - Arm 5 (3-State KF-GMO): $R^2 = 0.7287$, $\text{NRMSE} = 18.80\%$ (online causal optimality). Resolves **DEF-03**.

- **MS04P06 (Held-Out Machine Scalability & Generalization):** CLOSED (Simulation-Verified, 2026-11-13).
  - *Dataset 1 (Machine A Unseen Discontinuous Impact Steps):* Under sharp rectangular $50\,\mathrm{kN}$ rock impact steps, the baseline linear constant-acceleration KF acts as a low-pass filter, exhibiting transient step-edge lag ($R^2 = 0.1755$, $\text{NRMSE} = 26.23\%$, `STEP_LAG_WARN`). M5 Adaptive Innovation Gate eliminates this lag, achieving $R^2 = 0.8920$, $\text{NRMSE} = 9.49\%$ (a $63.8\%$ error reduction).
  - *Dataset 2 (Machine B TZ WK-20 Deployment Target at NCL Amlohri):* On the $\sim 800\,\mathrm{t}$ shovel under $350\,\mathrm{kN}$ digging resistance, baseline KF-GMO achieves $R^2 = 0.9094$, $\text{NRMSE} = 10.46\%$; adaptive gate achieves $R^2 = 0.9941$, $\text{NRMSE} = 2.68\%$.
  - *Critical Machine B Parameter Caveat:* Machine B parameters ($M_d = 56,500\,\mathrm{kg}$, $I_{zz} = 305,000\,\mathrm{kg}\cdot\mathrm{m}^2$) are engineering priors scaled from Rasuli (2014) based on the Amlohri OEM specification sheet, NOT verified OEM CAD data. True field validation requires parameter identification on site at NCL Amlohri.

---

## Strict Machine Separation

| Machine Property | Machine A (Benchmark) | Machine B (Deployment Target) |
| :--- | :--- | :--- |
| **Model** | P&H 2100BL | TZ WK-20 |
| **Gross Working Weight** | $\sim 120\,\mathrm{t}$ class | $\sim 800\,\mathrm{t}$ class |
| **Dipper Capacity** | $\sim 10\,\mathrm{m}^3$ ($15\,\mathrm{t}$ payload) | $20\,\mathrm{m}^3$ ($40\,\mathrm{t}$ payload) |
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
