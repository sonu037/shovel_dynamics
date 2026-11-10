# Paper 1: Analytical Identifiability Geometry and Two-Branch Discrete Generalized Momentum Observer for Electric Rope Shovels

**Target Journal:** *IEEE/ASME Transactions on Mechatronics* (TMECH) / *Mechanical Systems and Signal Processing* (MSSP)  
**Authors:** Suhail Majeed Sheikh, Sunita Das Mishra, Aurobinda Routray  
**Affiliations:** Department of Mining Engineering & Department of Electrical Engineering, Indian Institute of Technology Kharagpur, India  
**Project:** CMPDI / Coal India R&D Project `IIT/SRIC/R/AEH/2026/104` (MT-189)

---

## 1. Overview & Contents

This directory contains the self-contained publication package for **Paper 1**, presenting the mathematical breakthroughs, observer formulations, and dynamic simulation evidence across Milestones M1, M2, and M3:

- `paper1_tmech.tex`: Complete, peer-review-ready IEEEtran journal manuscript formatted with double-column layout, detailed mathematical derivations, theorems, proofs, and exhaustive quantitative tables.
- `references.bib`: Complete BibTeX bibliography containing all foundational and recent literature (Rasuli 2014, Fu 2022, De Luca 2003/2006, Wahrburg 2015, Hu 2017, Frimpong 2004/2008, Swevers 1997).
- `generate_paper1_figures.m`: Publication figure generation script that produces vector PDFs and 300-dpi PNGs directly from verified MATLAB `.mat` simulation archives.
- `figures/`: Vector and high-resolution figures referenced in the paper:
  - `fig1a_machine.pdf`: Shovel anatomy and major structural sub-assemblies.
  - `fig1b_abstraction.pdf`: 2-DOF planar kinematic abstraction with time-varying lever arm $r(t) = d_4(t) + c$.
  - `fig_identifiability_svd.pdf`: SVD subspace angle $\theta(d_4)$ showing the single-channel collinearity collapse ($\theta = 0.23^\circ$) vs. 2-channel restoration ($\theta_{\min} = 29.70^\circ$).
  - `fig_discrete_foh_tracking.pdf`: 30 Hz discrete First-Order Hold (FOH) observer tracking and strict Coulomb friction discrepancy bound ($|\hat{\tau}_A - \hat{\tau}_B| \le f_c$).
  - `fig_kfgmo_noise_suppression.pdf`: Configuration-adaptive 3-state KF-GMO under $3\ \mathrm{mrad/s}$ resolver jitter, demonstrating $81.3\%$ noise suppression and dynamic $2\sigma_P(d_4)$ confidence coverage ($97.47\%$).
  - `fig_torquedriven_deflection.pdf`: Forward torque-driven closed-loop compliance and deflection under cutting disturbance impact.

---

## 2. Compilation Instructions

### Option A: Overleaf (Recommended)
1. Zip the contents of `papers/paper1_identifiability_gmo/` (including `figures/`, `paper1_tmech.tex`, `references.bib`).
2. Open [Overleaf](https://www.overleaf.com), click **New Project** $\to$ **Upload Project**, and select the zip file.
3. Select `pdfLaTeX` as the compiler and compile `paper1_tmech.tex`.

### Option B: Local Command Line (Linux / macOS / Windows with TeX Live or MiKTeX)
```bash
cd papers/paper1_identifiability_gmo
pdflatex paper1_tmech
bibtex paper1_tmech
pdflatex paper1_tmech
pdflatex paper1_tmech
```

---

## 3. Key Quantitative Findings Reported in Paper 1

1. **Inertia Differentiation Bias:** Sensed vs. numerical velocity differentiation ($\dot{q} \to \ddot{q}$) introduces a $+56.1\%$ systematic bias on rotational inertia $I_{zz}$, providing the mathematical justification for acceleration-free momentum observers.
2. **Identifiability SVD Geometry:** Single-channel saddle observation exhibits near-total collinearity with dipper gravity leverage ($r = 0.9999919$, $\theta = 0.23^\circ$, condition number $\kappa_2 = 497.71$). Joint 2-channel observation $[\tau_3; \bar{s} F_4]$ with $\bar{s} = 10.0\ \mathrm{m}$ restores identifiability ($\theta_{\min} = 29.70^\circ$, $\kappa_2 = 3.77$).
3. **Discrete FOH Discrepancy Invariance:** Closed-form First-Order Hold (FOH) state update guarantees $100.00\%$ coverage of Coulomb friction uncertainty across velocity reversals ($|\hat{\tau}_{L,A} - \hat{\tau}_{L,B}| \le f_c$).
4. **3-State KF-GMO Harmonic Lag Elimination:** Upgrading from a 2-state random-walk to a 3-state linear-trend disturbance model ($[p; \tau_L; \dot{\tau}_L]$) reduces harmonic tracking RMSE from $120.82\ \mathrm{kN\cdot m}$ to $16.26\ \mathrm{kN\cdot m}$ (an $86.5\%$ error reduction).
5. **Deep Velocity Jitter Suppression:** Under $3\ \mathrm{mrad/s}$ resolver noise, the 3-state KF-GMO reduces saddle torque RMS chatter from $193.15\ \mathrm{kN\cdot m}$ down to $36.10\ \mathrm{kN\cdot m}$ (an $81.3\%$ noise reduction), with empirical error residual coverage of $97.47\%$ within the dynamic $2\sigma_P(d_4)$ bounds.
6. **Compliant Closed-Loop Validation:** In forward torque-driven simulation, cutting impacts induce joint deflections of $\Delta q_3 = 1.38^\circ$ and $\Delta d_4 = 84.1\ \mathrm{mm}$, with the discrete momentum observer achieving lag-compensated tracking $\mathrm{NRMSE} < 0.42\%$.
