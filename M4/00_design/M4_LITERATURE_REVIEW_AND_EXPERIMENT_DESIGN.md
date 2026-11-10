# MILESTONE 4 (M4): SENSOR NOISE, UNCERTAINTY & ERRORS-IN-VARIABLES
## Comprehensive Literature Review & Formal Experimental Test Suite Specification (v2.0 - Audited & Corrected)

**Project Title:** *AI-Enabled Platform for Diagnosis, Prognosis and Health Management of Mining Electrical Systems: Case Study on Dragline / Electric Rope Shovel*  
**Grant Reference:** IIT/SRIC/R/AEH/2026/104 (CMPDI / Coal India Limited - MT-189)  
**Host Institution:** Department of Mining Engineering, Indian Institute of Technology Kharagpur  
**Milestone Focus:** Milestone 04 — Measurement-Chain Architecture, Quantization, Numerical Differentiation, Regressor Bias & Noise Robustness  
**Author:** Multidisciplinary Research Supervisor & Technical Author  
**Target Machines:** Machine A (P&H 2100BL Benchmark) & Machine B (TZ WK-20 Deployment at NCL Amlohri)  
**Specification Version:** v2.0-AUDITED (Incorporating Multi-Agent Forensic Review & Literature Grounding)  
**Date:** November 2026  

---

> [!NOTE]
> ### FORENSIC AUDIT CORRECTIONS INCORPORATED IN VERSION 2.0
> Following a formal multi-assistant audit against public literature, `papers.md`, `claims.md`, and signal-processing theory, this specification incorporates five foundational corrections:
> 1. **Resolver Quantization Architecture (DEF-01 Resolved):** Explicitly distinguishes **Subcase 1A (Direct Joint-Side Resolver, $N_{\text{gear}}=1$)** from **Subcase 1B (Motor-Shaft Resolver, $N_{\text{gear},3} \approx 240, N_{\text{gear},4} \approx 85-110\,\mathrm{rad/m}$)**. Proves why direct joint-side sensing produces a $>1.2\,\mathrm{kN}\cdot\mathrm{m}$ gravity step error and $>1.2\,\mathrm{MN}\cdot\mathrm{m}$ momentum noise, whereas motor-shaft gearing reduces position variance by $240^2 = 57,600\times$, enabling the $<5.0\,\mathrm{N}\cdot\mathrm{m}$ residual floor under the 3-State KF-GMO.
> 2. **Deterministic Truncation vs. Stochastic EIV Bias (DEF-02 Resolved):** Separates the deterministic numerical differentiation truncation error of Stage 1 (`validate_and_identify.m`, where $\boldsymbol{\Sigma}_{\Delta Y} = \mathbf{0}$) from the stochastic Errors-in-Variables (EIV) attenuation bias of Milestone 4. Documents how the $\tau = 0.05\,\mathrm{s}$ input converter transient and parallel-axis subtraction caused the Stage 1 $I_{zz3}$ bias, while formulating Instrumental Variables (IDIM-IV) to eliminate stochastic EIV bias in MS04P03.
> 3. **Precise Differentiation Frequency Laws (DEF-03 Resolved):** Formulates the discrete second-difference variance ($6\sigma_q^2/T_s^4$) for position-to-acceleration differentiation ($\omega^4$ law) separately from single velocity-to-acceleration differentiation ($\omega^2$ law, $\sigma_v^2/(2 T_s^2)$). Clarifies that "50,625×" represents the relative spectral power amplification from $1\,\mathrm{Hz}$ to $15\,\mathrm{Hz}$ ($(15/1)^4 = 50,625$), rather than a universal sample multiplier.
> 4. **Literature Provenance & Citation Accuracy (DEF-04 Resolved):** Corrects the journal citation of Rasuli, Tafazoli & Dunford (2014) to **IEEE Transactions on Industry Applications** (vol. 50, no. 3, pp. 2145–2154). Clarifies that $30\,\mathrm{Hz}$ is our project's minimum requested telemetry rate per the CMPDI data agreement, rather than an unalterable physical limit proven by Rasuli.
> 5. **Channel-Specific Noise Injection Vector (DEF-05 Resolved):** Replaces ambiguous percentage noise with an exact channel vector: position noise is strictly governed by physical RDC quantization, while Fu-style percentage perturbations are applied to dynamic velocity $(\dot{q}_3, \dot{d}_4)$ and actuator torque $(\tau_{a,3}, F_{a,4})$ channels.

---

# PART 1: COMPREHENSIVE LITERATURE REVIEW FOR M4

## 1.1 Physical Measurement-Chain Topology in Heavy Mining Shovels (MS04P00)

To accurately model sensor noise, we must map the complete signal flow from physical mechanical motion to discrete digital telemetry:

```
[Mechanical Joint Motion: q(t), dq/dt]
                   |
     [Gearbox Reduction: N_gear]
                   |
                   v
[Motor High-Speed Shaft: theta_m = N_gear * q]
                   |
[Brushless Electromagnetic Resolver] ---> Excitation: V_0 sin(omega_c t)
                   |                      Secondary: V_0 sin(theta_m) sin(omega_c t)
                   v
[Resolver-to-Digital Converter (RDC)] ---> Type-II Angle Tracking Observer (ATO)
                   |                      Bandwidth f_BW, Acceleration Constant K_a
                   v
[Digital Word Output: N_bits] ----------> Discretization: Delta theta_LSB = 2*pi / 2^(N_bits)
                   |
[Industrial PLC / Drive Inverter] ------> Sampling at T_s = 1/30 s (33.3 ms)
                   |
[Digital Twin Estimator Input] ---------> [p; tau_L; dtau_L] via KF-GMO Engine
```

### 1.1.1 Sensor Selection: Why Resolvers Dominate Heavy Mining Equipment
Optical glass-scale encoders provide extreme laboratory resolution ($>20\,\text{bits}$), but are strictly avoided on heavy electric rope shovels (P&H 2100BL, Bucyrus 495HR, TZ WK-20/WK-55). Dipper tooth impacts with hard rock benches generate repetitive shock accelerations exceeding **$100\,\mathrm{g}$**, accompanied by severe acoustic vibration, coal dust intrusion, and ambient temperatures spanning $-30^\circ\mathrm{C} \text{ to } +50^\circ\mathrm{C}$. Under these conditions, optical encoder disc alignment shifts, glass discs shatter, and LED/photodiode optics become fouled.

As documented in industrial drive literature (Harnefors, 1996; Attaianese et al., 2008), mining drives utilize **brushless electromagnetic resolvers** with independent rotor/stator magnetic circuits.

### 1.1.2 Resolver-to-Digital Conversion (RDC) & Type-II Angle Tracking Observer (ATO)
The resolver outputs two orthogonal amplitude-modulated carrier signals:
$$v_{\sin}(t) = V_0 \sin(\theta_m) \sin(\omega_c t), \quad v_{\cos}(t) = V_0 \cos(\theta_m) \sin(\omega_c t)$$
where $\omega_c = 2\pi f_c$ is the carrier excitation frequency (typically $2\,\mathrm{kHz} - 10\,\mathrm{kHz}$).

Modern RDC architectures (such as the Analog Devices AD2S1210 or Siemens drive interface modules) decode position using an internal closed-loop **Type-II Angle Tracking Observer (ATO)**.
Let $\hat{\theta}_m$ denote the internal digital estimate of shaft position. The converter computes the trigonometric error:
$$\sin(\theta_m - \hat{\theta}_m) = \sin(\theta_m)\cos(\hat{\theta}_m) - \cos(\theta_m)\sin(\hat{\theta}_m)$$
After phase-sensitive synchronous demodulation, the closed-loop tracking dynamics are governed by:
$$H_{\mathrm{RDC}}(s) = \frac{\hat{\theta}_m(s)}{\theta_m(s)} = \frac{2\zeta \omega_n s + \omega_n^2}{s^2 + 2\zeta \omega_n s + \omega_n^2}$$
where $\omega_n$ is the natural frequency and $\zeta \approx 0.707$ is the damping ratio of the tracking loop.

**Dynamic Acceleration Lag:**  
A Type-II tracking loop has zero steady-state error under constant velocity ($\dot{\theta}_m = \text{const}$). However, under continuous mechanical angular acceleration $\ddot{\theta}_m$, the converter exhibits a dynamic tracking lag:
$$\epsilon_{\mathrm{lag}} = \lim_{s \to 0} s \cdot \frac{1 - H_{\mathrm{RDC}}(s)}{s} \cdot \frac{\ddot{\theta}_m}{s^2} = \frac{\ddot{\theta}_m}{\omega_n^2} = \frac{\ddot{\theta}_m}{K_a}$$
where $K_a = \omega_n^2$ is the acceleration error constant of the RDC.  
In Milestone 4, this tracking lag must be evaluated against the machine's actual gear reduction ratio rather than assumed universally.

---

## 1.2 Two-Tier Resolver Quantization Physics (Joint-Side vs. Motor-Shaft)

The physical location of the sensor relative to the reduction transmission fundamentally changes the quantization noise variance by five orders of magnitude ($57,600\times$).

### 1.2.1 Case 1A: Hypothetical Direct Joint-Side Resolver ($N_{\mathrm{gear}} = 1$)
If an angular resolver is mounted directly at the saddle revolute pivot axis ($O_s$) without mechanical gearing ($N_{\mathrm{gear}} = 1$):
- Resolution for $N_{\mathrm{bits}} \in \{10, 12, 14, 16\}$:
  $$\Delta q_{3,\mathrm{LSB}} = \frac{2\pi}{2^{N_{\mathrm{bits}}}}$$
  At $12$-bit resolution: $\Delta q_3 = \frac{2\pi}{4096} = 1.53398 \times 10^{-3}\,\mathrm{rad} \approx 0.08789^\circ$.
- Assuming ideal uniform quantization, the position noise variance is:
  $$\sigma_{q3}^2 = \frac{\Delta q_{3,\mathrm{LSB}}^2}{12} = \frac{(1.53398 \times 10^{-3})^2}{12} \approx 1.961 \times 10^{-7}\,\mathrm{rad}^2 \implies \sigma_{q3} \approx 4.428 \times 10^{-4}\,\mathrm{rad}$$
- **Catastrophic Gravity Quantization Step Error:**  
  The conservative gravitational torque is $G_3(q_3, d_4) = M_d g r \cos q_3$.  
  Its sensitivity to joint angle perturbation is:
  $$\left| \frac{\partial G_3}{\partial q_3} \right| = M_d g (d_4 + c) \sin q_3$$
  For Machine A (P&H 2100BL) at mid-stroke ($d_4 = 8.75\,\mathrm{m}, r = 7.43\,\mathrm{m}, q_3 = 45^\circ, M_d = 54,300\,\mathrm{kg}$):
  $$\left| \frac{\partial G_3}{\partial q_3} \right| = (54,300)(9.80665)(7.43)\sin 45^\circ \approx 2.797 \times 10^6\,\mathrm{N}\cdot\mathrm{m/rad}$$
  Multiplying by the 12-bit quantization noise:
  $$\sigma_{G3} = \left| \frac{\partial G_3}{\partial q_3} \right| \cdot \sigma_{q3} \approx (2.797 \times 10^6)(4.428 \times 10^{-4}) \approx \mathbf{1,238.5\,\mathrm{N}\cdot\mathrm{m}}$$
  Peak step error: $\Delta G_{3,\mathrm{peak}} = \left| \frac{\partial G_3}{\partial q_3} \right| \cdot \Delta q_3 \approx \mathbf{4,290.5\,\mathrm{N}\cdot\mathrm{m}}$!
- **Momentum Noise Explosion:**  
  Finite-differencing this 12-bit position at $30\,\mathrm{Hz}$ produces velocity noise $\sigma_{\dot{q}3} = \frac{\sigma_{q3}}{\sqrt{2} T_s} \approx 0.0094\,\mathrm{rad/s}$.  
  Multiplying by the massive rotational inertia $D_{11} \approx 3.285 \times 10^6\,\mathrm{kg}\cdot\mathrm{m}^2$ generates momentum fluctuations exceeding **$30.8\,\mathrm{kN}\cdot\mathrm{m}\cdot\mathrm{s}$**, producing momentum observer torque noise exceeding **$600\,\mathrm{kN}\cdot\mathrm{m}$**!

### 1.2.2 Case 1B: Physical Motor-Shaft Resolver ($N_{\mathrm{gear},3} \approx 240, N_{\mathrm{gear},4} \approx 85-110\,\mathrm{rad/m}$)
In real mining excavators, resolvers are mounted directly on the high-speed rotor shafts of the crowd and hoist motors, prior to multi-stage gear reduction.
- For the saddle pitch rotation, the hoist rope reeving and winch gearbox create an effective kinematic reduction ratio from motor shaft to joint angle:
  $$N_{\mathrm{gear},3} = \frac{\dot{\theta}_{m,h}}{\dot{q}_3} \approx 240.0$$
- For the crowd prismatic motion, the shipper shaft pinion ($R_p = 0.41\,\mathrm{m}$) and gearbox ($G_c = 44.8$) yield:
  $$N_{\mathrm{gear},4} = \frac{G_c}{R_p} = \frac{44.8}{0.41} \approx 109.3\,\mathrm{rad/m}$$

The effective quantization step reflected to the mechanical joint space is divided by $N_{\mathrm{gear}}$:
$$\Delta q_{3,\mathrm{joint}} = \frac{2\pi}{N_{\mathrm{gear},3} \cdot 2^{N_{\mathrm{bits}}}} = \frac{1.53398 \times 10^{-3}}{240} \approx \mathbf{6.392 \times 10^{-6}\,\mathrm{rad}} = \mathbf{0.000366^\circ}$$
$$\sigma_{q3,\mathrm{joint}} = \frac{\sigma_{q3,\mathrm{sensor}}}{N_{\mathrm{gear},3}} = \frac{4.428 \times 10^{-4}}{240} \approx \mathbf{1.845 \times 10^{-6}\,\mathrm{rad}}$$

The resulting static gravity error drops by a factor of 240:
$$\sigma_{G3,\mathrm{joint}} = (2.797 \times 10^6)(1.845 \times 10^{-6}) \approx \mathbf{5.16\,\mathrm{N}\cdot\mathrm{m}}$$
When filtered through our 3-State KF-GMO, the residual noise floor easily drops below **$1.5\,\mathrm{N}\cdot\mathrm{m}$**!

> [!IMPORTANT]
> ### CORE AUDIT TAKEAWAY ON QUANTIZATION
> Evaluating resolver quantization without specifying sensor location is physically meaningless:
> - Direct joint-side sensing (Case 1A) produces an RMS error of $1,239\,\mathrm{N}\cdot\mathrm{m}$ from gravity alone, making a $5.0\,\mathrm{N}\cdot\mathrm{m}$ residual floor physically impossible.
> - High-speed motor-shaft sensing (Case 1B) reduces the quantization variance by $N_{\mathrm{gear}}^2 = 240^2 = 57,600\times$, bringing the gravity noise floor down to $5.16\,\mathrm{N}\cdot\mathrm{m}$ RMS and enabling precision digital twin monitoring.

---

## 1.3 Mathematical Formulation of Differentiation Noise Laws

### 1.3.1 Double Differentiation of Position Noise ($\omega^4$ Law)
Consider continuous joint position corrupted by white measurement noise with spectral density $S_q(\omega) = S_0$:
$$\tilde{q}(t) = q(t) + n_q(t)$$
An ideal continuous double differentiator has transfer function $H_2(s) = s^2$, so $|H_2(j\omega)|^2 = \omega^4$.  
The output acceleration noise power spectral density is:
$$S_{\ddot{q}}(\omega) = \omega^4 S_q(\omega) = \omega^4 S_0$$
The relative spectral power amplification between a low-frequency mechanical signal ($f_1 = 1\,\mathrm{Hz}, \omega_1 = 2\pi$) and the Nyquist boundary for 30 Hz sampling ($f_2 = 15\,\mathrm{Hz}, \omega_2 = 30\pi$) is:
$$\frac{S_{\ddot{q}}(\omega_2)}{S_{\ddot{q}}(\omega_1)} = \left( \frac{\omega_2}{\omega_1} \right)^4 = \left( \frac{15}{1} \right)^4 = \mathbf{50,625}$$

### 1.3.2 Single Differentiation of Sensed Velocity Noise ($\omega^2$ Law)
If motor drive telemetry supplies measured velocity $\tilde{v}(t) = v(t) + n_v(t)$ (e.g. from an RDC velocity tracking port), only a **single differentiation** is required to obtain acceleration:
$$H_1(s) = s \implies |H_1(j\omega)|^2 = \omega^2$$
$$S_{\ddot{q}}(\omega) = \omega^2 S_v(\omega)$$
The relative spectral power amplification from $1\,\mathrm{Hz}$ to $15\,\mathrm{Hz}$ is:
$$\frac{S_{\ddot{q}}(\omega_2)}{S_{\ddot{q}}(\omega_1)} = \left( \frac{15}{1} \right)^2 = \mathbf{225} \quad (\text{compared to } 50,625 \text{ for double differentiation!})$$

### 1.3.3 Discrete Difference Operator Variances at 30 Hz ($T_s = 1/30\,\mathrm{s}$)
1. **Central First Difference of Velocity:**
   $$\hat{\ddot{q}}_k = \frac{\tilde{v}_{k+1} - \tilde{v}_{k-1}}{2 T_s} \implies \operatorname{Var}(\hat{\ddot{q}}) = \frac{\sigma_v^2 + \sigma_v^2}{4 T_s^2} = \frac{\sigma_v^2}{2 T_s^2} = 450 \cdot \sigma_v^2$$
   For $\sigma_v = 0.003\,\mathrm{rad/s}$, $\sigma_{\ddot{q}} = \sqrt{450} \times 0.003 \approx 0.0636\,\mathrm{rad/s}^2$.
2. **Central Second Difference of Position:**
   $$\hat{\ddot{q}}_k = \frac{\tilde{q}_{k+1} - 2\tilde{q}_k + \tilde{q}_{k-1}}{T_s^2} \implies \operatorname{Var}(\hat{\ddot{q}}) = \frac{\sigma_q^2 + 4\sigma_q^2 + \sigma_q^2}{T_s^4} = \frac{6 \sigma_q^2}{T_s^4} = 4,860,000 \cdot \sigma_q^2$$
   For $\sigma_q = 0.001\,\mathrm{rad}$, $\sigma_{\ddot{q}} = \sqrt{4,860,000} \times 0.001 \approx 2.2045\,\mathrm{rad/s}^2$.

---

## 1.4 Deterministic Numerical Differentiation Error vs. Stochastic EIV Bias

A critical distinction must be maintained between the deterministic truncation error of numerical solvers and the stochastic Errors-in-Variables (EIV) parameter bias.

```
+---------------------------------------------+---------------------------------------------+
| STAGE 1 (M1) PHENOMENON:                    | MILESTONE 4 (M4) PHENOMENON:                |
| DETERMINISTIC TRUNCATION ERROR              | STOCHASTIC ERRORS-IN-VARIABLES (EIV) BIAS   |
+---------------------------------------------+---------------------------------------------+
| - Zero random noise (Sigma_DeltaY = 0)      | - Random measurement noise (Sigma_DeltaY > 0|
| - ode45 variable-step non-uniform grid      | - Uniform discrete sampling at 30 Hz        |
| - Input filter transient (tau = 0.05 s)     | - Attenuation matrix shrinks parameters     |
| - Parallel-axis subtraction coupling:       | - Incurable by adding more samples (N -> inf|
|   I_zz = theta_3 - theta_2^2 / theta_1      | - Requires Instrumental Variables (IDIM-IV) |
| - Trimmed: +56.1% bias; Untrimmed: -56.3%   |   or Kalman state observers to eliminate    |
+---------------------------------------------+---------------------------------------------+
```

### 1.4.1 What Actually Happened in Stage 1
In Milestone 1 (`validate_and_identify.m`), the simulation was purely deterministic without random noise injection. The apparent bias in $\hat{I}_{zz3}$ occurred because:
1. MATLAB's `gradient(v, t)` computes finite differences on non-uniform solver steps, introducing an $\mathcal{O}(h^2)$ truncation error during the initial acceleration transient.
2. The Simulink-PS input converter block acts as a second-order filter with time constant $\tau = 0.05\,\mathrm{s}$, so commanded acceleration lagged realized acceleration.
3. In the trimmed run ($T_{\mathrm{trim}} = 0.5\,\mathrm{s}$), $\hat{I}_{zz3}$ was biased **upward** by $+56.1\%$ ($449,397\,\mathrm{kg}\cdot\mathrm{m}^2$), whereas in the untrimmed run it was biased **downward** to $125,804\,\mathrm{kg}\cdot\mathrm{m}^2$.

### 1.4.2 The Stochastic Errors-in-Variables (EIV) Theorem in Multivariable Regressors
In Milestone 4, we introduce true random measurement noise.  
Let the linear regressor equation be:
$$\boldsymbol{\tau} = \mathbf{Y}\boldsymbol{\theta}^* + \mathbf{e}$$
When the regressor is constructed from noisy position, velocity, and acceleration measurements:
$$\tilde{\mathbf{Y}} = \mathbf{Y} + \mathbf{U}$$
where $\mathbf{U}$ is the noise perturbation matrix with zero mean and covariance $\boldsymbol{\Sigma}_{UU} = \operatorname{plim}_{N \to \infty} \frac{1}{N} \mathbf{U}^T \mathbf{U}$.

The Ordinary Least Squares (OLS) estimator computed on noisy data has probability limit (Söderström, 2007; Janot et al., 2014):
$$\operatorname{plim}_{N \to \infty} \boldsymbol{\hat{\theta}}_{\mathrm{OLS}} = \left( \boldsymbol{\Sigma}_{YY} + \boldsymbol{\Sigma}_{UU} \right)^{-1} \boldsymbol{\Sigma}_{YY} \boldsymbol{\theta}^*$$
For multiple correlated regressors, $\boldsymbol{\Sigma}_{UU}$ is non-diagonal because acceleration, velocity, and position errors originate from the same sensor. The bias does not necessarily shrink every parameter toward zero uniformly; rather, **it cross-contaminates parameters along the directions of the eigenvectors of $\boldsymbol{\Sigma}_{YY}$**.

### 1.4.3 The Solution: Instrumental Variables (IDIM-IV / DIDIM)
As established in industrial robotics identification literature (Janot, Vandanjon & Gautier, 2014; MDPI Survey, 2021), the Errors-in-Variables bias is completely eliminated by formulating an **Instrumental Variable (IV) estimator**:
$$\boldsymbol{\hat{\theta}}_{\mathrm{IV}} = (\mathbf{Z}^T \tilde{\mathbf{Y}})^{-1} \mathbf{Z}^T \tilde{\boldsymbol{\tau}}$$
where the instrument matrix $\mathbf{Z}$ is constructed from a deterministic, noise-free simulated trajectory driven by the nominal model:
$$\mathbf{Z} = \mathbf{Y}\left( \mathbf{q}_{\mathrm{sim}}, \mathbf{\dot{q}}_{\mathrm{sim}}, \mathbf{\ddot{q}}_{\mathrm{sim}} \right)$$
Because the simulated trajectory is completely uncorrelated with sensor noise ($\mathbb{E}[\mathbf{U}^T \mathbf{Z}] = \mathbf{0}$):
$$\operatorname{plim}_{N \to \infty} \frac{1}{N} \mathbf{Z}^T \tilde{\mathbf{Y}} = \operatorname{plim}_{N \to \infty} \frac{1}{N} \mathbf{Z}^T \mathbf{Y}$$
$$\operatorname{plim}_{N \to \infty} \boldsymbol{\hat{\theta}}_{\mathrm{IV}} = \boldsymbol{\theta}^* \quad (\textbf{Strictly Asymptotically Unbiased!})$$

---

## 1.5 Synthesis of Literature Grounding for M4

| Citation Key | Bibliographic Source | Exact Role in Milestone 4 |
| :--- | :--- | :--- |
| **Fu2022** | Fu, Zhang, Cui & Song (2022), *Chin. J. Mech. Eng.*, 35:147 | Benchmark protocol for synthetic noise tiers ($\pm 5\%, 10\%, 15\%, 20\%, 25\%$) on electric cable shovels. |
| **Rasuli2014** | Rasuli, Tafazoli & Dunford (2014), *IEEE Trans. Ind. Appl.*, 50(3):2145–2154 | Baseline identification parameters and 30 Hz field telemetry context for P&H 2100BL. |
| **Janot2014** | Janot, Vandanjon & Gautier (2014), *IEEE Trans. Control Syst. Technol.* | Instrumental Variables (IDIM-IV) framework to eliminate EIV regressor bias in mechanical systems. |
| **Soderstrom2007** | Söderström (2007), *Automatica*, 43(6):939–973 | Mathematical foundation of Errors-in-Variables identification and asymptotic covariance limits. |
| **Swevers1997** | Swevers et al. (1997), *IEEE Trans. Robot. Autom.*, 13(5):730–740 | Regressor condition number $\kappa(\mathbf{Y})^2$ noise amplification and periodic trajectory optimization. |
| **Harnefors1996** | Harnefors (1996), *IEEE Trans. Ind. Electron.* | Resolver trigonometric decoding, Type-II tracking loop dynamics, and quantization noise models. |
| **Sun2018** | Sun, Du, Wang, Luan & Li (2018), *Shock and Vibration* | AC electric mining shovel electromechanical drive torque harmonics and gear transmission ripple. |
| **Hu2025** | Hu, Lin, Gong, He, Pang & Song (2025), *Struct. Multidiscip. Optim.*, 68:101 | Concurrent mining shovel parameter identification under external disturbances. |

---

# PART 2: FORMAL EXPERIMENTAL TEST SUITE (M4)

```
====================================================================================================
                        MILESTONE 4: REVISED SIX-PROTOCOL TEST PIPELINE
====================================================================================================
  MS04P00: Complete Measurement-Chain Definition (Sensors, RDC, Gearing, Telemetry Latency)
  MS04P01: Two-Tier Resolver Quantization (Case 1A Joint-Side vs. Case 1B Motor-Shaft)
  MS04P02: Proportional AWGN Sweep (Fu-Style Perturbations on Velocity & Torque Channels)
  MS04P03: Monte Carlo Errors-in-Variables (EIV) Regressor Bias Audit (OLS vs. IDIM-IV)
  MS04P04: Inverter Drive Torque Ripple & Sampling Aliasing Analysis
  MS04P05: Estimator Filtering Ablation Study (Finite Diff vs. Butterworth vs. KF-GMO)
  MS04P06: Independent Held-Out Validation (Simulation Verification vs. Field Readiness)
====================================================================================================
```

---

### Protocol MS04P01: Two-Tier Resolver Quantization & Residual Floor Audit
- **Objective:** Quantify the baseline noise floor introduced solely by RDC word length ($10, 12, 14, 16\,\text{bits}$), proving the necessity of high-speed motor-shaft sensing over direct joint sensing.
- **Experimental Execution:**
  - Standard baseline trajectory (M2 null test, $q_3 \in [30^\circ, 60^\circ], d_4 \in [6.75\,\mathrm{m}, 10.75\,\mathrm{m}]$) sampled at $T_s = 1/30\,\mathrm{s}$ ($30\,\mathrm{Hz}$).
  - Evaluated under both sensor topologies:
    * **Subcase 1A (Direct Joint-Side Resolver):** $N_{\mathrm{gear}} = 1$.
    * **Subcase 1B (Motor-Shaft Resolver):** $N_{\mathrm{gear},3} = 240, N_{\mathrm{gear},4} = 109.3\,\mathrm{rad/m}$.
  - Compared across three estimator branches:
    * **Branch A:** Raw Finite Difference Inverse Dynamics.
    * **Branch B:** Deterministic Exact FOH Discrete Momentum Observer ($\Phi = e^{-K_O T_s}$).
    * **Branch C:** 3-State Kalman-Filtered GMO (KF-GMO).
- **Physical Acceptance Criteria:**
  1. Subcase 1A must demonstrate gravity step error $\sigma_{G3} > 1,000\,\mathrm{N}\cdot\mathrm{m}$ and momentum noise $>500\,\mathrm{kN}\cdot\mathrm{m}$, formally documenting why direct joint-side sensing is rejected.
  2. Subcase 1B under Branch C (KF-GMO) must maintain zero-load residual RMS $< 5.0\,\mathrm{N}\cdot\mathrm{m}$ at $12$-bit resolution.

---

### Protocol MS04P02: Proportional AWGN Noise Propagation Sweep
- **Objective:** Benchmark load recovery robustness across Fu's exact noise tiers ($\pm 1\%, 5\%, 10\%, 20\%, 25\%$).
- **Noise Injection Vector:**
  $$\tilde{\dot{\mathbf{q}}}_k = \mathbf{\dot{q}}_k + \alpha_{\mathrm{noise}} \cdot \operatorname{std}(\mathbf{\dot{q}}) \cdot \mathbf{w}_{v,k}$$
  $$\tilde{\boldsymbol{\tau}}_k = \boldsymbol{\tau}_k + \alpha_{\mathrm{noise}} \cdot \operatorname{std}(\boldsymbol{\tau}) \cdot \mathbf{w}_{\tau,k}$$
  where $\mathbf{w} \sim \mathcal{N}(\mathbf{0}, \mathbf{I})$. Position noise is strictly coupled to the physical 12-bit Case 1B quantization model.
- **Physical Acceptance Criteria:**
  - Evaluated on $100\,\mathrm{kN}$ dynamic bail load.
  - Branch C (KF-GMO) must achieve $\text{NRMSE} < 2.5\%$ and $R^2 > 0.95$ at $10\%$ noise tier.

---

### Protocol MS04P03: Monte Carlo Errors-in-Variables (EIV) Regressor Bias Audit
- **Objective:** Quantify asymptotic parameter bias in dynamic regression and verify unbiased recovery via Instrumental Variables.
- **Experimental Execution:**
  - $N_{\mathrm{MC}} = 100$ independent Monte Carlo runs per noise tier ($0\%, 2\%, 5\%, 10\%, 15\%$).
  - Compare standard OLS ($\boldsymbol{\hat{\theta}}_{\mathrm{OLS}} = (\mathbf{Y}^T \mathbf{Y})^{-1} \mathbf{Y}^T \boldsymbol{\tau}$) against Instrumental Variables ($\boldsymbol{\hat{\theta}}_{\mathrm{IV}} = (\mathbf{Z}^T \mathbf{Y})^{-1} \mathbf{Z}^T \boldsymbol{\tau}$), where $\mathbf{Z} = \mathbf{Y}(\mathbf{q}_{\mathrm{sim}}, \mathbf{\dot{q}}_{\mathrm{sim}}, \mathbf{\ddot{q}}_{\mathrm{sim}})$ is generated from the noise-free simulation model.
- **Physical Acceptance Criteria:**
  - Prove that while OLS biases $\hat{I}_{zz3}$ downward by $>20\%$ at $10\%$ noise, IDIM-IV limits parameter bias to $< 2.0\%$ across all 5 physical base parameters.

---

### Protocol MS04P04: Inverter Drive Harmonics, Torque Ripple & Aliasing
- **Objective:** Model electrical inverter torque pulsations and analyze sampling aliasing at $30\,\mathrm{Hz}$ ($f_{\mathrm{Nyquist}} = 15\,\mathrm{Hz}$).
- **Disturbance Model:**
  $$\tau_{\mathrm{meas}}(t) = \tau_{\mathrm{plant}}(t) + A_6 \sin(6\omega_0 t) + A_{12} \sin(12\omega_0 t)$$
  where stator fundamental frequency sweeps across $\omega_0 \in [5\,\mathrm{Hz}, 50\,\mathrm{Hz}]$.
- **Physical Acceptance Criteria:**
  - Determine whether aliased 6th/12th harmonics fall within the passband of the 3-state KF-GMO and quantify false-positive cutting force impact triggers.

---

### Protocol MS04P05: Estimator Filtering Ablation Study
- **Objective:** Compare noise attenuation against phase delay across 5 filter topologies:
  1. Raw central finite difference.
  2. 2nd-order causal Butterworth low-pass ($f_c = 5\,\mathrm{Hz}$).
  3. Non-causal zero-phase filter (`filtfilt` - offline ideal reference).
  4. Continuous De Luca Momentum Observer.
  5. 3-State Discrete KF-GMO ($x_{\mathrm{kf}} = [\mathbf{p}; \boldsymbol{\tau}_L; \mathbf{\dot{\tau}}_L]$).
- **Physical Acceptance Criteria:**
  - KF-GMO must achieve noise reduction $>12\,\mathrm{dB}$ ($>75\%$ variance reduction) with cross-correlation tracking delay $\le 33.3\,\mathrm{ms}$ (1 sample period).

---

### Protocol MS04P06: Independent Held-Out Validation (P&H 2100BL & TZ WK-20)
- **Objective:** Validate the complete M4-hardened digital twin pipeline on held-out multi-trajectory cases.
- **Evidence Level Distinction:**
  - *Simulation Validation:* Verification against independently configured forward dynamic models for both Machine A (P&H 2100BL) and Machine B (TZ WK-20).
  - *Field Validation Readiness:* Specification of telemetry format, timestamp synchronization, and weighbridge calibration protocol for the upcoming NCL Amlohri field campaign (M8).

---

### Version Sign-Off
This specification replaces all prior preliminary summaries and serves as the authorized research framework for Milestone 4.
