# M3 — 30 Hz Discrete Momentum Observer Suite

## Status & Executive Summary

STATUS DEFINITIONS AND MILESTONE STATE (AUDITED 2026-11-17 / 2026-11-18):

- **Status:** **Code Check (Model-Matched Simulation)**.
- **Scientific Goal:** Estimate instantaneous external rock-cutting load torque $\tau_L(t) = Q_{\mathrm{ext}}(t) = -\mathbf{J}^\top \mathbf{F}_{\mathrm{dig}}$ on electric cable shovel drives at the native industrial drive logging rate ($30\,\mathrm{Hz}$, $T_s = 33.333\,\mathrm{ms}$) without requiring joint acceleration sensing $\ddot{q}$.
- **Core Verification Finding:** The discrete observer difference equations, exact First-Order Hold (FOH) matrix transitions, and 3-state linear-trend Kalman filter implementations are mathematically verified in code. However, in `build_M3_DiscreteObserver_30Hz.m`, the plant actuator torque is synthesized using the forward analytical dynamics equations with parameters $\mathbf{P}$ identical to those used by the observer. No Simscape simulation is executed during this test suite. In accordance with Research Integrity Rule 2 ("A model cannot validate itself"), M3 is classified as **Code Verification / Simulation Check**, pending independent closed-loop Simscape validation under parameter mismatch.
- **Update 2026-11-18 (MS03-IND1, run 20261118_160926 from commit 6f6ed82):** the same observer (`M3/scripts/m3_observer_30Hz.m`, identical to the archive) was run on the 30 Hz export of the torque-driven Simscape plant of MS02P08B rev 2, with true and with wrong parameters. True parameters: Fy error = the 50 ms lag only (rms 2.47 kN on a 100 kN load; 0.04 kN after accounting for the lag). Wrong parameters: the MS02P08B bias (+21.8 / +21.2 kN) is inherited unchanged. Status: **simulation check on independently integrated data (noise-free)**. Because plant and observer share the same equations, the numbers match the model-matched pre-check to ~3 decimals; noise, quantisation and unmodelled dynamics remain untested (M4). See claims.md row 'MS03-IND1'.

---

## Observer Architectures

Three observer branches are implemented and evaluated across the test suite:

1. **Branch A (Raw Generalized Momentum Observer — De Luca 2005):**
   Continuous-time momentum tracking:
   $$r(t) = k_O \left( p(t) - \int_0^t (h(q, \dot{q}) + r(\sigma))\, d\sigma - p(0) \right)$$
   where $p = \mathbf{D}(q)\dot{q}$ is generalized momentum, and $h(q, \dot{q}) = \tau_a - \mathbf{g}(q) - \mathbf{f}_v(\dot{q}) - \mathbf{f}_c(\dot{q}) + \mathbf{C}^\top(q, \dot{q})\dot{q}$.
   - Bandwidth: $k_O = 20.0\,\mathrm{s}^{-1}$ ($\tau = 50.0\,\mathrm{ms}$, cut-off $3.18\,\mathrm{Hz}$).
   - Forward Euler / ZOH discretization suffers severe phase lag and numerical drift under $30\,\mathrm{Hz}$ sampling.

2. **Branch B (Exact First-Order Hold Discrete GMO):**
   Transforms the scalar momentum filter $\dot{z} = -k_O z + u(t)$ with $u(t) = h(t) + k_O p(t)$ using exact FOH matrix exponentiation:
   $$z_{k+1} = \Phi z_k + \beta_0 u_k + \beta_1 u_{k+1}$$
   where:
   $$\Phi = \exp(-k_O T_s) = 0.513417$$
   $$\beta_1 = \frac{1 - \Phi(1 + k_O T_s)}{k_O^2 T_s} = 0.013506$$
   $$\beta_0 = \frac{1 - \Phi}{k_O} - \beta_1 = 0.010823$$
   $$\hat{\tau}_{L,k} = k_O (p_k - z_k)$$
   Eliminates the $26\,\mathrm{kNm}$ ZOH discretization artifact without requiring continuous sub-step integration.

3. **Branch C (3-State Linear-Trend Kalman-Filtered GMO):**
   State vector $\mathbf{x}_k = [p_k; \tau_{L,k}; \dot{\tau}_{L,k}]^\top$ with constant-slope disturbance assumption:
   $$\mathbf{x}_{k+1} = \begin{bmatrix} 1 & T_s & \frac{1}{2}T_s^2 \\ 0 & 1 & T_s \\ 0 & 0 & 1 \end{bmatrix} \mathbf{x}_k + \begin{bmatrix} T_s \\ 0 \\ 0 \end{bmatrix} h_k$$
   - Measurement equation: $y_k = p_k = \mathbf{D}(q_k)\dot{q}_k$.
   - State-dependent measurement noise covariance:
     $$R_k(d_4) = \left( D_{11}(d_4) \cdot \sigma_v \right)^2$$
     which dynamically scales with the $4.75\times$ dipper inertia variation across crowd stroke $d_4$.

---

## Verified Archive Results (`M3_discrete_30Hz_verification.mat`)

Audited from the saved `.mat` archive generated on 2026-11-16:

| Test Case | Condition | Branch B (Plain FOH) | Branch C (3-State KF-GMO) | Honest Comparison / Key Finding |
|---|---|---|---|---|
| **Case 1: ZERO_LOAD** | $F_{\mathrm{dig}} = 0$, friction active | $53.47\,\mathrm{Nm}$ (Branch A) | $1.36\,\mathrm{Nm}$ (saddle), $0.07\,\mathrm{N}$ (crowd) | Numerical zero-load floor confirmed under matched parameters. |
| **Case 2: SIN_100kN** | $100\,\mathrm{kN}$, $1/9\,\mathrm{Hz}$ harmonic load | $\mathrm{RMSE} = 16.66\,\mathrm{kNm}$ | $\mathrm{RMSE} = 16.26\,\mathrm{kNm}$ | **$2.4\%$ reduction** in harmonic lag error (the legacy $86.5\%$ / $120.82\,\mathrm{kNm}$ claim was from an older 2-state filter run). Saddle lag coverage is $75.37\%$ (reconciling legacy $100\%$ claim). |
| **Case 3: NOISY_RESOLVER** | $3\,\mathrm{mrad/s}$ velocity noise | $\mathrm{RMSE} = 193.15\,\mathrm{kNm}$ | $\mathrm{RMSE} = 36.10\,\mathrm{kNm}$ | **$81.3\%$ noise reduction**; dynamic 2-sigma coverage is $97.47\%$. |
| **Case 4: STEP_50kN** | $50\,\mathrm{kN}$ sharp step load | Dynamic lag observed | $250\,\mathrm{ms}$ transient edge lag | Linear constant-acceleration KF acts as low-pass filter on rock impact steps; directly motivates M5 adaptive innovation gating. |
| **Case 5: WK20_HARMONIC** | Machine B parameter set | Framework verified | Framework verified | Evaluates heavy shovel scaling; dedicated Machine B archive generation pending. |

---

## Confidence Bound Separation

The suite establishes strict separation between distinct physical and algorithmic error sources:
- **Friction Uncertainty Bound:** $\pm f_{c3} = \pm 380\,\mathrm{Nm}$. Note that $\text{CovFricPct} = 100\%$ on noiseless runs is mathematically automatic because $f_{c3}$ is $\approx 27,000\times$ larger than the noiseless residual floor ($0.0136\,\mathrm{Nm}$).
- **Filter Phase Lag:** Fixed-gain Branch B exhibits $50\,\mathrm{ms}$ delay, resulting in $\text{CovRawPct} = 0.80\%$ under rapid harmonic digging.
- **Composite Dynamic Bound:** $f_c + 2\sigma_{P,k}$ captures $97.47\%$ of observed errors under realistic $3\,\mathrm{mrad/s}$ velocity noise.

---

## Traceability & Next Steps

1. **Current Code Status:** Standalone discrete execution verified via `M3/scripts/build_M3_DiscreteObserver_30Hz.m`.
2. **Unified Simscape Validation:** Planned for execution in Phase 3 against a compliant Simscape Multibody plant with deliberately perturbed estimator parameters to eliminate the "tested against itself" limitation.
