# MS02P08B Pre-Registration Specification (Revision 2)
# Closed-Loop Simscape Plant vs. M2 Dynamic Residual Estimator under Parameter Mismatch

**Status: FROZEN SPECIFICATION.**
**Date:** 2026-11-18  
**Project:** IIT/SRIC/R/AEH/2026/104 (CMPDI MT-189) — IIT Kharagpur  
**Model & Runner:** `M2/model/M2_TorqueDriven_Validation.slx`, `M2/scripts/build_M2_TorqueDriven_Validation.m`, `M2/scripts/run_MS02P08B_simscape.m`  
**Predictor Scripts:** `M2/experiments/08_held_out/rev2/ms02p08b_predict.py`, `M2/experiments/08_held_out/rev2/ms02p08b_closedloop_check.py`  

---

## Historical Context & Supersession Notice

- **Revision 1 (2026-11-09 & 2026-11-16):** Evaluated `M2_TorqueDriven.slx` via forward-Euler ODE script integration (`build_M2_TorqueDriven.m`, archived in `M2/experiments/08_held_out/M2_p08_torque_driven_results.mat`). While mathematically sound, the plant equations were integrated within the test script using matched parameters; the Simscape `.slx` model itself was not simulated with torque inputs under parameter mismatch.
- **Revision 2 (2026-11-18, This Document):** Clones the verified Simscape physical model (`M2_LoadInjection.slx`), converts revolute and prismatic joints to torque/force actuation with independent computed-torque + PID control loops, injects external load, and subjects the resulting physical plant to the M2 dynamic residual estimator under $+5\%$ mass, $+10\%$ inertia, and $+50\,\text{mm}$ center-of-gravity mismatch.

---

## 1. Scientific Question

Given drive torques $\tau_{\mathrm{cmd}}$ and sensed kinematics $(q, \dot{q}, \ddot{q})$ from an **independently solved, forward-dynamic Simscape plant** (`ode15s` numerical solver) under closed-loop torque control:
1. Does the M2 inverse-dynamics residual estimator
   $$\hat{Q} = \tau_{\mathrm{model}}(\hat{P}; q, \dot{q}, \ddot{q}) - \tau_{\mathrm{cmd}}$$
   recover the true generalized bail load $Q_{\mathrm{true}}$ to solver precision when $\hat{P} = P_{\mathrm{truth}}$ (algebraic identity)?
2. Under pre-registered physical parameter mismatch $\hat{P} \neq P_{\mathrm{truth}}$ ($+5\%\,M_d$, $+10\%\,I_{zz}$, $+50\,\text{mm}\,c$), does the estimated vertical payload force $\hat{F}_y$ bias by the exact pre-registered amount predicted before running the simulation?

---

## 2. Hypotheses

- **H1 (Structural Exactness):** With $\hat{P} = P_{\mathrm{truth}}$, the control residual $\hat{Q}_c - Q_{\mathrm{true}}$ vanishes to numerical solver precision ($\le 10^{-6}$ relative error), confirming that the Simscape plant dynamics, joint sensor transforms, and smooth Coulomb friction model are structurally consistent with the M1/M2 equations of motion.
- **H2 (Mismatch Predictability):** Under parameter mismatch $\hat{P} = \{1.05\,M_d,\, 1.10\,I_{zz},\, c + 0.05\,\text{m}\}$, the mean bias and RMS error of the recovered payload force $\hat{F}_y$ match the pre-registered analytical predictions within $10\%$.

---

## 3. Experimental Setup & Simulation Parameters

| Item | Specification / Value | Scientific Rationale / Source |
|---|---|---|
| **Simscape Plant** | `M2_TorqueDriven_Validation.slx` | Programmatically cloned from `M2_LoadInjection.slx` via `build_M2_TorqueDriven_Validation.m`. Source model untouched. |
| **Truth Parameters** | $M_d = 54,300\,\text{kg}$, $c = -1.32\,\text{m}$, $I_{zz} = 287,900\,\text{kg}\cdot\text{m}^2$<br>$f_{v3} = 3,800\,\text{N}\cdot\text{m}\cdot\text{s/rad}$, $f_{c3} = 340\,\text{N}\cdot\text{m}$<br>$f_{v4} = 1,473\,\text{N}\cdot\text{s/m}$, $f_{c4} = 480\,\text{N}$<br>$g = 9.80665\,\text{m/s}^2$, $x_{\mathrm{ref}} = 1.50\,\text{m}$, $\epsilon_c = 10^{-3}$ | Rasuli (2014) Table II; `claims.md` rows 9–16. Read back directly from blocks (Gate G0). |
| **Coulomb Friction** | $f_c \tanh(\dot{q}/\epsilon_c)$, $\epsilon_c = 10^{-3}$ | Continuous regularized Coulomb representation; identical in Simscape plant and M1/M2 estimator. |
| **Actuation & Control** | Independent joint PID with gravity compensation:<br>$\omega_c = 15\,\text{rad/s}$, $K_p = J\omega_c^2$, $K_d = 2J\omega_c$, $K_i = K_p\omega_c/4$<br>Integrators pre-loaded with static gravity torque. | Ensures tight trajectory tracking ($|\Delta q_3| < 0.02^\circ$) while allowing natural dynamic deflection. |
| **"Measured" Torque** | Controller output $\tau_{3,\mathrm{cmd}}, F_{4,\mathrm{cmd}}$ | Sensed drive effort before internal joint friction. |
| **Injected Bail Load** | $F_y(t) = F_{0,y} \sin(\omega_F t)$, $F_{0,y} = 100\,\text{kN}$, $\omega_F = 2\pi/9 \approx 0.6981\,\text{rad/s}$ ($T_F = 9.0\,\text{s}$). $F_x = 0$. | Applied at bail frame (World $+Y$), identical to M2 benchmark load. |
| **Kinematic Cases** | **Case E0 (Digging envelope):**<br>$q_3 = 45 \pm 15^\circ$, $d_4 = 8.75 \pm 2.00\,\text{m}$, $\omega = \pi/10 \approx 0.3142\,\text{rad/s}$ ($T = 20\,\text{s}$, exactly 3 integer cycles in 60 s).<br>**Case E1 (Field lifting envelope):**<br>$q_3 = -12 \pm 9^\circ$, $d_4 = 7.325 \pm 0.275\,\text{m}$, $\omega = 0.51\,\text{rad/s}$ ($T = 12.32\,\text{s}$, 4.87 cycles; peak velocity $0.080\,\text{rad/s}$). | E0 matches heavy digging stroke geometry; E1 matches authentic Rasuli (2014) Fig. 7 field lifting telemetry. |
| **Simulation Timing** | Total duration $T_{\mathrm{stop}} = 65.0\,\text{s}$.<br>Transient trim $T_{\mathrm{trim}} = 5.0\,\text{s}$.<br>Evaluated window: $[5.0,\, 65.0]\,\text{s}$ ($60.0\,\text{s}$ steady-state, exactly 1 minute). | Closed-loop dominant poles at $-4.35 \pm 4.55j\,\text{s}^{-1}$ ($\tau \approx 0.230\,\text{s}$). At $t = 5.0\,\text{s}$ ($21.7\tau$), transient envelope residual is $e^{-21.7} = 3.58 \times 10^{-10}$, strictly below Simscape solver numerical truncation ($\mathrm{AbsTol} = 10^{-9}$). |
| **Numerical Solver** | `ode15s` (stiff/multibody), $\mathrm{RelTol} = 10^{-8}$, $\mathrm{AbsTol} = 10^{-9}$, $\mathrm{MaxStep} = 10^{-3}\,\text{s}$. | Precision integration avoiding numerical damping artifacts. |
| **Signal Logging** | Internal log at $300\,\text{Hz}$ (`Ts_log = 1/300 s`). Decimated $30\,\text{Hz}$ output (`S30`) exported for M3 observer. | Common uniform time grid for all kinematic and kinetic channels. |
| **Estimator Mismatch** | $\hat{M}_d = 1.05\,M_d$ ($+5\%$), $\hat{I}_{zz} = 1.10\,I_{zz}$ ($+10\%$), $\hat{c} = c + 0.05\,\text{m}$ ($+50\,\text{mm}$). Friction unchanged. | Represents unmodeled payload carry-back, dipper wear, and center-of-gravity migration. |

---

## 4. Verification Gates (Strict Pre-Registration)

All gates must evaluate to `PASS`. Tolerances are frozen prior to simulation:

| Gate | Criterion Description | Evaluation Method | Pass Tolerance |
|---|---|---|---|
| **G0** | **Plant Block Readback** | Read back block parameters directly from `.slx` system XML/handles; verify empty Model Workspace. | Parameter identity $\le 10^{-9}$ relative (viscous friction $\le 10^{-4}$ due to degree/radian conversion). |
| **G1** | **Applied Load Verification** | Peak realized bail force in steady-state window. | $|F_{\mathrm{peak}} - F_{0,y}| / F_{0,y} \le 0.5\%$. |
| **G2** | **Trajectory Realization** | Realized joint midpoints and half-ranges vs. reference targets. | $q_3$ mid $\le 0.5^\circ$, $d_4$ mid $\le 0.02\,\text{m}$; half-ranges within $5\%$. |
| **G3** | **Joint Actuator Wiring** | Verified torque balance: $\tau_{\mathrm{joint}} = \tau_{\mathrm{cmd}} - \tau_{\mathrm{Coulomb}}$. | Relative residual $\le 10^{-6}$. |
| **G4** | **Structural Identity (Control)** | Estimator residual with $\hat{P} = P_{\mathrm{truth}}$ vs. true generalized load $Q_{\mathrm{true}}$. | $\max|\hat{Q}_{3,c} - Q_3| / \max|Q_3| \le 10^{-6}$; $\max|\hat{Q}_{4,c} - Q_4| / F_{0,y} \le 10^{-6}$. |
| **G5** | **Mismatch Predictability** | Saddle channel recovered $F_y$ bias and RMS error vs. pre-registered values. | Within $10\%$ of pre-registered predictions (Table in Section 5). |

---

## 5. Pre-Registered Predictions (Computed 2026-11-18)

Predictions evaluated across the 60.0 s steady-state window ($t \in [5.0,\, 65.0]\,\text{s}$) via `ms02p08b_predict.py` and cross-verified via closed-loop ODE integration `ms02p08b_closedloop_check.py`:

| Case | Trajectory Characteristics | Pre-Registered $F_y$ Bias | Pre-Registered $F_y$ RMS Error | Mean $\Delta Q_3$ Residual |
|---|---|---|---|---|
| **E0** | $q_3 \in [30^\circ, 60^\circ]$, $d_4 \in [6.75, 10.75]\,\text{m}$, $\omega = \pi/10$ | **$+21.90\,\mathrm{kN}$** | **$21.90\,\mathrm{kN}$** | $+152.0\,\mathrm{kNm}$ |
| **E1** | $q_3 \in [-21^\circ, -3^\circ]$, $d_4 \in [7.05, 7.60]\,\text{m}$, $\omega = 0.51$ | **$+21.32\,\mathrm{kN}$** | **$21.32\,\mathrm{kN}$** | $+182.6\,\mathrm{kNm}$ |

*Physical Note:* The mismatch bias is dominated by unmodeled static gravity moment ($\Delta M_d \cdot g \cdot r \cos q_3 + M_d \cdot g \cdot \Delta c \cos q_3$). Because this gravitational moment is in-phase with handle reach and dipper pitch, the resulting force error $\Delta F_y = \Delta Q_3 / (s \cos q_3)$ is strictly positive and virtually constant across the stroke, yielding $\mathrm{RMS} \approx \mathrm{Bias}$.

---

## 6. Failure Interpretation Protocol

1. **Failure of G0–G3:** Indicates a model construction or converter wiring failure. Simulation halts, model XML is inspected, builder script is revised, and build receipt is updated. No experimental claims are made.
2. **Failure of G4:** Indicates a fundamental structural discrepancy between Simscape multibody physics and the M1 analytical equations (e.g., sign conventions, coordinate frame origins, or friction implementation). Must be treated as a scientific finding and resolved prior to evaluating mismatch.
3. **Pass G4, Failure of G5:** The structural identity holds, but the parameter sensitivity deviates from analytical predictions. The deviation must be reported truthfully; no retuning of estimator parameters is permitted.

---

## 7. Scope & Claim Boundaries

- **What Revision 2 Proves:** Proves that the M2 dynamic residual estimator correctly recovers generalized loads from an independently solved Simscape multibody forward-dynamic plant under closed-loop torque control, and quantitatively confirms the exact bias sensitivity under parameter mismatch.
- **What Revision 2 Does Not Claim:** Does not claim validation against physical field telemetry (Milestones 5/6), flexible boom/stick structural modes, hoist rope compliance, or motor electrical transients. Terminology in publications must state: *"Simulation validation under parameter mismatch (independent Simscape plant)"*.

---

## 8. Output Archival & Non-Deletion Rule

All simulation runs write timestamped archives:
- MAT file: `M2/experiments/08_held_out/rev2/MS02P08B_simscape_<timestamp>.mat`
- Execution Log: `M2/experiments/08_held_out/rev2/MS02P08B_log_<timestamp>.txt`
- Exported Data: Sub-sampled $30\,\text{Hz}$ signals (`S30`) preserved within the MAT archive for direct consumption by the M3 discrete observer.
- In accordance with the Non-Deletion Historical Protocol, all previous Revision 1 data and scripts remain permanently preserved.

---

## Amendment 1 (2026-11-18, recorded before the first full run)

**What changed:** the builder now sets the model's output option to `RefineOutputTimes` (`build_M2_TorqueDriven_Validation.m`, solver block). The model is rebuilt with `build_M2_TorqueDriven_Validation('Overwrite', true)` and committed together with its new build receipt.

**Why:** the model inherited `OutputOption = SpecifiedOutputTimes` with `OutputTimes = 0:0.01:30` from `M2_LoadInjection.slx`. The smoke test (`T_stop = 2 s`) then stops with: *"Output times for the block diagram 'M2_TorqueDriven_Validation' must be within the specified simulation start (0.0) and stop (2.0) times."* (MATLAB console, 2026-11-18 13:04, run stamp 20261118_130401, recorded DIRTY, nothing saved). In the 65 s run the same setting would cap `tout` at 30 s. A hand edit of the model at 12:41 made the same change without a receipt; it was reverted with `git checkout` and is replaced by this logged rebuild.

**What did not change:** plant parameters, controller, cases E0/E1, gates G0–G5, tolerances and the pre-registered predictions (§4–§5). No simulation result had been produced or inspected when this amendment was made.
