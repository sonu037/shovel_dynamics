# Research Log — Shovel Dynamics (Project IIT/SRIC/R/AEH/2026/104)

JRF: Suhail Majeed Sheikh · PI: Dr. Sunita Das Mishra · Dept. of Mining Engineering, IIT Kharagpur
Objective: recover instantaneous load torque on mining shovel drives from motor
signals alone, via τ_load(t) = τ_measured(t) − τ_model(t), for payload estimation
and health management. Roadmap: validate 2-DOF → 4-DOF → digging forces → PINN residual.

Rules for this log:
- One entry per work session. Write it BEFORE closing MATLAB.
- Format: What changed / Key result / Next step / Open questions.
- Numbers get a source. Files get a version. Decisions get a reason.

---

## 2026-07-04 — First forward validation attempt (pre-log, reconstructed)

**What was done:** Built 2-DOF Simscape model (ShovelSimulator.slx): revolute
saddle joint q3 + prismatic crowd d4, sine-driven motion, sensed tau_hoist.
Wrote first validation script comparing Simscape torque against hand-derived
point-mass Euler-Lagrange formula: τ = m(d_eff²·q̈ + 2·d_eff·ḋ·q̇ + g·d_eff·cos q).

**Key result:** Curves tracked in shape but with systematic ~9–18% offset;
predicted above measured everywhere.

**Next step:** Root-cause the offset instead of accepting "roughly matches."

---

## 2026-07-06 (session 1) — Model dissection via raw .slx XML

**What was done:** Unpacked ShovelSimulator.slx (it is a zip of XML) and read
every block parameter directly, instead of trusting assumptions.

**Findings (all verified in XML, not from memory):**
1. Solver = VariableStepAuto → the script's linspace() time vector was WRONG.
   Fix: always use out.tout.
2. Dipper Handle brick: mass 90,300 kg, COM [−5 0 0], custom Izz = 839,000 kg·m²
   → the point-mass formula was missing the Izz·q̈ parallel-axis term.
3. Rigid Transform between joints carries a 1.5 m translation → lever arm is
   not simply d−5.
4. Simulink-PS input converters filter with 2nd-order, τ=0.05 s → startup
   transient at t<0.5 s is expected, must be trimmed.
5. Revolute joint already had SenseAcceleration=on (unused 'b' port).

**Decisions:** Rewrote script (validate_and_identify.m) with: tout time base,
gradient() fallback for accelerations, Izz term, startup trim, and the
linear-in-parameters identification Y·θ=τ with θ=[m, mc, mc²+Izz]
(Tafazoli-1999/Rasuli-2014 regrouping trick).

---

## 2026-07-06 (session 2) — The axis bug hunt (three geometry bugs found)

**Symptom:** Measured torque only ±35 kNm; expected ~4.4 MNm from m·g·r alone.
R² catastrophically negative. Identification returned negative mass/Izz.

**Diagnosis chain (keep this — it is a thesis-methodology exhibit):**
- g=0 test: torque UNCHANGED with gravity off ⇒ gravity contributed nothing.
- Torque waveform at 2× excitation frequency ⇒ pure Coriolis signature
  (velocity×velocity products double frequency).
- ⇒ Revolute spin axis was VERTICAL (Simscape default Z ∥ gravity): the
  mechanism was a horizontal carousel; gravity has no moment about a
  vertical axis. Confirmed by model tree label "Rz".
- After first fix attempt, identification "fit perfectly with wrong params":
  R²_fit=0.9999 with effective m·r = 70,672 kg·m ⇒ r_eff ≈ 1.30 m, CONSTANT.
  Only constant ~1.3 m in the model: |1.5 − 2.82| = 1.32 m ⇒ the prismatic
  joint was sliding ALONG the spin axis (axial), not radially. Lesson:
  when least squares fits perfectly with wrong parameters, it is telling
  you the true geometry of the system actually built.

**Fix (model v4, "tilt gravity not the mechanism"):**
- Rigid Transform1 rotation reverted to None.
- Middle Rigid Transform: rotation StandardAxis +Y 90° → slide made radial.
- Dipper COM moved to [0 0 −2.82] (along slide axis); inertia to [Izz 0 0]
  (axis parallel to spin).
- Gravity set in-plane: [0 −9.80665 0]. Lever arm becomes r = d4 − 1.32.
- One remaining sign flip in sensed torque fixed in script (tau = +out.tau_hoist).

**Parameter provenance decision:** replaced the arbitrary built-in body values
(90,300 / −5 / 839,000 — traceable to NO paper; checked Frimpong 2004/2006/2008
and all project docs) with Rasuli 2014 Table II (dynamic case):
Md=54,300 kg, L_COG=2.82 m, Izz3=287,900 kg·m², fv3=3,800 N·m·s/rad,
fc3=340 N·m, fv4=1,473 N·s/m, fc4=480 N. NOTE: I initially misread Table II
(swapped fv3/fc3 columns); caught by checking the rendered table against the
plain-text extraction. Always verify table reads against the PDF image.

**Excitation fix:** Sine Wave1 (d4) amplitude 0.25→2 m; cond(Y) scaled
dropped from ~10^4–10^5 territory to 87 (healthy).

---

## 2026-07-07 (session 1) — External review + file consolidation

**What happened:** Independent review (Gemini) of the v3 script found 4 valid
issues: (1) stale comments contradicting the model, (2) isfield/isprop on
SimulationOutput always false → sensed accelerations never used, (3) friction
missing from τ_pred, (4) crowd equation logged but never validated.

**Verification of the review (do not skip this step for ANY external file):**
- Confirmed all four issues real.
- Confirmed their G7: logged q_d is in rad/s (q plotted ×180/π gave degrees),
  so θ4 is already N·m·s/rad; my old ×180/π printout was wrong.
- Confirmed their G8: the Rasuli paper contains NO "R²=0.942" anywhere —
  a citation I (Claude session) had fabricated from bad memory. Paper actually
  reports: payload 8,402 vs 8,420 kg actual, σ=104 kg (swing);
  8,340 kg, σ=579 kg (lifting). Struck the false number everywhere.
- Found one bug in THEIR file: corr() needs Statistics Toolbox → replaced
  with corrcoef() (base MATLAB).

**Adopted:** shovel_params.m as single source of truth (all numbers, one file)
+ validate_and_identify_4.m (fixed). HPINN paper (Fu 2022) re-read: confirms
R²/MAE/RMSE are the field-standard metrics (their Eq. 18) and its hybrid loss
J = α·J_lagrangian + η·J_energy is the Stage-4 blueprint.

---

## 2026-07-07 (session 2) — STAGE 1 CLOSED

**What was done:** Wired the two acceleration ports (revolute 'b' → q_3_ddot,
prismatic 'a' → d_4_ddot, each via PS-Simulink → To Workspace). Ran
validate_and_identify_4 with sensed accelerations.

**Key result (stage1_results.mat, archived in experiments/2026-07-07_stage1_final/):**
- Revolute equation:  R² = 1.000000, RMSE = 0 Nm
- Prismatic (crowd) equation (validated for the FIRST time): R² = 1.000000, RMSE = 0 N
- Identification: m=54,300.0±0.0 | c=−1.3200±0.0000 | Izz=287,900±0 |
  fv3=3,800.0±0.0 | fc3=0.0±0.0 — every parameter exact, machine precision.
- Residuals at 1e−13 scale (floating-point floor).

**Interpretation (honest version for the thesis):** with noise-free sensed
signals, the hand-derived equations are term-for-term identical to the
multibody solution; ALL previous residual (0.79% NRMSE, spikes at velocity
zero-crossings, 56% Izz bias) was numerical differentiation error. Zero-width
CIs are a simulation-only phenomenon; field data will reopen them — expected.

**Status vs objective:** objective NOT yet achieved — Stage 1 built and proved
the instrument (a model whose residual channel is clean to 1e−13). Novelty
begins at Phase C.

**Next step (Phase C):** inject a known time-varying external force at the
dipper in Simscape (unknown to the analytical model); demonstrate
τ_load(t) = τ_meas − Y·Φ̂ recovers it. Then Phase D: momentum observer
(no q̈ needed → deployable on real machines).

**Also produced:** CMPDI data-requirements document (meetings/) for the
2026-07-08 Coal India meeting — shovel phase only.

---

## TEMPLATE — copy for each new session

## YYYY-MM-DD — <one-line title>

**What was done:**

**Key result:**

**Decisions & why:**

**Next step:**

**Open questions:**
