# Learning Curriculum — Load-Torque Estimation for Shovel Health
### The complete study map for the PhD, keyed to milestones M1–M9
*File this at repo root as `LEARNING.md`. Work one module at a time. A module is DONE when you can answer its self-test questions out loud, from memory, to an imaginary examiner.*

---

## How to study (read once, follow forever)

1. **Derive, don't read.** For every equation module: close the book, derive it on paper, compare, find your error. Reading a derivation feels like understanding; reproducing it *is* understanding.
2. **Active recall beats re-reading.** The self-test questions in each module are the module. Answer them cold, out loud. If you can't, that's the signal — not a failure, a to-do.
3. **Spaced repetition.** When a module is done, re-answer its self-tests after 1 week, then after 1 month. Ten minutes each time. This is what makes it permanent.
4. **Everything lands in the repo.** Each module has a deliverable exercise that produces a script, a derivation PDF, or a log entry. Learning that leaves no artifact evaporates.
5. **Budget:** ~5 focused hours/week on the *current* module only. The curriculum follows the milestone order, so you always learn just-in-time, never just-in-case.

---

## Module 0 — Foundations audit (NOW, ~1 week)
*You have a B.Tech ME — this is a refresh, not a course.*

**Why:** every later module silently assumes these.
**Master:**
- Vector kinematics in rotating frames: position, velocity, acceleration of a point whose radius changes while the frame rotates (this IS your machine)
- Work–energy theorem; kinetic energy of a rigid body = ½mv²_COM + ½I_COM ω² (König's theorem)
- Parallel-axis theorem — and *why* it exists (COM motion carries kinetic energy)
- Small matrix algebra: rank, invertibility, what (AᵀA)⁻¹ means geometrically

**Self-test:**
1. A bead slides outward on a rotating rod. Write its acceleration in polar coordinates from memory. Which term is Coriolis? What direction does it point?
2. Why does swinging a longer pendulum of the same mass require more torque — in energy language?
3. If a matrix is nearly singular, what happens to the solution of Ax = b when b has small noise?

**Deliverable:** one handwritten page — the polar-coordinates acceleration derivation — scanned into `learning/M0_foundations.pdf`.
**Resource:** any undergrad dynamics text (Hibbeler/Meriam), Ch. on curvilinear motion + rigid-body KE.

---

## Module 1A — Lagrangian mechanics (supports M1 — mostly done, consolidate now)

**Why:** it produced your two equations; the examiner WILL probe it.
**Master:**
- Generalized coordinates: why (q₃, d₄) and not (x, y) — constraints vanish
- L = T − V; the Euler–Lagrange operator; where τᵢ comes from (generalized force = virtual-work conjugate)
- Why the formalism cannot produce friction (it's conservative) and the standard fix (append dissipative terms; Rayleigh dissipation function as the formal route)
- The origin of EVERY term in your two equations: parallel-axis inertia, the factor 2 in Coriolis (product rule on r²q̇₃), gravity's cos/sin split
- Manipulator form D(q)q̈ + C(q,q̇)q̇ + G(q) = τ; properties: D symmetric positive-definite, (Ḋ − 2C) skew-symmetric (you'll need this for M3!)

**Self-test:**
1. Derive the saddle equation from L, on paper, in under 20 minutes.
2. Where exactly does the factor 2 in 2M_d r ḋ₄ q̇₃ come from? (Point to the differentiation step.)
3. Why is gravity's moment arm r·cos q₃ and not r·sin q₃?
4. Why can't Coulomb friction come out of the Lagrangian? What is the Rayleigh dissipation function?
5. What physical statement is the skew-symmetry of (Ḋ − 2C)? *(Answer: internal forces do no net work — energy bookkeeping.)*

**Deliverable:** the full two-equation derivation, typed or scanned → `learning/M1A_lagrangian_derivation.pdf` (this becomes thesis Appendix material directly).
**Resources:** Spong, Hutchinson & Vidyasagar, *Robot Modeling and Control*, Ch. 7 (the canonical manipulator-dynamics treatment); Goldstein Ch. 1–2 if you want depth.

---

## Module 1B — Multibody simulation internals (supports M1, M6)

**Why:** "how do you know Simscape is right?" is a guaranteed committee question.
**Master:**
- How multibody engines work: constrained Newton–Euler equations + Lagrange multipliers, assembled from part definitions — i.e., a genuinely different mathematical route than your algebra (this is the non-circularity argument)
- Variable-step vs fixed-step solvers; local error tolerance; why you must use the solver's own time base (your first bug!)
- Joint actuation modes: motion-prescribed vs torque-prescribed (inverse vs forward dynamics)
- What a rigid transform is; frame conventions; where gravity lives

**Self-test:**
1. Explain to a non-expert why agreement between your algebra and Simscape is not circular.
2. Your torque signal looks jagged at a fixed 0.01 s grid but smooth on the solver time base. Why?
3. What's the difference between prescribing motion and sensing torque, vs prescribing torque and sensing motion? Which did Stage 1 use and why?

**Deliverable:** ½-page note in `research_log.md`: "How Simscape builds equations, and why our validation is non-circular" (you'll reuse it verbatim in the thesis).
**Resource:** MATLAB Simscape Multibody documentation, "How Simscape Models Work"; Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 1–3 (skim).

---

## Module 1C — System identification I: least squares (supports M1 — done, consolidate)

**Why:** identification is half your thesis; you must own it beyond the recipe.
**Master:**
- Linear-in-parameters: why regrouping θ = [m, mc, mc²+I_zz, f_v, f_c] makes a nonlinear model linear in the unknowns (Tafazoli's trick)
- LSE derivation from scratch: minimize ‖τ − Yθ‖² → normal equations → θ̂ = (YᵀY)⁻¹Yᵀτ
- Statistics of θ̂: Cov(θ̂) = σ̂²(YᵀY)⁻¹; 95% CIs; the delta method for derived quantities (how you got CI on I_zz from CI on mc²+I_zz)
- Condition number: what cond(YᵀY) means, why scaling columns matters, why richer excitation (your d₄ amplitude fix, 0.25→2 m) cures ill-conditioning
- Identifiability: why gravity (2.5 MN·m) dominates and inertia (10 kN·m) is fragile — signal-to-signal ratios inside Y
- Persistent excitation (the formal name for "the trajectory must wiggle enough")

**Self-test:**
1. Derive the normal equations from the cost function, on paper.
2. Why did widening the d₄ excitation fix the conditioning? Which columns of Y became distinguishable?
3. Your identification returns a perfect fit but wrong parameters. List the possible causes. *(Geometry mismatch — your 1.32 m X-ray story; insufficient excitation; parameter redundancy.)*
4. What exactly does a 95% CI of ±0.0000 mean in simulation, and why will field CIs never look like that?

**Deliverable:** a 30-line MATLAB script that identifies a 1-DOF pendulum's (m, ℓ, f_v) from simulated data with noise — written WITHOUT looking at your Stage-1 code → `learning/M1C_pendulum_id.m`.
**Resources:** Ljung, *System Identification: Theory for the User*, Ch. 1–2, 7; your own Rasuli 2014 + Tafazoli 1999 papers (read the identification sections again — they'll read differently now).

---

## Module 2 — Residuals and disturbance estimation (supports M2 — NEXT)

**Why:** M2 is literally this module.
**Master:**
- The residual as estimator: τ_load = τ_meas − τ_model; error budget (model error + measurement error + numerical error all land in the residual — why M1's exactness matters)
- Superposition: why the residual equals the injected force exactly in the linear-measurement case
- Force→torque mapping through geometry: a force at the dipper appears in the saddle equation as F·(moment arm) — you must know the Jacobian mapping J(q)ᵀF
- Acceptance testing: how to design the M2 experiment (inject known F(t), recover, quantify NRMSE, target 2–3%)

**Self-test:**
1. A 100 kN vertical force acts at the dipper, r = 9 m, q₃ = 45°. What torque does it add to the saddle equation? Write the general J(q)ᵀF expression.
2. List every error source that can appear in the M2 residual even with a perfect model.
3. Why must the injected force be *time-varying* for the test to be convincing?

**Deliverable:** M2 experiment design note (1 page, in `experiments/` before running it): injection profile, expected residual, acceptance criterion. *(We will do this together — this module runs concurrently with the actual M2 work.)*
**Resource:** Spong Ch. on force control (Jacobian transpose); De Luca & Mattone, "Sensorless robot collision detection" (the classic residual-method paper — it's your M2/M3 in robot form).

---

## Module 3 — The momentum observer (supports M3)

**Why:** the single most important trick for field deployment — kills the acceleration requirement your C1 study proved fatal.
**Master:**
- Generalized momentum p = D(q)q̇; its time derivative via the equations of motion
- The key identity: ṗ = τ + Cᵀq̇ − G − friction (uses the skew-symmetry from Module 1A!) — note: NO q̈ anywhere
- The observer: r(t) = K_O [p(t) − ∫(τ + Cᵀq̇ − G − fric + r)dt]; r → τ_load with first-order dynamics
- Gain K_O trade-off: bandwidth vs noise amplification
- Why this is the industry/robotics standard for collision detection (same math, different application)

**Self-test:**
1. Show that ṗ contains no acceleration. Where did q̈ go? *(It was absorbed into d(Dq̇)/dt.)*
2. What does the observer output converge to, and with what dynamics?
3. Why does a large K_O make the estimate faster but noisier?

**Deliverable:** derivation page + a simulation of the observer on your validated 2-DOF model → `learning/M3_momentum_observer.pdf` + script.
**Resource:** De Luca & Mattone (again — Sections on the momentum observer); Haddadin et al., "Robot collisions: a survey" §III (the definitive treatment).

---

## Module 4 — Measurement noise and numerical differentiation (supports M4)

**Why:** your C1 experiment already met this enemy; M4 fights it systematically.
**Master:**
- Noise models: white Gaussian, quantization; SNR; what ±5–25% injected noise means (Fu's protocol)
- Why differentiation amplifies noise (d/dt multiplies each frequency by ω — high frequencies explode); why your gradient() run biased I_zz by 56% specifically (acceleration terms carry the inertia information)
- Filtering: moving average, Butterworth low-pass, zero-phase (filtfilt); the bias-variance trade-off of filtering before identification
- Monte Carlo methodology: run N noisy identifications, report distribution of θ̂ — this is how M4's results table will be built
- Bias vs variance in estimators: noise on Y (regressor) causes BIAS (errors-in-variables), not just spread — the deep reason sensed acceleration matters

**Self-test:**
1. White noise of σ = 0.01 rad on q₃ sampled at 30 Hz — estimate the noise σ on a finite-difference q̈₃. (Order of magnitude.)
2. Why does noise in the regressor Y bias θ̂ (not just widen its CI)? What is errors-in-variables?
3. Design the M4 study: what noise levels, how many Monte Carlo runs, what plots?

**Deliverable:** extend your pendulum script: add noise sweeps, plot θ̂ distributions vs noise level → `learning/M4_noise_study.m`.
**Resource:** Ljung Ch. on errors-in-variables (skim); Fu et al. 2022 §4 (their noise protocol — your template).

---

## Module 5 — Motor physics: from current to torque (supports M5)

**Why:** the bridge from your mechanical model to the only signals the mine actually has.
**Master:**
- DC machine fundamentals: τ = K·i_a, back-EMF e = K·ω, and the crucial detail K = K'·Φ(i_f) — the torque constant depends on FIELD current (Rasuli's iHA·iCA products come from exactly this)
- Ward-Leonard drives and modern DC/AC shovel drives — what P&H/Taiyuan machines actually use
- Gearbox and hoist-rope transmission: ratios, efficiency η, reflected inertia (why motor-side inertia = load inertia/N² + motor's own)
- Saturation of the magnetic circuit: why Φ(i_f) is nonlinear and how Rasuli handled it (lookup/polynomial)
- What you can and can't get from drive PLCs: torque reference vs actual, sampling, latency

**Self-test:**
1. Why does τ = K i_a fail if field current changes? Write the corrected expression.
2. A gearbox N = 30, η = 0.92: motor torque 1 kN·m → joint torque? Joint inertia 288,000 kg·m² → inertia seen by the motor?
3. Which of Rasuli's regressor terms exist purely because of motor physics rather than mechanics?

**Deliverable:** 1-page note "The current→torque map for our shovel class, and its three uncertainty sources" → feeds directly into the CMPDI data-requirements conversation.
**Resources:** Fitzgerald, *Electric Machinery*, DC machine chapters; Rasuli 2014 §II (re-read the electrical part — it will finally fully click).

---

## Module 6 — 3-D rotations and the 4-DOF model (supports M6)

**Master:** rotation matrices, frame composition; Denavit–Hartenberg convention; deriving multi-DOF D(q), C(q,q̇), G(q) systematically (Christoffel symbols); swing–crowd–hoist coupling terms.
**Self-test:** derive the 2-link planar arm's D(q) via Christoffel symbols; explain why adding swing creates gyroscopic coupling with the saddle.
**Deliverable:** 2-link derivation → `learning/M6_two_link.pdf`.
**Resource:** Spong Ch. 3 (kinematics) + Ch. 7 (dynamics, Christoffel).

---

## Module 7 — Soil–tool interaction (supports M7)

**Master:** Zelenin's empirical cutting-resistance formula (compactness index C₀, depth, width); Frimpong's parabolic dig-path force profile; Coulomb passive earth pressure basics (Frimpong & Hu's alternative); why all such models are empirical and material-generic — the argument for learning the residual instead (M9).
**Self-test:** sketch F_r along a dig path and justify its shape; list the parameters a mine would have to give you for Zelenin, and which are realistically obtainable.
**Deliverable:** ½-page comparison note: "Zelenin vs Coulomb-wedge vs learned residual — what M7 will use and why."
**Resource:** your Frimpong & Hu 2008 paper §2–3; Zelenin via its citations.

---

## Module 8 — Field-data craft (supports M8, start EARLY)

**Master:** time-sync of multi-channel logs; resampling; cycle segmentation from drive states (hoist/crowd/swing signatures); outlier and gross-error screening; validating against weighbridge tickets (per-pass vs per-truck aggregation, statistics of agreement: mean error, σ, Bland–Altman thinking); uncertainty of the "ground truth" itself.
**Self-test:** design the complete processing chain from raw historian export to "estimated payload per pass vs weighbridge" scatter plot; what can make weighbridge truth wrong?
**Deliverable:** a segmentation dry-run script on synthetic multi-phase data → `learning/M8_segmentation_dryrun.m`. *(Do this module the month before CMPDI data lands.)*
**Resource:** Rasuli 2014 §IV–V (their field campaign — read as a manual, note every practical decision they mention and every one they DON'T).

---

## Module 9 — Scientific ML: PINNs, DeLaN, RLS (supports M9)

**Master:**
- Recursive Least Squares: the update equations, forgetting factor λ, how drift tracking works — parameter drift AS the health indicator (the thesis's destination)
- DeLaN: a network *structured as* Lagrangian mechanics (learns L, differentiates it) — physics as architecture
- PINN: physics as a LOSS term; Fu's hybrid J = αJ_dynamics + ηJ_energy; why the energy term needs central differences (t is not a network input)
- Cholesky-factorized inertia (Fu's trick to guarantee D(q) positive-definite by construction)
- When learning is legitimate: only for the residue physics can't express (media–dipper force, unmodeled compliance) — your anti-LSTM argument, now with its constructive counterpart
- Out-of-distribution honesty: why a learned component must carry an applicability domain statement

**Self-test:**
1. Write the RLS update equations from memory; what does λ = 0.99 mean in "memory length"?
2. Why does Fu factor the inertia matrix through Cholesky instead of predicting its entries directly?
3. In one paragraph: the division of labor between physics and learning in YOUR final architecture, and why a pure LSTM was rejected at Stage 1.

**Deliverable:** RLS drift-tracking demo on your 2-DOF model (slowly ramp f_v3 +20% over simulated "weeks", watch RLS track it) → `learning/M9_rls_drift.m`. *This script is a thesis figure waiting to happen.*
**Resources:** Fu et al. 2022 (full, slow read — every equation); Lutter et al., "Deep Lagrangian Networks" (DeLaN original); Ljung Ch. 11 (recursive identification).

---

## Cross-cutting module — Research craft (continuous)
You're already practicing it; keep the habits deliberate: claims ledger for every number, one-change-one-verify, regenerable figures, log-before-leaving, report-section-before-slide. Add one new habit per month: this month, **cite-while-writing** (never "add citations later").

---

## The schedule at a glance

| When | Module | Feeds |
|---|---|---|
| Now → +2 wk | M0 + M1A consolidation | thesis Ch. 3 defense |
| Concurrent with M2 work | Module 2 | M2 |
| +1 month | Module 3 | M3 |
| +2 months | Module 4 | M4 |
| +3 months | Modules 1B, 5 | M5, committee questions |
| +4–5 months | Module 6 | M6 |
| Month before field data | Modules 7, 8 | M7, M8 |
| Alongside P1 writing | Module 9 | M9, P3 |

---

*Standing offer: for any module, say "start module N" in chat — I run it as a tutoring session: I probe, you derive, I correct, we quiz. The self-tests above are exactly what I'll ask.*
