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

## 2026-07-13/14 — Communication pipeline built; PI docs generated from repo

**What was done:** Built the comms/ infrastructure (committed 92073ab):
equations_master.tex (10 macros E1–E10) + render_equations.py (Computer Modern,
600 dpi, navy+black PNG pairs); LaTeX report skeleton (thesis-time use);
deck content_map.md (19-slide control table); figure_register.md (FIG-001…010);
fig_architecture (5-layer research architecture, FIG-001); export_for_pi.py
(generates Progress_Note.docx, Revision_History.docx, Figure_Register.xlsx
FROM the repo). Proposal deck Shovel_Health_Proposal_v2.pptx QA'd slide-by-slide
(timeline overflow, valign, contents alignment fixed). PhD strategy session:
paper map P1 (~mo 12–15, M1–M4 sim-only) / P2 (~mo 24–30, field) / P3 (M9);
writing order §5→§2–4→§1→§6→abstract; report-section-before-slide rule.

**Decisions & why:** PI documents are GENERATED from repo truth, never
hand-maintained (export_for_pi.py) — duplicated documents drift like duplicated
constants. External "PhD_Research_Master" pack REJECTED as a parallel system;
absorbed two ideas only (figure register concept, architecture-figure design).
Report to PI written in Word (equations as _black.png) for compatibility;
LaTeX skeleton retained for thesis. Assumption ledger A1–A9 finalized with
removal milestones. LSTM rejected at Stage 1 with recorded rationale (no
training data; attribution required for diagnosis; OOD failure = fault regime;
Fu 2022 used LSTM only for unmodelable media–dipper residue under energy
constraint — that is the M9 slot, not the M1 slot).

**Next step:** Stage-1 case-study report generation; slide-by-slide deck
rebuild with mentoring loop.

---

## 2026-07-14 — Stage-1 report drafted; accidental trim ablation; repo audit

**What was done:** Generated Stage1_Report.docx (first full draft). Learning
curriculum LEARNING.md created (10 modules keyed to M1–M9). Full walkthrough
of validate_and_identify internals (trim rationale, inertia-axis bookkeeping,
oracle independence). GitHub audit from a fresh clone found the committed
ShovelSimulator.slx had NO acceleration wiring (6 logged vars only) — the
machine-precision model was never committed (data-loss incident, since
resolved 07-15). Ran validation with trim set to 0: revolute (gradient
fallback due to a block-name typo, see 07-15) collapsed to R²=0.933 /
NRMSE 2.58% with uninformative identification (Izz 79,336±143,359;
fv3 −68,447±242,596), while prismatic (sensed) held R²=1.000000 through the
startup spike.

**Key result:** Accidental controlled ablation: the startup transient is
genuine rigid-body dynamics (sensed equation exact through it); what the
spike breaks is numerical differentiation, not physics. Startup-spike
physics established: commanded sine has non-zero velocity at t=0 vs joint
at rest → impulse demand; input filter (τ=0.05 s) converts it to a finite
~MN·m catch-up transient; trim = 10τ = 0.5 s.

**Next step:** Rebuild report to the approved skeleton; restore wiring and
commit the true model.

---

## 2026-07-15 — v5.1 instrument; C1 reproduced bit-exact; doubt register; two provenance strikes

**What was done:**
- Compared two stray .slx files: ShovelSimulatorr2025b.slx = live model
  (8 logged vars, sensed ON, ±2 m excitation, v4 geometry) with ONE defect —
  To Workspace name q_3ddot (missing underscore) → silent gradient fallback
  on the revolute. Fixed, verified: SENSED ×2, R²=1.000000 ×2, blind ID exact.
  ShovelSim.slx identified as a pre-v4 relic (no logging, ±0.25 m, no 1.5 m
  offset) — superseded.
- Figure factory rerun (all 5 PDFs regenerated). Two lessons: Windows locks
  files open in a PDF viewer ("Permission denied" = file held by Acrobat);
  make_stage1_figures.m patched to anchor outdir to mfilename('fullpath')
  instead of the current folder.
- Stage-1 report REBUILT to the approved skeleton (Stage1_Report_M1.docx):
  exact title/sections/nomenclature/appendices; Appendix A full derivation
  (9 steps, CM-rendered); Appendix B decision ledger + full parameter table +
  provenance; inline symbols upgraded to true math runs (italic + subscripts).
- Doubt register created from 11 handwritten pages: Doubt_Register_Stage1.docx,
  39 doubts verbatim, numbered D1–D39. D1–D10 resolved in discussion.
- TWO provenance strikes from the doubt review (claims.md entries required):
  (1) Rasuli's field machine is a P&H 2100 (verified via literature search),
  NOT the 4100XPB named in our report title — title/§2.3 fix approved,
  regeneration pending. (2) The "US$100,000/hr downtime" figure has NO
  receipt-grade source (best available: industry press, "losses exceeding
  US$300,000" per outage, basis unstated) — to be softened or cited as such.
- Script upgraded v4 → v5 → v5.1 (function form). H-series fixes:
  [H1] declared config block (T_TRIM, FORCE_GRADIENT), printed every run;
  [H2] TRIM BUG: v4 shipped with keep=true(size(t)) — the trim was silently
  OFF in every run since the 07-14 ablation edit (comment-vs-code drift,
  same disease as G1); [H3] FORCE_GRADIENT switch; [H4] results struct
  self-describes config+version; [H5] datetime; [H6] covariance by solve;
  [H7] loud guards; [H8] function form with arguments block —
  validate_and_identify(out, ForceGradient=true, T_TRIM=0).

**Key result:** C1 reproduced BIT-EXACT on demand with
validate_and_identify(out, ForceGradient=true): Izz = 125,804 ± 53,945
(truth 287,900, −56%); fv3 = 6,476 ± 5,114; m and c tight (54,306.4 ± 98.4;
−1.3178 ± 0.0130); forward NRMSE 0.79%. Identical numbers to the original
study — reproducibility demonstrated, not asserted.

**Residual-shape law (new diagnostic skill, record it):** white noise at the
numerical floor = model complete, only arithmetic left; smooth signal-locked
waveform = deterministic error present (here: differentiation truncation,
absorbed partly into biased parameters); a shape matching a known input =
the signal itself (what M2's injected force will look like rising from the
~1e−12 kNm floor — the residual figure is the channel's zero-point
calibration).

**Decisions & why:** No-delete policy adopted for file copies (owner's call);
authority declared by CHANGELOG instead of renaming — model/CHANGELOG.md v5
entry flags ShovelSimulatorr2025b.slx as AUTHORITATIVE; canonical rename
deferred but REQUIRED before any M2 model edit. MATLAB access interim:
colleague's campus account for license only; institute email + campus
license requested via PI (SRIC route); own accounts for Drive and GitHub.

**Next step:** Push this commit; verify on GitHub; regenerate report with the
three approved edits (machine provenance, K_H/K_C annotation, downtime
figure); resume doubt register at Page 3 (D11–D14); then M2.

**Open questions:** Origin/purpose of model/FourDOFShovelSimulator.slx
(+.slx.r2024a autosave) — found untracked 07-15, committed per no-delete
policy, undocumented. Owner to state provenance before it is used or archived.

---

## TEMPLATE — copy for each new session

## YYYY-MM-DD — <one-line title>

**What was done:**

**Key result:**

**Decisions & why:**

**Next step:**