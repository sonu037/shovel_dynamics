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
## 2026-07-23 — Report v2, Rasuli verification, residual channel ruling

- Report v2 built: 11 surgical edits (P&H-class title + P&H 2100 provenance; downtime
  reinstated with thesis receipt [2]; datum-convention para (our d4 = his d4 - 1.5 m,
  a3 = 0); L_COG_Offset symbol fidelity + eq. 56 note; K_H = 1940 / K_C = 3950 N/A
  banked for M5; frame-convention clause (Izz = local-x slot, 90deg/+y); residual-channel
  reading in 5.1; trim-ablation addendum in 5.4 (cond 82.2 -> 13.4 untrimmed);
  theta/Phi nomenclature note; Konig citation [3]; section-symbol spelled out; new
  assumption row A10 (slide-through-pivot, inherited from [1], sin q3 detection
  signature at M8). Results untouched. claims.md: downtime line reinstated + 9 new
  verified lines (thesis source, eq. 56, a3 = 0 quote, static-table discrepancy
  56,900 vs 56,960, m_Load-at-d4 rule, payload-exclusion structural proof,
  effective-parameter caveat, M8 field constraints).
- Rasuli 2012 UBC thesis adopted as reference [2] (open access); to be filed in
  papers_library/.
- Figure factory verified against v5.1 output: Figures 2-6 reproduce bit-identical;
  identification-residual plot confirmed as the tau_load channel display.
- tau_load standalone figure DEFERRED to M2 — v2 carries the framing in 5.1 text;
  residual structure at 1e-12 understood as signal-correlated rounding (ulp
  territory), not physics. Registered as FIG-B4, reserved for the M2 before/after
  pair.
- Figure 1 defect noted (boom drawn to lower corner, should be upper); FIG-001-v2
  rebuild queued, code-first with collision audit. Internet figures ruled out
  (copyright + unverifiable geometry).
---
## 2026-08-25 — M2 modules 01, 02 closed; first identifiability result

Commits: 43f08bf → ac4f460 → 04328fd → 33fb3bf
Script: `M2/scripts/recover_load.m` (new — replaces command-window analysis)
Model:  `M2/model/M2_LoadInjection.slx`

### Model defects found and corrected

1. **Sine Wave2 held literal values** (Amplitude `100000`, Frequency `2*pi/9`).
   Force amplitude could not be set from a script, so no valid zero-force run
   was possible. Now reads `F0_y` / `omega_F`, set via `Simulink.SimulationInput`.
2. **Simulink-PS Converter2 had input filtering enabled.**
   Mask read `FilteringAndDerivatives = filter`, `SimscapeFilterOrder = 2`,
   `InputFilterTimeConstant = 0.05` → group delay ≈ 2τ = 0.1 s.
   Force inputs require no derivatives (unlike motion inputs), so the filter
   served no purpose. Set to `zero`.
3. **Applied force was not logged.** Analysis compared the residual against a
   MATLAB reconstruction of the *commanded* force while the mechanism received
   the *filtered* force. Added To Workspace block `outF`, sample time 0.01 s.
4. **Time base mismatch.** `tout` followed solver steps (10000 points at tight
   tolerance) while To Workspace blocks logged 3001. Fixed via
   `OutputOption = SpecifiedOutputTimes`, `OutputTimes = 0:0.01:30`.
   To Workspace sample times pinned to 0.01 s.

### The filter defect — diagnostic chain

Worth recording as a worked example: the residual statistics located the fault
before the block was opened.

- Case 01 gave NRMSE 2.1032% / 1.9903% but peak-magnitude error of only
  0.1081% / 0.0789%. Amplitudes agreeing while the whole curve disagrees can
  only mean a timing error.
- Implied lag from ρ = √R²:  φ = arccos ρ, Δt = φ/ω.
  q₃: R² 0.996742 → Δt 0.0818 s.  d₄: R² 0.996759 → Δt 0.0816 s.
- The two channels agreed to 0.2%. A *common* lag cannot arise from geometry,
  station, or Jacobian errors — those corrupt the channels differently.
  Therefore the fault lay in the shared force input path.
- Predicted NRMSE for a pure lag φ: 2·sin(φ/2)/2 ≈ 2.02%. Measured 2.10% / 1.99%.
  The lag accounted for essentially the entire error.
- Block mask then confirmed 2τ = 0.1 s independently.

**Consequence for the inherited target.** The "2–3% NRMSE" figure carried over
from M1 documentation was never an acceptance criterion for M2 — it was
approximately the size of this artifact. Had it been treated as a pass mark,
a filter delay would have been certified as physics. Acceptance criteria must
be derived from the milestone's own floor, not inherited.

### Module 01 — zero-force gate  (`M2/experiments/01_zero_force/M2_null.mat`)

| channel | max abs | rms | relative |
|---|---|---|---|
| q₃ | 1.999550e-06 N·m | 1.407555e-06 N·m | 7.037e-13 |
| d₄ | 2.910383e-10 N | 7.837256e-11 N | 6.461e-16 |

The two channels differ by roughly three orders of magnitude in *relative*
floor, because the torque scale is ~10⁶ N·m and the force scale ~10⁵ N. The
specification's refusal to freeze 1e-12 as a universal constant was correct:
a single threshold would have been meaningless across the two channels.

This floor also resolved the unexplained intercept from the original Case 01
(β₃ = 2679.874 N·m, β₄ = 321.179 N). No counterpart exists at zero force, so
the offset was an artifact of the filtered force path, not model–plant mismatch.

### Module 02 — known load, correct station
(`M2/experiments/02_known_load/M2_100kN_0p1111Hz_sp1p50.mat`)

F_y = 100 kN sin(2π/9 · t), bail station s = d₄ + 1.5 m, trim t ≥ 5 s.

| metric | q₃ | d₄ |
|---|---|---|
| slope (expect −1) | −1.000000 | −1.000000 |
| intercept | −0.000 | 0.000 |
| R² | 1.000000 | 1.000000 |
| max err / peak ref | 2.724e-12 | 3.868e-15 |
| implied lag | 0.000000 s | 0.000000 s |

Recovery error sits at the module-01 floor, which is the correct ceiling on
the claim.

**Scope of the claim.** This verifies the force→generalised-force mapping, the
application station, the sign convention, and the subtraction identity. It does
*not* establish load identifiability: the residual contained exactly one
unmodelled term, placed there by construction. Under motion-prescribed joints
the result is an algebraic identity, and the oracle shares assumptions A1–A3
with the analytical model, so M2 is structurally blind to errors in those.

### Sign convention (now confirmed empirically, not only algebraically)

D q̈ + C q̇ + G + f = τ_actuator + Q_ext, hence τ_actuator = τ_model − Q_ext, so

    residual = τ_measured − τ_model = −JᵀF

Regression slope of residual on Q_pred is therefore −1, measured as −1.000000.

### Module 06 — first identifiability result (unplanned)

A run with `Station = -1.32` was made in the mistaken belief that it moved the
force to the COG. It does not — `Station` alters the *analysis assumption* only;
the force remains physically applied at the bail. The run is retained because
it measures something more useful: how distinguishable two stations 2.82 m
apart actually are.

| | wide stroke (d₄ = 8.75 ± 2 m) | narrow stroke (d₄ = 8.75 ± 0.27 m) |
|---|---|---|
| slope | −1.378454 | −1.380943 |
| intercept | −1662.762 N·m | −220.037 N·m |
| R² | 0.997308 | 0.999953 |
| NRMSE | 13.7187% | 11.7131% |
| peak magnitude error | 39.7425% | 39.3560% |
| d₄ channel | −1.000000, R² 1.000000 | −1.000000, R² 1.000000 |

Files: `M2/experiments/02_known_load/M2_100kN_0p1111Hz_s-1p32.mat`,
       `M2/experiments/06_identifiability/M2_wrongstation_narrowband.mat`

**Reading.**

1. The d₄ channel is unaffected in both cases because Q_d₄ = F_y sin q₃ carries
   no lever arm. Only q₃ can distinguish stations. Consistent with module 05's
   note that observability redistributes between coordinates.

2. The slope measures ρ(d₄) = (d₄+1.5)/(d₄−1.32) empirically, weighted across
   the trajectory. Wide stroke: ρ ∈ [1.2990, 1.5193], measured 1.378454.
   Narrow stroke: ρ ∈ [1.3662, 1.3939], measured 1.380943 — near the midpoint.
   Derivation and measurement agree; the algebra of module 06 is confirmed.

3. **The result.** Assuming a station 2.82 m wrong still gave R² = 0.9973 over
   the wide stroke and R² = 0.99995 over the narrow one. Discriminating
   information fell from 0.2692% to 0.0047% — a factor of 57 — for a sevenfold
   reduction in stroke. In a noise-free simulation with exact sensing.

4. Detectability and identifiability separate cleanly here: peak-magnitude
   error stayed near 39% in both runs, so the load remains obviously present
   and obviously mis-scaled. What collapses is the ability to determine *which
   model* produced it.

**Caveats.**
- The narrow band has the right *width* (0.54 m, matching Bi 2020's operational
  extension band) but the wrong *centring* (8.75 m rather than ~9.77 m). Since
  |dρ/dd₄| = 2.82/(d₄−1.32)² is larger at short extension, correct centring
  should worsen the confounding further. To be tested, not assumed.
- This is station-versus-station confounding, not load-versus-parameter-error
  confounding. Related but distinct; the latter requires the sensitivity matrix.

### Crowd excitation

Sine Wave1 amplitude temporarily set to 0.27 for the narrow-band run, then
restored to 2 and the model saved. Verified by `get_param`.

### Open items carried forward

- **Izz ambiguity** (already flagged in `shovel_params.m`): whether Rasuli
  Table II's 287,900 kg·m² is about the COM or grouped about the pivot. The
  inertia column is the whole basis of the load-versus-mass separability
  argument, so this must be resolved *before* κ(S) is computed.
- Modules 01 and 02 were run with blank acceptance criteria. Fill and freeze
  retroactively, using the measured floor.
- Next: centred narrow band; then sensitivity matrix S = [∂r/∂F | ∂r/∂δm] and
  κ(S) computed with and without the inertia and Coriolis terms.
  [SUPERSEDED 2026-08-31 - see Izz correction at the end of this file.]
## 2026-08-28 — Rev 24 corrections; station confounding measured inside the Bi operational band

Commit: (fill in after committing)

### Specification corrections (rev 24)

Two defects found in `M2_Experiment_Design_Specification.md` module 06 and
`M2_nomenclature.md`, corrected today.

**1. s_dm was incomplete.** The spec gave the mass-error sensitivity as
gravity only:

    s_dm = g (d4 - 1.32) cos q3

The full derivative is

    s_dm = d(tau3)/d(M_d) = r^2 q3ddot + 2 r d4dot q3dot + g r cos q3,  r = d4 - 1.32

because M_d appears in the inertia, Coriolis AND gravity terms of the saddle
equation. Derivation verified independently in three separate sessions.

Consequence: the spec's conclusion that "separability comes from d4 variation
alone; q3 variation contributes nothing" is an artifact of the truncation. The
inertia and Coriolis terms carry no cos q3 factor, so the cancellation that
produced that conclusion does not occur. With the full s_dm the load/mass ratio
depends on q3, q3dot, q3ddot, d4 and d4dot.

This matters because a vertical load produces NO inertial and NO Coriolis
signature, while a mass error produces both. These are terms the load cannot
imitate, and they do not require d4 spread — a second candidate separation
mechanism (now labelled Mechanism B, against Mechanism A for geometric
separation through d4).

The gravity-only form is retained in the spec as the QUASI-STATIC
APPROXIMATION, with the condition stated explicitly and flagged as
trajectory-dependent rather than a property of the machine.

**2. rho was used for two different quantities.** Split into:

    rho_station(d4) = (d4+1.5)/(d4-1.32)        dimensionless
    rho_LM(t)       = s_load / s_dm             s^2/m, trajectory-dependent

They differ by a factor of g and by dimension. rho_LM reduces to
(1/g) rho_station only under the quasi-static approximation.

**Correction to earlier reasoning (mine and Claude's).** The 250:1
gravity-to-inertia ratio was being quoted as though it settled whether
Mechanism B is viable. It does not. The dynamic terms do not need to be large
relative to gravity; they need to be large relative to the residual
UNCERTAINTY. A term at 0.4% of total torque is usable if the torque estimate is
accurate to 0.01% and useless if it is accurate to 2%. That comparison arrives
at M5 with the motor-current-to-torque map. Also noted: 250:1 is a property of
the current sine trajectory, not of the machine.

### New experiment — station confounding inside the Bi operational band

**Motivation.** The earlier narrow-band run (8.75 +/- 0.27 m) had the correct
stroke WIDTH (0.54 m, matching Bi et al. 2020) but the wrong CENTRING. Bi's
measured operational extension band for the WK-55 at Anjialing is 9.50-10.04 m.
All prior statements about "the operational band" were therefore extrapolations
from a window positioned ~1 m below it.

**Setup.** Sine Wave1 set to Bias 9.77, Amplitude 0.27. Verified trajectory:
d4 in [9.5001, 10.0399] m. Force 100 kN at 2*pi/9 rad/s, wrong station assumed
(Station = -1.32 against a true station of d4 + 1.5).
File: `M2/experiments/06_identifiability/M2_wrongstation_bi_band.mat`

**Prediction made before running.** rho_station spans 1.3447 to 1.3234 over the
Bi band (1.6% spread) against roughly 1.3939 to 1.3662 over the 8.75-centred
window (2.0% spread). Since discrimination scales roughly with the square of the
available spread, 1-R^2 was predicted at approximately 3e-05, i.e. a ratio near
1.6 relative to the 8.75-centred case.

**Result.**

| trajectory | d4 range | slope | 1 - R^2 |
|---|---|---|---|
| wide | 6.75 - 10.75 m | -1.378454 | 2.691764e-03 |
| narrow, off-band | 8.48 - 9.02 m | -1.380943 | 4.695204e-05 |
| Bi operational band | 9.50 - 10.04 m | -1.334832 | 2.998519e-05 |

Measured ratio (off-band / Bi-band) = 1.566 against 1.6 predicted, within 3%.

Wide to operational band: **89.8-fold collapse** in discriminating information.

**Independent corroboration.** The measured slope of 1.334832 matches the
midpoint of rho_station over the Bi band (1.3447 to 1.3234, midpoint ~1.334).
Two separate quantities agreeing.

**Defensible claim.** In a noise-free simulation with exact sensing, two
candidate force-application stations separated by 2.82 m become nearly
indistinguishable when the crowd stroke is confined to the operational envelope
of a working cable shovel. Only 3.0e-05 of the residual variance carries
information distinguishing them.

**Mechanism.** A wrong station produces Q_wrong = rho_station(d4) * Q_true. When
d4 is confined, rho_station is nearly constant, and a constant is exactly what a
free regression slope absorbs. The fit is excellent and the physical answer is
wrong. This is confounding, not a fitting failure.

**Limits of the claim.**
- This is STATION versus STATION, not LOAD versus MASS ERROR. The mechanism
  looks analogous but has not been measured for the load/mass case.
- Noise-free with exact sensing, so this is a LOWER BOUND on the difficulty.
  Real measurement error can only worsen discrimination.
- The Bi-band and 8.75-centred runs differ in window position; the stroke width
  is the same (0.54 m) but the whole trajectory shape changes because d4 enters
  the lever arm throughout. Not a clean single-variable comparison.
- Bi's limits are for a WK-55 at Anjialing, not a P&H 2100. The gate still
  requires P&H-specific stroke and velocity limits.

**Significance for the programme.** Mechanism A (geometric separation through
d4 variation) is close to exhausted inside the real operating envelope —
measured, not argued. This is why the s_dm correction is consequential: if
Mechanism A is dead in the band where machines work, Mechanism B is the only
remaining route, and there is now a numerical reason to look there.

### Housekeeping

- `M2_case01_100kN.mat` (pre-filter-fix, no provenance) moved via `git mv` to
  `M2/experiments/00_archive/M2_case01_100kN_SUPERSEDED.mat` with a README
  marking it as evidence for the filter diagnosis and not to be cited.
- F0_x, F0_y, omega_F assigned as defaults in the model workspace, so the model
  opens self-contained rather than depending on base-workspace variables.
  recover_load still overrides them via setVariable.
- Stray files at repo root deleted (untitled*.fig, CM2_Acceleration.fig);
  *.slxc and slprj/ added to .gitignore.
- Crowd excitation restored to Bias 8.75, Amplitude 2 and verified.

### Open

- Crowd excitation has now been hand-edited and restored three times. Add
  'CrowdBias' and 'CrowdAmplitude' options to recover_load. It is currently the
  only experimental variable with no provenance in the saved .mat.
- Next: load-vs-mass signature correlation on the existing trajectory, using the
  FULL s_dm. Sequence: magnitude -> shape/correlation -> conditioning ->
  uncertainty.
- Izz ambiguity in shovel_params.m still unresolved and is now blocking, since
  the inertia column is the basis of Mechanism B.
  [SUPERSEDED 2026-08-31 - see Izz correction at the end of this file.]
- Modules 01 and 02 acceptance criteria still blank.
- Trim window t >= 5 s still unjustified since the filter removal.
## 2026-08-28 (continued) — Load-vs-mass sensitivity diagnostic; Mechanism B not demonstrated

Trajectory: the existing M2 known-load run (q3 = 45 +/- 15 deg, d4 = 8.75 +/- 2 m,
both coordinates at omega = pi/10).
File: `M2/experiments/06_identifiability/M2_sensitivity_diagnostic.mat`

### Metric correction — and why it is not a matter of taste

The diagnostic was first run using cosine similarity on raw sensitivity vectors:

    corr(load, gravity-only) = 0.998605
    corr(load, full)         = 0.999361
    corr(load, inertia)      = -0.411699
    corr(load, coriolis)     = +0.828721

The -0.4117 was read as evidence that the inertial component has a temporal
structure the load cannot produce, i.e. support for a dynamic separation
mechanism. That reading is WRONG.

s_load has mean 6.872 and std 0.430 — a large positive offset with a small
variation. s_in has mean -0.583 and std 1.109. Cosine similarity between a
mostly-positive vector and a zero-crossing one is dragged negative by the offsets
regardless of shape. Mean-removed, corr(load, inertia) = 0.9613.

The justification for Pearson is structural, not stylistic. The estimator is

    y(t) = beta0 + s_load(t) F + s_full(t) dM_d + eps(t)

so beta0 is a nuisance parameter, and removing means is exactly PROJECTING OUT
THE INTERCEPT DIRECTION. Cosine similarity on raw vectors answers a question
about a model WITHOUT an intercept, which is not the model being fitted. The
ranking of components reverses entirely between the two choices.

Formal identifiability must include the intercept explicitly or project it out.

### Pearson correlation matrix (mean-removed)

|          | load   | gravity | inertia | coriolis |
|----------|--------|---------|---------|----------|
| load     | 1      | 0.5389  | 0.9613  | 0.4688   |
| gravity  | 0.5389 | 1       | 0.2863  | 0.9919   |
| inertia  | 0.9613 | 0.2863  | 1       | 0.2106   |
| coriolis | 0.4688 | 0.9919  | 0.2106  | 1        |

corr(load, full mass) = 0.8222

RMS relative to gravity: inertia 2.563%, Coriolis 0.980%, total dynamic 2.484%.

### Result — Mechanism B is NOT DEMONSTRATED on this trajectory

The inertial component correlates 0.9613 with the load in shape. It is nearly
parallel to the load and therefore cannot separate load from mass error on this
trajectory.

Including the dynamic terms increases the load-mass sensitivity correlation from
0.5389 (gravity only) to 0.8222 (full), so NO SEPARATION BENEFIT IS OBSERVED in
this experiment. Whether this constitutes a degradation of identifiability is a
question for the conditioning analysis, not for a pairwise correlation.

Scope: ONE trajectory, with omega_q3 = omega_d4 = pi/10. This does not disprove
the mechanism generally; it tests one instance of it.

The earlier "sign test" (corr with grav-dyn = 0.9972 versus grav+dyn = 0.9993) is
explained by the same fact: subtracting a load-parallel component moves the sum
away from the load. It was not evidence of a hidden separating direction.

### Structure of the correlation matrix

Two strongly correlated pairs on the tested trajectory: {gravity, Coriolis} at
0.9919 and {load, inertia} at 0.9613, with the pairs largely independent of each
other (gravity-inertia 0.2863).

HYPOTHESIS (untested): this pairing arises from SINGLE-FREQUENCY excitation
rather than being intrinsic to the shovel dynamics. Both coordinates are driven
at omega = pi/10, so the available signals may collapse into a small number of
independent shapes. Different frequencies on the two coordinates would test this.

The term "near-degenerate" is reserved for the singular-value structure of the
actual sensitivity matrix and is not used here.

### The 250:1 figure was wrong

Preliminary unmeasured estimate used throughout the session, including in the
rev 24 spec text: ~250:1 gravity-to-dynamic.

Measured for this trajectory: RMS(s_dyn)/RMS(s_grav) = 0.024843, i.e. 40.25:1.
Six times more favourable than quoted, and trajectory-dependent rather than a
machine property. Both values retained here for the audit trail; only the
measured value should appear in any interpretation.

### Status changes

- Rev 24's mathematical correction to s_dm STANDS (three terms, verified
  independently three times).
- Rev 24's JUSTIFICATION — that the dynamic terms constitute a second separation
  mechanism — is not demonstrated on this trajectory. The Mechanism A /
  Mechanism B framing in the trilemma section should NOT be declared dead on the
  basis of one trajectory, and should not be treated as settled either way.
- The three station-experiment results are unaffected. Different computation
  (regression on simulation output), different question (station vs station,
  physical cause held fixed). 1-R^2 = 2.998519e-05 and the 57.3x collapse remain
  valid within their stated scope.

### Unreconciled

Load-vs-gravity Pearson is 0.5389, meaning their variations differ substantially.
Yet the station experiment gives 1-R^2 = 3.0e-05 inside the Bi band, meaning
d4-based geometric discrimination is nearly exhausted. Both concern d4 weighting.
These have NOT been reconciled. Likely they are different questions —
station-vs-station holds the physical cause fixed while load-vs-mass compares
different causes — but that is a hypothesis, not an answer.

### Next

- **Izz ambiguity is now BLOCKING.** The sensitivity matrix requires the inertia
  column, and whether Rasuli Table II's 287,900 kg m^2 is about the COM or
  grouped about the pivot is still unresolved in shovel_params.m. Resolve before
  the SVD or the result inherits the ambiguity.
  [SUPERSEDED 2026-08-31 - see Izz correction at the end of this file.]
- Verify Bi et al. 2020 numbers (9.50-10.04 m, 0.76 m/s, 0.5 m/s^2) verbatim
  from the PDF into claims.md. Record that these are for a WK-55 at Anjialing,
  not a P&H 2100. Wording for any write-up: "literature-derived reference
  envelope used to constrain the numerical experiment", NOT "actual shovel data".
- Build S with all parameter columns plus the constant, and compute the angle
  between the load column and span(S), plus the singular values. This answers the
  identifiability question directly for any trajectory, without needing a
  mechanism to be proposed first. It also settles whether the correlation pairing
  above reflects genuine rank deficiency.
- Reconcile the station result against the Pearson result.
## 2026-08-28 (session 2) — Load-vs-mass sensitivity diagnostic, wide and Bi-band

Script: `M2/scripts/sensitivity_diagnostic.m`
Artifact: `M2/experiments/06_identifiability/M2_sensitivity_diagnostic.mat`
Sources: `02_known_load/M2_100kN_0p1111Hz_sp1p50.mat`,
         `06_identifiability/M2_wrongstation_bi_band.mat`
Trim: t >= 5 s via R.trimIdx, 2501 of 3001 samples in both cases.

### Sign convention

Columns are defined WITHOUT negation: s_load = d(tau3)/d(Fy) and
s_dm = d(tau3)/d(M_d). The residual carries the minus sign separately
(residual = tau_measured - tau_model = -J'F). Pearson correlation is invariant
under a sign flip applied to both columns; verified numerically to exactly zero
difference. The convention matters at the CONDITIONING step, where estimate signs
depend on it, not at the correlation step.

### Metric — the choice is structural, not stylistic

Mean-removed Pearson. The estimator fits a free intercept,
y = beta0 + s_load*F + s_dm*dM_d + eps, so beta0 is a nuisance parameter and
mean-removal is exactly the projection removing the constant direction. Cosine
similarity on raw vectors answers a question about a model WITHOUT an intercept.

Same two signals, both cases:

| case | raw cosine(load, inertia) | Pearson(load, inertia) |
|---|---|---|
| wide | -0.411699 | 0.961309 |
| Bi band | -0.055816 | 0.998613 |

Offsets responsible: wide, s_load mean 6.8725 std 0.4297 against s_in mean
-0.5834 std 1.1089. Bi band, s_load mean 7.5597 std 1.3285 against s_in mean
-0.3020 std 1.2926.

On the Bi-band case the raw metric would have indicated near-orthogonality
(-0.056) in exactly the case where the two columns are 0.9986 aligned — the
strongest possible false support for Mechanism B.

### Results

| quantity | wide | Bi operational band |
|---|---|---|
| d4 range | 6.7505 - 10.7495 m | 9.5001 - 10.0399 m |
| d4 span | 3.9990 m | 0.5399 m |
| max abs(d4dot) | 0.628164 m/s | 0.084802 m/s |
| max abs(q3ddot) | 0.025832 rad/s^2 | 0.025832 rad/s^2 |
| r(load, gravity-only) | 0.5388857718 | 0.9999729034 |
| r(load, FULL mass) | 0.8222157044 | 0.9999919261 |
| 1 - r(load, full) | 1.777843e-01 | **8.073857e-06** |
| inertia / gravity RMS | 2.563048 % | 2.353814 % |
| Coriolis / gravity RMS | 0.979654 % | 0.127604 % |
| dynamic / gravity RMS | 2.484269 % | 2.341438 % |
| gravity : dynamic | 40.25 : 1 | 42.71 : 1 |

Pearson matrix, wide:

|          | load    | gravity | inertia | coriolis |
|----------|---------|---------|---------|----------|
| load     | 1       | 0.53889 | 0.96131 | 0.46882  |
| gravity  | 0.53889 | 1       | 0.28627 | 0.99189  |
| inertia  | 0.96131 | 0.28627 | 1       | 0.21061  |
| coriolis | 0.46882 | 0.99189 | 0.21061 | 1        |

Pearson matrix, Bi band:

|          | load    | gravity | inertia | coriolis |
|----------|---------|---------|---------|----------|
| load     | 1       | 0.99997 | 0.99861 | 0.16100  |
| gravity  | 0.99997 | 1       | 0.99820 | 0.16826  |
| inertia  | 0.99861 | 0.99820 | 1       | 0.10892  |
| coriolis | 0.16100 | 0.16826 | 0.10892 | 1        |

### Reading

**Wide trajectory.** The four columns fall into two groups, {load, inertia} at
0.9613 and {gravity, Coriolis} at 0.9919, with the groups largely independent
(gravity-inertia 0.2863). Including the dynamic terms moves load-vs-mass from
0.5389 to 0.8222: no separation benefit observed, because the inertia column is
nearly parallel to the load.

**Bi operational band.** The structure collapses. Load, gravity and inertia are
mutually correlated above 0.998. Only Coriolis retains independence (0.109-0.168),
and it falls from 0.980% to 0.128% of gravity by RMS.

**Finding.** The one column retaining an independent direction is the one the
operating envelope suppresses most. Coriolis depends on d4dot, and confining the
crowd to a 0.54 m stroke at the same frequency necessarily reduces d4dot by 7.4x
(0.628 -> 0.085 m/s). This is a concrete statement of the excitation trilemma.

**Headline.** 1 - r(load, full) = 8.07e-06 in the Bi band against 1.78e-01 wide.
Load and mass error are close to indistinguishable in shape once the crowd is
confined to a realistic envelope.

### Mechanism B — NOT DEMONSTRATED on either trajectory

The inference "a load produces no inertial signature, therefore the inertial
column separates load from mass error" does not follow. A column the load cannot
produce may nonetheless resemble the load. Measured: 0.9613 (wide), 0.9986 (Bi
band).

Dynamic MAGNITUDE and dynamic IDENTIFIABILITY are distinct. The dynamic
contribution is 2.34% of gravity in the Bi band — not negligible — yet contributes
essentially nothing to separation.

Scope: two trajectories, both with omega_q3 = omega_d4 = pi/10. This does not
settle the mechanism generally, and the gate should NOT be rewritten to declare
it dead.

### Confound — stated explicitly

The Bi-band case differs from the wide case in TWO ways: narrower d4 window AND
7.4x slower crowd. The coupling is imposed by the physical constraint, not by
careless design, but the collapse cannot yet be attributed to the window alone.

Note max abs(q3ddot) is IDENTICAL in both cases (0.025832 rad/s^2), since only
the crowd excitation was changed. The inertia/gravity ratio is accordingly
similar (2.56% vs 2.35%), while the Coriolis ratio falls by 7.7x.

Attribution of the wide-vs-Bi difference to d4 spread alone is therefore NOT
established. Excitation differences have not been isolated.

### The 250:1 figure was wrong

Quoted repeatedly and written into the rev 24 spec text as a machine property.
Never measured. Measured: 40.25:1 (wide), 42.71:1 (Bi band). Six times more
favourable, and trajectory-dependent. Both retained for the audit trail; only
measured values to be used in interpretation.

Note also that an RMS sensitivity ratio is not equivalent to a torque-accuracy
requirement. Whether a 2.34% component is exploitable depends on measurement
noise, model uncertainty and estimator conditioning — not on the ratio alone.

### Status

- Rev 24's mathematical correction to s_dm STANDS.
- Rev 24's JUSTIFICATION for Mechanism B is not demonstrated on either trajectory.
- Station-vs-station confounding: experimentally demonstrated by regression on
  simulation output. Load-vs-mass confounding: diagnosed by correlation only.
  Formal conditioning and uncertainty outstanding.
- Station results unaffected. Different computation, different question.
- Izz: NOT blocking for this diagnostic, since d(tau3)/d(M_d) contains no I_zz.
  STILL BLOCKING for the sensitivity matrix, which needs an inertia column
  d(tau3)/d(I_zz) = q3ddot whose scaling depends on the parameter value. The M1
  inertia ambiguity is not resolved.
  [SUPERSEDED 2026-08-31 - see Izz correction at the end of this file.]

### Unreconciled

Load-vs-gravity Pearson on the WIDE trajectory is 0.5389 — substantially
different shapes — while the wide station experiment gives 1-R^2 = 2.69e-03. Both
concern d4 weighting. Not reconciled. Probably different questions
(station-vs-station holds the physical cause fixed; load-vs-mass compares
different causes), but that is a hypothesis.

### Next

1. Same 0.54 m window at higher crowd frequency, restoring d4dot toward Bi's
   0.76 m/s. Separates "narrow window" from "slow crowd".
2. Resolve Izz before building S.
[SUPERSEDED 2026-08-31 - see Izz correction at the end of this file.]
3. Verify Bi et al. 2020 numbers verbatim from the PDF. WK-55 at Anjialing, not
   a P&H 2100.
4. Formal S: all parameter columns plus the constant. Note that kappa(S) requires
   physically justified column scaling, since s_load is in m and s_dm in m^2/s^2.
   The angle between s_load and span(S) is scale-invariant per column and may be
   the better measure.
### Correction (2026-09-02) — Izz was never open

Five entries in this log state that the Izz ambiguity is unresolved or
blocking. All are wrong.

The question was closed on 2026-07-23 and recorded in claims.md line 13,
status V: Rasuli's 287,900 kg m^2 is the about-COM value, and Phi carries
the composite (Izz3 + Md*L_COG_Offset^2) separately, so the parallel-axis
usage does not double-count.

Cause: the comment in M1/scripts/shovel_params.m was never updated after the
closure and still read "must be resolved against Rasuli Eq. 23/26". That
comment was read as current and the claim repeated into this log five times
without checking claims.md. The comment is now corrected.

Nothing is blocked by Izz. The sensitivity matrix can be built.

Lesson: single source of truth applies to STATUS, not only to values. A
resolved question that still reads as open in the code will keep being
reopened.
## 2026-09-02 — Module 03 amplitude sweep CLOSED; setVariable override found and fixed

Spec frozen at 074fac6 before running (fill-freeze-run rule observed).
Script: `M2/scripts/recover_load.m`
Results: `M2/experiments/03_amplitude_sweep/M2_03_{10,50,100,200,500,1000}kN.mat`

### Defect found: model workspace shadows setVariable

The first attempt returned SIX BIT-IDENTICAL results across a 100x amplitude
range, including an intercept of -2.464e-07 N m that cannot be
amplitude-independent. Confirmed by `max(abs(R.sig.Fy))` = 100000 in both the
nominal 10 kN and 1000 kN runs.

Cause: F0_x, F0_y and omega_F were placed in the MODEL WORKSPACE on 2026-08-28
(commit 4fe0621) to make the model open self-contained. Model-workspace
variables SHADOW `Simulink.SimulationInput.setVariable`. From that commit
onward every 'Amplitude', 'Fx0' and 'Frequency' argument was silently ignored.

This also explains the earlier `M2_100kN_horizontal.mat`, whose metrics matched
the vertical run exactly. Both defects have the same root cause.

Fix: `recover_load` now assigns directly to the model workspace via
`mw.assignin` and READS THE VALUE BACK, erroring if the write did not take.

**Prior results verified unaffected.** Requested and applied amplitude match in
M2_null (0 kN), both 02_known_load cases, and both 06_identifiability station
runs. Modules 01 and 02 and all station-confounding results stand. Only module
03 was void, because it is the first module to vary amplitude and therefore the
first to exercise the broken path.

### Results (valid run)

| amp (kN) | applied (kN) | max abs Fx (N) | slope q3 | slope d4 | intercept (N m) | 1-R^2 q3 | max err / peak |
|---|---|---|---|---|---|---|---|
| 10 | 10.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 7.673e-22 | 2.723e-11 |
| 50 | 50.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 3.069e-23 | 5.446e-12 |
| 100 | 100.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 7.673e-24 | 2.724e-12 |
| 200 | 200.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 1.918e-24 | 1.361e-12 |
| 500 | 500.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 3.069e-25 | 5.448e-13 |
| 1000 | 1000.000 | 0 | -1.000000 | -1.000000 | -2.464e-07 | 7.674e-26 | 2.726e-13 |

Absolute maximum q3 recovery error across the sweep:

| amp (kN) | max abs err (N m) | ref peak (N m) | relative |
|---|---|---|---|
| 10 | 1.999448e-06 | 7.342640e+04 | 2.723064e-11 |
| 50 | 1.999433e-06 | 3.671320e+05 | 5.446088e-12 |
| 100 | 1.999782e-06 | 7.342640e+05 | 2.723520e-12 |
| 200 | 1.999317e-06 | 1.468528e+06 | 1.361443e-12 |
| 500 | 2.000015e-06 | 3.671320e+06 | 5.447674e-13 |
| 1000 | 2.001412e-06 | 7.342640e+06 | 2.725739e-13 |

### Reading

The ABSOLUTE error is constant to within 0.1% across a 100x amplitude range, at
approximately 2.00e-06 N m — the module 01 zero-force floor. The reference peak
scales exactly 100x. The RELATIVE error therefore falls as 1/F, from 2.723e-11
to 2.726e-13.

The 0.1% drift in absolute error is not zero and should not be reported as
"constant". It rises very slightly with amplitude, consistent with
floating-point rounding on larger operands. A perfectly fixed floor would show
no drift.

1-R^2 falls as 1/F^2 for the same reason: 7.673e-22 to 7.674e-26 across 100x.

The intercept is -2.464e-07 N m at every amplitude, unchanged. It is a fixed
arithmetic artefact of the trim window and the regression, four orders below the
module 01 floor. Note that it was the CONSTANCY of this number that first
revealed the override, since a real intercept would have scaled with force.

### Acceptance criteria — all four met

(a) slope = -1.000000 to six decimals at every amplitude, both channels. MET.
(b) intercept 2.464e-07 N m, below the module 01 floor of 2.00e-06 N m. MET.
(c) max err / peak ref scales as 1/F, consistent with a fixed floor and not with
    an amplitude-dependent scaling error. MET, with the mechanism demonstrated
    directly rather than inferred from the ratio.
(d) no trend in slope with amplitude. MET.

Validity check: applied amplitude equals requested at every level; max abs Fx
exactly zero, confirming the Fx = 0 condition; both channels recover.

### Scope of the claim

M03 demonstrates that, under the specified baseline trajectory, station, force
direction, solver settings and implemented linear force-injection model,
residual recovery remains proportional to applied force over 10-1000 kN, with
maximum absolute error at the module 01 numerical floor.

It does NOT demonstrate: that real digging forces are linear over this range;
that the physical shovel is linear over this range; that 1000 kN is a realistic
operating load (it is a deliberate numerical stress test); that payload recovery
behaves identically; or that identifiability is resolved.

### Next

Module 04, frequency sweep. Duration becomes a design variable there and must be
justified in the Setup field: 30 s was inherited from modules 01-03 and contains
about 1.5 saddle cycles at omega = pi/10. Slower or multi-frequency excitation
will need longer.

Still open: trim window justification (t >= 5 s inherited from the filter era);
blank acceptance criteria on modules 01 and 02; Bi et al. verbatim verification.

## TEMPLATE — copy for each new session

## YYYY-MM-DD — <one-line title>

**What was done:**

**Key result:**

**Decisions & why:**

**Next step:**