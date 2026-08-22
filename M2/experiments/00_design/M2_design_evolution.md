# M2 — Design Evolution

Record of how the M2 experiment design was arrived at. Twenty-three revisions,
each triggered by finding a specific scientific weakness. Kept because the
reasoning is the defence: a reviewer or examiner asking "why is it done this way"
should find the answer here.

Status: design FROZEN. Further revision is drift, not improvement.
The remaining work is analysis (the trilemma) and building, not redesign.

---

## Group A — Defining what M2 measures

**V0. Original idea.** Inject a known force, subtract the model, recover it.
NRMSE < 2-3%.
Weakness found: too simple to be meaningful. "You injected a force you already
knew - why does subtracting a model prove anything?"

**V1. tau_model is evaluated on the OBSERVED motion**, never a hypothetical
force-free one. The model is a calculator applied to reality, not a simulator of
an alternative reality.

  CORRECTION (recorded): an earlier version of this note stated that the injected
  load changes the trajectory. That is FALSE for this setup. The M2 baseline is
  MOTION-PRESCRIBED - Sine Wave -> PS Converter -> joint, with Simscape computing
  the required torque. Injecting a force does not change the commanded trajectory;
  it raises the required torque. The trajectory-changing case is a separate
  torque-driven variant, run later as a generalisation check.

**V2. Circularity resolved.** Two independent paths:
    oracle path      F -> machine -> tau_measured
    analytical path  q, qdot, qddot -> M1 model -> tau_model
The analytical model is NEVER given F. That the load influences qddot is not
circularity: the model answers "what torque would the nominal machine require for
this motion?", and the excess is the residual.
Decision: tau_measured comes from the Simscape joint sensor. Computing it from the
same equations and parameters would merely reproduce tau_model.

---

## Group B — The mechanics

**V3. Injection point.** Bail station on the handle axis, distance d4 + 1.5 m from
the saddle pivot, eccentricity e = 0, consistent with report assumption A10.
Note: four papers in the library (Raza & Frimpong, Feng, Fu, Frimpong) apply
resistive forces at the DIPPER TIP instead. The bail choice is deliberate but
must be justified explicitly in any write-up.

**V4. Jacobian derived, not assumed.**

    p = [ (d4+1.5) cos q3 ; (d4+1.5) sin q3 ]

    J = [ -(d4+1.5) sin q3    cos q3 ]
        [  (d4+1.5) cos q3    sin q3 ]

    Q = J^T F

    Q_q3 = (d4+1.5) (Fy cos q3 - Fx sin q3)    [N m]
    Q_d4 = Fx cos q3 + Fy sin q3                [N]

Units check: q3 is dimensionless (rad), so dp/dq3 carries a length; d4 is already
a length, so dp/dd4 is dimensionless. Each generalized force therefore carries
whatever units make Q * dq come out in joules.

**V5. Physical reading of the two coordinates.**
Q_d4 = component of F along the handle -> carried by the RACK AND PINION into the
crowd drive (not by the saddle bearing).
Q_q3 = perpendicular component x lever arm -> opposed by the hoist rope.
One external force produces two generalized effects, split purely by geometry.

---

## Group C — Observability and identifiability  [THE PUBLISHABLE CORE]

**V6. "Blind at q3 = 90 deg" corrected.** For a vertical load, Q_q3 -> 0 but
Q_d4 -> -F. Observability REDISTRIBUTES between coordinates rather than vanishing.
Consequence: two per-coordinate observability maps, not one blindness map.

**V7. Load can masquerade as parameter error.** For a vertical load:

    s_load = (d4 + 1.5) cos q3
    s_dm   = g (d4 - 1.32) cos q3

At fixed d4 these are proportional. A naive M2 could appear to recover the load
while actually mixing load and parameter error.

**V8. cos q3 CANCELS from the ratio.**

    s_load / s_dm = (1/g) * rho(d4),    rho(d4) = (d4+1.5)/(d4-1.32)

Therefore, for a vertical load, q3 variation contributes NOTHING to this
separability. The entire burden falls on d4.

Distinguish:
  structural identifiability  - d4 must take at least two distinct values
  practical identifiability   - d4 must vary ENOUGH for acceptable conditioning

**V9. "Short extension" reasoning corrected.** The first justification (the ratio
is larger) was wrong. The correct one:

    drho/dd4 = -2.82 / (d4 - 1.32)^2

Separability comes from the SPREAD of rho across the run, and |drho/dd4| is
largest at short extension. Same conclusion, defensible reason.

**V10. The result is conditional.** For Fx != 0,

    s_load = (d4+1.5)(Fy cos q3 - Fx sin q3)

is not proportional to cos q3, and q3 variation DOES contribute. The d4-only
result holds for M2's vertical loading and must not be stated as general.

---

## Group D — Experiment design

**V11. Excitation, and what a sweep proves.** Spectral separation established from
the actual FFT, not from checking harmonic relationships. And because the channel
is configuration-dependent, a frequency sweep characterises LOCAL behaviour only -
single-frequency results do not compose to broadband performance. A separate
multisine or chirp test is required for any broadband claim.

**V12. Physical admissibility.** Trajectories must respect real machine limits on
q, qdot, qddot. An artificially rich trajectory that violates machine constraints
is scientifically meaningless.

**V13. Preprocessing as an experiment.** From M1's finding that numerical
differentiation biased Izz by 56%: document the full chain
q_raw -> processed -> qdot -> qddot -> tau_model, and test whether processing
changes the recovered load.

**V14. Error budget.**

    e_total = e_numerical + e_measurement + e_Jacobian + e_model + e_observer

M2 deliberately drives numerical, measurement and model terms to ~0 so that
e_Jacobian is what remains - the thing under test. Each is then switched back on
individually.

---

## Group E — Validation rigour

**V15. Multiple criteria, not one number.** Recovery NRMSE; zero-load residual at
the numerical floor; linearity; frequency behaviour; configuration dependence;
model perturbation.

**V16. "Blind" corrected to "held-out."** We generate the force, so we know the
answer. Truly blind requires a third party to generate the load file with truth
revealed afterwards.

**V17. Fingerprint library.** Deliberately set dm, dIzz, dc, dfv and record each
residual signature. Turns M2 from demonstration into diagnosis, and is directly
reusable at M9.

**V18. Correlation rejected as diagnostic.** corr(e, qddot3) fails because qddot3
and cos q3 are themselves correlated on sinusoidal trajectories, so a non-zero
correlation does not establish causation. Replaced by regression onto the
sensitivity basis:

    e(t) ~ dm*phi_m(t) + dIzz*phi_I(t) + dc*phi_c(t) + dfv*phi_f(t)

**V19. Staged progression.** Zero force -> known load -> amplitude -> frequency ->
configuration -> identifiability -> model perturbation -> held-out.

---

## Group F — Scope

**V20. Point load, not distributed.** A known time-varying POINT load gives a
uniquely defined Q = J^T F. Distributed loading would require
Q = integral of J(s,q)^T f(s,t) ds, adding spatial distribution as another
unknown. Deferred to a later milestone.

---

## Group G — Added after reading the primary literature in full

**V21. Report peak-magnitude and peak-time error alongside NRMSE.**
Source: Feng et al. 2022, Machines 10:1203, who validate on peak value, peak
timing and trend. Rationale: a residual can track well on average while the
transient is shifted in time, and the diagnostic value of the channel lives in
the transients.

**V22. Payload is a MASS change, not a force.**
Source: Fu et al. 2022, CJME 35:147, who use m_d = m_0 + m_t with the payload mass
appearing in the inertia, Coriolis AND gravity terms.
A mass m_t at distance L contributes:
    m_t L^2 qddot3  +  2 m_t L d4dot q3dot  +  m_t g L cos q3
A force injection reproduces only the third term.
CONSEQUENCE, to be stated as a limitation: M2 validates the force-to-generalized-
force mapping. It does NOT validate payload recovery.

**V23. Two-station injection test.**
Source: Raza & Frimpong 2017, who track the payload centroid as a DYNAMIC point
that migrates as the dipper fills. Our formulation assumes a fixed application
point. Injecting the same force at two different stations tests whether the
Jacobian handles application-point variation before it matters at M7.

---

## Which revisions actually raise the work

Most of the twenty-three are competence. Six would be noticed by a reviewer:

  V7 + V8   the identifiability degeneracy and its resolution. No paper in the
            69-paper library does this. Palomba (2019, MSSP) met the same
            ill-conditioning and handled it by REGULARISATION, not by analysing
            what is and is not separable. This is the opening.
  V17       the fingerprint library - turns "does it work" into "what does
            failure look like"
  V14       the error budget - allows failure to be attributed, not just observed
  V22       stating that M2 does not validate payload recovery, before a reviewer
            finds it
  V11       admitting the frequency response is local, not broadband
  V6        correcting a naive blindness claim into a two-channel result

---

## Open, not yet resolved

1. THE TRILEMMA. Dynamic richness (Frimpong 2005) vs physical admissibility
   (Fu 2024, Bi 2020) vs identifiability (wants large d4 variation at SHORT
   extension, exactly where a real crowd is most constrained). Surfaced twice in
   the literature reading, resolved neither time. Must be formulated as a
   constrained design problem: maximise separability subject to the machine's real
   envelope. THIS IS THE ACTUAL RESEARCH CONTRIBUTION and it is still unwritten.

2. Two derivations owed: dp/dq3 (worked, but must be reproduced independently) and
   the four sensitivity functions phi_I, phi_m, phi_c, phi_f.

3. Positioning. All twenty-three revisions frame M2 as proving a method. After the
   literature review, its publishable content is the identifiability analysis -
   Palomba (2019) already performs through-dig estimation on hydraulic excavators,
   and the residual identity appears in US patent 12,480,286.
