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

**V24. Full mass-error sensitivity derived.**

The previous M2 identifiability analysis used the gravity-only approximation

    s_dm = g r cos(q3),

    r = d4 - 1.32.

This is incomplete because M_d appears in three terms of the saddle equation:

    tau3 =
        (I_zz + M_d r^2) q3ddot
        + 2 M_d r d4dot q3dot
        + M_d g r cos(q3)
        + f_v3 q3dot.

Therefore the complete mass sensitivity is

    s_dm = d(tau3)/d(M_d)

         = r^2 q3ddot
           + 2 r d4dot q3dot
           + g r cos(q3).

The three terms are respectively the inertia, Coriolis and gravity
contributions.

The previous gravity-only expression is retained only as a QUASI-STATIC
APPROXIMATION. Whether that approximation is adequate must be evaluated
against the actual trajectory and the measurement/model uncertainty relevant
to the estimator; it must not be justified solely by comparison with the
gravity term.

CONSEQUENCE: the previous conclusion that cos(q3) cancels from the
load/mass-error ratio and that separability depends only on d4 variation is
conditional on the gravity-only quasi-static approximation. It is NOT the
general result for the full mass sensitivity.

The dimensionless station ratio and the trajectory-dependent load/mass ratio
are kept distinct:

    rho_station(d4) = (d4 + 1.5)/(d4 - 1.32)

and

    rho_LM(t) = s_load(t)/s_dm(t).

This is a mathematical correction to the M2 analysis, not a new experimental
result. The full derivative must be used for the primary load-vs-model-error
identifiability analysis.

---

**V25. Full mass sensitivity tested; dynamic separation mechanism not demonstrated.**

The full mass-error sensitivity introduced in V24 contains two dynamic terms:

    s_inertia  = r^2 q3ddot

    s_Coriolis = 2 r d4dot q3dot.

This suggested a hypothesis:

    the dynamic components of the mass-error signature might provide an
    additional direction that allows an external vertical load to be
    distinguished from a mass error even when d4 variation is small.

This was tested using two existing M2 trajectories.

For the WIDE trajectory,

    d4 = 6.7505 to 10.7495 m,

the mean-removed Pearson correlations were:

    r(load, gravity)  = 0.53888577
    r(load, inertia)  = 0.96130851
    r(load, Coriolis) = 0.46882128
    r(load, full mass)= 0.82221570.

For the BI operational-band trajectory,

    d4 = 9.500067 to 10.039933 m,

the corresponding values were:

    r(load, gravity)  = 0.9999729034
    r(load, inertia)  = 0.99861
    r(load, Coriolis) = 0.161
    r(load, full mass)= 0.9999919261.

The dynamic-to-gravity RMS ratios were:

    WIDE:
        inertia  = 2.563048 %
        Coriolis = 0.979654 %
        dynamic  = 2.484269 %
        gravity:dynamic = 40.25:1

    BI BAND:
        inertia  = 2.353814 %
        Coriolis = 0.127604 %
        dynamic  = 2.341438 %
        gravity:dynamic = 42.71:1.

The diagnostic was originally inspected using raw cosine similarity. This was
rejected because non-zero signal means can dominate the result. For example,
the wide load/inertia raw cosine similarity was -0.411699 whereas the
mean-removed Pearson correlation was +0.9613.

Pearson correlation is used for the primary shape comparison because the
intended recovery formulation contains a free intercept; a constant offset is
therefore absorbed by the estimator and does not constitute an independent
separating direction.

RESULT:

The dynamic terms do NOT demonstrate a useful second separation mechanism on
the trajectories tested.

In particular, on the BI-band trajectory the full mass-error signature is
almost perfectly correlated with the load signature:

    r(load, full mass) = 0.9999919261.

Therefore the inference

    "the load cannot produce the dynamic signature, therefore the dynamic
     terms provide separability"

is NOT supported by these experiments.

This does NOT establish that dynamic excitation can never improve
identifiability. The present tests use controlled single-frequency
trajectories. Alternative excitation remains an open experimental question.

The wide and BI-band trajectories also differ in d4 operating range and crowd
velocity, so their difference cannot be attributed to excitation frequency
alone.

The previously quoted approximately 250:1 gravity-to-dynamic ratio was an
unmeasured estimate. The measured values are approximately:

    40.25:1  (wide trajectory)
    42.71:1  (BI-band trajectory)

These ratios are trajectory-dependent and must not be treated as fixed
machine properties.

SCIENTIFIC STATUS:

    V24 mathematical correction:
        CONFIRMED.

    Dynamic-separation hypothesis:
        NOT DEMONSTRATED on the tested trajectories.

    Load/model-error identifiability:
        OPEN.

NEXT:

    Test alternative admissible excitation and evaluate the resulting
    sensitivity matrix using formal conditioning, rather than relying on
    correlation alone.

---

## Which revisions actually raise the work

Most of the twenty-three earlier revisions are competence. Several revisions
are especially important to the scientific contribution:

    V7 + V8
        identification of the load/model-error degeneracy and its treatment.

    V17
        fingerprint-library formulation: turns "does it work?" into
        "what does each failure/model-error source look like?"

    V14
        explicit error budget, allowing failure to be attributed rather than
        merely observed.

    V22
        explicitly stating that M2 does not yet validate payload recovery.

    V11
        recognizing that frequency response is local to the tested trajectory,
        rather than making a broadband claim.

    V6
        correcting a naive blindness claim into a two-channel observability
        result.

    V24
        correcting the mass-error sensitivity from the gravity-only expression
        to the complete inertia + Coriolis + gravity derivative.

    V25
        experimentally testing the resulting dynamic-separation hypothesis and
        finding that it is not demonstrated on the trajectories tested.

---

## Open, not yet resolved

1. THE CONSTRAINED IDENTIFIABILITY PROBLEM.

   The earlier formulation attributed separability primarily to d4 variation
   under a gravity-only approximation. V24 showed that the full mass
   sensitivity also contains inertial and Coriolis terms; V25 showed that
   these terms do not provide a demonstrated second separation mechanism on
   the tested single-frequency trajectories.

   The remaining question is therefore broader:

       Can an admissible trajectory be designed that makes the external-load
       sensitivity sufficiently independent of the relevant model-error
       sensitivities?

   This should be formulated as a constrained sensitivity/conditioning
   problem:

       minimise kappa(S)

       subject to the machine's feasible trajectory envelope.

   The answer is not yet known.

2. TWO DERIVATIONS.

   dp/dq3 is worked but must be reproduced independently.

   The four sensitivity functions

       phi_I, phi_m, phi_c, phi_f

   still need to be derived and verified.

3. POSITIONING.

   Earlier M2 revisions framed M2 primarily as proving a load-estimation
   method. After the literature review, the potentially publishable
   contribution is more specifically the identifiability analysis:

       which external-load and model-error signatures are distinguishable,
       under which trajectories, and subject to which physical constraints?

   Palomba (2019, MSSP) provides an important comparison because it addresses
   ill-conditioning in through-dig estimation using regularisation. The
   present work instead investigates the physical source of the
   non-identifiability and asks whether trajectory/excitation design can
   reduce it.

4. PHYSICAL OPERATING ENVELOPE.

   The Bi et al. 2020 values currently used to define the BI-band trajectory
   must be verified directly from the source PDF before being treated as
   authoritative machine limits.

   In addition, P&H 2100-specific trajectory limits remain required for the
   final constrained-design gate. WK-55 limits must not be silently treated
   as P&H 2100 limits.

5. EXCITATION.

   Frequency variation, multisine/chirp excitation and transient/realistic
   trajectories remain to be tested. The purpose is to determine whether
   alternative admissible trajectories reduce sensitivity collinearity.

6. MODEL-ERROR FINGERPRINTS.

   The mass-error sensitivity has been tested, but the broader fingerprint
   library remains incomplete. Planned perturbations include inertia,
   geometry/centre-of-mass parameters, viscous friction and Coulomb friction.

7. FORCE DIRECTION.

   The current baseline primarily investigates vertical loading. Horizontal
   force and combined planar loading remain to be tested before any broader
   planar-load identifiability claim is made.

8. UNCERTAINTY AND VALIDATION.

   Measurement noise, model mismatch and parameter uncertainty remain to be
   introduced systematically after the deterministic sensitivity structure
   has been established.

9. HELD-OUT VALIDATION.

   A held-out experiment remains required before claiming generalisation of
   the identification procedure.