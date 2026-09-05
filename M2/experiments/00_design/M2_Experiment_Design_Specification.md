# M2 Experiment Design Specification

STATUS: template. Fill each module before running it. Once filled, FREEZE.
The purpose of freezing is to stop the design drifting while building starts.

Per-module format:
  Question | Hypothesis | Setup | Variables | Equations | Metric |
  Acceptance criterion | Failure interpretation | Repository location

---

## OPEN GATE BEFORE ANY CODE — the excitation trilemma

Three requirements pull against each other and none is soft:

  1. DYNAMIC RICHNESS      quasi-static testing does not exercise the residual
                           (Frimpong 2005 shows kinematic/dynamic response varies
                           strongly through the excavation trajectory)
  2. PHYSICAL ADMISSIBILITY trajectories must respect real limits on q, qdot, qddot
                           (Fu 2024, Bi 2020, Zhang 2022)
  3. IDENTIFIABILITY        two candidate separation mechanisms exist, and they
                           are not equivalent:

                           MECHANISM A - geometric. Through d4 variation.
                           |drho_station/dd4| = 2.82 / (d4 - 1.32)^2 is largest at
                           SHORT extension - exactly where a real crowd is most
                           constrained.

                           MECHANISM B - dynamic. Through qddot3, q3dot, d4dot.
                           A vertical load produces NO inertial and NO Coriolis
                           signature; a mass error produces both. These are terms
                           the load cannot imitate, and they do not require d4
                           spread. See module 06.

RESOLUTION REQUIRED: state the P&H 2100 crowd stroke and velocity limits, then
choose the d4 range and excitation that maximise separability subject to those
limits. Report the resulting conditioning. This is a constrained design problem,
not three separate constraints to list.

Note that requirement 1 and requirement 3 are in tension only under Mechanism A.
Under Mechanism B, dynamic richness WOULD SERVE identifiability if the mechanism
holds. Which mechanism dominates within the admissible envelope is an open
question and is the substance of this gate.

This is the gate. Nothing downstream is meaningful until it is answered.

---

## 01_zero_force
Question:
Hypothesis:
Setup:
Variables:
Equations:
Metric:                 zero-load residual, absolute AND normalised by torque scale
Acceptance criterion:   at or below the independently established numerical floor
                        (do not freeze 1e-12 as a universal constant - it depends
                        on solver, tolerances, signal magnitude, sampling)
Failure interpretation: injection point, reference frame, sign convention,
                        attachment body, or time alignment is wrong. STOP.
Repository location:    M2/experiments/01_zero_force/

## 02_known_load
Question:
Hypothesis:
Setup:
Variables:
Equations:
Metric:                 NRMSE + peak-magnitude error + peak-time error
Acceptance criterion:
Failure interpretation:
Repository location:    M2/experiments/02_known_load/

## 03_amplitude_sweep
Question:               Does residual recovery remain exact and proportional as the
                        injected load amplitude varies?

Hypothesis:             Recovery remains linear with load amplitude, with an ideal
                        regression slope of -1 under the adopted residual sign
                        convention. Any increase in relative error at low amplitude
                        is expected to arise from the approximately fixed
                        Module-01 numerical floor while the recoverable signal
                        decreases, rather than from physical nonlinearity.

Setup:                  Baseline wide trajectory, unchanged: q3 = 45 +/- 15 deg,
                        d4 = 8.75 +/- 2 m, both at omega = pi/10. Vertical force at
                        the bail station s = d4 + 1.5. Force frequency 2*pi/9 rad/s.
                        Trim per Module 01 measurement.

Variables:              F0_y ONLY. Amplitudes 10, 50, 100, 200, 500, 1000 kN.
                        Everything else held fixed, including the model file.
                        The 1000 kN point is a deliberate numerical stress test,
                        not a claim about a realistic operating load.

Equations:              Q_q3 = (d4+1.5) Fy cos q3
                        Q_d4 = Fy sin q3
                        residual = tau_measured - tau_model = -J'F

Metrics:                slope; intercept; 1-R^2 (direct, ssRes/ssTot); NRMSE;
                        max err / peak ref; peak-magnitude error; peak-time error.
                        Across the sweep also report max abs(slope + 1) and the
                        trend of relative error against amplitude.

Acceptance criterion:   (a) slope = -1.000000 to six decimals at EVERY amplitude;
                        (b) intercept remains below the Module-01 residual floor
                        when expressed in the same units;
                        (c) max err / peak ref remains consistent with a
                        floor-limited increase at low amplitude rather than a
                        systematic amplitude-dependent scaling error;
                        (d) no systematic trend in slope with amplitude.

Failure interpretation: A departure from linearity in a motion-prescribed model
                        with a linear mapping indicates an IMPLEMENTATION defect
                        — saturation, overflow, or a unit error — not a physical
                        nonlinearity. Investigate the force path before recording
                        it as a result. First compare any low-amplitude deviation
                        against the Module-01 numerical floor and any high-amplitude
                        deviation against possible force-path saturation or
                        numerical limits. If a transition does appear, refine with
                        additional amplitudes around it.

Repository location:    M2/experiments/03_amplitude_sweep/

## 04_frequency_sweep
Question:               Does varying the mechanism trajectory frequency improve
                        separability between the external-load sensitivity and
                        the mass-error sensitivity?
Hypothesis:             Raising omega_traj grows the dynamic terms as omega^2.
                        If the inertia or Coriolis column is directionally
                        distinct from the load, separability should improve.
Setup:                  Sine Wave and Sine Wave1 Frequency parameterised to the
                        model-workspace variable omega_traj. Amplitudes and
                        biases held fixed. omega_F fixed at 2*pi/9. Force 100 kN
                        vertical at the bail station. omega_traj = 0.314, 0.5,
                        0.7, 1.0, 1.36 rad/s on two stroke families: WIDE
                        d4 = 8.75 +/- 2 m, NARROW d4 = 9.77 +/- 0.27 m.
Variables:              omega_traj and crowd stroke. All else fixed.
Equations:              s_load = (d4+1.5) cos q3
                        s_dm = s_in + s_co + s_grav, r = d4 - 1.32
                        R_dyn = RMS(s_in + s_co) / RMS(s_grav)
Metric:                 mean-removed Pearson r(s_load, s_grav), r(s_load, s_in),
                        r(s_load, s_full); R_dyn; recovery slope and 1-R^2.
Acceptance criterion:   RETROACTIVE - see failure note. Recovery slope must be
                        -1.000000 at every frequency on both channels (met).
                        R_dyn must follow omega^2 scaling, confirming the
                        trajectory changed as intended (met: 18.3-18.7x over a
                        4.33x frequency increase).
Failure interpretation: PROCESS DEFECT: this module was RUN before these fields
                        were filled, violating the fill-freeze-run rule. Same
                        defect as modules 01 and 02. Fields written retroactively
                        2026-09-05 and marked as such rather than presented as
                        pre-registered. The scientific prediction that WAS made
                        before the narrow run (r(s_load,s_in) stroke-independent
                        at ~0.96) was refuted; see research_log 2026-09-04.
Repository location:    M2/experiments/04_frequency_sweep/
Note: results are LOCAL to the operating trajectory. Do not claim broadband
performance from single-frequency tests.

Note (rev 24/25): this module's purpose is enlarged. Beyond robustness, it tests
whether alternative excitation can improve load/model-error identifiability.

The inertial contribution to s_dm scales with qddot3, so changing excitation
frequency changes the magnitude and temporal structure of the dynamic
sensitivity terms. Whether this produces a sufficiently independent sensitivity
direction is an open experimental question and must be evaluated using
correlation and, ultimately, sensitivity-matrix conditioning subject to the
physical velocity and acceleration limits.

The existing wide and Bi-source trajectories also differ in d4 range and crowd
velocity, so their difference cannot be attributed to excitation frequency alone.

## 05_configuration_sweep
[same fields]
Note: produce TWO maps, one per generalized coordinate. Observability
redistributes between coordinates rather than vanishing - at q3 -> 90 deg,
Q_q3 -> 0 but Q_d4 -> -F.

## 06_identifiability
### METHOD — frozen 2026-09-02; not yet executed

1. Residual:        r(t) = tau_measured - tau_model

2. Load column:     s_F = d(tau3)/d(Fy)        [m]

3. Mass column:     s_m = d(tau3)/d(M_d)       [m^2/s^2], using the full three
                   terms of the mass sensitivity.

4. Further columns:
                   d(tau3)/d(I_zz) = q3ddot
                   d(tau3)/d(c)
                   d(tau3)/d(f_v3)
                   and the CONSTANT column for the regression intercept.

5. Scaling:         The sensitivity columns carry different physical units.
                   Therefore kappa(S) on the raw columns measures unit mismatch
                   as well as physical conditioning. Scale each column using a
                   resolution prior (dF, dM_d, dI_zz, etc.) chosen as an engineering
                   requirement and JUSTIFIED IN WRITING. Record that the resulting
                   kappa depends on those choices.

6. Intercept:       Include the constant column explicitly, or project the
                   sensitivity columns off the constant direction. Do not leave
                   the intercept implicit.

7. Decomposition:   S = U Sigma V'.
                   Report singular values, kappa, and the ANGLE between the load
                   sensitivity column s_F and the subspace spanned by the
                   remaining nuisance columns. The angle is scale-invariant per
                   column and is the preferred measure; kappa is reported
                   alongside for comparability with the literature.

8. Decision:        Well conditioned -> potentially identifiable.
                   Poorly conditioned -> ambiguity remains.
                   Near rank-deficient -> not separable under this excitation.

9. Scope:           Every identifiability verdict is LOCAL to the trajectory
                   tested. State the complete trajectory, amplitude, frequency,
                   configuration, and model conditions with every reported number.

Prerequisites:      Modules 03, 04, and 05 must be complete before executing
                   Module 06. Executing Module 06 before them yields a conditioning
                   result for one amplitude, one frequency, and one configuration
                   without establishing how those factors affect identifiability.

Core result to test:

  s_load = (d4 + 1.5) cos q3                                   [m]

  s_dm   = r^2 qddot3 + 2 r d4dot q3dot + g r cos q3           [m^2/s^2]
           where r = d4 - 1.32

  rho_LM(t) = s_load / s_dm                                    [s^2/m]

s_dm is the FULL derivative d(tau3)/d(M_d). M_d appears in the inertia, Coriolis
and gravity terms of the saddle equation, so all three contribute. Derivation
verified independently, rev 24.

QUASI-STATIC APPROXIMATION. Retaining only the gravity term requires

    r^2 qddot3        <<  g r cos q3
    2 r d4dot q3dot   <<  g r cos q3

This is TRAJECTORY-DEPENDENT and must be evaluated over the operating interval,
not assumed. Under the approximation, s_dm -> g r cos q3, the cos q3 factor
cancels from rho_LM, and

    rho_LM  ->  (1/g) * rho_station(d4) = (1/g) * (d4 + 1.5)/(d4 - 1.32)

so separability reduces to d4 variation alone (Mechanism A).With the full s_dm there is no common cos q3 factor, so rho_LM depends on
q3, q3dot, q3ddot, d4 and d4dot. Dynamic excitation therefore provides a
CANDIDATE second separation mechanism (Mechanism B), but its practical
usefulness is not established. V25 found that the tested trajectories did not
produce demonstrated load/mass separation; alternative admissible excitation
remains to be tested.

Do NOT compute the final conditioning from the gravity-only form. Doing so would
yield a result that is a property of the truncation rather than of the machine.

Conditional on Fx = 0 throughout. For a general force the q3 dependence in s_load
returns and the analysis must be redone.

Distinguish structural identifiability (d4 must take at least two distinct values)
from practical identifiability (d4 must vary enough for acceptable conditioning).

MAGNITUDE. Practical identifiability is set by the size of the discriminating
signal relative to the residual UNCERTAINTY, not relative to the gravity term.
Measured: RMS(s_dyn)/RMS(s_grav) = 0.024843 (wide trajectory) and 0.023414
(Bi band), i.e. 40.25:1 and 42.71:1. TRAJECTORY-DEPENDENT; re-evaluate for any
new excitation. An earlier figure of ~250:1 was an unmeasured estimate and must
not be used. An RMS sensitivity ratio is not equivalent to a torque-accuracy
requirement; exploitability depends on noise, model uncertainty and conditioning.

STATUS (2026-08-28): r(load, full s_dm) = 0.9999919 on the Bi-band trajectory and
0.8222 on the wide trajectory (mean-removed Pearson). Mechanism B not demonstrated
on either. See research_log.
Assessment sequence: magnitude -> shape/correlation -> conditioning -> uncertainty.
Do not stop at the magnitude ratio; it is not an identifiability measure.

## 07_model_perturbation
[same fields]
Build the fingerprint library: deliberately set dm, dIzz, dc, dfv and dfc and
record each residual signature. The Coulomb-friction perturbation uses the
identified/provisional fc3 and fc4 parameters but remains explicitly labelled as
a model-error sensitivity until the Coulomb-friction model is activated and
verified. Reusable at M9.

## 08_held_out
[same fields]
Naming: "held-out validation" is the honest label. "Blind" applies only if a third
party generates the load file and the truth is revealed afterwards.

## Follow-on: torque-driven variant
Repeat 02 with the joints torque-driven so the load alters the motion. This tests
whether tau_model generalises to an unseen trajectory - a stronger claim than the
motion-prescribed baseline.

## Follow-on: two-station injection
Inject the same force at two different stations along the handle. Consistent
recovery demonstrates the Jacobian handles application-point variation. Motivated
by Raza and Frimpong, who show the real payload centroid MIGRATES as the dipper
fills, so the application point is genuinely time-varying at M7.