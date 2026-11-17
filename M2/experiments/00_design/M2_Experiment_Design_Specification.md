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
Note: produce TWO maps, one per generalized coordinate. Observability
redistributes between coordinates rather than vanishing - at q3 -> 90 deg,
Q_q3 -> 0 but Q_d4 -> -F.

STATUS:                 FILLED 2026-11-17 (drafted with Claude; every input
                        checked against claims.md, shovel_params.m,
                        shovel_params_wk20.m and Rasuli 2014 Figs. 7-9).
                        FROZEN 2026-11-17 by Suhail.
                        The 2026-11-09 runs (M2_05_d4bias_{7p75,8p75,9p77}m.mat)
                        were RUN before these fields were filled - PROCESS
                        DEFECT, same as modules 01, 02 and 04. They are kept as
                        a PILOT: they show exact recovery (slope -1.000000) at
                        three d4 biases at q3 = 45 +/- 15 deg, and nothing more.
                        They are not this module's result.

Question:               Across the configurations a working shovel actually
                        occupies, where does each generalized coordinate observe
                        a vertical load at the bail station, and where can that
                        load be separated from an error in the empty-dipper
                        mass M_d when both channels are used together?

Hypothesis:             Observability of the load redistributes between the two
                        channels but never vanishes (note above). SEPARABILITY
                        of load from mass error does vanish: in the quasi-static
                        limit the two unknowns load the two channels in the same
                        ratio wherever sin q3 cos q3 = 0, i.e. on q3 = 0 (handle
                        horizontal), which lies inside every working envelope
                        below. Near q3 = 0 the only separating information is
                        the dynamic crowd-channel term d4ddot - r q3dot^2, which
                        is small at field-observed rates.

Setup:                  PART 1 - analytic maps (the main result).
                        Grid: q3 in [-60, +70] deg, 1 deg steps; d4 in
                        [4.0, 11.0] m, 0.1 m steps (project datum: s = d4 + 1.5,
                        r = d4 - 1.32). Parameters from shovel_params.m
                        (Machine A). The maps depend on the geometry
                        (x_ref = 1.50, L_COG = 2.82) and not on M_d, so they
                        hold for the WK-20 while its estimated geometry equals
                        Machine A's (as set in shovel_params_wk20.m).

                        Three envelopes are overlaid, each with its status:
                        E0  M2 test region [design choice, modules 02-04]:
                            q3 = 30-60 deg; d4 = 6.75-10.75 m (wide) and
                            9.50-10.04 m (Bi band).
                        E1  Field-observed, Machine A [source fact, read from
                            figures - mark V only after reading the PDF
                            yourself]: Rasuli 2014 Fig. 7 (dipper lifting)
                            theta3 ~ -21 to -3 deg, peak q3dot ~ 4.6 deg/s
                            (0.080 rad/s), peak q3ddot ~ 3.5 deg/s^2;
                            Fig. 9 (swing) theta3 ~ -18 to -22 deg;
                            Fig. 8 d4 ~ 8.55-9.10 m in his datum = 7.05-7.60 m
                            in ours, peak d4dot ~ 0.27 m/s, |d4ddot| ~ 0.2 m/s^2.
                            Lifting and swing only - Rasuli recorded digging
                            but did not use it, because the external forces
                            during digging were not measurable.
                        E2  WK-20 declared envelope [design choice/estimate]:
                            q3 in [-45, +25] deg [D]; d4 in [4.2, 8.8] m
                            [ESTIMATE]; |d4dot| <= 0.54 m/s [V, OEM sheet];
                            |d4ddot| <= 0.5 m/s^2 [D, Bi 2020, WK-55];
                            |q3dot| <= 0.45 rad/s [D, no source].
                            CITATION DEFECT: shovel_params_wk20.m tags the
                            -45 deg bound "Rasuli 2014 Fig. 6", but Fig. 6 is
                            the estimated-payload plot. Corrected 2026-11-17.

                        PART 2 - Simscape spot checks (independent validation of
                        the analytic sensitivities). M2_LoadInjection.slx,
                        100 kN vertical at the bail station, motion-prescribed.
                        Requires the q3 Sine Wave Bias and Amplitude to be
                        parameterised (B_q3, A_q3) exactly as A_d4/B_d4 were on
                        2026-09-07: a read-back at 45/15 deg must reproduce
                        M2_100kN_0p1111Hz_sp1p50 to every printed digit, and a
                        control run must differ.
                        Spot centres: q3 = -40, -20, -10, -3, 0, +10, +20, +45
                        deg at d4 = 7.3 m; plus q3 = 0 deg at d4 = 4.5 and 8.5 m.
                        Excitation about each centre: A_q3 = 2 deg,
                        A_d4 = 0.25 m, omega_traj = 0.5 rad/s (peak
                        q3dot 0.017 rad/s, d4dot 0.125 m/s, d4ddot
                        0.0625 m/s^2 - inside both E1 and E2).

Variables:              Independent: configuration centre (q3, d4); envelope
                        (E0/E1/E2) for overlays only.
                        Held fixed: model file, parameter file, force amplitude,
                        force frequency (2*pi/9), force station, solver, trim
                        (0 s, claims.md 2026-09-02), noise-free sensing, Fx = 0.

Equations:              Load columns (per unit Fy):
                          s_F3 = (d4 + 1.5) cos q3                    [m]
                          s_F4 = sin q3                               [-]
                        Mass-error columns (per unit dM_d), r = d4 - 1.32:
                          s_M3 = r^2 q3ddot + 2 r d4dot q3dot + g r cos q3
                                                                      [m^2/s^2]
                          s_M4 = d4ddot - r q3dot^2 + g sin q3        [m/s^2]
                        Quasi-static determinant (static columns only):
                          det = s_F3 s_M4 - s_M3 s_F4
                              = (s - r) g sin q3 cos q3
                              = 2.82 g sin q3 cos q3 = 13.83 sin(2 q3)  [m^2/s^2]
                        Whitened column angle: theta_W = angle between
                        W^(1/2) s_F and W^(1/2) s_M, W = diag(sigma_tau3^-2,
                        sigma_f4^-2). The ZERO SET of theta_W does not depend
                        on W; its size does.

Metric:                 Map 1: |s_F3|(q3, d4).
                        Map 2: |s_F4|(q3, d4).
                        Map 3: quasi-static |det| and theta_W, for noise ratios
                          L_n = sigma_tau3 / sigma_f4 = 1, 3, 10 m [design
                          choice; replace with measured drive noise when
                          available]. L_n = 10 m reproduces the earlier s_bar
                          for comparability.
                        Map 4: dynamic separability bound near q3 = 0,
                          max |d4ddot - r q3dot^2| / g, under E1 and E2 rates.
                        Spot checks: slope, intercept, 1-R^2, max err / peak ref,
                          per channel (as module 02).
                        Code check: the new map script reproduces the 12-case
                          table of m2_formal_identifiability_svd.m.

Prediction:             Written before running; hand calculation.
                        P1  Map 1 never vanishes inside E1 or E2:
                            min |s_F3| >= (4.2 + 1.5) cos 45 deg = 4.03 m.
                        P2  Map 2 is zero on the line q3 = 0 for every d4;
                            at q3 = -3 deg, |s_F4| = 0.052.
                        P3  The quasi-static det is zero on q3 = 0 for every d4
                            and every weighting, and does not depend on d4.
                            Relative to the E0 centre (q3 = 45 deg) it is
                            0.67 at -21 deg, 0.34 at -10 deg, 0.10 at -3 deg,
                            0 at 0 deg.
                        P4  Rasuli's own lifting trajectory (E1, -21 to -3 deg)
                            runs from 0.67 to 0.10 of the E0 separability,
                            heading to zero.
                        P5  Dynamic rescue near q3 = 0 is at most
                            0.2 + 6.28 x 0.080^2 = 0.24 m/s^2 (2.4 % of g) at E1
                            rates, and at most 0.5 + 7.48 x 0.45^2 = 2.0 m/s^2
                            (20 % of g) at the E2 limits. At field-observed
                            rates the collapse at q3 = 0 is predicted to stand.
                        P6  Spot checks recover slope -1.000000 on both channels
                            at every centre, including q3 = 0, where the
                            d4-channel load signal is small but exactly recovered.

Acceptance criterion:   Validity of the module (NOT the hypothesis):
                        (a) B_q3/A_q3 read-back reproduces the baseline to every
                            printed digit, and the control run differs;
                        (b) spot checks: slope -1.000000 to six decimals and
                            1-R^2 at the Module-01 floor, both channels, every
                            centre;
                        (c) the numeric zero locus of the det map lies within
                            one grid step (1 deg) of q3 = 0 for every d4;
                        (d) the code check reproduces the 12-case table, with
                            cond_joint within 1 % (NarrowBI omega 1.36 -> 3.77).
                        P1-P5 may be refuted. A refutation is a result and is
                        recorded as one.

Failure interpretation: (a) fails -> parameter not connected (silent-override
                            class, 2026-09-02). STOP.
                        (b) fails only near q3 = 0 -> check the q3 datum (from
                            horizontal?), gravity axis and force axis before any
                            physical explanation.
                        (c) fails -> the sin q3 dependence of Q_d4 is wrong in
                            the model or the script: sign or frame convention.
                            STOP.
                        (d) fails -> the new script does not compute the same
                            metric. Fix it before using any map.
                        If P2-P5 hold: payload/tare separation on the WK-20
                        cannot rely on quasi-static two-channel separability
                        near a horizontal handle. It must come from the cycle
                        phase (empty swing-return tare) or from configuration
                        diversity. Record this as a design constraint for later
                        milestones.

Scope:                  Fx = 0 and one force station. Says nothing about payload
                        versus digging force (two force components and a
                        migrating centroid) - that needs the two-component
                        follow-on below. Noise-free; the noise weighting is a
                        design choice until drive noise is measured.

Repository location:    M2/experiments/05_configuration_sweep/rev2/ (the
                        2026-11-09 pilot files stay where they are).

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