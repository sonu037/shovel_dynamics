# M2 — Known External-Load Residual

## Status

STATUS DEFINITIONS AND MILESTONE STATE (AUDITED & EXECUTED 2026-11-09):

- MS02P01: CLOSED (Simulation-Verified). Numerical floor: 2.00e-06 N m (q3), 2.91e-10 N (d4).
- MS02P02: CLOSED (Simulation-Verified). Exact recovery under motion-prescribed trajectory: slope -1.000000, 1-R^2 = 7.67e-24 (q3).
- MS02P03: CLOSED (Simulation-Verified, 2026-09-02). Amplitude sweep 10-1000 kN linear, absolute error constant at ~2.00e-06 N m.
- MS02P04: CLOSED (Simulation-Verified, 2026-09-04). Frequency sweep: s_in/s_co ratio invariant under common frequency; stroke-dependent identifiability measured across 10 trajectories.
- MS02P05: CLOSED (Simulation-Verified, 2026-11-09). Configuration sweep across crowd bias 7.75, 8.75, 9.77 m executed in Simscape and verified to machine precision (slopes -1.000000, 1-R^2 ~ 1e-24).
- MS02P06: CONDITIONAL RESULT (2026-11-09). Formal SVD across 12 trajectories: single-channel saddle collinearity (kappa_2 ~ 497.71, theta_min = 0.23 deg) is mitigated in joint 2-channel estimation (kappa_2 = 3.77, theta_min = 29.70 deg) using an explicit dimensional weighting factor s_bar = 10.0 m. Condition number is NOT scale-invariant; only theta_min is scale-invariant.
- MS02P07: ANALYTICAL REGRESSION VERIFIED (2026-11-09). 10 parameter perturbation cases evaluated on analytical inverse equations: basis regression recovers mass errors exactly under nominal geometry. Geometric error (+0.10 m in lever arm c) couples into mass, producing +756.6 kg misattribution.
- MS02P08: CLOSED (Dynamic Torque-Driven & In-Sample Consistency Verified 2026-11-09).
  - Part A (In-Sample Archive Consistency): 17 motion-prescribed archives show NRMSE_q3 < 4.3e-10%.
  - Part B (Forward Torque-Driven Dynamic Deflection, build_M2_TorqueDriven.m): Evaluated across 6 calibration and held-out cases (Impact Step, Stochastic, Oblique, Stall) for both P&H 2100BL and Amlohri TZ WK-20. Proves dynamic deflection (max |dq3| = 1.38 deg, max |dd4| = 0.084 m) and verified discrete 30 Hz observer tracking (lag-compensated NRMSE < 0.42%).

The excitation-trilemma gate is conditionally resolved via joint 2-channel ([tau3; s_bar*f4]) estimation with s_bar = 10.0 m.
I_zz ambiguity was resolved on 2026-07-23 (claims.md line 13, status V).

## Downstream Milestone Status
- M3 (Momentum Observer): Native discrete 30 Hz Simulink observer (build_M3_DiscreteObserver_30Hz.m) and 3-state KF-GMO verified across 5 canonical benchmark cases. Exact FOH matrix update achieves CovFricPct = 100.0%, CovKFCompPct = 100.0% (deterministic regimes) and 97.47% (under 3 mrad/s resolver noise, rounded to 97.5%), eliminates harmonic tracking lag (120.8 kNm -> 16.26 kNm), suppresses resolver velocity noise by 81.3% (193.15 kNm -> 36.10 kNm), and reaches zero-load noise floor = 53.47 Nm (BrA) / 1.36 Nm (KF-GMO).
- M4 (Noise and Uncertainty): Design stage; KF-GMO architecture formulated.

## Part identification

M2 is Milestone 02. Each experimental Part is identified as MS02Pxx,
where MS02 denotes Milestone 02 and Pxx denotes the Part number.

The Part identifier is the research identity; the existing directory names
remain unchanged.

    MS02P01  01_zero_force
    MS02P02  02_known_load
    MS02P03  03_amplitude_sweep
    MS02P04  04_frequency_sweep
    MS02P05  05_configuration_sweep
    MS02P06  06_identifiability
    MS02P07  07_model_perturbation
    MS02P08  08_held_out

Results summary:

    MS02P01  01_zero_force
             numerical floor: 2.00e-06 N m (q3), 2.91e-10 N (d4);
             relative 7.04e-13 and 6.46e-16

    MS02P02  02_known_load
             exact recovery: slope -1.000000, 1-R^2 = 7.67e-24 (q3),
             max error / peak reference = 2.7e-12

    MS02P03  03_amplitude_sweep
             Six verified force levels: 10, 50, 100, 200, 500 and 1000 kN.
             Recovery slope was -1.000000 on both generalized-force channels.
             Relative error decreased approximately as 1/F.

    MS02P04  04_frequency_sweep
             10 Simscape trajectories (5 wide, 5 narrow). Dynamic ratio
             follows omega^2 scaling. Demonstrates common-frequency limitations.

    MS02P05  05_configuration_sweep
             Simscape grid runs across d4 bias (7.75, 8.75, 9.77 m).
             Slopes -1.000000 on both channels, 1-R^2 in [6.20e-24, 9.23e-24].

    MS02P06  06_identifiability
             Formal SVD suite on 12 trajectories. Single-channel saddle
             collinearity (kappa_2 = 497.71) resolved by joint 2-channel
             estimation (kappa_2 = 3.77).

    MS02P07  07_model_perturbation
             10 parameter perturbation cases. Basis regression achieves exact
             parameter error attribution (dMd_hat exact to 0.0 kg error).

    MS02P08  08_held_out
             17 independent Simscape archives cross-validated with max NRMSE < 4.3e-10%.

Scope of these results. Module 02 and P05 verify the force-to-generalized-force
mapping and the subtraction identity under motion-prescribed joints with exact
sensing. Identifiability requires joint crowd-saddle estimation. Payload mass
recovery alters D(q), C(q,qdot), G(q) and is evaluated separately (Milestone M7).

## Scientific question
Can the frozen, independently validated M1 mechanical model recover a known
external generalized load from an independent measured torque signal?

Core residual:

    tau_load(t) = tau_measured(t) - tau_model(q(t), qdot(t), qddot(t))

The M1 model's equations and parameters are FROZEN. Its OUTPUT is not constant:
tau_model is recomputed from the observed state at every instant.

## Excitation mode (important)
The baseline M2 experiments are MOTION-PRESCRIBED: the joints follow a commanded
trajectory and Simscape computes the torque required. Injecting an external load
therefore does NOT change the trajectory; it raises the required torque. This
isolates the force-to-generalized-force mapping as the only thing under test.

A TORQUE-DRIVEN variant, in which the load does alter the motion, is run
separately (08_held_out / follow-on) as a model-generalisation check.

## Independence of the measurement
tau_measured must come from the Simscape joint-torque sensor (independent oracle).
It must NEVER be computed from q, qdot, qddot and the known parameters, which
would merely reproduce tau_model and make the test circular.

The model is given only kinematic observables. It is never given the injected
force. That the load influences qddot is not circularity: the model answers
"what torque would the nominal machine require for this motion?", and the excess
is the residual.

## Load definition (baseline scope)
Baseline M2 uses a KNOWN TIME-VARYING POINT LOAD applied at the bail station on
the handle axis, distance d4 + 1.5 m from the saddle pivot, with eccentricity
e = 0 (consistent with report assumption A10).

    p = [ (d4 + 1.5) cos q3 ; (d4 + 1.5) sin q3 ]
    Q = J^T F

    Q_q3 = (d4 + 1.5) * (Fy cos q3 - Fx sin q3)      [N m]
    Q_d4 = Fx cos q3 + Fy sin q3                      [N]

Naming note: "bail station" is a convenience label for the prismatic joint's
follower-frame origin. The 1.5 m datum is a v4 Simscape construction choice, not
a machine dimension (see M2_nomenclature.md), and correspondence to the physical
bail on a P&H 2100 is NOT established. Immaterial to M2, which verifies a
mapping; material at M7, where the payload centroid migrates as the dipper fills.

The station has been confirmed empirically rather than from the model tree: a
deliberately wrong station moves the regression slope to -1.378454, while the
assumed station d4 + 1.5 returns -1.000000.

Distributed / contact loading, which would require Q = integral of J(s,q)^T f(s,t) ds,
is deliberately deferred to a later milestone.

## Known limitation to state in any write-up
Payload is physically a MASS change, not a pure external force. A mass m_t at the
bail contributes to the inertia, Coriolis AND gravity terms; a force injection
reproduces only the gravity contribution. M2 therefore validates the
force-to-generalized-force mapping; it does NOT validate payload recovery.

## M2 sequence
    00_design                 frozen experiment design specification
    00_archive                superseded results, retained with provenance
    01_zero_force             numerical floor; GATE - do not proceed if this fails
    02_known_load             baseline recovery against truth
    03_amplitude_sweep        linearity
    04_frequency_sweep        local bandwidth (see caveat below)
    05_configuration_sweep    two per-coordinate observability maps
    06_identifiability        load vs parameter-error separability
    07_model_perturbation     fingerprint library (dm, dIzz, dc, dfv)
    08_held_out               held-out validation (blinded only if a third party
                              generates the load file)

Caveat for 04: the channel is configuration-dependent, so a frequency response is
LOCAL to the operating trajectory. Single-frequency results do not compose to
broadband performance; a separate multisine or chirp test is required for any
broadband claim.

## Metrics
Report NRMSE together with peak-magnitude error and peak-time error. A residual
can track well on average while the transient is shifted in time, and the
diagnostic information lives in the transients.

Report 1-R^2 computed directly as ssRes/ssTot, never as 1 - R2: at R^2 ~ 0.99995
the subtraction discards most of the significant figures, and the identifiability
results depend on that quantity.

Diagnose contamination by regressing the residual onto the sensitivity basis
(phi_m, phi_I, phi_c, phi_f), NOT by correlating the error with qddot3: on
sinusoidal trajectories qddot3 and cos q3 are themselves correlated, so a
non-zero correlation does not establish causation.

For shape comparison between sensitivity columns, use mean-removed Pearson
correlation, not cosine similarity on raw vectors. The estimator fits a free
intercept, so the constant direction carries no information and must be projected
out. The two metrics give different RANKINGS - see LEARNING.md.

## Repository rule
M1 is the frozen foundation. M2 may use the validated M1 model but must not
silently modify M1 evidence or parameters.

## Scripts
    M2/scripts/recover_load.m            run a case, compute metrics, save with
                                         git provenance
    M2/scripts/sensitivity_diagnostic.m  load-vs-mass column comparison
    M2/scripts/plot_m2.m                 publication figure set

## Shared project files
LEARNING.md, research_log.md, claims.md, README.md remain at repository root
because they are continuous across M1 -> M2.