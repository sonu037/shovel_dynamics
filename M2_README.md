# M2 — Known External-Load Residual

## Status

STATUS DEFINITIONS AND MILESTONE STATE (AUDITED 2026-11-17):

- MS02P01: CLOSED (Simulation-Verified). Numerical floor: 2.00e-06 N m (q3), 2.91e-10 N (d4).
- MS02P02: CLOSED (Simulation-Verified). Exact recovery under motion-prescribed trajectory: slope -1.000000, 1-R^2 = 7.67e-24 (q3).
- MS02P03: CLOSED (Simulation-Verified, 2026-09-02). Amplitude sweep 10-1000 kN linear (slope -1.000000). Absolute error is approximately 2.00e-06 N m, varying by 0.1% across a 100x amplitude range due to finite-precision round-off; relative error scales as 1/F (2.7e-11 to 2.7e-13). Matches claims.md.
- MS02P04: CLOSED (Simulation-Verified, 2026-09-04). Frequency sweep: s_in/s_co ratio invariant under common frequency; stroke-dependent identifiability measured across 10 trajectories.
- MS02P05: SPECIFICATION FROZEN (2026-11-17, Commit 2dd61f7). The 9 Nov runs (d4_bias 7.75, 8.75, 9.77 m at q3 = 45 deg) were pilot spot-checks. The complete P05 objective is the full 2D configuration sensitivity and quasi-static determinant map computed on the grid q3 in [-60, +70] deg (1 deg steps) and d4 in [4.0, 11.0] m (0.1 m steps), with operational envelopes E0 (pilot), E1 (Machine A field lifting [-21, -3] deg), and E2 (Machine B [-45, +25] deg, [4.2, 8.8] m) overlaid, explicitly investigating the horizontal handle singularity locus at q3 = 0 deg (det = 13.83*sin(2*q3) = 0).
- MS02P06: RE-EVALUATED SAME-MOTION CONDITIONING (2026-11-17). Formal SVD across 12 trajectories resolves single-channel saddle collinearity (theta = 0.23 deg) via joint 2-channel estimation:
  - Like-for-Like at handle rate comparable to Rasuli field data (omega_traj = 0.314 rad/s, peak q3dot ~ 0.082 rad/s): Single-channel kappa_2 = 497.71 -> Joint 2-channel kappa_2 = 22.45 (a 22x reduction at s_bar = 10 m, provisional until noise-based weighting via P05 Map 3).
  - Fast Trajectory Stress Test (omega_traj = 1.36 rad/s, 4.4x field speed): Single-channel kappa_2 = 153.71 -> Joint 2-channel kappa_2 = 3.77 (41x reduction).
  - Note on Dimensional Weighting: Both kappa_2 and the joint subspace angle depend on crowd weighting s_bar (ratio of N m to N): for Narrow Bi at omega_traj = 0.314 rad/s, theta_joint changes with s_bar (1.08 deg at s_bar=1 m, 5.10 deg at s_bar=10 m, 1.30 deg at s_bar=100 m; verified in m2_check_kappa_repro.m).
- MS02P07: FULL PARAMETER LEAKAGE REPORTED (2026-11-17). Basis regression on analytical inverse equations confirms that a +0.10 m error in COG lever arm c leaks severely into all parameters:
  - Rotational Inertia: +415,995 kg m^2 (+144.5% leakage)
  - Viscous Friction: +7,294 N m s/rad (+191.9% leakage)
  - Dipper Mass: +756.6 kg (+1.4% leakage)
  - Coulomb Friction: -368 N m
  - Scientific Takeaway: Front-end geometry and COG lever arms must be calibrated accurately before rotational inertia or friction identification can be trusted.
- MS02P08: AUDITED & RECLASSIFIED (2026-11-17):
  - Part A (In-Sample Archive Cross-Check): 17 motion-prescribed Simscape archives cross-checked; algebraic recovery holds to numerical precision (NRMSE_q3 < 4.3e-10%).
  - Part B (Forward Torque-Driven Code Verification): Initial execution in build_M2_TorqueDriven.m used an internal forward ODE solver with identical plant and observer parameters (code verification, not independent Simscape sensor validation). Planned for re-run against compliant Simscape Multibody plant with deliberately perturbed estimator parameters.

The excitation-trilemma gate is conditionally resolved; fails near q3 = 0 deg where quasi-static separability vanishes.
I_zz ambiguity was resolved on 2026-07-23 (claims.md line 13, status V).

## Downstream Milestone References
For downstream implementations and verified results, refer to their respective milestone documentation:
- **M3 (Momentum Observer):** Acceleration-free 30 Hz discrete observer suite in [M3/](M3/).
- **M4 (Noise and Uncertainty):** Six sensor stress and EIV protocols audited in [M4_README.md](M4_README.md).
- **M5 (Electromechanical Digital Twin):** Multi-rate drive model and adaptive innovation gating in [M5/](M5/).
- **M6 (3D Kinematics & Compliance):** 4-DOF Euler-Lagrange front-end model in [M6/](M6/).

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
             Relative error decreased as 1/F; absolute error varies by ~0.1%
             (finite-precision round-off).

    MS02P04  04_frequency_sweep
             10 Simscape trajectories (5 wide, 5 narrow). Dynamic ratio
             follows omega^2 scaling. Demonstrates common-frequency limitations.

    MS02P05  05_configuration_sweep
             Pilot spot-checks across d4 bias (7.75, 8.75, 9.77 m). Full 2D
             configuration and singularity map (q3 in [-45, +25] deg) frozen.

    MS02P06  06_identifiability
             Formal SVD suite on 12 trajectories. Like-for-like at 0.314 rad/s:
             saddle collinearity (kappa_2 = 497.71) resolved by joint 2-channel
             estimation (kappa_2 = 22.45, 22x reduction). Fast trajectory reaches
             kappa_2 = 3.77 under 1.36 rad/s (4.4x normal speed).

    MS02P07  07_model_perturbation
             10 parameter perturbation cases. Confirms severe parameter leakage
             under lever arm error dc = +0.10 m (+144% inertia, +192% viscous).

    MS02P08  08_held_out
             Part A: 17 in-sample Simscape archives cross-checked (NRMSE < 4.3e-10%).
             Part B: Torque-driven forward ODE code verification (planned for Simscape re-run).

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