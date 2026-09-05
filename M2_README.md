# M2 — Known External-Load Residual

## Status

MS02P01 and MS02P02 CLOSED. MS02P03 CLOSED 2026-09-02.

MS02P06 partially complete, run out of sequence.

MS02P04, MS02P05, MS02P07 and MS02P08 not started. The excitation-trilemma
gate in 00_design remains OPEN.

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

Results to date (see research_log.md for full context and caveats):

    MS02P01  01_zero_force
             numerical floor: 2.00e-06 N m (q3), 2.91e-10 N (d4);
             relative 7.04e-13 and 6.46e-16

    MS02P02  02_known_load
             exact recovery: slope -1.000000, 1-R^2 = 7.67e-24 (q3),
             max error / peak reference = 2.7e-12

    MS02P03  03_amplitude_sweep
             CLOSED 2026-09-02. Six verified force levels: 10, 50, 100,
             200, 500 and 1000 kN. Achieved applied force matched the
             requested value at every level; Fx remained zero. Recovery
             slope was -1.000000 on both generalized-force channels at
             every amplitude. Absolute recovery error was approximately
             2.00e-06 N m, varying by about 0.1% across the 100x amplitude
             range, so relative error decreased approximately as 1/F over
             the tested range.

    MS02P06  06_identifiability
             station confounding on three trajectories. Assuming a station
             2.82 m wrong still gives 1-R^2 = 2.69e-03 (wide stroke),
             4.70e-05 (narrow, off-band) and 3.00e-05 (Bi-number-matched d4 interval, 9.50-10.04 m). An 89.77-fold collapse from wide stroke to that numerical interval. Bi et al.'s 9.50-10.04 m values are optimized initial polar-diameter values rho_0, not a validated d4 envelope.

             Load-vs-mass separability diagnosed by correlation:
             r(load, full s_dm) = 0.822 (wide), 0.99999 (Bi-number-matched d4 interval).
Scope of these results. Module 02 verifies the force-to-generalized-force
mapping and the subtraction identity under motion-prescribed joints with exact
sensing. It does NOT establish load identifiability: the residual contained one
unmodelled term by construction, and the Simscape oracle shares assumptions
A1-A3 with the analytical model, so M2 is structurally blind to errors in those.

The load-vs-mass results are DIAGNOSTIC (correlation only). Formal conditioning
and uncertainty analysis are outstanding.

Blocking: the I_zz ambiguity flagged in shovel_params.m must be resolved before
the sensitivity matrix is built, since the inertia column's scaling depends on it.

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