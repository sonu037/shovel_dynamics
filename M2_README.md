# M2 — Known External-Load Residual

## Status
M2 structure only. No experimental result is implied by this directory.

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

Distributed / contact loading, which would require Q = integral of J(s,q)^T f(s,t) ds,
is deliberately deferred to a later milestone.

## Known limitation to state in any write-up
Payload is physically a MASS change, not a pure external force. A mass m_t at the
bail contributes to the inertia, Coriolis AND gravity terms; a force injection
reproduces only the gravity contribution. M2 therefore validates the
force-to-generalized-force mapping; it does NOT validate payload recovery.

## M2 sequence
    00_design                 frozen experiment design specification
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

Diagnose contamination by regressing the residual onto the sensitivity basis
(phi_m, phi_I, phi_c, phi_f), NOT by correlating the error with qddot3: on
sinusoidal trajectories qddot3 and cos q3 are themselves correlated, so a
non-zero correlation does not establish causation.

## Repository rule
M1 is the frozen foundation. M2 may use the validated M1 model but must not
silently modify M1 evidence or parameters.

## Shared project files
LEARNING.md, research_log.md, claims.md, README.md remain at repository root
because they are continuous across M1 -> M2.
