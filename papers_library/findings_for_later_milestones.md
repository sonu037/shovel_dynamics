# Findings Parked for Later Milestones

Extracted from full-text reading, August 2026. These do not affect M1 or M2 but
will be needed when the relevant milestone begins. Recorded now so they are not
rediscovered later.

===========================================================================
M7 — SOIL AND DIGGING FORCE MODEL
===========================================================================

## Two candidate soil models, of very different complexity

### (a) Simple linear form — Bi et al. 2020, Energies 13:3118, Eq. 14

    F_Et(t) = K * b * c(t)          tangential force at the bucket tip
    F_En(t) = lambda * F_Et(t)      normal force

  K       unit resistance to excavation, depends on soil type
  b       bucket width
  c(t)    current cutting thickness of the soil
  lambda  ratio of normal to tangential force

  b * c(t) is the area of the current digging cross-section.

  Their caveat: "large numbers of boulders or oversized rocks would lead to
  significant fluctuation of digging force during the excavation, which makes it
  difficult to describe the digging process numerically."

### (b) Bearing-capacity form — Feng et al. 2022, Machines 10:1203

    F11 = sigma * (gamma*g*d^2*N_gamma + c*d*N_c + gamma*v^2*d*N_a)

  sigma    dipper width
  gamma    material density
  d        excavating depth
  c        material cohesion
  v        speed of the dipper teeth
  N_gamma, N_c, N_a   bearing-capacity factors, given explicitly in the paper in
                      terms of digging angle epsilon, repose angle alpha, and
                      internal friction angle phi

  Structure: quadratic in depth (weight), linear in depth (cohesion), and a
  velocity-squared term (dynamic).

  Feng's total tangential resistance is F_tau = F11 + F12 + F13, where F12 is
  resistance caused by speed and F13 is resistance from material extrusion on
  the side walls.

### Cross-check with the electrical collaboration
The EKATWA internship report (reference/) uses F_dig = gamma*g*d^2 + c_soil*d + q.
The first two terms match Feng's form exactly; the velocity term is absent. So
the electrical team is using a truncated version of the same Balovnev-type
formulation. Worth reconciling at M5/M7.

## The payload application point MIGRATES

Raza & Frimpong 2017, IntechOpen, decompose excavation loading into six forces
(after Hemami): f1 payload weight, f2 material movement resistance, f3 wall
friction, f4 cutting/penetration, f5 payload inertia, f6 empty dipper.

Verbatim: "The forces, f1 and f5, are the dynamic forces, where f1 changes both
in magnitude AND THE POINT OF APPLICATION, and f5 depends on the bucket
acceleration."

And: "The centroid of each material geometry, a polygon inside the dipper, is
computed using a special algorithm. This centroid is a dynamic point, which is
used as the point of application for the dynamic force f1."

CONSEQUENCE FOR M7: the Jacobian becomes time-varying in a way M2 does not test,
because M2 uses a fixed application point. The two-station injection test in M2
partially probes this sensitivity.

CONFLICT TO RESOLVE: Feng et al. take the opposite approach - "We simplify the
gravity center of the excavated material to the gravity center of the dipper for
calculation purposes." Two published papers, two incompatible treatments of the
same physical quantity. A choice must be made and justified at M7.

## Assumptions the residual method could TEST rather than make

Raza & Frimpong set f2 = 0 by choosing a trajectory where the dipper does not
compress material, and f5 = 0 by assuming the dipper moves at uniform velocity.

f5 is payload inertia. Our residual channel would SEE f5 if it were non-zero.
So the method can test an assumption their method is forced to make. This is a
defensible point of differentiation for M7.

## Force direction rule

Bi et al. and Raza & Frimpong both state that the cutting force acts along the
TANGENT to the dipper trajectory at the tooth tip, resolved into components along
and normal to the dipper base. This is a physically motivated direction rule,
better justified than an arbitrary vertical or along-handle choice.

===========================================================================
M9 — HEALTH MONITORING AND THE PHYSICS-INFORMED LAYER
===========================================================================

## The only genuine wear signature found in the library

Bi et al. 2020, Eq. 14 commentary:

  "The parameter of lambda ... is the ratio between the tangential force and the
   normal force, whose value depends on the soil type AND THE SERVICE TIME OF THE
   BUCKET."

Bucket service time means TOOTH WEAR. So wear changes lambda, which changes the
RATIO of normal to tangential force, which changes the DIRECTION of the resultant
at the tooth tip, which changes how the load splits between Q_q3 and Q_d4.

This is a GEOMETRIC signature of wear, not a magnitude change. Two channels
observing the same force at a changing angle should show a correlated shift that a
single-channel monitor could not distinguish from a change in soil.

This is the strongest health-monitoring hook found in the entire 69-paper library
and should be developed at M9.

## Model bias is an acknowledged problem

Long et al. 2025, J. Mech. Sci. Tech., acknowledge that physics-based digging
models carry bias from idealised assumptions and incomplete knowledge. Fu et al.
2022 state that "some parts of load are non-observable and latent, which cannot be
measured from actual system sensing."

Supports the deliberate parameter-perturbation experiment (M2 level 05) and the
physics-informed residual layer at M9.

## History dependence

Fu et al. 2022: "during the digging process, the mass, moment of inertia, and
centroid of the dipper-medium system are constantly changing, and the required
digging force depends not only on the current working conditions but also on the
HISTORICAL digging trajectory."

This is why they use an LSTM. It is also the deepest difficulty in the programme:
the plant drifts while the disturbance is being estimated. Published
acknowledgement that it is a recognised problem, not an invented one.

## Comparative baselines M9 must beat

  Fu et al. 2022 HPINN vs classical analytical:
    RMSE 31.8% lower, MAE 27.9% lower, R^2 8.6% higher

  Their criticism of classical methods: "the classical method assumes or
  simplifies the dynamic digging process as a steady process and does not consider
  the influence of time-varying dipper penetration and digging speed. Therefore,
  this approach can predict only A RANGE of crowd force and hoist force and cannot
  sensitively capture small changes."

  Hu & Xiong 2017, IEEE TIE: semiparametric model (rigid-body physics + learned
  residual) with a disturbance Kalman filter. The classical baseline a PINN
  approach must outperform.

===========================================================================
M5 — ROPE AND MOTOR CURRENT CHAIN
===========================================================================

## Modern machines are AC, not DC

Rasuli's machine used DC drives. Current ultra-class shovels (P&H 4100XPC,
4800XPC, Cat 7495) use AC drives with regenerative capability. The
current-to-torque mapping differs, so K_H and K_C from Rasuli are structurally
correct but numerically specific to DC machines.

## Drive-side measurement is how the field does it

Fu et al. 2022 obtained digging forces "uploaded through the drive inverter" on
their 1:7 scale WK-55 - i.e. force inferred from motor current. That is exactly
the M5 chain, and it is established practice.

## Field-measured drive ratings, WK-55 (Bi et al. 2020, Table 2)

  Hoist: 33,695 N m torque, 1,987 kN force, 930 rpm, 1,735 kW
  Crowd:  9,356 N m torque, 1,002 kN force, 772 rpm,   754 kW

===========================================================================
M8 — FIELD VALIDATION
===========================================================================

## Accuracy benchmark to beat

Commercial payload systems achieve approximately 1-3% of full scale when
calibrated. Research systems (calibrated dynamic modelling, 2025, 25 t excavator)
report about 1%. Haul-truck strut scales at the load point achieve only about
20%, which is the historical motivation for on-shovel weighing.

The relevant target for M8 is therefore 1-2% of payload against weighbridge
ground truth. Achieving that DURING ACTIVE DIG - where commercial systems work in
swing or carry - would be the distinguishing result.

## Field validation standard in this literature

Feng et al. 2022 validate on peak value, peak timing and overall trend, reporting
approximately 5% peak-force deviation, under 2% peak-time deviation, and R^2
around 0.9. Wu et al. 2025 report R^2 greater than 0.85 with about 5% deviation.

This is the field's standard of evidence: trend agreement and single-digit
percentage deviation. M1's machine-precision verification is a different and
stronger kind of claim, but it is a claim about the EQUATIONS, not about the
machine.
