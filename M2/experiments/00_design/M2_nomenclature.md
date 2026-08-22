# M2 — Nomenclature

Every symbol used in M2, with units. Anything appearing in an M2 equation or
figure must have an entry here first.

## Coordinates

| symbol | meaning | units |
|---|---|---|
| q3 | saddle rotation angle, measured from horizontal | rad |
| d4 | crowd extension of the handle, from the 1.5 m datum | m |
| q3dot, q3ddot | angular velocity, angular acceleration | rad/s, rad/s^2 |
| d4dot, d4ddot | crowd velocity, crowd acceleration | m/s, m/s^2 |

## Distances — three different ones

| symbol | from the pivot to | value | used in |
|---|---|---|---|
| r | the COG | d4 - 1.32 | all M1 terms |
| r_bail | the bail | d4 + 1.5 | M2 injection point |
| L_COG_Offset | COG to bail | 2.82 m | Rasuli Table II |

The 1.5 m datum is a modelling choice from the v4 Simscape construction, not a
machine dimension. Only r is physical; the offset is absorbed exactly into the
identified constant c = -1.32 m. Blind identification returned c = -1.3200 with
standard error 5e-15. Rasuli measures d4 from the pivot itself, so his coordinate
and ours differ by this constant. Feng et al. 2022 use the same construction
(d2 + d20, where d20 is the initial elongation).

## Forces and torques

| symbol | meaning | units |
|---|---|---|
| F = (Fx, Fy) | injected external force, world frame | N |
| Q_q3 | generalized force on q3 — a TORQUE | N m |
| Q_d4 | generalized force on d4 — a FORCE | N |
| tau_measured | Simscape joint-torque sensor reading (the oracle) | N m |
| tau_model | what the frozen M1 equations say the observed motion required | N m |
| tau_load | the residual, tau_measured - tau_model | N m |

## Geometry and mapping

| symbol | meaning | units |
|---|---|---|
| p | position of the bail in world coordinates | m |
| J | Jacobian, dp/dq — how p moves per unit change in each coordinate | — |
| delta | a VIRTUAL (imagined) infinitesimal change, not a real motion | — |
| dp/dq3 | how the bail moves when only q3 changes | m/rad |
| dp/dd4 | how the bail moves when only d4 changes | dimensionless |

## Parameters — all frozen from M1

| symbol | meaning | value |
|---|---|---|
| M_d | combined dipper + handle mass | 54,300 kg |
| I_zz | inertia about the COG | 287,900 kg m^2 |
| c | net lever constant, r = d4 + c | -1.32 m |
| f_v3 | viscous friction, saddle | 3,800 N m s/rad |
| f_v4 | viscous friction, crowd | 1,473 N s/m |
| g | gravitational acceleration | 9.80665 m/s^2 |

## Analysis symbols

| symbol | meaning |
|---|---|
| rho(d4) | separability ratio, (d4+1.5)/(d4-1.32) |
| s_load | the load's signature in the residual over time |
| s_dm | a mass error's signature in the residual over time |
| dm, dIzz, dc, dfv | small deliberate errors in the identified parameters |
| phi_I, phi_m, phi_c, phi_f | sensitivity functions — what multiplies each parameter |
| kappa(S) | condition number of the sensitivity matrix |
| NRMSE | normalised root-mean-square error |
