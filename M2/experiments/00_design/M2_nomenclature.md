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

| symbol | meaning | units |
|---|---|---|
| rho_station(d4) | force-application-station ratio, (d4+1.5)/(d4-1.32) | dimensionless |
| rho_LM(t) | load-vs-mass sensitivity ratio, s_load / s_dm | s^2/m |
| s | scalar station distance, d4 + Station (recover_load.m line 169). For the normal bail injection Station = 1.5 m, so s = r_bail. Stored in R.sig | m |
| s_load | saddle-residual load sensitivity, r_bail cos q3 = (d4+1.5) cos q3 | m |
| s_dm | mass-error signature, r^2 q3ddot + 2 r d4dot q3dot + g r cos q3 | m^2/s^2 |
| s_grav | gravity part of s_dm, g r cos q3, r = pivot-to-COG | m^2/s^2 |
| s_in | inertia part of s_dm, r^2 q3ddot, r = pivot-to-COG | m^2/s^2 |
| s_co | Coriolis part of s_dm, 2 r d4dot q3dot, r = pivot-to-COG. Named s_co in the code; do NOT write s_cor | m^2/s^2 |
| s_dyn | dynamic part of s_dm, s_in + s_co | m^2/s^2 |
| s_full | complete mass sensitivity, s_grav + s_dyn. Mathematically identical to s_dm; s_full is the name sensitivity_diagnostic.m uses for the analysis column | m^2/s^2 |
| omega_traj | MECHANISM trajectory angular frequency; drives Sine Wave (q3) and Sine Wave1 (d4) | rad/s |
| omega_F | EXTERNAL FORCE angular frequency; drives Sine Wave2. Set by the 'Frequency' argument of recover_load. INDEPENDENT of omega_traj | rad/s |
| R_dyn | RMS(s_dyn) / RMS(s_grav) | dimensionless |
| dm, dIzz, dc, dfv | small deliberate errors in the identified parameters | — |
| phi_I, phi_m, phi_c, phi_f | sensitivity functions — what multiplies each parameter | — |
| S | sensitivity matrix; columns are parameter-error signatures | mixed |
| kappa(S) | condition number of the sensitivity matrix | dimensionless |
| NRMSE | normalised root-mean-square error | % |

Notes on rho_station and rho_LM (rev 24). These are two different quantities and
were previously both called rho.

  rho_station is the ratio of the same load's signature evaluated at two candidate
  application stations. It is a pure geometry ratio and is what the wrong-station
  experiment measures. Verified: the regression slope of a wrong-station fit
  equals the x^2-weighted mean of rho_station over the trajectory, to 7e-06 (wide
  stroke) and 6e-06 (narrow stroke).

  rho_LM compares two different physical causes — an external load and a mass
  error. It carries units of s^2/m because s_load is torque per newton (m) while
  s_dm is torque per kilogram (m^2/s^2). It is TRAJECTORY-DEPENDENT in general,
  not a function of d4 alone. Only under the quasi-static approximation, where
  s_dm reduces to g r cos q3, does it collapse to (1/g) rho_station(d4).

s_dm is the full derivative d(tau3)/d(M_d). M_d appears in the inertia, Coriolis
and gravity terms of the saddle equation, so all three contribute. See module 06.

Decomposition: s_dm = s_in + s_co + s_grav. All three use r = d4 - 1.32, the
pivot-to-COG distance. Note that s_load uses r_bail = d4 + 1.5 instead — the two
sensitivities are evaluated at DIFFERENT stations, which is the whole reason
rho_station exists.

Caution on the letter s. Bare `s` is the scalar station distance in metres, used
in the Jacobian. The `s_*` family are torque sensitivities: s_load in m, and
s_dm, s_grav, s_in, s_co, s_dyn, s_full in m^2/s^2. They are unrelated quantities
sharing a prefix.

## Internal collision: three meanings of rho

    rho_station(d4)   station ratio, (d4+1.5)/(d4-1.32)              dimensionless
    rho_LM(t)         s_load / s_dm                                  s^2/m
    rho (local)       sqrt(R^2), correlation coefficient, used at  recover_load.m line 458 for the implied lag    dimensionless

NEVER write bare rho in M2 prose.

## Symbol collisions with cited sources

| our symbol | our meaning | conflicting use | source |
|---|---|---|---|
| c | net lever constant, -1.32 m | soil cutting thickness c(t) in F_Et = K b c(t); also c = tan(beta_f - delta_d) | Bi et al. 2020, Eqs 3 and 14 |
| rho | not used bare; see rho_station, rho_LM above | polar diameter, pivot-to-tooth; also rho_soil, bulk density 1700 kg/m^3 | Bi et al. 2020 |
| L | not used bare in this project | L_1, L_2 crowd-arm lengths | Frimpong 2008 |

Bi's rho_0 is what caused the 9.50-10.04 m misattribution: those are initial
polar diameters from Tables 6-7, not a d4 crowd band.

## omega_traj vs omega_F

Until 2026-09-04 these were conflated. The 'Frequency' argument of recover_load
sets omega_F only; the trajectory blocks held pi/10 as a literal, so no M2
experiment before MS02P04 varied the mechanism frequency. Both are now
model-workspace variables and must be reported separately in every result.