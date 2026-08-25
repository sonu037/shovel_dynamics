# M2 — Literature Findings That Change the Design

Extracted from full-text reading of the primary sources, August 2026.
Only findings that alter M2's design or interpretation are recorded here.
Soil-model and health-monitoring material is parked separately in
papers_library/findings_for_later_milestones.md.

---

## 1. The operating envelope is now a measured quantity, not a guess

Bi, Wang, Wang, Yao & Hall (2020), Energies 13:3118, performed field tests on a
WK-55 cable shovel at the Anjialing mining site specifically to establish
trajectory-optimisation constraints. Table 2 of that paper gives:

                                          Hoist        Crowd
  Maximum mechanism velocity              1.76 m/s     0.76 m/s
  Maximum mechanism acceleration          0.8 m/s^2    0.5 m/s^2
  Maximum motor torque                    33,695 N m   9,356 N m
  Maximum motor force                     1,987 kN     1,002 kN
  Maximum motor speed                     930 rpm      772 rpm
  Maximum motor power                     1,735 kW     754 kW
  Average digging cycle                   20 s
  Minimum digging cycle                   15 s

Table 7 gives the operational extension band: the initial polar diameter rho_0
ranges from 9.50 m to 10.04 m across every tested pile profile and digging
distance, with an optimised digging phase of approximately 10 s.

CONSEQUENCE FOR M2: the excitation trajectory must satisfy
    |d4_dot|  <= 0.76 m/s
    |d4_ddot| <= 0.5 m/s^2
over a cycle of order 10-20 s.

CAUTION: rho_0 = 9.5-10.04 m is the OPERATIONAL band observed in the field, not
the mechanical stroke limit. The machine evidently does not work near full
retraction. This matters directly for identifiability (see item 4).

---

## 2. The startup-transient contradiction is documented in the field literature

Bi et al. 2020, Section 2, verbatim:

  "both the speed curves start and end with a non-zero value. However, the working
   mechanism is commonly locked in a fixed position at the beginning and the ending
   moment of the digging process, which means the values of the instantaneous speed
   and acceleration should be zero. Therefore, the contradiction between the digging
   theory and real operational situation makes it hard to implement the theoretical
   speed curves."

This is precisely the mismatch that produces our M1 startup spike: the commanded
trajectory demands non-zero velocity at t = 0 while the machine begins at rest.

CONSEQUENCE: the T_TRIM decision is no longer merely a housekeeping choice. It
engages a documented contradiction in cable-shovel trajectory theory. The report
should cite this rather than simply asserting that the transient was removed.

Their formal remedy (Eqs. 34-35) is a set of endpoint and continuity constraints:
  - at the terminal moment t_d: velocity AND acceleration are zero for both drives
  - at intermediate transition moments t_1, t_4: velocity and acceleration are
    continuous
  - maximum acceleration bounded by the driving system

These are adopted as M2's admissible-trajectory conditions.

---

## 3. A published, physically-motivated excitation profile

Feng, Wu, Guo & Lin (2022), Machines 10:1203, Section 2.5:

  "The change trend for the crowd speed and hoist speed can be divided into three
   stages: acceleration, uniform speed, and deceleration. In order to reduce the
   sudden change in driving force caused by acceleration mutation, a smooth
   transition should be ensured at the corner of the speed, so the step driving
   function was used."

Their implementation (Table 5, high-speed case, 9 s cycle):
    crowd: step(time,0,0,3,30) + step(time,6,0,9,-30)

That is: ramp up over 3 s, hold, ramp down over 3 s, with smoothed corners so the
driving force has no jump discontinuity.

CONSEQUENCE: M2's excitation adopts this three-stage shape rather than a raw
sinusoid, satisfying both the endpoint conditions of item 2 and the acceleration
bound of item 1. This is a citable template, not an invented profile.

---

## 4. Excavating resistance is depth-driven, not speed-driven

Feng et al. 2022, Section 3.1, verbatim:

  "the excavating resistance is mainly affected by the excavating depth, and the
   impact of the speed is very small"

They observed that peak magnitude and peak location were "basically the same"
across high, medium and low speed combinations.

CONSEQUENCE FOR IDENTIFIABILITY: the real dig load is primarily a function of
POSITION, not velocity. A mass-parameter error is also position-dependent
(through cos q3). Neither therefore carries a distinguishing velocity signature.
This STRENGTHENS the finding that separability must come from geometry - the d4
spread - rather than from dynamic content.

Combined with item 1, this sharpens the trilemma: the machine operates at
rho_0 = 9.5-10 m, whereas |drho/dd4| = 2.82/(d4-1.32)^2 is largest at SHORT
extension. The operating band may lie outside the region of best separability.
This must be quantified, not assumed away.

---

## 5. Observability: the hoist channel carries roughly twice the power

Feng et al. 2022, Section 3.1: hoist power is about 2x crowd power during
excavation, "explaining that the hoist motion plays a leading role in the
excavating process."

CONSEQUENCE: in the two-channel observability maps, Q_q3 (driven through the
hoist) is the higher-signal channel. The redistribution at q3 -> 90 deg therefore
moves load INTO the weaker channel, not merely between equals.

---

## 6. Payload is a mass change, not an external force

Fu, Zhang, Cui & Song (2022), CJME 35:147, Eq. 17, uses m_d = m_0 + m_t, where
m_t is the loaded material. The payload mass appears in the inertia, Coriolis AND
gravity terms.

A mass m_t at distance L contributes three terms:
    m_t L^2 q3_ddot  +  2 m_t L d4_dot q3_dot  +  m_t g L cos q3

A pure force injection reproduces only the third.

CONSEQUENCE, to be stated as an explicit limitation: M2 validates the
force-to-generalized-force mapping. It does NOT validate payload recovery.

Possible extension: inject a MASS (time-varying body property) as well as a force,
and confirm the residual carries all three terms as predicted.

---

## 7. Conventional application point is the tooth tip, not the bail

Four independent papers place the digging resistance at the dipper tooth tip:
  - Raza & Frimpong 2017: resistive forces defined in frame {4} at the dipper tip
  - Feng et al. 2022: F_tau and F_n are "tangential and normal excavating
    resistances of the dipper tooth tip"
  - Fu et al. 2022: tangential force enters with lever arm (L_d + r)
  - Bi et al. 2020: "digging forces acting on the tip of the bucket"

M2 injects at the BAIL station to keep eccentricity e = 0, consistent with report
assumption A10.

CONSEQUENCE: this is a deliberate deviation from field convention and must be
justified explicitly in any write-up, not passed over. The two-station injection
test (design evolution V23) partly addresses the sensitivity this creates.

---

## 8. Independent confirmations of M1 modelling choices

  2-DOF (1R-1P) abstraction:
    Feng et al. 2022: "the ECS scale model can be regarded as a 1R-1P system"
    Fu et al. 2022: the digging mechanism treated as two degrees of freedom
    -> answers "why only 2 DOF?" with published precedent

  Datum-offset construction:
    Feng et al. 2022 use d2 + d20, d20 being the initial handle elongation
    -> structurally identical to our d4 + 1.5

  Hoist-rope angle:
    Fu et al. 2022 Eq. 17: hoist enters as F_r * r * sin(theta), theta measured
    between the hoist rope and the dipper handle
    -> confirms our alpha_3 formulation and that it is measured from the handle,
       not from horizontal

  Residual architecture:
    Raza & Frimpong 2017: D(Theta)Theta_ddot + C(Theta,Theta_dot) + G(Theta)
                          = F - F_load
    -> rearranged, this is tau_load = tau_measured - tau_model, published
       independently in the mining literature

  Inertia is non-negligible:
    Bi et al. 2020: "The mass of the bucket-handle assembly for a large cable
    shovel cannot be ignored as it will cause non-negligible inertia force"
    -> supports the dynamic-richness requirement
