# CHANGELOG — ShovelSimulator.slx

One entry per model version. The .slx in this folder is always the CURRENT one;
old versions live in git history, not as _v1/_v2/_(1) filename copies.

## v4 — 2026-07-06 — planar geometry made consistent (CURRENT)
- Rigid Transform1 rotation: reverted to None (superseded by gravity tilt).
- Middle Rigid Transform (the 1.5 m one): rotation StandardAxis +Y 90°
  → prismatic slide axis made RADIAL (was axial along the spin axis).
- Dipper Handle: COM [0 0 −2.82] (moved onto slide axis);
  MomentsOfInertia [287900 0 0] (moved to the axis parallel to spin).
- Mechanism Configuration: gravity [0 −9.80665 0] — in-plane
  ("tilt gravity, not the mechanism" — standard planar-model practice).
- Net lever arm: r = 1.5 + d4 − 2.82 = d4 − 1.32.
- Post-v4 wiring (2026-07-07): revolute 'b' → q_3_ddot, prismatic 'a' →
  d_4_ddot (each via PS-Simulink → To Workspace). Stop time 30 s.
- Validation state: BOTH equations R² = 1.000000 with sensed accelerations.
- LIMITATION to remember: planar trick valid only while the subsystem is
  planar; the 4-DOF extension with swing needs genuine 3-D frames.

## v3 — 2026-07-06 — Rasuli Table II parameters adopted
- Dipper mass 90,300 → 54,300 kg; COM −5 → −2.82 m; Izz 839,000 → 287,900.
- Reason: old values traced to NO source (checked Frimpong 2004/06/08 and all
  project docs — absent); Table II values are published and citable.
- Superseded the same day by v4 (geometry was still wrong).

## v2 — 2026-07-06 — friction + excitation fixes ("withFriction")
- Revolute DampingCoefficient: 0 → 66.322512 N·m/(deg/s) (= 3,800 N·m·s/rad).
  (First patch used 5.934 = 340×π/180 from a MISREAD of Table II; corrected.)
- Prismatic DampingCoefficient: 0 → 1,473 N/(m/s) (same correction history).
- Sine Wave1 (d4) amplitude 0.25 → 2 m — fixed regressor ill-conditioning
  (cond(Y) scaled ~1e4–1e5 → ~85).
- Rigid Transform1 rotation StandardAxis +X 90° added (partial axis fix —
  fixed gravity coupling into the revolute but left the slide axial;
  fully superseded by v4).

## v1 — original — as first built
- 2-DOF chain: World → Boom Base → Revolute (Rz) → Rigid Transform
  [1.5 0 0] → Prismatic (Pz) → Dipper Handle.
- Dipper: 90,300 kg, COM [−5 0 0], Izz 839,000 (provenance unknown).
- Gravity [0 0 −9.80665]; both joint axes on Simscape default Z
  → spin axis vertical (the root geometry bug), slide axial.
- Sine inputs: q3 45°±15°, d4 8.75±0.25 m, both ω = π/10; stop 10 s;
  solver VariableStepAuto.
