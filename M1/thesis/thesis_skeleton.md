# Thesis Skeleton — fill sections WHILE milestones are fresh, not at the end

## Ch 1. Introduction
- Problem: no direct load measurement on shovels; motor current is the only free signal
- Objective: instantaneous load-torque recovery -> payload + health management
- Contributions list (grow as they happen; Stage 1 is rigor, not a contribution)

## Ch 2. Literature Review
- Source: papers_library/ (58-paper md, tiered ranking, 15-paper deep analysis)
- Anchors: Rasuli 2014 (regressor ID, swing-phase payload, limitations L1-L8);
  Fu 2022 HPINN (physics-embedded loss, Stage-4 blueprint);
  Tafazoli 1999 (linear-in-parameters trick); Frimpong (digging force models)

## Ch 3. Dynamic Modelling & Verification  <-- WRITE NOW, it is DONE
- 2-DOF Euler-Lagrange derivation (both equations, term meanings)
- Why friction is added post-Lagrangian (non-conservative)
- Simscape as independent oracle (why not circular)
- Debugging-as-isolation-protocol narrative (g=0 test, frequency doubling,
  perfect-fit-wrong-params geometry deduction) — research_log 2026-07-06
- Results: both eqs R^2=1.000000; identification exact; identifiability
  analysis (CI reading, cond(Y), Izz vs sensed accel) — claims.md
- Honest scope: simulation-only, planar, no Coulomb yet, zero-noise CIs

## Ch 4. Instantaneous Load-Torque Recovery (Phase C/D)  <-- NEXT
- Force injection; residual = tau_load(t)
- Momentum observer (acceleration-free) for field deployability
- Noise robustness (Fu-style +/-5..25% injection)

## Ch 5. Field Application (CMPDI data)
- Data per meetings/CMPDI_Data_Requirements doc
- Current->torque mapping incl. field-current dependence of K_H

## Ch 6. AI Layer (PINN residual / health indicators)
## Ch 7. Conclusions
