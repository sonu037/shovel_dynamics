# Figure Register — master (git-tracked)
One row per figure. This file is the TRUTH; Figure_Register.xlsx for the PI
is GENERATED from it by comms/export_for_pi.py — never edited by hand.
Status: final | draft | planned.   Source = the script/file that regenerates it.
| ID | Title | Thesis Ch | Paper | Deck slide | Status | Source (regenerable from) | File |
|----|-------|-----------|-------|-----------|--------|---------------------------|------|
| FIG-001 | Research architecture (5 layers) | Ch 1 | P1,P2,P3 | 11 | final | comms/figures_src/fig_architecture.svg | thesis/figures/fig_architecture.pdf |
| FIG-002 | Milestone fishbone | Ch 1 | — | 17 | final | chat-designed SVG (thesis/figures/fishbone_thesis.svg) | thesis/figures/fishbone_thesis.pdf |
| FIG-003a | Machine anatomy (numbered callouts + key) | Ch 3 | P1 | 6 | final | comms/figures_src/fig1a_machine.py | comms/figures/fig1a_machine.pdf |
| FIG-003b | Stage-1 2-DOF abstraction (frames, lever arm r = d4 + c) | Ch 3 | P1 | 6 | final | comms/figures_src/fig1b_abstraction.py | comms/figures/fig1b_abstraction.pdf |
| FIG-004 | Excitation trajectories q3, d4 | Ch 3 | P1 | — | final | scripts/make_stage1_figures.m (A3) | experiments/2026-07-07_stage1_final/figures/fig_A3_excitation.pdf |
| FIG-005 | Revolute forward validation + residual | Ch 3 | P1 | 14 | final | scripts/make_stage1_figures.m (B1) | experiments/.../fig_B1_revolute_validation.pdf |
| FIG-006 | Prismatic (crowd) forward validation | Ch 3 | P1 | — | final | scripts/make_stage1_figures.m (B2) | experiments/.../fig_B2_prismatic_validation.pdf |
| FIG-007 | Identification CIs: gradient vs sensed | Ch 3 | P1 | 15 | final | scripts/make_stage1_figures.m (B3) | experiments/.../fig_B3_identification_ci.pdf |
| FIG-008 | Differentiation-noise comparison | Ch 3 | P1 | — | final | scripts/make_stage1_figures.m (C1) | experiments/.../fig_C1_gradient_vs_sensed.pdf |
| FIG-009 | Simulink/Simscape model diagram | Ch 3 | P1 | — | planned | export from ShovelSimulator.slx (print -dpdf) | model/ |
| FIG-010 | M2 load recovery: injected vs recovered | Ch 4 | P1 | — | planned | M2 experiment script (to be written) | experiments/M2_* |
| FIG-011 | 2-D kinematic skeleton | Ch 3 | — | — | draft | comms/figures_src/make_skeleton.py (script not yet in repo) | comms/figures/fig_skeleton_2dof.pdf |
| FIG-012 | Isometric 3-D skeleton | — | — | 6 | draft | comms/figures_src/make_skeleton_3d.py (script not yet in repo) | comms/figures/fig_skeleton_3d.pdf |
| FIG-B4 | Identification residual (tau_load channel, empty at Stage 1) | Ch 3 | — | — | final | validate_and_identify.m (plot output) | experiments/2026-07-07_stage1_final/figures/fig_B4_residual_channel.pdf |
