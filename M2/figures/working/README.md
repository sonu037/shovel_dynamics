# M2 working figures — Case 01 (PRE-FILTER-FIX)

Saved 2026-08-25 during the original interactive session, before
plot_m2.m existed. Filenames were MATLAB defaults (untitled*.fig)
and have been renamed from their axis titles.

## IMPORTANT: these show the CONTAMINATED run

All nine figures were generated from the Case 01 data containing the
0.082 s force-path lag (Simulink-PS Converter2, second-order filter,
tau = 0.05 s, group delay ~0.1 s). That run gave NRMSE 2.1032% and
slope -0.996490. After the filter was disabled the same case gives
slope -1.000000 and max error / peak ref = 2.7e-12.

Superseded by M2/experiments/02_known_load/M2_100kN_0p1111Hz_sp1p50.mat
and the figures produced by plot_m2.m. DO NOT CITE these as results.

## Why they are kept

They are the visual record of the pre-fix state. M2case01_residual_vs_JTF
and M2case01_normalized_recovery are the clearest illustration of a point
recorded in LEARNING.md: two curves at 1e5 magnitude differing by 2% look
identical to the eye. None of these nine figures could distinguish a 2%
error from a 1e-12 error. That is why the log-scale error plot
(M2_fig2_recovery_error) exists.

## Contents

| file | shows |
|---|---|
| M2case01_kinematic_trajectory | q3, d4, q3dot, d4dot |
| M2case01_accelerations | q3ddot, d4ddot |
| M2case01_measured_efforts | measured tau3 and f4 |
| M2case01_measured_vs_M1model | measured vs frozen M1 prediction |
| M2case01_residuals | e_tau3 and e_f4 |
| M2case01_analytical_JTF | Q_q3 and Q_d4 from the Jacobian mapping |
| M2case01_residual_vs_JTF | residual overlaid on -J'F (main validation) |
| M2case01_residual_regression | residual vs predicted load, scatter |
| M2case01_normalized_recovery | both channels normalised |

Status: [RECORD] — not thesis figures, not appendix figures.
