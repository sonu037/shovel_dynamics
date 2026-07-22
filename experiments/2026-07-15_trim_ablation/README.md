Question: does including the first 0.5 s (solver startup) change the identification?
Varied: T_TRIM = 0 vs 0.5 s; same simulation output, same script v5.1, all else identical.
Verdict: No effect at Stage 1 - both conditions recover all parameters exactly (R2=1.000000, RMSE=0 both equations). No-trim keeps 373/374 and improves conditioning (13.4 vs 82.2): startup accelerations enrich excitation. Trim retained as hygiene for noisy/differentiated data (field phase).
Date: 2026-07-22 (rerun of the 2026-07-14 observation)
Script: validate_and_identify v5.1 (2026-07-15)
