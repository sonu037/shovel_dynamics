# End-to-End Test Suite Execution Report

**Date:** 2026-11-18 18:15:31 UTC  
**Environment:** Python 3.14.6, NumPy 2.5.1, SciPy 1.18.0  
**Platform:** Windows-11-10.0.26300-SP0  
**Overall Verdict:** ✅ **PASS** (100% Verified)  

---

## 1. Executive Summary

| Metric | Value |
|---|---|
| Total Tests Executed | `160` |
| Passed | `160` |
| Failed | `0` |
| Errors | `0` |
| Pass Rate | `100.00%` |
| Total Execution Duration | `6.50 s` |

### Tier Breakdown

| Tier | Name | Total | Passed | Failed | Errors | Status |
|:---:|---|:---:|:---:|:---:|:---:|:---:|
| `tier_1` | Tier 1: Feature Coverage (Nominal) | 68 | 68 | 0 | 0 | ✅ PASS |
| `tier_2` | Tier 2: Boundary, Corner & Negative Cases | 72 | 72 | 0 | 0 | ✅ PASS |
| `tier_3` | Tier 3: Pairwise Combinatorial Interactions | 13 | 13 | 0 | 0 | ✅ PASS |
| `tier_4` | Tier 4: Real-World Excavation Scenarios | 7 | 7 | 0 | 0 | ✅ PASS |

---

## 2. Invariants Enforced Across Test Tiers

1. **Mass Matrix Positive Definiteness**: $\min_q \lambda_{\min}(M(q)) \ge 1.4\times 10^3 > 0$.
2. **Passivity Skew-Symmetry**: $\|\dot{M} - 2C + (\dot{M}-2C)^T\|_F < 10^{-12}$.
3. **Uniform Time Grid Regularity**: $|\Delta t - 1/30| < 10^{-9}$ s over 1951 samples.
4. **Actuator OEM Physical Envelopes**: $|F_{crowd}| \le 745.0$ kN, $|v_{crowd}| \le 0.540$ m/s.
5. **Thermodynamic Energy Dissipation**: Mechanical power dissipation $P_{diss} \ge 0$.
6. **Unilateral Cable Tension**: Hoist wire-rope tension $T_{rope} \ge 0$.
7. **Dataset Provenance**: Exact MD5 hash match (`5eaa1131f51257e72b24b2596a83598d`).

---

## 3. Detailed Test Results Matrix

| # | Test ID | Tier | Features | Test Name | Duration | Status |
|---|---|:---:|:---:|---|:---:|:---:|
| 1 | `TC-T1-F01-01` | T1 | F1 | Mass Matrix Positive Definiteness on Operational Envelope | 0.003s | PASS |
| 2 | `TC-T1-F01-02` | T1 | F1 | Passivity Skew-Symmetry Invariant Verification | 0.000s | PASS |
| 3 | `TC-T1-F01-03` | T1 | F1 | 30 Hz Uniform Time Grid on P08B Rev2 Simscape Export | 0.118s | PASS |
| 4 | `TC-T1-F01-04` | T1 | F1 | Cryptographic Provenance Hash on Frozen Simscape Telemetry | 0.005s | PASS |
| 5 | `TC-T1-F01-05` | T1 | F1 | OEM Actuator Physical Bounds Enforcement | 0.119s | PASS |
| 6 | `TC-T1-F01-06` | T1 | F1 | Code Inspection: Elimination of 'bytes > 500' Naive Assertions | 0.002s | PASS |
| 7 | `TC-T1-F02-01` | T1 | F2 | Verification of Complete Frozen M1-M9 Roadmap Table | 0.001s | PASS |
| 8 | `TC-T1-F02-02` | T1 | F2 | Verification of Codified M5 Second Audit Findings | 0.001s | PASS |
| 9 | `TC-T1-F02-03` | T1 | F2 | Verification of Codified M6 Second Audit Withdrawals | 0.001s | PASS |
| 10 | `TC-T1-F02-04` | T1 | F2 | Verification of P08B Rev2 and MS03/04-IND1 Status Rows | 0.001s | PASS |
| 11 | `TC-T1-F02-05` | T1 | F2 | Verification of Unplanned DIRTY Rerun Disclaimers | 0.001s | PASS |
| 12 | `TC-T1-F03-01` | T1 | F3 | Verify M5 Report STATUS NOTICE Banner Presence | 0.001s | PASS |
| 13 | `TC-T1-F03-02` | T1 | F3 | Verify M5 Report AUDIT ADDENDUM Banner Presence | 0.001s | PASS |
| 14 | `TC-T1-F03-03` | T1 | F3 | Verify M6 Report STATUS NOTICE Banner Presence | 0.001s | PASS |
| 15 | `TC-T1-F03-04` | T1 | F3 | Verify M6 Report AUDIT ADDENDUM Banner Presence | 0.001s | PASS |
| 16 | `TC-T1-F03-05` | T1 | F3 | Verify M5 Forensic Audit Receipt Header Banner | 0.001s | PASS |
| 17 | `TC-T1-F03-06` | T1 | F3 | Verify M6 Forensic Audit Receipt Header Banner | 0.001s | PASS |
| 18 | `TC-T1-F04-01` | T1 | F4 | Calibration on Full 2-DOF Case E0 Steady-State Window | 0.118s | PASS |
| 19 | `TC-T1-F04-02` | T1 | F4 | Calibration on Full 2-DOF Case E1 Field Envelope | 0.116s | PASS |
| 20 | `TC-T1-F04-03` | T1 | F4 | Comparison of 2-DOF Stacking vs Revolute-Only Subspace | 0.117s | PASS |
| 21 | `TC-T1-F04-04` | T1 | F4 | Verification of Regressor Torque Reconstruction Fit Quality | 0.121s | PASS |
| 22 | `TC-T1-F04-05` | T1 | F4 | Calibration Invariance Under External Load Subtraction | 0.119s | PASS |
| 23 | `TC-T1-F05-01` | T1 | F5 | Zero-Payload Baseline Estimation (Tare Check, 0 t) | 0.136s | PASS |
| 24 | `TC-T1-F05-02` | T1 | F5 | Nominal 5-Tonne Payload Mass Estimation (5 t) | 0.136s | PASS |
| 25 | `TC-T1-F05-03` | T1 | F5 | Medium 10-Tonne Payload Mass Estimation (10 t) | 0.138s | PASS |
| 26 | `TC-T1-F05-04` | T1 | F5 | Heavy 15-Tonne Payload Mass Estimation (15 t) | 0.136s | PASS |
| 27 | `TC-T1-F05-05` | T1 | F5 | Comparative Estimator Verification: Branch B vs KF-GMO | 0.194s | PASS |
| 28 | `TC-T1-F05-06` | T1 | F5 | Kinematic Invariance Across Trajectory Envelopes (E0 vs E1) | 0.393s | PASS |
| 29 | `TC-T1-F06-01` | T1 | F6 | Case 1A Joint-Side Resolver Static Gravity Step Error > 1000 Nm | 0.000s | PASS |
| 30 | `TC-T1-F06-02` | T1 | F6 | Case 1B Motor-Shaft Encoder Static Gravity Step Error <= 10 Nm | 0.000s | PASS |
| 31 | `TC-T1-F06-03` | T1 | F6 | Case 1B Motor-Shaft Encoder Relative Dynamic Noise Floor <= 0.10% | 0.234s | PASS |
| 32 | `TC-T1-F06-04` | T1 | F6 | Noise Attenuation of KF-GMO Over Central Differencing >= 15.0 dB | 0.237s | PASS |
| 33 | `TC-T1-F06-05` | T1 | F6 | Quantization Word Length Monotonicity (10, 12, 14, 16 bits) | 0.000s | PASS |
| 34 | `TC-T1-F07-01` | T1 | F7 | Identification Under Realistic Noise (N1, N2, N4) Using IDIM-IV | 0.122s | PASS |
| 35 | `TC-T1-F07-02` | T1 | F7 | Verification of EIV Attenuation Bias in Unweighted OLS | 0.121s | PASS |
| 36 | `TC-T1-F07-03` | T1 | F7 | Non-Oracle Digital Twin Instrument Generation | 0.121s | PASS |
| 37 | `TC-T1-F07-04` | T1 | F7 | Orthogonality Condition Verification of Instrument Z | 0.000s | PASS |
| 38 | `TC-T1-F07-05` | T1 | F7 | Realistic Noise vs Legacy 10% AWGN Over-Estimation Ratio | 0.000s | PASS |
| 39 | `TC-T1-F08-01` | T1 | F8 | Matched Sinusoidal Excitation on Machine A and Machine B | 0.000s | PASS |
| 40 | `TC-T1-F08-02` | T1 | F8 | Adaptive Innovation-Gated KF-GMO Tracking of 50 kN Boulder Step | 0.018s | PASS |
| 41 | `TC-T1-F08-03` | T1 | F8 | Impact Phase Lag Reduction with Adaptive Innovation Gating | 0.184s | PASS |
| 42 | `TC-T1-F08-04` | T1 | F8 | Cross-Machine Generalization Under Matched Conditions | 0.002s | PASS |
| 43 | `TC-T1-F08-05` | T1 | F8 | Decoupling of Excitation Artifacts from Machine Structural Identity | 0.000s | PASS |
| 44 | `TC-T1-F09-01` | T1 | F9 | VFD Stator Frequency Sweep (0 to 60 Hz) Mapping 6th Harmonics | 0.000s | PASS |
| 45 | `TC-T1-F09-02` | T1 | F9 | Detection of DC Aliasing Resonance at Integer Multiples of 5 Hz | 0.000s | PASS |
| 46 | `TC-T1-F09-03` | T1 | F9 | Mapping of Observer Passband Intrusion (fc = 3.18 Hz) | 0.000s | PASS |
| 47 | `TC-T1-F09-04` | T1 | F9 | Analog Butterworth Pre-Filter Rejection > 30 dB at 69 Hz | 0.000s | PASS |
| 48 | `TC-T1-F09-05` | T1 | F9 | Passband Fidelity of Analog Pre-Filter at Digging Frequencies | 0.000s | PASS |
| 49 | `TC-T1-F10-01` | T1 | F10 | Cross-Correlation Peak at +10 Samples (+333.3 ms Physical Lag) | 0.001s | PASS |
| 50 | `TC-T1-F10-02` | T1 | F10 | Proof of Sign Convention Bug in Legacy delay_ms Formula | 0.001s | PASS |
| 51 | `TC-T1-F10-03` | T1 | F10 | DARE Dominant Closed-Loop Pole Frequency Calculation | 0.001s | PASS |
| 52 | `TC-T1-F10-04` | T1 | F10 | Analytical Phase Lag at Digging Frequency w_F = 0.698 rad/s | 0.001s | PASS |
| 53 | `TC-T1-F10-05` | T1 | F10 | Verification of Causal Physical Filter Operation (Lag > 0) | 0.000s | PASS |
| 54 | `TC-T1-F11-01` | T1 | F11 | Dimensional Sensitivity of Condition Number in Unweighted Regressor | 0.001s | PASS |
| 55 | `TC-T1-F11-02` | T1 | F11 | Noise Covariance Weighting Ln = sigma_tau3 / sigma_f4 approx 2.8 m | 0.000s | PASS |
| 56 | `TC-T1-F11-03` | T1 | F11 | Improvement of Joint 2-Channel Condition Number Over Single Saddle | 0.000s | PASS |
| 57 | `TC-T1-F11-04` | T1 | F11 | Parameter Subspace Angle theta_joint >= 2.5 deg | 0.000s | PASS |
| 58 | `TC-T1-F11-05` | T1 | F11 | Gauss-Markov Invariance Under Unit Scaling | 0.000s | PASS |
| 59 | `TC-T1-F12-01` | T1 | F12 | Peak Coulomb Force Mismatch Evaluation (166.7 Nm) | 0.000s | PASS |
| 60 | `TC-T1-F12-02` | T1 | F12 | Proof of Branch B Structural Immunity to eps_obs | 0.155s | PASS |
| 61 | `TC-T1-F12-03` | T1 | F12 | Branch A Zero-Crossing Torque Leakage | 0.000s | PASS |
| 62 | `TC-T1-F12-04` | T1 | F12 | KF-GMO Low-Pass Attenuation of Coulomb Mismatch Spike | 0.012s | PASS |
| 63 | `TC-T1-F12-05` | T1 | F12 | Integrated Payload Mass Error Invariance Under Coulomb Mismatch | 0.016s | PASS |
| 64 | `TC-T1-F13-01` | T1 | F13 | Twin 52 mm Wire-Rope Axial Stiffness Calculation | 0.000s | PASS |
| 65 | `TC-T1-F13-02` | T1 | F13 | Two-Mass Bounce Resonance Frequency Calculation (3.6 to 4.4 Hz) | 0.000s | PASS |
| 66 | `TC-T1-F13-03` | T1 | F13 | Rigid Observer Transmission of Bounce Mode (-4.6 dB) | 0.000s | PASS |
| 67 | `TC-T1-F13-04` | T1 | F13 | Rack-and-Pinion Backlash Contact Shock Pulse Modeling | 0.003s | PASS |
| 68 | `TC-T1-F13-05` | T1 | F13 | Notch Filter Rejection of Bounce Resonance (> 20 dB) | 0.000s | PASS |
| 69 | `TC-T2-F01-01` | T2 | F1 | Detection and Rejection of Indefinite Mass Matrix (Negative Mass) | 0.003s | PASS |
| 70 | `TC-T2-F01-02` | T2 | F1 | Detection and Rejection of Corrupted Coriolis Passivity | 0.000s | PASS |
| 71 | `TC-T2-F01-03` | T2 | F1 | Detection and Rejection of Jittered or Missing 30 Hz Grid Samples | 0.000s | PASS |
| 72 | `TC-T2-F01-04` | T2 | F1 | Detection and Rejection of Altered Provenance Hash | 0.006s | PASS |
| 73 | `TC-T2-F01-05` | T2 | F1 | Detection of OEM Physical Bound Exceedance (Fc = 755 kN) | 0.000s | PASS |
| 74 | `TC-T2-F01-06` | T2 | F1 | Defeating Trivial File-Size Bypass (Dummy 501-Byte File) | 0.000s | PASS |
| 75 | `TC-T2-F01-07` | T2 | F1 | Detection and Rejection of Non-Finite Values (NaN/Inf) in Signals | 0.000s | PASS |
| 76 | `TC-T2-F02-01` | T2 | F2 | Detection of Missing Milestone Row in Roadmap Table | 0.001s | PASS |
| 77 | `TC-T2-F02-02` | T2 | F2 | Detection of Unapproved Status Mutation (Flipping Frozen to Verified) | 0.001s | PASS |
| 78 | `TC-T2-F02-03` | T2 | F2 | Detection of Omitted M6 Second Audit Withdrawal Caveats | 0.001s | PASS |
| 79 | `TC-T2-F02-04` | T2 | F2 | Markdown Table Delimiter and Column Count Integrity | 0.001s | PASS |
| 80 | `TC-T2-F02-05` | T2 | F2 | Cross-Section Semantic Consistency Check | 0.001s | PASS |
| 81 | `TC-T2-F02-06` | T2 | F2 | Read-Only Registry Non-Mutation Enforcement | 0.001s | PASS |
| 82 | `TC-T2-F03-01` | T2 | F3 | Detection of Missing Notice Banner in Report | 0.001s | PASS |
| 83 | `TC-T2-F03-02` | T2 | F3 | Detection of Diluted or Tampered Status Language | 0.001s | PASS |
| 84 | `TC-T2-F03-03` | T2 | F3 | Detection of Banner Relocation (Banner Pushed Below Fold) | 0.001s | PASS |
| 85 | `TC-T2-F03-04` | T2 | F3 | Detection of Truncated Audit Receipt (Missing Header Notice) | 0.001s | PASS |
| 86 | `TC-T2-F03-05` | T2 | F3 | Case Sensitivity and Punctuation Rigidity of Banner Matchers | 0.000s | PASS |
| 87 | `TC-T2-F03-06` | T2 | F3 | Preservation of Subordinate Report Text | 0.001s | PASS |
| 88 | `TC-T2-F04-01` | T2 | F4 | Degenerate Horizontal Posture (q3 = 0 Static Pose) | 0.000s | PASS |
| 89 | `TC-T2-F04-02` | T2 | F4 | Zero-Acceleration Maneuver (Inertial Subspace Collapse) | 0.000s | PASS |
| 90 | `TC-T2-F04-03` | T2 | F4 | Extreme Velocity Zero-Crossing Behavior (Regularized Coulomb) | 0.000s | PASS |
| 91 | `TC-T2-F04-04` | T2 | F4 | Ill-Conditioned Trajectory Detection (kappa > 1e6) | 0.000s | PASS |
| 92 | `TC-T2-F04-05` | T2 | F4 | Robustness to Transient Start-Up Inclusion (t in [0, 5] s) | 0.125s | PASS |
| 93 | `TC-T2-F04-06` | T2 | F4 | Calibration Under Sensed Noise (Errors-in-Variables Attenuation) | 0.121s | PASS |
| 94 | `TC-T2-F05-01` | T2 | F5 | Singularity Near Vertical Pitch Posture (cos(q3) -> 0) | 0.000s | PASS |
| 95 | `TC-T2-F05-02` | T2 | F5 | Negative Payload Detection (-2.0 t Tare Drift Alarm) | 0.139s | PASS |
| 96 | `TC-T2-F05-03` | T2 | F5 | Extreme Sensor Noise Stress Test (10x N1 Speed, 10x N2 Torque) | 0.139s | PASS |
| 97 | `TC-T2-F05-04` | T2 | F5 | Massive Dipper Overload (45 t > 1.25x Capacity) | 0.142s | PASS |
| 98 | `TC-T2-F05-05` | T2 | F5 | Tooth-Tip Location Offset Sensitivity (+/- 200 mm) | 0.138s | PASS |
| 99 | `TC-T2-F05-06` | T2 | F5 | Observer Cold-Start Transient Rejection Window (t < 5.0 s) | 0.000s | PASS |
| 100 | `TC-T2-F05-07` | T2 | F5 | Joint-Side 12-Bit Encoder Quantization Failure (N3 Noise Breakdown) | 0.124s | PASS |
| 101 | `TC-T2-F06-01` | T2 | F6 | Low-Resolution 8-Bit Encoder Stress Test | 0.000s | PASS |
| 102 | `TC-T2-F06-02` | T2 | F6 | High-Resolution 16-Bit Encoder Noise Floor Saturation | 0.000s | PASS |
| 103 | `TC-T2-F06-03` | T2 | F6 | Zero-Velocity Limit Behavior Under Quantized Steps | 0.000s | PASS |
| 104 | `TC-T2-F06-04` | T2 | F6 | Extreme Gear Ratio Bounds (Ngear = 1 vs Ngear = 500) | 0.000s | PASS |
| 105 | `TC-T2-F06-05` | T2 | F6 | Phase Jitter Sensitivity in Discrete Differencing | 0.000s | PASS |
| 106 | `TC-T2-F07-01` | T2 | F7 | Parameter Mismatch Stress on Digital Twin Instrument (+30%) | 0.122s | PASS |
| 107 | `TC-T2-F07-02` | T2 | F7 | Collinear Excitation Trajectory (Rank Deficiency Detection) | 0.000s | PASS |
| 108 | `TC-T2-F07-03` | T2 | F7 | High Friction Noise Regime (SNR < -40 dB) | 0.120s | PASS |
| 109 | `TC-T2-F07-04` | T2 | F7 | Non-Gaussian Impulsive Noise Injection Stability | 0.005s | PASS |
| 110 | `TC-T2-F07-05` | T2 | F7 | Sample Size Lower Bound (N < 7 Parameters) | 0.000s | PASS |
| 111 | `TC-T2-F08-01` | T2 | F8 | Discontinuous Multi-Step Boulder Impacts (+100 kN Series) | 0.028s | PASS |
| 112 | `TC-T2-F08-02` | T2 | F8 | Mismatched Bail Station Sensitivity (1.5 m vs 2.2 m) | 0.121s | PASS |
| 113 | `TC-T2-F08-03` | T2 | F8 | Innovation Threshold Boundary Sensitivity (chi_th in [1, 5]) | 0.042s | PASS |
| 114 | `TC-T2-F08-04` | T2 | F8 | High Covariance Boost Stability Limit (gamma = 500) | 0.039s | PASS |
| 115 | `TC-T2-F08-05` | T2 | F8 | Negative Force Shock Step (Sudden Loss of Bank Reaction) | 0.019s | PASS |
| 116 | `TC-T2-F09-01` | T2 | F9 | Exact Nyquist Boundary Frequency Excitation (15.0 Hz) | 0.000s | PASS |
| 117 | `TC-T2-F09-02` | T2 | F9 | Zero-Stator Frequency DC Limit (f0 -> 0 Hz) | 0.005s | PASS |
| 118 | `TC-T2-F09-03` | T2 | F9 | 12th Inverter Harmonic Foldover Mapping (12*f0) | 0.000s | PASS |
| 119 | `TC-T2-F09-04` | T2 | F9 | High-Amplitude Harmonic Ripple Stress (10% Torque Ripple) | 0.011s | PASS |
| 120 | `TC-T2-F09-05` | T2 | F9 | Pre-Filter Group Delay Distortion at Boundary Frequencies | 0.000s | PASS |
| 121 | `TC-T2-F10-01` | T2 | F10 | Zero-Frequency DC Delay Singularity | 0.000s | PASS |
| 122 | `TC-T2-F10-02` | T2 | F10 | High-Frequency Asymptotic Phase Lag Limit (180 deg) | 0.000s | PASS |
| 123 | `TC-T2-F10-03` | T2 | F10 | Negative Lag Detection as Invariant Violation (Acausal Alarm) | 0.000s | PASS |
| 124 | `TC-T2-F10-04` | T2 | F10 | Cross-Correlation Window Size Lower Bound | 0.001s | PASS |
| 125 | `TC-T2-F10-05` | T2 | F10 | White Noise Cross-Correlation Spurious Peak Trapping | 0.000s | PASS |
| 126 | `TC-T2-F11-01` | T2 | F11 | Zero Crowd Channel Weight Limit (s_bar -> 0, Saddle Collapse) | 0.000s | PASS |
| 127 | `TC-T2-F11-02` | T2 | F11 | Infinite Crowd Channel Weight Limit (s_bar -> inf) | 0.123s | PASS |
| 128 | `TC-T2-F11-03` | T2 | F11 | Single-Joint Motion Trajectory (Zero Crowd Speed d4d = 0) | 0.000s | PASS |
| 129 | `TC-T2-F11-04` | T2 | F11 | Severe Noise Covariance Anisotropy (Ratio > 100) | 0.002s | PASS |
| 130 | `TC-T2-F11-05` | T2 | F11 | Parameter Sensitivity Under Weight Perturbation (+/- 20%) | 0.128s | PASS |
| 131 | `TC-T2-F12-01` | T2 | F12 | Discontinuous Signum Friction Limit (eps -> 0) | 0.000s | PASS |
| 132 | `TC-T2-F12-02` | T2 | F12 | High Smoothing Threshold Limit (eps -> 0.1 rad/s) | 0.000s | PASS |
| 133 | `TC-T2-F12-03` | T2 | F12 | High-Frequency Chattering Trajectory Across Zero Velocity | 0.000s | PASS |
| 134 | `TC-T2-F12-04` | T2 | F12 | Inverted Smoothing Mismatch (eps_plant < eps_obs) | 0.000s | PASS |
| 135 | `TC-T2-F12-05` | T2 | F12 | Asymmetric Joint Smoothing Thresholds (eps3 != eps4) | 0.000s | PASS |
| 136 | `TC-T2-F13-01` | T2 | F13 | Complete Wire-Rope Slack Condition (T_rope = 0 for 500 ms) | 0.000s | PASS |
| 137 | `TC-T2-F13-02` | T2 | F13 | Snatch Impact Tension Spike Exceeding Safe Limit (T_rope > 800 kN) | 0.000s | PASS |
| 138 | `TC-T2-F13-03` | T2 | F13 | Zero-Damping Cable Resonance (Sustained Bounce Ringing) | 0.000s | PASS |
| 139 | `TC-T2-F13-04` | T2 | F13 | Infinite Stiffness Rigid Rope Limit (K_rope -> inf) | 0.000s | PASS |
| 140 | `TC-T2-F13-05` | T2 | F13 | Asymmetric Backlash Gap During Bidirectional Flutter | 0.000s | PASS |
| 141 | `TC-T3-01` | T3 | F4,5 | Pair (F4, F5): Calibration Output -> Observer Input Data Flow | 0.142s | PASS |
| 142 | `TC-T3-02` | T3 | F1,4 | Pair (F1, F4): Invariant Audit on Calibrated Plant Model | 0.137s | PASS |
| 143 | `TC-T3-03` | T3 | F5,6 | Pair (F5, F6): Payload Weighing Under Quantization Sweeps | 0.165s | PASS |
| 144 | `TC-T3-04` | T3 | F7,11 | Pair (F7, F11): Realistic Noise ID with Noise Covariance Weighting | 0.134s | PASS |
| 145 | `TC-T3-05` | T3 | F5,9 | Pair (F5, F9): Payload Mass Estimation Under VFD Harmonics | 0.155s | PASS |
| 146 | `TC-T3-06` | T3 | F8,10 | Pair (F8, F10): Matched Step Load and Corrected Filter Phase Lag | 0.020s | PASS |
| 147 | `TC-T3-07` | T3 | F5,12 | Pair (F5, F12): Dynamic Payload Under Coulomb Mismatch | 0.155s | PASS |
| 148 | `TC-T3-08` | T3 | F5,13 | Pair (F5, F13): Payload Mass Estimation Under Unmodelled Rope Bounce | 0.148s | PASS |
| 149 | `TC-T3-09` | T3 | F1,13 | Pair (F1, F13): Invariant Audit on Elastic Wire-Rope Dynamics | 0.000s | PASS |
| 150 | `TC-T3-10` | T3 | F6,9 | Pair (F6, F9): Combined Quantization Noise and Inverter Harmonics | 0.184s | PASS |
| 151 | `TC-T3-11` | T3 | F2,3 | Pair (F2, F3): claims.md Registry and Report Banner Consistency | 0.002s | PASS |
| 152 | `TC-T3-12` | T3 | F7,8 | Pair (F7, F8): Realistic Noise Identification Across Matched Profiles | 0.150s | PASS |
| 153 | `TC-T3-13` | T3 | F4,11 | Pair (F4, F11): 2-DOF SVD Identifiability on P08B Simscape Export | 0.140s | PASS |
| 154 | `SCEN-01` | T4 | F1,4,5,6,11 | Full Digging Cycle (Tuck, Dig, Breakout, Swing, Dump, Return) | 0.011s | PASS |
| 155 | `SCEN-02` | T4 | F5,8,10,12 | Boulder Strike During Crowding (+250 kN Shock Step) | 0.020s | PASS |
| 156 | `SCEN-03` | T4 | F1,5,6,13 | Bank Collapse & Sudden Drop (Unilateral Rope Tension & Free-Fall) | 0.000s | PASS |
| 157 | `SCEN-04` | T4 | F1,4,5,11 | Dynamic Swing Payload Estimation (Centrifugal & Spatial Decoupling) | 0.004s | PASS |
| 158 | `SCEN-05` | T4 | F1,6,8,12 | High-Speed Actuator Stall (Bedrock Engagement at vc = 0.54 m/s) | 0.000s | PASS |
| 159 | `SCEN-06` | T4 | F6,9,10,13 | Harmonic Vibration During Heavy Rock Engagement (Coupled 69 Hz & Chatter) | 0.000s | PASS |
| 160 | `SCEN-07` | T4 | F1,5,12,13 | Wire-Rope Slack & Snatch Transient (Bank Crest Unweighting) | 0.000s | PASS |
