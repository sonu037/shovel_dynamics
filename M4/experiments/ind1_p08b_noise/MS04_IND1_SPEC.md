# MS04-IND1 — Sensor noise and encoder steps on the M3 observer, on MS02P08B Simscape data

```
STATUS:                 DRAFT 2026-11-18 (drafted with Claude; inputs checked
                        against the MS02P08B and MS03-IND1 results files,
                        M3/scripts/m3_observer_30Hz.m and the M4 P01 quantisation
                        conventions). FROZEN when Suhail commits this file
                        together with M4/scripts/run_MS04_IND1_noise_on_P08B.m,
                        before the run.

Question:               How much does the load estimate of the M3 30 Hz observer
                        degrade when the clean Simscape signals of MS02P08B are
                        replaced by what real sensors deliver: noisy speed, noisy
                        drive torque, and positions from a 12-bit encoder (on
                        the joint or on the motor) with speed obtained by
                        differencing at 30 Hz?

Why:                    MS03-IND1 ran the observer on the noise-free Simscape
                        data and found 0.04 kN error against the lag-filtered
                        truth (true parameters). That number says nothing about a
                        real machine. The existing M4 results (P01-P06) were run
                        on the observer's own model-generated data. This is the
                        first M4 test on data the observer did not generate.

Hypothesis:             The Branch B estimate is linear in the measured signals:
                          tau_hat = kO s/(s + kO) p  -  kO/(s + kO) h,
                          p3 = D11(d4) q3dot.
                        So speed noise reaches the load estimate through a HIGH-
                        pass with gain kO (amplified by D11), and torque noise
                        through a LOW-pass (attenuated). Fy noise from the saddle
                        channel = that noise / ((d4 + 1.5) cos q3). The KF-GMO,
                        tuned for 3 mrad/s speed noise (R = (D11 x 0.003)^2),
                        smooths the momentum and should suppress speed noise.

Setup:                  Data. M2/experiments/08_held_out/rev2/
                        MS02P08B_simscape_20261118_130959.mat (committed in
                        581d494), MD5 5eaa1131f51257e72b24b2596a83598d. Cases E0
                        (q3 45 +/- 15 deg, d4 8.75 +/- 2 m, omega 0.314 rad/s) and
                        E1 (q3 -12 +/- 9 deg, d4 7.325 +/- 0.275 m, omega
                        0.51 rad/s); 100 kN vertical sine load at the bail,
                        period 9 s; 65 s; S30 (every 10th sample of the 300 Hz
                        log): t, q_3, q_3_dot, d_4, d_4_dot, tau3_cmd, f4_cmd,
                        outF.

                        Observer. M3/scripts/m3_observer_30Hz.m (proven identical
                        to the M3 archive in MS03-IND1), kO = 20 1/s,
                        Ts = 1/30 s, Branch B and KF-GMO with M3's own tuning.
                        TRUE parameters only, read from the data file (Md 54,300
                        kg, Izz 287,900 kg m^2, c -1.32 m, fv3 3,800, fv4 1,473,
                        fc3 340, fc4 480). The wrong-parameter bias is already
                        measured (MS03-IND1) and adds to what is measured here.

                        Load recovery as in MS03-IND1:
                          Fy_hat = tau_hat_L3 / ((d4m + 1.5) cos q3m),
                        with the MEASURED (noisy or quantised) d4m, q3m.
                        Scoring window t >= 5 s.

Conditions:             Each for E0 and E1. Random noise: rng(20261118 + 10 j + i,
                        'twister'), j = condition number 1-6 (N0-N5), i = case
                        (1 E0, 2 E1). One realisation per condition.
                        N0  clean (the MS03-IND1 input).
                        N1  white noise added to q3dot (3 mrad/s rms) and d4dot
                            (3 mm/s rms). 3 mrad/s is the level the KF-GMO was
                            tuned for (M3 "noisy resolver").
                        N2  white noise added to tau3_cmd and f4_cmd, rms = 1 % of
                            the rms of that channel over the 65 s record.
                        N3  12-bit encoder on the JOINT (M4 Case 1A convention):
                            q3m = round(q3/dq) dq, dq = 2 pi/4096 rad; d4m the same
                            with dd = 2 pi/4096 m. Speeds by backward difference
                            at 30 Hz: v_k = (x_k - x_{k-1})/Ts, first sample
                            forward difference. No speed sensor.
                        N4  12-bit encoder on the MOTOR (M4 Case 1B design
                            values): dq = 2 pi/(4096 x 240) rad,
                            dd = 2 pi/(4096 x 109.3) m. Gear ratios 240 and
                            109.3 rad/m are M4 design values, NOT sourced from
                            a machine (claims.md flags them). Speeds as N3.
                        N5  N4 and N2 together.

Metrics:                Per case, condition and branch (B, KF-GMO):
                        noise-induced error = rms over t >= 5 s of
                          Fy_hat(condition) - Fy_hat(N0)   [kN];
                        also rms of Fy_hat - outF (total error vs truth).
                        Crowd channel not scored.

Validity gates:         G1  observer identity: m3_observer_30Hz.m on the archived
                            case M3_DISC_SIN_100kN reproduces
                            M3/06_RESULTS_MAT/M3_discrete_30Hz_verification.mat
                            to <= 1e-9 relative (Branch B and KF-GMO).
                        G2  data provenance: MD5 above; S30 1951 samples on an
                            exact 1/30 s grid, 0-65 s, E0 and E1.
                        G3  clean path: N0 reproduces the true-parameter Fy_hat
                            of MS03-IND1 (M3/experiments/ind1_p08b/
                            MS03_IND1_20261118_160926.mat, results.E*.control.FyB
                            and .FyK) to <= 1e-6 N everywhere.

Prediction:             Written before the run, from the hypothesis; no data
                        values used. Branch B noise-induced rms of Fy [kN]:

                                    E0        E1      band
                          N1      27.10     13.17    +/- 15 %
                          N2       1.915     1.829   +/- 15 %
                          N3     189.3      92.0     +/- 50 %
                          N4       0.789     0.383   +/- 25 %
                          N5       2.071     1.869   +/- 15 %
                        (N5 = sqrt(N4^2 + N2^2).)

                        Derivation. Exact discrete gains of the M3 FOH filter
                        (Phi = 0.513417, kO beta1 = 0.270126, kO(Phi beta1 +
                        beta0) = 0.355152), as rms output per rms input:
                          g_p = 0.8390  high-pass, white input (speed noise)
                          g_d = 0.9356  high-pass, differenced uniform input
                                        (quantisation + backward difference)
                          g_h = 0.4942  low-pass, white input (torque noise)
                        Fy noise (speed)  = g kO sigma_v rms(D11/arm),
                        Fy noise (torque) = g_h sigma_tau rms(1/arm),
                        arm = (d4 + 1.5) cos q3, D11 = Izz + Md (d4 + c)^2.
                        Quantisation + backward difference:
                          sigma_v = sqrt(2) step / (sqrt(12) Ts)
                          = 18.79 mrad/s (N3), 0.0783 mrad/s (N4).
                        Trajectory factors from the COMMANDED E0/E1 sines (not
                        from the data), window t >= 5 s:
                          rms(D11/arm) = 5.384e5 / 2.617e5 kg m,
                          rms(1/arm) = 0.1449 / 0.1170 1/m;
                        torque rms (model inverse dynamics with the load,
                        0-65 s) = 2.674 / 3.163 MN m, so sigma_tau = 26.74 /
                        31.63 kN m. Crowd-speed noise does not enter the saddle
                        channel (p3 uses q3dot only).
                        Bands: +/- 15 % covers the sampling error of one 60 s
                        realisation (~3 %) and tracking error; N4 +/- 25 % for
                        the uniform-error model of quantisation; N3 +/- 50 %
                        because at 1.5 mrad steps the error is correlated near
                        motion reversals.

                        P-N1 ... P-N5  Branch B within the band, both cases.
                        P-KF  KF-GMO noise-induced rms <= 0.35 x Branch B under
                              N1, both cases (the M3 archive claimed 0.19 on its
                              own data).
                        KF-GMO under N2-N5 is reported, not predicted.

Synthetic pre-check:    Disclosure. The script was tested in GNU Octave 8.4 on a
                        SYNTHETIC, model-matched stand-in for the data file (the
                        commanded E0/E1 trajectories and torques from the
                        model's own equations; not the Simscape data), with an
                        MS03-IND1 result produced on the same stand-in. The N3/N4
                        gain g_d and all bands were fixed before this test.
                        Results there (Branch B / KF-GMO noise-induced rms, kN,
                        E0; E1):
                          N1  27.04 / 4.13;   13.16 / 2.58
                          N2   1.92 / 1.60;    1.77 / 1.57
                          N3 197.0 / 11.3;    97.8 / 7.9
                          N4   0.82 / 0.073;   0.39 / 0.073
                          N5   2.13 / 1.71;    1.86 / 1.66
                        All predictions held there; G1 and G3 passed (G2 failed,
                        as it must on a stand-in file). Octave's random numbers
                        differ from MATLAB's, so N1/N2/N5 will not repeat to the
                        digit. The Simscape data file has not been run.

Independent check:      After the run: N0, N3 and N4 contain no random numbers,
                        so they will be recomputed in Python from the data file
                        and must match to 3 decimals. N1, N2, N5 are checked
                        statistically against the prediction only.

Acceptance criterion:   Validity of the run = G1, G2 and G3. Predictions may be
                        refuted; a refutation is a result and is recorded.

Failure interpretation: G1 fails -> the observer function changed. STOP.
                        G2 fails -> wrong or changed data file. STOP.
                        G3 fails -> the clean path differs from MS03-IND1
                            (fields read, trim, arm formula). STOP and find the
                            difference before any noise result is used.
                        P-N1 / P-N2 refuted -> check the noise went into the
                            right channel and units, the window, and the
                            trajectory factor on the actual Simscape motion,
                            before any claim that the observer is non-linear.
                        P-N3 / P-N4 refuted -> the uniform-white model of
                            quantisation is wrong for this resolution and
                            motion; report the measured value; that is the
                            result.
                        P-KF refuted -> KF-GMO tuning on this data; report, do
                            not retune after seeing the result.

Scope:                  Simulation only. One realisation per noise condition.
                        No anti-alias filter, no speed sensor (speeds from
                        position differences), no motor-current model (torque
                        noise is white at 1 %; current ripple and sensor
                        dynamics are M5). True parameters only. Gear ratios for
                        N4 are unsourced design values. The plant and the
                        observer share the same rigid-body structure, so this
                        isolates sensor effects; it does not test unmodelled
                        dynamics. Machine A (Rasuli 2014) parameters. The load is
                        a force at the bail, not a payload mass (V27).

Repository location:    M4/experiments/ind1_p08b_noise/

Run:                    addpath(fullfile(pwd,'M4','scripts'))
                        R = run_MS04_IND1_noise_on_P08B();   % clean tree required
```
