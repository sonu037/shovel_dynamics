# MS03-IND1 — M3 30 Hz observer on independent Simscape data (MS02P08B rev 2)

```
STATUS:                 DRAFT 2026-11-18 (drafted with Claude; inputs checked
                        against the MS02P08B results file, claims.md rows 101-102
                        and M3/scripts/build_M3_DiscreteObserver_30Hz.m).
                        FROZEN when Suhail commits this file together with
                        M3/scripts/m3_observer_30Hz.m and
                        M3/scripts/run_MS03_IND1_on_P08B.m, before the run.

Question:               Does the M3 30 Hz momentum observer recover the bail load
                        from data it did not generate - the torque-driven
                        Simscape plant of MS02P08B rev 2 - (i) with the true
                        parameters and (ii) with the MS02P08B wrong parameters?

Why:                    Every M3 result so far is a code check: the observer was
                        tested on torques computed from its own equations and
                        parameters (claims.md row 102). MS02P08B produced a
                        torque-driven Simscape run whose 30 Hz export (S30)
                        holds exactly the signals the observer needs and no
                        acceleration. This is the first M3 test on data the
                        observer did not generate.

Hypothesis:             The observer output is its first-order filter (k_O) of
                        the generalized load the model "sees". With the true
                        parameters that is the true load (plus at most the
                        Coulomb term Branch B leaves out); with wrong parameters
                        it is the low-pass-filtered MS02P08B inverse-dynamics
                        residual, so the bias is the same as in MS02P08B and only
                        the 50 ms lag is added.

Setup:                  Data. M2/experiments/08_held_out/rev2/
                        MS02P08B_simscape_20261118_130959.mat (committed in
                        581d494), MD5 5eaa1131f51257e72b24b2596a83598d. Cases E0
                        (q3 45 +/- 15 deg, d4 8.75 +/- 2 m, omega 0.314 rad/s) and
                        E1 (q3 -12 +/- 9 deg, d4 7.325 +/- 0.275 m, omega
                        0.51 rad/s); 100 kN vertical sine load at the bail,
                        period 9 s; 65 s each. S30 = every 10th sample of the
                        300 Hz log: t, q_3, q_3_dot, d_4, d_4_dot, tau3_cmd,
                        f4_cmd, outF. The drive command tau3_cmd / f4_cmd is the
                        "measured" torque; the plant subtracted Coulomb friction
                        (340 N m, 480 N, tanh, eps 1e-3) from it.

                        Observer. M3/scripts/m3_observer_30Hz.m: the Branch B
                        (exact FOH, friction left out of h) and KF-GMO update
                        equations copied line for line from
                        build_M3_DiscreteObserver_30Hz.m. k_O = 20 1/s,
                        Ts = 1/30 s, Phi = 0.513417, beta0 = 0.010823 (old
                        sample), beta1 = 0.013506 (new sample). KF-GMO keeps
                        M3's own tuning and its Coulomb smoothing eps = 3e-4
                        (the plant used 1e-3; this mismatch is part of the test).

                        Parameters, read from the same file (not retyped):
                          true      Md 54,300 kg, Izz 287,900 kg m^2, c -1.32 m
                          mismatch  Md 57,015 kg (+5 %), Izz 316,690 kg m^2
                                    (+10 %), c -1.27 m (+0.05 m)
                          fv3 3,800, fv4 1,473, fc3 340, fc4 480, g 9.80665.

                        Load recovery, as in MS02P08B (saddle channel):
                          Fy_hat = tau_hat_L3 / ((d4 + 1.5) cos q3).
                        Scoring window t >= 5 s (same trim as MS02P08B).

Variables:              Independent: case (E0, E1); parameter set (true,
                        mismatch); observer branch (B, KF-GMO).
                        Held fixed: data file, k_O, Ts, trim, Fy formula.

Equations:              Generalized momentum p = [ (Izz + Md r^2) q3dot ; Md d4dot ],
                        r = d4 + c.
                        h_B = tau - G - Fv q_dot + [0 ; Md r q3dot^2],
                        G = [Md g r cos q3 ; Md g sin q3].
                        z_k = Phi z_{k-1} + beta0 u_{k-1} + beta1 u_k,
                        u = h_B + k_O p,  tau_hat_L = k_O (p - z).
                        Lag-matched reference: the true Q3 = (d4 + 1.5) cos q3 Fy
                        passed through the same exact first-order filter.

Metrics (per case, parameter set and branch):
                        bias and rms of Fy_hat - Fy (kN); for Branch B also the
                        rms of Fy_hat - Fy_filt (lag-matched). Crowd channel
                        rms of tau_hat_L4 - Fy sin q3 is reported, not scored.

Validity gates:         G1  code identity: m3_observer_30Hz.m re-run on the
                            archived case M3_DISC_SIN_100kN reproduces
                            M3/06_RESULTS_MAT/M3_discrete_30Hz_verification.mat,
                            Branch B and KF-GMO, to <= 1e-9 relative.
                        G2  data provenance: the file MD5 equals the value above;
                            S30 has 1951 samples on an exact 1/30 s grid, 0-65 s,
                            for E0 and E1.

Prediction:             Written before the run; hand calculation, then a
                        synthetic pre-check (below).
                        P1  True parameters, Branch B, both cases:
                            |bias| < 0.3 kN; rms 2.0-3.0 kN; rms against the
                            lag-matched reference < 0.2 kN.
                            Why: the 50 ms first-order lag on a 100 kN sine at
                            2 pi/9 rad/s leaves an error of amplitude
                            100 kN x (omega/k_O) / sqrt(1 + (omega/k_O)^2)
                            = 3.49 kN, i.e. rms 2.47 kN; the configuration change
                            of (d4 + 1.5) cos q3 adds at most ~0.7 kN in E0.
                            Its mean over the 60 s window (6.7 load periods) is
                            at most 2 x 3.49 / (0.698 x 60) = 0.17 kN; the
                            Coulomb term Branch B leaves out is at most
                            340 N m / min((d4 + 1.5) cos q3) = 0.08 kN (E0).
                        P2  Wrong parameters, Branch B, both cases:
                            bias within 10 % of the MS02P08B inverse-dynamics
                            bias on the same data (E0 +21.90 kN, E1 +21.32 kN,
                            read from the file), and rms within 10 % of
                            sqrt(bias^2 + 2.47^2) (E0 22.04 kN, E1 21.46 kN).
                            Why: the observer converges to the first-order
                            filtered residual of the wrong model; filtering keeps
                            the mean and adds the lag.
                        P3  KF-GMO: true parameters |bias| < 0.5 kN; wrong
                            parameters bias within 10 % of Branch B's. No rms
                            prediction (its response depends on its tuning).

Synthetic pre-check:    Disclosure. The script was tested in GNU Octave 8.4 on a
                        SYNTHETIC stand-in for the data file (analytic
                        trajectories of E0/E1, torques from the model's own
                        equations - model-matched, not the Simscape data). There
                        P1-P3 held: control Branch B bias -0.13 / -0.11 kN,
                        rms 2.47 / 2.46 kN, lag-matched rms 0.04 / 0.03 kN;
                        mismatch bias 21.77 / 21.21 kN. The P1 bias bound was
                        set to 0.3 kN from the window-mean calculation above
                        (0.17 + 0.08 kN), not from that test. The Simscape data
                        file itself has not been run through the observer.

Acceptance criterion:   Validity of the run = G1 and G2. P1-P3 may be refuted;
                        a refutation is a result and is recorded as one.

Failure interpretation: G1 fails -> m3_observer_30Hz.m is not the M3 observer
                            (copy error). STOP and fix before any result.
                        G2 fails -> wrong or changed data file. STOP.
                        P1 fails on the lag-matched rms -> the observer sees
                            something its model lacks in the Simscape data.
                            Check, in order: time alignment of S30 (q and
                            tau3_cmd from the same sample), sign of tau3_cmd
                            against the plant's Coulomb subtraction, units (rad,
                            m), the 1.5 m datum, before any physical explanation.
                        P1 fails only on the bias -> window effect; report the
                            value with the window length.
                        P2 fails -> filtering does not simply preserve the
                            residual mean here; check the start-up transient and
                            that the wrong parameters enter D(q) as well as G.
                        P3 fails -> KF-GMO tuning (process noise) on this data;
                            report, do not retune after seeing the result.

Scope:                  Noise-free; 30 Hz by taking every 10th sample (no
                        anti-alias filter; the signals are smooth). The Simscape
                        plant and the observer model have the same rigid-body
                        structure and friction law, so this tests the observer
                        on independently integrated data and with wrong
                        parameter values - not unmodelled dynamics, sensor noise
                        (M4), quantisation, or motor current (M5). Machine A
                        (Rasuli 2014) parameters. The load is a force at the
                        bail, not a payload mass (V27).

Repository location:    M3/experiments/ind1_p08b/

Run:                    R = run_MS03_IND1_on_P08B();   % clean tree required
```
