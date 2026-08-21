# Estimation-Theory Papers — Two Rankings

Added 2026-07-30. These eleven papers close the observer/estimation gap in
`papers_library.bib`, which previously held 58 papers and none on estimation.

---

## RANKING 1 — Relevance to this project

Ordered by how directly each bears on M2, M3, M7, M9. Read top-down.

| # | Paper | Milestone | Why it ranks here |
|---|-------|-----------|-------------------|
| **1** | Haddadin, De Luca & Albu-Schäffer 2017, *Robot Collisions: A Survey* | M3, M9 | Field map. Defines detection / isolation / identification, the standard metrics, and the vocabulary. One paper that orients everything else. **Read first.** |
| **2** | De Luca & Mattone 2003, *Generalized Momenta* | **M3** | The method M3 will implement. Estimates external torque with no acceleration term. Directly answers M1's 56% Izz bias under differentiated velocities. |
| **3** | Swevers et al. 1997, *Optimal Robot Excitation* | **M2**, retrospective M1 | Turns excitation from a judgement call into a designed input. Fourier-series excitation gives periodicity, averaging, noise characterisation, analytic derivatives. |
| **4** | De Luca & Mattone 2005, *Sensorless Collision Detection* | M3, M7 | Moves the residual from actuator faults to external contact forces — the exact structure of τ_load. |
| **5** | Armstrong 1989, *Finding Exciting Trajectories* | **M2** | Condition number of the persistent-excitation matrix governs convergence and noise immunity. Direct commentary on our cond(Y) = 82.2, which was chosen by intuition. |
| **6** | Hu & Xiong 2017, *Semiparametric + Disturbance Kalman* | **M9** | Physics + learned residual, done classically. The baseline M9's PINN must outperform, or M9 has no claim. |
| **7** | Garofalo et al. 2019, *Sliding-Mode Momentum Observers* | M3 | Recovers joint acceleration as a by-product — interesting precisely because acceleration is what we are trying to avoid measuring. |
| **8** | Wahrburg et al. 2015, *Kalman + Generalized Momentum* | M3, M8 | Adds uncertainty quantification. Route to putting error bars on τ_load instead of a point estimate. |
| **9** | De Luca et al. 2006, *DLR-III Hardware* | **M8** | Hardware validation. Evidence the method survives real sensors and real model error. |
| **10** | Chen et al. 2018, *Extended State Observer* | M3 | Alternative observer. Useful as a comparison baseline, not as the primary method. |
| **11** | Gautier et al. 2013, *DIDIM* | M3 (alternative) | Identification from torque alone. A different route to the same goal; worth knowing, not on the critical path. |

**Critical path for the next three months:** 1 → 2 → 3. Those three unblock M2's
experiment design and M3's method choice. The rest are supporting.

---

## RANKING 2 — Venue quality

Ordered by publication venue standing. Impact factors move annually and should be
verified on JCR/Scimago before quoting in any document.

| # | Venue | Type | Standing | Papers here |
|---|-------|------|----------|-------------|
| **1** | *IEEE Transactions on Robotics* (T-RO) | Journal | **Q1, flagship of the field.** IF in the high single digits. | Haddadin 2017 |
| **2** | *The International Journal of Robotics Research* (IJRR) | Journal | **Q1, the other flagship.** Historically the most prestigious robotics journal. | Armstrong 1989 |
| **3** | *IEEE Transactions on Automatic Control* (TAC) | Journal | **Q1, flagship of control theory.** Highly theoretical. | Gautier 2013 |
| **4** | *IEEE Transactions on Industrial Electronics* (TIE) | Journal | **Q1, very high IF**, strong applied-engineering readership. | Hu & Xiong 2017 |
| **5** | *IEEE Transactions on Robotics and Automation* (T-RA) | Journal | Q1 in its era; predecessor of T-RO. | Swevers 1997 |
| **6** | *Control Engineering Practice* | Journal | **Q1**, applied control, mid-to-high IF. | Chen 2018 |
| **7** | *IEEE ICRA* | Conference | **Top-tier robotics conference.** In robotics, ICRA/IROS papers carry weight comparable to journals — unlike most engineering fields. | De Luca 2003, De Luca 2005, Garofalo 2019 |
| **8** | *IEEE/RSJ IROS* | Conference | Top-tier, ranked just below ICRA. | De Luca 2006 |
| **9** | *IEEE CASE* | Conference | Respectable, second tier. | Wahrburg 2015 |

### What this tells us about where to publish

Your programme currently cites mining and mechanical journals — *Journal of
Terramechanics*, *International Journal of Geomechanics*, *Chinese Journal of
Mechanical Engineering*, *Machines*. All legitimate, mostly Q1–Q2, but **modest
impact relative to T-RO, IJRR, or TIE**.

The estimation framing opens higher-impact venues:

| Target | Fit | Angle |
|---|---|---|
| *IEEE Trans. Industrial Electronics* | **strong** | Load estimation on production machinery with drive-side measurement. Applied, high IF. |
| *Mechanical Systems and Signal Processing* | **strong** | Condition monitoring, residual-based diagnosis. Very high IF, and it is where machine-health work lands. |
| *Control Engineering Practice* | **strong** | Real-plant validation on a CMPDI machine is exactly this journal's remit. |
| *IEEE Trans. Robotics* | possible | Only if the contribution is genuinely methodological, not applied. |
| *Journal of Terramechanics* | safe | Where the soil-force work (M7) naturally belongs. |

**Strategic read:** the M8 field-validation result is your highest-value output and
belongs in MSSP or CEP, not a mining journal. The soil-mechanics work goes to
*Terramechanics*. Splitting the programme across two venues by content type is
better than forcing it all into one.

---

## Positioning warning

A 2025 paper combines a **physics-informed neural network with a momentum observer**
for contact-force estimation on industrial robots. That is M3 + M9 together,
already published.

**Consequence:** M9 cannot be positioned as "we combined PINN with residual
estimation." The defensible novelty is:

1. A machine with a **configuration-dependent lever arm** (r = d₄ + c) and a
   **time-varying payload** — unlike a fixed-inertia manipulator
2. Operation **through the digging cycle**, the regime Rasuli explicitly excluded,
   where external load and parameter drift are entangled
3. **Validation on real production-machine data** against weighbridge ground truth

Item 3 is the strongest and the hardest to contest. **M8 is the novelty, not M9.**
