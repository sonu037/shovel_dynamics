# Deck content map — slide-by-slide control document
Workflow per slide: SPEC -> report prose written -> slide built -> rendered -> APPROVED -> next.
A slide's status only moves forward. Math on slides = PNGs from equations/png (CM font).
Rule: report section for a work unit is written BEFORE its slide (compress, never expand).

| # | Slide | Claim (one sentence) | Visual | Equations | Report § | Status |
|---|-------|----------------------|--------|-----------|----------|--------|
| 1 | Title | Health-first framing of the project | — | — | title | draft-v2 |
| 2 | Contents | 7-part narrative | numbered list | — | — | draft-v2 |
| 3 | Motivation | Downtime economics make drive health the objective | stat cards / site photo (pending CMPDI) | — | §1 | draft-v2 |
| 4 | Observability | Health is the smallest of three torque causes | 3-layer diagram | — | §1 | draft-v2 |
| 5 | Core idea | One subtraction defines the programme | E1 large | eqCore | §1 | draft-v2 |
| 6 | Anatomy | 2-DOF abstraction of the front end | anatomy schematic | eqLever | §2.1 | draft-v2 |
| 7 | Lit: Rasuli | Parent method; swing-only limitation | results table | — | §3 intro | draft-v2 |
| 8 | Lit: Frimpong | Digging-force structure for M7 | Fr profile sketch | — | §3 | draft-v2 |
| 9 | Lit: Fu | Metrics + loss blueprint for M4/M9 | mini architecture | eqHPINN, eqMetrics | §3 | draft-v2 |
|10 | Gap matrix | The combination column is the thesis | capability matrix | — | §1 | draft-v2 |
|11 | Objectives | Health primary; payload sub-objective | labeled list | — | §1 | draft-v2 |
|12 | Methodology | Residual pipeline | 4-block flow | eqSaddle | §2.2 | draft-v2 |
|13 | Model+oracle | Independent verification is non-circular | eqCrowd + callout | eqCrowd, eqManip | §2, §3 | draft-v2 |
|14 | MONEY | Machine-precision validation | fig_B1 + stat cards | — | §5 | draft-v2 |
|15 | Identification | Exact blind recovery; instrumentation lesson | fig_B3 + table | eqRegressor, eqLS | §4, §5 | draft-v2 |
|16 | Protocol | Verification by designed isolation | 3 diagnostic rows | — | §3 | draft-v2 |
|17 | Roadmap | Nine milestones to the reality gate | fishbone | — | §6 | draft-v2 |
|18 | Timeline | Data-gated M8; parallel simulation track | gantt-lite | — | §6 | draft-v2 |
|19 | References | IEEE numbered + one-sentence thesis | — | — | bib | draft-v2 |

## Per-slide iteration protocol (what we do in chat)
1. I post the slide SPEC: claim, bullets, visual, notes script.
2. You approve/edit the spec.
3. I build ONLY that slide, render it, show you the image.
4. You approve -> status = FINAL; we move to the next slide.
5. Any equation change happens in equations_master.tex first, then re-render.
