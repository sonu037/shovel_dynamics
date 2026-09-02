# shovel-dynamics
Load-torque estimation & health monitoring for mining electric rope shovels.
Project IIT/SRIC/R/AEH/2026/104 - IIT Kharagpur Mining Engineering.

START EVERY SESSION: read last entry of research_log.md
END EVERY SESSION:   write an entry + git commit + push

Layout:
- research_log.md   what happened, when, why (thesis methodology source)
- claims.md         every number + its source (nothing cited from memory)
- LEARNING.md       transferable lessons, not tied to a milestone
- M1/               frozen: model, scripts, experiments, Stage 1 report
- M2/               active: 00_design, 00_archive, 01..08 experiments,
                    scripts, figures, model. See M2_README.md
- papers_library/   papers.md, .bib, rankings
- meetings/         CMPDI docs, minutes
- reference/        source material

Rules:
1. No _v2/_final/_(1) filenames - git history is the version.
2. A figure that cannot be regenerated from its experiment folder does not go in a paper.
3. External/AI files get a provenance line in research_log before use.
4. Every number in a log or a paper must trace to a script that regenerates it.
   Results computed in the command window are not results.