# shovel-dynamics
Load-torque estimation & health monitoring for mining electric rope shovels.
Project IIT/SRIC/R/AEH/2026/104 - IIT Kharagpur Mining Engineering.

START EVERY SESSION: read last entry of research_log.md
END EVERY SESSION:   write an entry + git commit + push

Layout:
- research_log.md   what happened, when, why (thesis methodology source)
- claims.md         every number + its source (nothing cited from memory)
- model/            ShovelSimulator.slx (current only) + CHANGELOG.md
- scripts/          validate_and_identify.m, shovel_params.m (params = single source of truth)
- experiments/      one frozen folder per reportable result (script+params+mat+figs)
- thesis/           skeleton + sections written while fresh
- papers_library/   papers.md, .bib, rankings
- meetings/         CMPDI docs, minutes

Rules:
1. No _v2/_final/_(1) filenames - git history is the version.
2. A figure that cannot be regenerated from its experiment folder does not go in a paper.
3. External/AI files get a provenance line in research_log before use.
