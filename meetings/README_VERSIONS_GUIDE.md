# CMPDI Project MT-189: Presentation Version & Storage Guide

## Why Presentations Looked Similar When Opened
Previously, all presentations shared the official Coal India / CMPDI presentation template (same cover layout, background colors, and slide title locations). Because Slide 1 only differed by a small text date ("1–2 November 2026" vs "Mid-November 2026"), opening multiple files in PowerPoint made them look like identical clones.

To permanently solve this confusion, all versions are now:
1. **Separated into numbered, dedicated directories** with clear, descriptive filenames.
2. **Equipped with prominent visual branding badges** on Slide 1 and Slide 2 of the final presentation so you immediately know which version is open.
3. **Universally formatted with Arial typography** and the official **Material Dynamics Lab footer logo** on every single slide of Version 5.0.

---

## Master Version Comparison Matrix

| Property | Version 3.0 (Baseline) | Version 4.0 (Text Update) | Version 5.0 (Final Pro Research Deck) ★ **USE THIS** |
| :--- | :--- | :--- | :--- |
| **Directory** | `meetings/01_Baseline_v3_Sept2026/` | `meetings/02_TextUpdate_v4_Nov2026/` | `meetings/03_Final_Research_ProDeck_v5/` |
| **File Name** | `CMPDI_Progress_v3_Baseline_39Slides.pptx` | `CMPDI_Progress_v4_TextUpdate_51Slides.pptx` | `CMPDI_Progress_v5_Final_Research_ProDeck.pptx` |
| **Total Slides** | **39 Slides** | **51 Slides** (+12 slides) | **51 Slides** |
| **Scope Covered** | Milestone M1 complete; M2 preliminary | Milestones M1, M2, M3, M4 full scope | Milestones M1, M2, M3, M4 full scope |
| **Typography** | Calibri / Calibri Light | Calibri / Calibri Light | **Arial & Arial Bold Globally** |
| **Slide 1 Header** | Plain white band with logos | Plain white band with logos | **Bold Navy & Gold Version 5.0 Banner** |
| **Simscape Canvases** | None (Text notes only) | None (Text notes only) | **Physical Multibody Canvases (Slides 15 & 16)** |
| **Empirical Figures** | None | None | **6 High-Res 300 DPI Validation Curves (Slides 38–48)** |
| **Equation Cards** | Text approximations | Text approximations | **Rendered Mathematical Vector Cards** |
| **Slide Footer** | Generic template footer | Generic template footer | **Material Dynamics Lab Official Logo (All 51 slides)** |
| **File Size** | ~6.90 MB | ~6.92 MB | **~8.18 MB** |
| **Primary Purpose** | Archived historical baseline (Sept 2026) | Intermediate text draft | **Final Defense & Review Deck for PI / CMPDI** |

---

## How to Verify You Have the Right Version Open in PowerPoint

When you double-click `CMPDI_Progress_v5_Final_Research_ProDeck.pptx`:
1. **Slide 1 (Top Center):** You will immediately see a prominent dark-navy banner bordered in gold:
   > `★ VERSION 5.0 — RESEARCH & EXPERIMENTAL VERIFICATION DECK ★`
   > `51 Slides | Milestones M1–M4 Complete | Simscape Canvases (S15–S16) | Empirical Curves (S38–S48)`
2. **Slide 1 (Subtitle Area):** Contains the blue capsule:
   > `FINAL RESEARCH PRO DECK: Universal Arial Typography | Simscape Digital Twins | Material Dynamics Lab`
3. **Slide 2 (Bottom Bar):** A dedicated callout banner summarizing the expansion from 39 to 51 slides.
4. **Slides 15 & 16:** Right-hand side features high-resolution block diagrams of the Simscape Multibody model.
5. **Slides 38, 41, 44, 46, 48:** High-resolution empirical research plots with callout annotations.
6. **All Slides (Bottom-Left):** The official Material Dynamics Lab logo is embedded on every slide footer.

---

## Directory Layout in `meetings/`
```
meetings/
│
├── 01_Baseline_v3_Sept2026/
│   └── CMPDI_Progress_v3_Baseline_39Slides.pptx        <-- Untouched September baseline
│
├── 02_TextUpdate_v4_Nov2026/
│   └── CMPDI_Progress_v4_TextUpdate_51Slides.pptx       <-- Text & table milestone expansion
│
├── 03_Final_Research_ProDeck_v5/
│   └── CMPDI_Progress_v5_Final_Research_ProDeck.pptx   <-- ★ DEFINITIVE PRESENTATION FOR PI
│
├── figures_generated/                                  <-- All 300 DPI plots & canvas diagrams
└── README_VERSIONS_GUIDE.md                            <-- This master documentation guide
```
