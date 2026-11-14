import os
import shutil
import pptx
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN

DECK_PATH = r"meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx"
BACKUP_PATH = r"meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.backup.pptx"

# Create a safety backup if not already present
if not os.path.exists(BACKUP_PATH):
    shutil.copy2(DECK_PATH, BACKUP_PATH)
    print(f"Created safety backup at {BACKUP_PATH}")

import io

# Read into memory buffer so the file handle is closed immediately
with open(DECK_PATH, "rb") as f:
    deck_bytes = io.BytesIO(f.read())

prs = pptx.Presentation(deck_bytes)
print(f"Loaded deck with {len(prs.slides)} slides.")

# ==============================================================================
# 1. UPDATE SLIDE 41 (index 40): M4 P06 Boulder Impact
# ==============================================================================
s41 = prs.slides[40]
print(f"\n--- Updating Slide 41: {s41.shapes[0].text_frame.text.strip()} ---")

# Shape 2 is the text box on the left card
tf41 = s41.shapes[2].text_frame

# Existing paragraphs:
# P0: "Discontinuous Rock Stall Discovery" (title)
# P1: Stress Test...
# P2: Smooth cutting...
# P3: Metrics Degrade...
# P4: Physical Mechanism...
# P5: Bode's Sensitivity...
# P6: Key Research Insight...

# We want to add/update text to show boulder stall phase lag has been RESOLVED via M5 Adaptive Innovation Gate:
# 'RESOLVED via M5 Adaptive Innovation Gate: NIS-based covariance scaling collapses lag from 250 ms to < 33 ms (single 30 Hz PLC sample), boosting R^2 from 0.1755 to 0.8920 and slashing NRMSE from 26.23% to 9.49% (63.8% error reduction).'

# Adjust existing paragraphs to 13 pt font so everything fits with comfortable margins
for p in tf41.paragraphs[1:]:
    p.font.name = "Arial"
    p.font.size = Pt(13)
    for r in p.runs:
        r.font.name = "Arial"
        r.font.size = Pt(13)

# Check if paragraph already exists or needs to be added
resolved_text_prefix = "RESOLVED via M5 Adaptive Innovation Gate:"
has_resolved_para = any(resolved_text_prefix in p.text for p in tf41.paragraphs)

if not has_resolved_para:
    p7 = tf41.add_paragraph()
    p7.space_before = Pt(4)
    p7.space_after = Pt(2)
    
    r1 = p7.add_run()
    r1.text = "RESOLVED via M5 Adaptive Innovation Gate: "
    r1.font.name = "Arial"
    r1.font.size = Pt(13)
    r1.font.bold = True
    r1.font.color.rgb = RGBColor(0x1F, 0x4E, 0x9C) # Classic Navy
    
    r2 = p7.add_run()
    r2.text = "NIS-based covariance scaling collapses lag from 250 ms to < 33 ms (single 30 Hz PLC sample), boosting R² from 0.1755 to 0.8920 and slashing NRMSE from 26.23% to 9.49% (63.8% error reduction)."
    r2.font.name = "Arial"
    r2.font.size = Pt(13)
    r2.font.bold = False
    r2.font.color.rgb = RGBColor(0x33, 0x33, 0x33)

print("Slide 41 paragraphs after update:")
for idx, p in enumerate(tf41.paragraphs):
    print(f"  P{idx}: {p.text}")

# ==============================================================================
# 2. UPDATE SLIDE 48 (index 47): Conclusions & Support
# ==============================================================================
s48 = prs.slides[47]
print(f"\n--- Updating Slide 48: {s48.shapes[0].text_frame.text.strip()} ---")

# Extend background cards and text frames for visual balance
# Shape 1 (left card), Shape 3 (right card)
s48.shapes[1].height = Emu(4500000)
s48.shapes[3].height = Emu(4500000)

# Shape 2 (left text box), Shape 4 (right text box)
s48.shapes[2].height = Emu(4350000)
s48.shapes[4].height = Emu(4350000)

tf48_concl = s48.shapes[2].text_frame
tf48_supp = s48.shapes[4].text_frame

# Standardize font size on Support Requested to 16 pt
for p in tf48_supp.paragraphs[1:]:
    p.font.name = "Arial"
    p.font.size = Pt(16)
    p.space_after = Pt(6)
    for r in p.runs:
        r.font.name = "Arial"
        r.font.size = Pt(16)

# Standardize existing Conclusions paragraphs to 16 pt
for p in tf48_concl.paragraphs[1:]:
    p.font.name = "Arial"
    p.font.size = Pt(16)
    p.space_after = Pt(6)
    for r in p.runs:
        r.font.name = "Arial"
        r.font.size = Pt(16)

# Bullets to add under Conclusions:
# 1. 'Prismatic Coulomb friction physically integrated on crowd joint (Fc4 = 480 N continuous tanh, zero solver chatter, 6.3s run)'
# 2. 'Boulder impact step-lag collapsed from 250 ms down to < 33 ms via M5 Adaptive Innovation Gate (NRMSE 26.2% -> 9.5%)'

bullet1_text = "Prismatic Coulomb friction physically integrated on crowd joint (Fc4 = 480 N continuous tanh, zero solver chatter, 6.3s run)"
bullet2_text = "Boulder impact step-lag collapsed from 250 ms down to < 33 ms via M5 Adaptive Innovation Gate (NRMSE 26.2% -> 9.5%)"

existing_texts = [p.text.strip() for p in tf48_concl.paragraphs]

if bullet1_text not in existing_texts:
    p_new1 = tf48_concl.add_paragraph()
    p_new1.space_after = Pt(6)
    r = p_new1.add_run()
    r.text = bullet1_text
    r.font.name = "Arial"
    r.font.size = Pt(16)
    r.font.color.rgb = RGBColor(0x33, 0x33, 0x33)

if bullet2_text not in existing_texts:
    p_new2 = tf48_concl.add_paragraph()
    p_new2.space_after = Pt(6)
    r = p_new2.add_run()
    r.text = bullet2_text
    r.font.name = "Arial"
    r.font.size = Pt(16)
    r.font.color.rgb = RGBColor(0x33, 0x33, 0x33)

print("Slide 48 Conclusions paragraphs after update:")
for idx, p in enumerate(tf48_concl.paragraphs):
    print(f"  P{idx}: {p.text}")

# ==============================================================================
# 3. SAVE PRESENTATION
# ==============================================================================
prs.save(DECK_PATH)
print(f"\nSuccessfully saved updated presentation to {DECK_PATH}")
