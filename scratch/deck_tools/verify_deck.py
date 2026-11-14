import os
import zipfile
import pptx
from pptx.util import Pt

DECK_PATH = r"meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx"

print("=" * 70)
print("COMPREHENSIVE PPTX VERIFICATION")
print("=" * 70)

# 1. File existence & size
file_size = os.path.getsize(DECK_PATH)
print(f"File exists: {DECK_PATH}")
print(f"File size: {file_size:,} bytes ({file_size / (1024*1024):.2f} MB)")

# 2. ZIP integrity check (pptx is an OPC zip package)
with zipfile.ZipFile(DECK_PATH, "r") as zf:
    bad_file = zf.testzip()
    if bad_file:
        raise ValueError(f"Corrupt file found in PPTX zip: {bad_file}")
    print(f"ZIP package integrity: 100% OK ({len(zf.namelist())} members verified)")

# 3. PPTX loading via python-pptx
prs = pptx.Presentation(DECK_PATH)
print(f"Total slides loaded: {len(prs.slides)}")
assert len(prs.slides) == 51, f"Expected 51 slides, found {len(prs.slides)}"

# 4. Detailed Check of Slide 41
s41 = prs.slides[40]
print("\n" + "=" * 50)
print("SLIDE 41 VERIFICATION")
print("=" * 50)
print("Title:", s41.shapes[0].text_frame.text.strip())
tf41 = s41.shapes[2].text_frame
print(f"Paragraphs count in left card: {len(tf41.paragraphs)}")
for i, p in enumerate(tf41.paragraphs):
    print(f"  [{i}] {p.text}")

resolved_found = False
for p in tf41.paragraphs:
    if "RESOLVED via M5 Adaptive Innovation Gate:" in p.text:
        resolved_found = True
        print("\n--> Found RESOLVED paragraph:")
        print(f"    Full text: '{p.text}'")
        for r in p.runs:
            print(f"    Run '{r.text}' | Font: {r.font.name} {r.font.size.pt if r.font.size else 'default'}pt | Bold: {r.font.bold}")

assert resolved_found, "Slide 41: RESOLVED text not found!"

# 5. Detailed Check of Slide 48
s48 = prs.slides[47]
print("\n" + "=" * 50)
print("SLIDE 48 VERIFICATION")
print("=" * 50)
print("Title:", s48.shapes[0].text_frame.text.strip())

tf48_concl = s48.shapes[2].text_frame
print(f"\nConclusions Paragraphs count: {len(tf48_concl.paragraphs)}")
for i, p in enumerate(tf48_concl.paragraphs):
    print(f"  [{i}] {p.text}")

bullet1_expected = "Prismatic Coulomb friction physically integrated on crowd joint (Fc4 = 480 N continuous tanh, zero solver chatter, 6.3s run)"
bullet2_expected = "Boulder impact step-lag collapsed from 250 ms down to < 33 ms via M5 Adaptive Innovation Gate (NRMSE 26.2% -> 9.5%)"

b1_found = any(bullet1_expected in p.text for p in tf48_concl.paragraphs)
b2_found = any(bullet2_expected in p.text for p in tf48_concl.paragraphs)

print(f"\nBullet 1 verified: {b1_found}")
print(f"Bullet 2 verified: {b2_found}")

assert b1_found, "Slide 48: Bullet 1 not found!"
assert b2_found, "Slide 48: Bullet 2 not found!"

tf48_supp = s48.shapes[4].text_frame
print(f"\nSupport Requested Paragraphs count: {len(tf48_supp.paragraphs)}")
for i, p in enumerate(tf48_supp.paragraphs):
    print(f"  [{i}] {p.text}")

print("\n" + "=" * 70)
print("ALL VERIFICATION CHECKS PASSED PERFECTION!")
print("=" * 70)
