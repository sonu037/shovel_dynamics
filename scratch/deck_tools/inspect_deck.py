import pptx
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor

prs = pptx.Presentation(r'meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx')
s48 = prs.slides[47]
tf = s48.shapes[2].text_frame
for i, p in enumerate(tf.paragraphs):
    print(f'P{i}: text="{p.text}", level={p.level}')
    for r in p.runs:
        print(f'   run: text="{r.text}", font={r.font.name}, size={r.font.size.pt if r.font.size else None}')
