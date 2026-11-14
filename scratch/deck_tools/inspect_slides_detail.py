from pptx import Presentation

prs = Presentation('meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx')
for num in [41, 42, 46, 48]:
    slide = prs.slides[num-1]
    print(f"\n=================== SLIDE {num} ===================")
    for idx, shape in enumerate(slide.shapes):
        if shape.has_text_frame:
            for p in shape.text_frame.paragraphs:
                txt = p.text.strip()
                if txt:
                    print(f"  [{idx}] {txt}")
