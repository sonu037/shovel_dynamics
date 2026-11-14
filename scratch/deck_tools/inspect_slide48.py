from pptx import Presentation

prs = Presentation('meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx')
print(f"Total slides: {len(prs.slides)}")

if len(prs.slides) >= 48:
    slide = prs.slides[47] # 0-indexed slide 48
    print("=== Slide 48 Shapes ===")
    for idx, shape in enumerate(slide.shapes):
        if shape.has_text_frame:
            for p in shape.text_frame.paragraphs:
                if p.text.strip():
                    print(f"  Shape {idx}: {p.text.strip()[:100]}")
