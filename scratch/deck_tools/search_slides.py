from pptx import Presentation

prs = Presentation('meetings/03_Final_Research_ProDeck_v5/CMPDI_Progress_v5_Final_Research_ProDeck.pptx')
for idx, slide in enumerate(prs.slides):
    text_content = ""
    for shape in slide.shapes:
        if shape.has_text_frame:
            text_content += " " + " ".join([p.text for p in shape.text_frame.paragraphs])
    for kw in ['boulder', 'ms04p06', 'coulomb', 'held-out', 'phase delay', '250 ms', 'innovation']:
        if kw in text_content.lower():
            print(f"Slide {idx+1} mentions '{kw}': {text_content[:120].strip()}...")
            break
