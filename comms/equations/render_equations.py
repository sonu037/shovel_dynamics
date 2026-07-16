#!/usr/bin/env python3
"""Render every equation in equations_master.tex to journal-look PNGs.
Computer Modern fontset = the glyph shapes IEEE/Elsevier papers use.
Output: png/<name>_navy.png (slides) and png/<name>_black.png (docs)."""
import re, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['mathtext.fontset'] = 'cm'    # Computer Modern: the journal look

os.makedirs('png', exist_ok=True)
txt = open('equations_master.tex').read()
txt = re.sub(r'(?<!\\)%.*', '', txt)

# capture body up to the newline that ends the newcommand (single-line bodies)
pat = re.compile(r'\\newcommand\{\\(eq\w+)\}\{\s*\n(.*?)\}\s*\n', re.S)

def clean(body):
    body = ' '.join(body.split())
    body = body.replace(r'\qquad', r'\ \ \ \ ').replace(r'\quad', r'\ \ ')
    return body

count = 0
for name, body in pat.findall(txt):
    tex = '$' + clean(body) + '$'
    for tag, color in (('navy', '#1F3864'), ('black', '#000000')):
        fig = plt.figure(figsize=(0.1, 0.1))
        fig.text(0, 0, tex, fontsize=30, color=color)
        out = f'png/{name}_{tag}.png'
        try:
            fig.savefig(out, dpi=600, bbox_inches='tight', pad_inches=0.05, transparent=True)
            count += 1
        except Exception as e:
            print('FAIL', name, '->', e)
        plt.close(fig)
print(f'{count} PNGs rendered')
