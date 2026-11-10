import re
import html
import os

def convert_md_to_html(md_path, html_path, title="Research Study Notebook"):
    with open(md_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    html_out = []
    in_code_block = False
    code_block_lang = ""
    code_buffer = []
    in_table = False
    table_header = True
    in_list = False
    list_type = None # 'ul' or 'ol'
    in_blockquote = False

    def close_list():
        nonlocal in_list, list_type
        if in_list:
            html_out.append(f"</{list_type}>")
            in_list = False
            list_type = None

    def close_table():
        nonlocal in_table, table_header
        if in_table:
            html_out.append("</tbody></table></div>")
            in_table = False
            table_header = True

    def close_blockquote():
        nonlocal in_blockquote
        if in_blockquote:
            html_out.append("</blockquote>")
            in_blockquote = False

    def format_inline(text):
        # We need to preserve $...$ while formatting **bold**, *italic*, `code`
        parts = re.split(r'(\$[^\$]+\$)', text)
        res = []
        for part in parts:
            if part.startswith('$') and part.endswith('$') and len(part) > 1:
                # Math part: keep as is
                res.append(part)
            else:
                t = part
                t = re.sub(r'`([^`]+)`', r'<code>\1</code>', t)
                t = re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', t)
                t = re.sub(r'\*([^*]+)\*', r'<em>\1</em>', t)
                t = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2">\1</a>', t)
                res.append(t)
        return "".join(res)

    i = 0
    while i < len(lines):
        line = lines[i].rstrip('\r\n')

        # Code block toggle
        if line.startswith('```'):
            if in_code_block:
                in_code_block = False
                escaped_code = html.escape("\n".join(code_buffer))
                html_out.append(f'<div class="code-container"><pre><code class="language-{code_block_lang}">{escaped_code}</code></pre></div>')
                code_buffer = []
                i += 1
                continue
            else:
                close_list()
                close_table()
                close_blockquote()
                in_code_block = True
                code_block_lang = line[3:].strip()
                code_buffer = []
                i += 1
                continue

        if in_code_block:
            code_buffer.append(line)
            i += 1
            continue

        # Check display math $$ ... $$
        if line.strip().startswith('$$'):
            close_list()
            close_table()
            close_blockquote()
            math_lines = [line]
            if not (line.strip().endswith('$$') and len(line.strip()) > 2):
                i += 1
                while i < len(lines):
                    l = lines[i].rstrip('\r\n')
                    math_lines.append(l)
                    if l.strip().endswith('$$'):
                        break
                    i += 1
            html_out.append(f'<div class="math-display">{"\n".join(math_lines)}</div>')
            i += 1
            continue

        # Empty line
        if not line.strip():
            close_list()
            close_table()
            close_blockquote()
            i += 1
            continue

        # Horizontal rule
        if line.strip() in ['---', '***', '___']:
            close_list()
            close_table()
            close_blockquote()
            html_out.append("<hr class='notebook-divider'>")
            i += 1
            continue

        # Headings
        if line.startswith('#'):
            close_list()
            close_table()
            close_blockquote()
            level = len(re.match(r'^#+', line).group(0))
            heading_text = line[level:].strip()
            formatted_title = format_inline(heading_text)
            html_out.append(f"<h{level}>{formatted_title}</h{level}>")
            i += 1
            continue

        # Blockquote / Notebook Callouts
        if line.startswith('>'):
            close_list()
            close_table()
            if not in_blockquote:
                html_out.append("<blockquote>")
                in_blockquote = True
            quote_text = line[1:].strip()
            # Handle alerts
            if quote_text.startswith('[!NOTE]') or quote_text.startswith('[!TIP]') or quote_text.startswith('[!IMPORTANT]') or quote_text.startswith('[!WARNING]') or quote_text.startswith('[!CAUTION]'):
                alert_type = quote_text[2:-1]
                html_out.append(f'<div class="callout-header"><span class="badge badge-{alert_type.lower()}">{alert_type}</span></div>')
            else:
                html_out.append(f"<p>{format_inline(quote_text)}</p>")
            i += 1
            continue
        else:
            close_blockquote()

        # Tables
        if '|' in line and line.strip().startswith('|') and line.strip().endswith('|'):
            close_list()
            if re.match(r'^\s*\|(?:\s*:?-+:?\s*\|)+\s*$', line):
                i += 1
                continue
            cols = [c.strip() for c in line.strip().split('|')[1:-1]]
            if not in_table:
                in_table = True
                table_header = True
                html_out.append('<div class="table-container"><table><thead><tr>')
                for c in cols:
                    html_out.append(f"<th>{format_inline(c)}</th>")
                html_out.append("</tr></thead><tbody>")
            else:
                html_out.append("<tr>")
                for c in cols:
                    html_out.append(f"<td>{format_inline(c)}</td>")
                html_out.append("</tr>")
            i += 1
            continue
        else:
            close_table()

        # Lists: Unordered
        m_ul = re.match(r'^\s*[-*]\s+(.*)$', line)
        if m_ul:
            close_table()
            if not in_list or list_type != 'ul':
                close_list()
                in_list = True
                list_type = 'ul'
                html_out.append("<ul>")
            html_out.append(f"<li>{format_inline(m_ul.group(1))}</li>")
            i += 1
            continue

        # Lists: Ordered
        m_ol = re.match(r'^\s*(\d+)\.\s+(.*)$', line)
        if m_ol:
            close_table()
            if not in_list or list_type != 'ol':
                close_list()
                in_list = True
                list_type = 'ol'
                html_out.append("<ol>")
            html_out.append(f"<li>{format_inline(m_ol.group(2))}</li>")
            i += 1
            continue

        # Regular Paragraph
        close_list()
        close_table()
        close_blockquote()
        html_out.append(f"<p>{format_inline(line)}</p>")
        i += 1

    close_list()
    close_table()
    close_blockquote()

    body_content = "\n".join(html_out)

    full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<script>
window.MathJax = {{
  tex: {{
    inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
    displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
    processEscapes: true
  }},
  chtml: {{
    scale: 1.05
  }},
  options: {{
    ignoreHtmlClass: 'tex2jax_ignore',
    processHtmlClass: 'tex2jax_process'
  }}
}};
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-chtml.js"></script>
<style>
  @page {{
    size: A4 portrait;
    margin: 12mm 12mm 12mm 12mm;
  }}

  /* Notebook Typography & Sizing */
  :root {{
    --primary-blue: #1d4ed8;
    --navy-header: #0f2744;
    --slate-text: #1e293b;
    --callout-bg: #f8fafc;
    --callout-border: #3b82f6;
    --notebook-rule: #e2e8f0;
  }}
  
  *, *:before, *:after {{
    box-sizing: border-box;
  }}

  body {{
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    font-size: 16.5px;
    line-height: 1.75;
    margin: 30px auto;
    max-width: 960px;
    padding: 0 25px;
    color: var(--slate-text);
    background-color: #fdfdfd;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  /* Notebook Cover & Headings */
  h1 {{
    color: var(--navy-header);
    border-bottom: 3px solid var(--primary-blue);
    padding-bottom: 12px;
    margin-top: 45px;
    font-size: 2.1em;
    line-height: 1.25;
    letter-spacing: -0.5px;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  h2 {{
    color: #1e3a8a;
    border-bottom: 2px solid #cbd5e1;
    padding-bottom: 8px;
    margin-top: 38px;
    font-size: 1.55em;
    line-height: 1.3;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  h3 {{
    color: var(--primary-blue);
    margin-top: 30px;
    font-size: 1.25em;
    line-height: 1.35;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  h4 {{
    color: #334155;
    margin-top: 22px;
    font-size: 1.1em;
    font-weight: 600;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  p {{
    margin: 16px 0;
    font-size: 1.0em;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  /* Notebook Math Display */
  .math-display {{
    overflow-x: auto;
    overflow-y: hidden;
    margin: 22px 0;
    padding: 12px 16px;
    text-align: center;
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 8px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.03);
    max-width: 100%;
  }}
  
  mjx-container[jax="CHTML"][display="true"] {{
    max-width: 100% !important;
    overflow-x: auto !important;
    overflow-y: hidden !important;
    margin: 0.5em auto !important;
  }}

  /* Notebook Code Blocks */
  .code-container {{
    margin: 22px 0;
    max-width: 100%;
  }}
  pre {{
    background: #0f172a;
    color: #f8fafc;
    border-radius: 8px;
    padding: 16px 20px;
    overflow-x: auto;
    white-space: pre-wrap;
    word-break: break-word;
    overflow-wrap: break-word;
    font-size: 13.5px;
    font-family: "Consolas", "Courier New", monospace;
    line-height: 1.5;
    border: 1px solid #334155;
    max-width: 100%;
  }}
  
  code {{
    font-family: "Consolas", "Courier New", monospace;
    background: #f1f5f9;
    color: #0f172a;
    padding: 2px 6px;
    border-radius: 4px;
    font-size: 0.9em;
    font-weight: 500;
    word-break: break-word;
  }}
  
  pre code {{
    background: transparent;
    color: inherit;
    padding: 0;
    font-weight: normal;
    white-space: pre-wrap;
    word-break: break-word;
  }}
  
  /* Notebook Tables */
  .table-container {{
    overflow-x: auto;
    margin: 28px 0;
    max-width: 100%;
  }}
  
  table {{
    border-collapse: collapse;
    width: 100%;
    max-width: 100%;
    table-layout: auto;
    font-size: 14.5px;
    background: #ffffff;
    box-shadow: 0 1px 4px rgba(0,0,0,0.04);
  }}
  
  th, td {{
    border: 1px solid #cbd5e1;
    padding: 10px 14px;
    text-align: left;
    line-height: 1.5;
    word-wrap: break-word;
    overflow-wrap: break-word;
  }}
  
  th {{
    background-color: #f1f5f9;
    color: #0f172a;
    font-weight: 700;
    font-size: 0.98em;
  }}
  
  tr:nth-child(even) {{
    background-color: #f8fafc;
  }}
  
  /* Notebook Highlight / Callout Blocks */
  blockquote {{
    border-left: 5px solid var(--primary-blue);
    margin: 24px 0;
    padding: 14px 22px;
    background: #eff6ff;
    color: #1e3a8a;
    border-radius: 0 8px 8px 0;
    font-size: 0.98em;
    box-shadow: 0 1px 3px rgba(37,99,235,0.06);
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  blockquote p {{
    margin: 6px 0;
  }}
  
  .badge {{
    display: inline-block;
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.82em;
    font-weight: 700;
    letter-spacing: 0.6px;
    text-transform: uppercase;
    margin-bottom: 6px;
  }}
  .badge-note {{ background: #dbeafe; color: #1e40af; }}
  .badge-tip {{ background: #dcfce7; color: #15803d; }}
  .badge-important {{ background: #fef3c7; color: #92400e; }}
  .badge-warning {{ background: #fee2e2; color: #b91c1c; }}
  .badge-caution {{ background: #fce7f3; color: #9d174d; }}
  
  ul, ol {{
    padding-left: 28px;
    margin: 16px 0;
  }}
  
  li {{
    margin: 8px 0;
    font-size: 1.0em;
    overflow-wrap: break-word;
    word-wrap: break-word;
  }}
  
  hr.notebook-divider {{
    border: 0;
    border-top: 2px dashed #cbd5e1;
    margin: 38px 0;
  }}
  
  /* Print Controls */
  .print-btn {{
    position: fixed;
    top: 20px;
    right: 28px;
    background: var(--primary-blue);
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 8px;
    font-size: 14px;
    font-weight: 600;
    cursor: pointer;
    box-shadow: 0 4px 10px rgba(0,0,0,0.18);
    z-index: 1000;
    transition: all 0.2s;
  }}
  
  .print-btn:hover {{
    background: #1e40af;
    transform: translateY(-1px);
  }}
  
  @media print {{
    .print-btn {{ display: none !important; }}
    
    body {{
      max-width: 100% !important;
      width: 100% !important;
      margin: 0 !important;
      padding: 0 !important;
      font-size: 10.5pt !important;
      line-height: 1.55 !important;
      color: #000 !important;
      background: #fff !important;
      overflow-wrap: break-word !important;
      word-wrap: break-word !important;
    }}
    
    h1 {{
      page-break-before: always;
      font-size: 1.6em !important;
      border-bottom: 2pt solid #000 !important;
      margin-top: 20pt !important;
      padding-bottom: 6pt !important;
    }}
    h1:first-of-type {{
      page-break-before: avoid;
      margin-top: 0 !important;
    }}
    h2 {{
      font-size: 1.3em !important;
      page-break-after: avoid;
      margin-top: 16pt !important;
      padding-bottom: 4pt !important;
    }}
    h3 {{
      font-size: 1.15em !important;
      page-break-after: avoid;
      margin-top: 12pt !important;
    }}
    h4 {{
      font-size: 1.05em !important;
      page-break-after: avoid;
      margin-top: 10pt !important;
    }}
    
    .math-display {{
      max-width: 100% !important;
      font-size: 0.85em !important;
      padding: 6px 8px !important;
      margin: 12px 0 !important;
      page-break-inside: avoid;
      overflow: hidden !important;
      border: 1pt solid #cbd5e1 !important;
    }}
    
    mjx-container[jax="CHTML"][display="true"] {{
      font-size: 86% !important;
      max-width: 100% !important;
    }}
    
    table, pre, blockquote {{
      page-break-inside: avoid;
    }}
    
    table {{
      font-size: 8.5pt !important;
      width: 100% !important;
      max-width: 100% !important;
      table-layout: fixed !important;
    }}
    th, td {{
      padding: 5px 6px !important;
      line-height: 1.35 !important;
      word-wrap: break-word !important;
      overflow-wrap: break-word !important;
      white-space: normal !important;
    }}
    
    pre {{
      font-size: 7.5pt !important;
      line-height: 1.35 !important;
      background: #f8fafc !important;
      color: #0f172a !important;
      border: 1pt solid #cbd5e1 !important;
      white-space: pre-wrap !important;
      word-break: break-word !important;
      overflow-wrap: break-word !important;
      padding: 8px 10px !important;
      max-width: 100% !important;
    }}
    
    code {{
      font-size: 8.5pt !important;
      word-break: break-word !important;
    }}
    
    blockquote {{
      font-size: 9.5pt !important;
      padding: 8px 14px !important;
      margin: 12px 0 !important;
      border-left: 4pt solid #1d4ed8 !important;
    }}
    
    p, li {{
      font-size: 10pt !important;
      line-height: 1.5 !important;
      overflow-wrap: break-word !important;
      word-wrap: break-word !important;
    }}

    a {{
      color: #000 !important;
      text-decoration: none !important;
    }}
  }}
</style>
</head>
<body class="tex2jax_process">
<button class="print-btn" onclick="window.print()">Print / Save as PDF (Ctrl+P)</button>
{body_content}
</body>
</html>"""

    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(full_html)
    print(f"Generated notebook HTML: {html_path} ({os.path.getsize(html_path)} bytes)")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2:
        convert_md_to_html(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "Study Notebook")
    else:
        convert_md_to_html(
            r"c:\Users\IIT KGP\MATLAB Drive\shovel-dynamics\study_notebook\Volume_01_Chapter_01.md",
            r"c:\Users\IIT KGP\MATLAB Drive\shovel-dynamics\study_notebook\Volume_01_Chapter_01.html",
            "Volume I - Chapter 1: Advanced Vector Mathematics, Linear Spaces & Matrix Calculus"
        )
