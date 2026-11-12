import zipfile, re

slx_path = r'M1/model/ShovelSimulatorr2025b_M1C.slx'
with zipfile.ZipFile(slx_path, 'r') as z:
    content = z.read('simulink/systems/system_root.xml').decode('utf-8', errors='ignore')

lines = re.findall(r'<Line>.*?</Line>', content, re.DOTALL)
print(f'Total lines: {len(lines)}')
for l in lines:
    if '63' in l or '29' in l or '37' in l:
        print(l.replace('\n', ' '))
