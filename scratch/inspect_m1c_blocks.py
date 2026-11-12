import zipfile
import re

slx_path = r'M1/model/ShovelSimulatorr2025b_M1C.slx'
with zipfile.ZipFile(slx_path, 'r') as z:
    content = z.read('simulink/systems/system_root.xml').decode('utf-8', errors='ignore')

blocks = re.findall(r'<Block\s+[^>]*>.*?</Block>', content, re.DOTALL)
print(f'Total blocks in system_root.xml: {len(blocks)}')
for b in blocks:
    if any(k in b.lower() for k in ['prismatic', 'friction', 'revolute', '480', '340']):
        name_m = re.search(r'Name="([^"]+)"', b)
        b_type = re.search(r'BlockType="([^"]+)"', b)
        print(f"Block: Name='{name_m.group(1) if name_m else '?'}' Type='{b_type.group(1) if b_type else '?'}'")
        # print parameters matching friction or damping
        for p in re.findall(r'<P Name="[^"]+">[^<]+</P>', b):
            if any(k in p.lower() for k in ['trq', 'force', 'damping', 'fric', 'col', 'brk', '340', '480', '1473', '3800']):
                print(f"   {p}")
