import zipfile, re

slx_path = r'M1/model/ShovelSimulatorr2025b_M1C.slx'
with zipfile.ZipFile(slx_path, 'r') as z:
    content = z.read('simulink/systems/system_root.xml').decode('utf-8', errors='ignore')

# Find lines containing SID 63 or Rotational Friction
for line in content.splitlines():
    if any(k in line for k in ['63', 'Rotational Friction', 'Revolute Joint', 'Prismatic']):
        print(line[:120])
