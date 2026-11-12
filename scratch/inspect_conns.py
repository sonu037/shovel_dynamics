import zipfile, re

slx_path = r'M1/model/ShovelSimulatorr2025b_M1C.slx'
with zipfile.ZipFile(slx_path, 'r') as z:
    content = z.read('simulink/systems/system_root.xml').decode('utf-8', errors='ignore')

# print any tag containing 63#
matches = re.findall(r'<[^>]+63#[^>]+>', content)
print('Direct 63# tags:', matches)
# or search for SID="63" or Src="63#..." or Dst="63#..."
conns = re.findall(r'<[A-Za-z]+[^>]*63[^>]*>', content)
print('All 63 tags:', conns)
