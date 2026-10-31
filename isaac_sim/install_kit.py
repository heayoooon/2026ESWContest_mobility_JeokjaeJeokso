"""Install isaac_twin_light.kit into the Isaac Sim 4.2 pip package (run once with the Isaac Sim python).

    python install_kit.py

run_forest_twin.py starts Kit with <python>/Lib/site-packages/isaacsim/apps/isaac_twin_light.kit. The Korean panel
font region file is referenced by absolute path, so this script writes this folder's path into the copy.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
target = Path(sys.executable).parent / 'Lib/site-packages/isaacsim/apps/isaac_twin_light.kit'
if not target.parent.exists():
    sys.exit(f'Isaac Sim apps folder not found: {target.parent} - run this with the env_isaacsim42 python')
text = (HERE / 'isaac_twin_light.kit').read_text(encoding='utf-8').replace('__ISAAC_SIM_DIR__', HERE.as_posix())
target.write_text(text, encoding='utf-8')
print('installed', target)
