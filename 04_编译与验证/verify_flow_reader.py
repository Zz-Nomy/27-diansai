"""Forward to the current regression; historical test sources are backed up."""
from pathlib import Path
import subprocess
import sys
script = Path(__file__).resolve().parent / "BH1750_半自动光源_20261025" / "verify_light_task.py"
raise SystemExit(subprocess.call([sys.executable, str(script), *sys.argv[1:]]))
