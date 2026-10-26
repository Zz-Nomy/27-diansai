"""Run current flight/parser regression; original supplier observations are archived."""
from pathlib import Path
import subprocess
import sys
script = Path(__file__).resolve().parent / "AUX3_一键起飞_20261025" / "verify_auto_flight.py"
raise SystemExit(subprocess.call([sys.executable, str(script), *sys.argv[1:]]))
