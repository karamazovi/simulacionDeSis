import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    base = Path(__file__).resolve().parent
    deps_file = base / "dependencies.json"

    with deps_file.open("r", encoding="utf-8") as f:
        data = json.load(f)

    packages = data.get("pip", [])
    if not packages:
        print("No packages to install.")
        return 0

    cmd = [sys.executable, "-m", "pip", "install"] + packages
    print("Running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
