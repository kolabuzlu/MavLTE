"""
Build MavLTE.exe, and a zip of it ready to attach to a GitHub release.

    python build_release.py

Makes  dist/MavLTE/MavLTE.exe  (with LICENSE beside it)  and  dist/MavLTE-<version>-windows.zip.

Windows only: PyInstaller freezes the Python it runs on, so a Windows exe is built on Windows.
Not needed to run MavLTE from source (MavLTE.pyw) - this is only for making the exe.
"""

import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
LICENSE = HERE.parent / "LICENSE"
# PyInstaller's work folder, the exe and the zip together stay well under 100 MB
REQUIRED_FREE_MB = 300


def app_version() -> str:
    text = (HERE / "mavrelay.py").read_text(encoding="utf-8")
    m = re.search(r'^__version__ = "([^"]+)"', text, re.M)
    return m.group(1) if m else "dev"


def remove_stale() -> None:
    """A stale build/ silently reuses old analysis results: "I fixed that but the exe still does it"."""
    for stale in ("build", "dist"):
        if Path(stale).exists():
            print(f"Removing the old {stale}/ ...")
            try:
                shutil.rmtree(stale)
            except OSError as e:
                sys.exit(f"Cannot remove {e.filename}. Is MavLTE still running from there? Close it and "
                         "run this again.")


def archive(src: Path, zip_path: Path) -> None:
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for path in sorted(src.rglob("*")):
            if path.is_file():
                # one MavLTE/ folder inside the zip, so it cannot spill loose files into Downloads
                z.write(path, Path(src.name) / path.relative_to(src))


def main() -> None:
    if sys.platform != "win32":
        sys.exit("MavLTE.exe has to be built on Windows (PyInstaller freezes the Python it runs on).")
    os.chdir(HERE)
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        sys.exit("PyInstaller is not installed. Run:  pip install pyinstaller")

    remove_stale()
    free_mb = shutil.disk_usage(HERE).free / 1e6
    if free_mb < REQUIRED_FREE_MB:
        sys.exit(f"Need about {REQUIRED_FREE_MB} MB free to build, and there are {free_mb:.0f} MB.")

    print("Running PyInstaller (this takes a minute) ...")
    subprocess.run([sys.executable, "-m", "PyInstaller", "--noconfirm", "--distpath", "build/exe",
                    "--workpath", "build/work", "MavLTE.spec"], check=True)
    exe = HERE / "build" / "exe" / "MavLTE.exe"
    if not exe.exists():
        sys.exit(f"The build finished but {exe} is missing.")

    # GPL v3 asks that the licence travel with the program: it goes beside MavLTE.exe, both in the
    # folder it runs from and in the zip.
    if not LICENSE.exists():
        sys.exit("LICENSE is missing, and the release has to carry it.")
    payload = HERE / "dist" / "MavLTE"
    payload.mkdir(parents=True)
    shutil.copy2(exe, payload / "MavLTE.exe")
    shutil.copy2(LICENSE, payload / "LICENSE")

    zip_path = HERE / "dist" / f"MavLTE-{app_version()}-windows.zip"
    print(f"Zipping -> {zip_path.name} ...")
    archive(payload, zip_path)

    print("\nDone.")
    print(f"  Executable: {payload / 'MavLTE.exe'}  ({(payload / 'MavLTE.exe').stat().st_size / 1e6:.0f} MB)")
    print(f"  Zip:        {zip_path}  ({zip_path.stat().st_size / 1e6:.0f} MB)")


if __name__ == "__main__":
    main()
