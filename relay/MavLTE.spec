# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller build spec for MavLTE.exe.

Build with:  python build_release.py        (it runs  pyinstaller MavLTE.spec)

A ONE-FILE build, like MavJOY: a single MavLTE.exe that carries the icon and can be put anywhere.
MavGCS is one-folder because QtWebEngine brings its own helper exe; MavLTE is the standard library,
Tk and Pillow (to show the aircraft's photos), which unpack from the exe in about a second.
"""

import re
from pathlib import Path

from PyInstaller.utils.win32.versioninfo import (FixedFileInfo, StringFileInfo, StringStruct, StringTable,
                                                 VarFileInfo, VarStruct, VSVersionInfo)

# The version is written in one place, mavrelay.py, so the exe cannot drift from it.
_source = Path(SPECPATH, "mavrelay.py").read_text(encoding="utf-8")
VERSION = re.search(r'^__version__ = "([^"]+)"', _source, re.M).group(1)
_numbers = [int(n) for n in re.findall(r"\d+", VERSION)][:4]
NUMBERS = tuple(_numbers + [0] * (4 - len(_numbers)))

# What Windows shows in the exe's Properties > Details, and the name in Task Manager (without it,
# Task Manager lists the bare file name).
VERSION_INFO = VSVersionInfo(
    ffi=FixedFileInfo(filevers=NUMBERS, prodvers=NUMBERS),
    kids=[
        StringFileInfo([StringTable("040904B0", [
            StringStruct("FileDescription", "MavLTE"),
            StringStruct("ProductName", "MavLTE"),
            StringStruct("FileVersion", VERSION),
            StringStruct("ProductVersion", VERSION),
            StringStruct("InternalName", "MavLTE"),
            StringStruct("OriginalFilename", "MavLTE.exe"),
            StringStruct("Comments", "4G/LTE MAVLink link for ArduPilot, GPL-3.0. "
                                     "https://github.com/kolabuzlu/MavLTE"),
        ])]),
        VarFileInfo([VarStruct("Translation", [0x0409, 1200])]),
    ],
)

a = Analysis(
    ["mavlte.py"],
    pathex=[],
    binaries=[],
    datas=[("mavlte.png", ".")],  # the window icon and the badge in the header
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # pyserial goes in, for the flight logs over the board's USB cable (boardusb.py). The rest are
    # big packages this app never imports; named so the exe does not depend on what the build
    # machine has installed for other projects.
    # Pillow (PIL) is in, for the JPEG photos in the viewer and thumbnail, but not its codecs for
    # other formats: AVIF alone would add 8 MB to an exe that unpacks on every start.
    excludes=["numpy", "pymavlink", "matplotlib", "yaml", "setuptools",
              "PIL._avif", "PIL.AvifImagePlugin", "PIL._webp", "PIL.WebPImagePlugin", "PIL.ImageCms",
              "PIL._imagingcms"],
    noarchive=False,
)

# Tcl's time zone database and its clock translations: 736 of the exe's 964 files, unpacked again on
# every start of a one-file exe, for Tcl's [clock format/scan], which Tk on Windows never calls (only
# its Unix file dialog does). Tk's own translations, _tk_data/msgs, stay.
a.datas = [d for d in a.datas if not d[0].replace("\\", "/").startswith(("_tcl_data/tzdata/", "_tcl_data/msgs/"))]

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="MavLTE",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    runtime_tmpdir=None,
    console=False,          # a window app: no console window behind it
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon="mavlte.ico",
    version=VERSION_INFO,
)
