#!/usr/bin/env python3
"""Build the SIL core as a shared library around an UNCHANGED AutoPark.cpp.

    python3 simulation/core/build.py                       # baseline: RobotFirmware/opmodes/AutoPark.cpp
    python3 simulation/core/build.py --source path/to/AutoPark.cpp --out build/candidate

The firmware file is never copied, edited or patched: the compiler is pointed at it as-is,
with simulation/core/sdk on the include path so its `#include "NRL.h"` finds the stand-in SDK.
Works on macOS (clang, produces .dylib) and Linux (g++/clang, produces .so).
"""
import argparse
import hashlib
import os
import platform
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
REPO = os.path.dirname(SIM)
DEFAULT_SRC = os.path.join(REPO, "RobotFirmware", "opmodes", "AutoPark.cpp")


def lib_name():
    return "libglitchsim.dylib" if platform.system() == "Darwin" else "libglitchsim.so"


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def build(source=DEFAULT_SRC, out_dir=None, quiet=False):
    source = os.path.abspath(source)
    if not os.path.isfile(source):
        raise FileNotFoundError(source)
    out_dir = out_dir or os.path.join(SIM, "build", "baseline")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, lib_name())
    stamp = os.path.join(out_dir, "source.sha256")
    digest = sha256(source) + ":" + sha256(os.path.join(HERE, "sim_core.cpp")) + ":" + sha256(os.path.join(HERE, "sdk", "NRL.h"))
    if os.path.exists(out) and os.path.exists(stamp) and open(stamp).read().strip() == digest + " " + source:
        return out
    cxx = os.environ.get("CXX") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise RuntimeError("No C++ compiler found. On macOS run: xcode-select --install")
    flags = ["-std=c++17", "-O2", "-fPIC", "-shared", "-w", "-I", os.path.join(HERE, "sdk")]
    if platform.system() == "Darwin":
        flags += ["-undefined", "dynamic_lookup"]
    cmd = [cxx] + flags + [os.path.join(HERE, "sim_core.cpp"), source, "-o", out]
    if not quiet:
        print("building:", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        raise RuntimeError("build failed for " + source)
    with open(stamp, "w") as f:
        f.write(digest + " " + source)
    with open(os.path.join(out_dir, "source_path.txt"), "w") as f:
        f.write(source + "\n" + sha256(source) + "\n")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=DEFAULT_SRC)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    print(build(a.source, a.out))
