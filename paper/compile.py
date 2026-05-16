#!/usr/bin/env python3
"""Compile LaTeX paper using tectonic (auto-downloads packages)."""
import subprocess
import sys
import os
from pathlib import Path

PAPER_DIR = Path(__file__).parent / "latex"
MAIN_TEX = PAPER_DIR / "main.tex"
VENV_BIN = Path(__file__).parent / ".venv" / "bin"
TECTONIC = VENV_BIN / "tectonic"

def compile_latex():
    if not MAIN_TEX.exists():
        print(f"ERROR: {MAIN_TEX} not found")
        sys.exit(1)
    
    cmd = [
        str(TECTONIC),
        "-X", "compile",
        str(MAIN_TEX),
        "--outdir", str(PAPER_DIR),
        "--keep-intermediates",
    ]
    
    print(f"Compiling {MAIN_TEX}...")
    result = subprocess.run(cmd, capture_output=False)
    
    if result.returncode == 0:
        pdf = PAPER_DIR / "main.pdf"
        if pdf.exists():
            size_mb = pdf.stat().st_size / (1024 * 1024)
            print(f"\nSUCCESS: {pdf} ({size_mb:.1f} MB)")
        else:
            print("\nWARNING: tectonic returned 0 but PDF not found")
    else:
        print(f"\nFAILED (exit code {result.returncode})")
    
    return result.returncode

if __name__ == "__main__":
    sys.exit(compile_latex())
