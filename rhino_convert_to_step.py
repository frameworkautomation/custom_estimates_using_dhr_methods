"""Batch convert all SolidWorks assemblies in clones/ to STEP using Rhino.

Run this from inside Rhino via:
    -_RunPythonScript (C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\rhino_convert_to_step.py)

Or launch headlessly with run_rhino_convert.bat
"""
import os
import datetime
import rhinoscriptsyntax as rs

PROJECT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
CLONES_DIR = os.path.join(PROJECT_DIR, "clones")
OUTPUT_DIR = os.path.join(PROJECT_DIR, "steps_from_SolidWorks")
LOG_FILE = os.path.join(PROJECT_DIR, "rhino_convert.log")


def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


def find_sldasm_files(root_dir):
    results = []
    for dirpath, _, files in os.walk(root_dir):
        for f in files:
            if f.upper().endswith(".SLDASM") and not f.startswith("~$"):
                results.append(os.path.join(dirpath, f))
    return results


def main():
    # Clear log file at start of each run
    with open(LOG_FILE, "w") as f:
        f.write("")
    log("Starting conversion...")

    sldasm_files = find_sldasm_files(CLONES_DIR)
    total = len(sldasm_files)
    log(f"Found {total} SLDASM files")

    skipped = 0
    converted = 0
    failed = []

    for i, sldasm in enumerate(sldasm_files):
        # Mirror the clones/ structure under steps_from_SolidWorks/
        rel = os.path.relpath(sldasm, CLONES_DIR)
        out = os.path.join(OUTPUT_DIR, os.path.splitext(rel)[0] + ".step")

        if os.path.exists(out):
            log(f"[{i+1}/{total}] Skip (exists): {os.path.basename(sldasm)}")
            skipped += 1
            continue

        log(f"[{i+1}/{total}] Converting: {os.path.basename(sldasm)}")

        try:
            os.makedirs(os.path.dirname(out), exist_ok=True)

            # Open the assembly. Double _Enter dismisses any missing-ref dialogs.
            rs.Command(f'-_Open "{sldasm}" _Enter _Enter', False)

            # Select everything and export as STEP.
            # Leading hyphen on _Export suppresses the options dialog.
            rs.Command("_SelAll", False)
            result = rs.Command(f'-_Export "{out}" _Enter _Enter', False)

            if os.path.exists(out):
                converted += 1
                log(f"  -> OK")
            else:
                failed.append(sldasm)
                log(f"  FAILED (no output file)")

        except Exception as e:
            failed.append(sldasm)
            log(f"  ERROR: {e}")

    log(f"Done. Converted: {converted}  Skipped: {skipped}  Failed: {len(failed)}")
    if failed:
        log("Failed files:")
        for f in failed:
            log(f"  {f}")


main()
