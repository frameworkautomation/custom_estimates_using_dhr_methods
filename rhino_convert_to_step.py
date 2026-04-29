"""Batch convert all SolidWorks assemblies in clones/ to STEP using Rhino.

Run this from inside Rhino via:
    -_RunPythonScript (C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\rhino_convert_to_step.py)

Or launch headlessly with run_rhino_convert.bat
"""
import os
import rhinoscriptsyntax as rs

PROJECT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
CLONES_DIR = os.path.join(PROJECT_DIR, "clones")
OUTPUT_DIR = os.path.join(PROJECT_DIR, "steps_from_SolidWorks")


def find_sldasm_files(root_dir):
    results = []
    for dirpath, _, files in os.walk(root_dir):
        for f in files:
            if f.upper().endswith(".SLDASM") and not f.startswith("~$"):
                results.append(os.path.join(dirpath, f))
    return results


def main():
    sldasm_files = find_sldasm_files(CLONES_DIR)
    total = len(sldasm_files)
    print(f"Found {total} SLDASM files")

    skipped = 0
    converted = 0
    failed = []

    for i, sldasm in enumerate(sldasm_files):
        # Mirror the clones/ structure under steps_from_SolidWorks/
        rel = os.path.relpath(sldasm, CLONES_DIR)
        out = os.path.join(OUTPUT_DIR, os.path.splitext(rel)[0] + ".step")

        if os.path.exists(out):
            print(f"[{i+1}/{total}] Skip (exists): {os.path.basename(sldasm)}")
            skipped += 1
            continue

        print(f"[{i+1}/{total}] Converting: {os.path.basename(sldasm)}")

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
                print(f"  -> {out}")
            else:
                failed.append(sldasm)
                print(f"  FAILED (no output file)")

        except Exception as e:
            failed.append(sldasm)
            print(f"  ERROR: {e}")

    print(f"\nDone. Converted: {converted}  Skipped: {skipped}  Failed: {len(failed)}")
    if failed:
        print("\nFailed files:")
        for f in failed:
            print(f"  {f}")


main()
