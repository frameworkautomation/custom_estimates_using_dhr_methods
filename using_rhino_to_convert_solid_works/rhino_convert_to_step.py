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
HISTORY_FILE = os.path.join(PROJECT_DIR, "rhino_convert_history.log")


def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    line = "[{0}] {1}".format(ts, msg)
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")
    with open(HISTORY_FILE, "a") as f:
        f.write(line + "\n")


def find_sldasm_files(root_dir):
    results = []
    for dirpath, _, files in os.walk(root_dir):
        for f in files:
            if f.upper().endswith(".SLDASM") and not f.startswith("~$"):
                results.append(os.path.join(dirpath, f))
    return results


def main():
    # Clear the live log each run; append a separator to the history log
    with open(LOG_FILE, "w") as f:
        f.write("")
    run_ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(HISTORY_FILE, "a") as f:
        f.write("\n=== Run started {0} ===\n".format(run_ts))
    log("Starting conversion...")

    sldasm_files = find_sldasm_files(CLONES_DIR)
    total = len(sldasm_files)
    log("Found {0} SLDASM files".format(total))

    skipped = 0
    converted = 0
    failed = []

    for i, sldasm in enumerate(sldasm_files):
        # Mirror the clones/ structure under steps_from_SolidWorks/
        rel = os.path.relpath(sldasm, CLONES_DIR)
        out = os.path.join(OUTPUT_DIR, os.path.splitext(rel)[0] + ".step")

        if os.path.exists(out):
            log("[{0}/{1}] Skip (exists): {2}".format(i+1, total, os.path.basename(sldasm)))
            skipped += 1
            continue

        log("[{0}/{1}] Converting: {2}".format(i+1, total, os.path.basename(sldasm)))

        try:
            out_dir = os.path.dirname(out)
            if not os.path.exists(out_dir):
                os.makedirs(out_dir)

            # Open the assembly. Double _Enter dismisses any missing-ref dialogs.
            rs.Command('-_Open "{0}" _Enter _Enter'.format(sldasm), False)

            # Select everything and export as STEP.
            # Leading hyphen on _Export suppresses the options dialog.
            rs.Command("_SelAll", False)
            result = rs.Command('-_Export "{0}" _Enter _Enter'.format(out), False)

            if os.path.exists(out):
                converted += 1
                log("  -> OK")
            else:
                failed.append(sldasm)
                log("  FAILED (no output file)")

        except Exception as e:
            failed.append(sldasm)
            log("  ERROR: {0}".format(e))

    log("Done. Converted: {0}  Skipped: {1}  Failed: {2}".format(converted, skipped, len(failed)))
    if failed:
        log("Failed files:")
        for f in failed:
            log("  {0}".format(f))


main()
