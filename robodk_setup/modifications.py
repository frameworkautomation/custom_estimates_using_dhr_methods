"""Modifications applied to the RoboDK station after loading.

Called automatically by setup_station.py — do not run standalone.
"""
import re
from robodk.robolink import Robolink

RDK = Robolink()
try:
    RDK.COM.settimeout(None)
except Exception:
    pass

# Matches: Machine<N>YarnTray<N>Slot2Base
YARN_TRAY_RE = re.compile(r"^Machine\d+YarnTray\d+Slot2Base$")

# Collect first, then delete (avoid mutating list while iterating)
to_delete = [item for item in RDK.ItemList() if YARN_TRAY_RE.match(item.Name())]

for item in to_delete:
    print("Deleting: " + item.Name())
    item.Delete()

print("Deleted {0} yarn tray item(s).".format(len(to_delete)))
