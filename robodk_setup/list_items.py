"""Print all item names currently in the RoboDK station.

Run from RoboDK: Tools > Run Script > list_items
Helps identify the exact names to target in modifications.py.
"""
from robodk.robolink import Robolink

RDK = Robolink()

items = RDK.ItemList()
print("Items in station ({0} total):".format(len(items)))
for item in items:
    print("  " + item.Name())
