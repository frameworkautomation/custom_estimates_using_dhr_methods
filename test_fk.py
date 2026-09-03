"""
Quick FK test — two poses to verify DH parameters look right.
Pose 1: all joints 0
Pose 2: j1=90, all others 0

Shows the arm skeleton for both poses in PyBullet.
"""

import math
import numpy as np
import pybullet as p
import pybullet_data
import time

# DH parameters for Fanuc R-2000iC 125L (approximate)
FANUC_DH = [
    (312.0,  -math.pi/2,  780.0,  0.0),
    (1075.0,  0.0,           0.0, -math.pi/2),
    (225.0,  -math.pi/2,    0.0,   0.0),
    (0.0,     math.pi/2,  1280.0,  0.0),
    (0.0,    -math.pi/2,    0.0,   0.0),
    (0.0,     0.0,          215.0,  0.0),
]


def dh_matrix(a, alpha, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,   0.0,  1.0],
    ])


def fk_all_links(joints_deg):
    joints_rad = [math.radians(j) for j in joints_deg[:6]]
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for i, (a, alpha, d, theta_off) in enumerate(FANUC_DH):
        theta = joints_rad[i] + theta_off
        T = T @ dh_matrix(a, alpha, d, theta)
        positions.append(T[:3, 3].copy())
    return positions


# Two test poses
pose1 = [0, 0, 0, 0, 0, 0]       # all zeros
pose2 = [90, 0, 0, 0, 0, 0]      # j1=90

pos1 = fk_all_links(pose1)
pos2 = fk_all_links(pose2)

print("Pose 1 (all zeros):")
labels = ["base", "J1", "J2", "J3", "J4", "J5", "TCP"]
for label, p_ in zip(labels, pos1):
    print(f"  {label:>5}: x={p_[0]:>8.1f}  y={p_[1]:>8.1f}  z={p_[2]:>8.1f}")

print("\nPose 2 (j1=90):")
for label, p_ in zip(labels, pos2):
    print(f"  {label:>5}: x={p_[0]:>8.1f}  y={p_[1]:>8.1f}  z={p_[2]:>8.1f}")

# Visualize
scale = 0.001  # mm to meters

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
p.loadURDF("plane.urdf", [0, 0, -0.01])

# Draw pose 1 in blue
for i in range(len(pos1) - 1):
    a = [pos1[i][0]*scale, pos1[i][1]*scale, pos1[i][2]*scale]
    b = [pos1[i+1][0]*scale, pos1[i+1][1]*scale, pos1[i+1][2]*scale]
    p.addUserDebugLine(a, b, [0, 0, 1], lineWidth=4, lifeTime=0)
    p.addUserDebugText(labels[i], a, [0, 0, 1], textSize=1.0)

# Draw pose 2 in red
for i in range(len(pos2) - 1):
    a = [pos2[i][0]*scale, pos2[i][1]*scale, pos2[i][2]*scale]
    b = [pos2[i+1][0]*scale, pos2[i+1][1]*scale, pos2[i+1][2]*scale]
    p.addUserDebugLine(a, b, [1, 0, 0], lineWidth=4, lifeTime=0)
    p.addUserDebugText(labels[i], a, [1, 0, 0], textSize=1.0)

# Camera
p.resetDebugVisualizerCamera(4.0, 45, -30, [0, 0, 0.5])

print("\nBlue = all zeros, Red = j1=90")
print("Middle mouse to orbit, scroll to zoom, Ctrl+middle to pan")
print("Close window or Ctrl+C to exit")

try:
    while True:
        p.stepSimulation()
        time.sleep(1/60)
except KeyboardInterrupt:
    pass

p.disconnect()
