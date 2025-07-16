#!/usr/bin/env python3
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ament_index_python.packages import get_package_share_directory

LOG_PATH = os.path.join(
    get_package_share_directory('sensor_emulator'),
    'logs',
    'detections.csv'
)

def read_data():
    points = []
    try:
        with open(LOG_PATH, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                points.append((x, y))
    except Exception as e:
        print(f"Error reading CSV: {e}")
    return points

fig, ax = plt.subplots()
sc = ax.scatter([], [], c='red', label='Fused Detections')
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_title("Fused Detections Over Time")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.legend()

def update(frame):
    data = read_data()
    if data:
        x_vals, y_vals = zip(*data)
        sc.set_offsets(list(zip(x_vals, y_vals)))
    return sc,

ani = animation.FuncAnimation(fig, update, interval=1000)
plt.show()

