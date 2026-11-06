#!/usr/bin/env python3
"""
Trajektorien-Visualisierung für den Bug3Controller 
mit PT2-Filterung Dieses Skript simuliert die Fahrt unseres Roboters um ein Reihe von
Hindernissen und speichert die Trajektorie als übersichtliche Grafik ab. 

"""

import csv
import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import os
import subprocess

# 1. Ordnerpfade festlegen
csv_path = "../data/trajektorie_slalom_daten.csv"
if not os.path.exists(csv_path):
    csv_path = "../data/trajektorie_slalom_daten.csv"

if not os.path.exists(csv_path):
    print(
        f" Fehler: Die Messdaten unter '{csv_path}' wurden noch nicht erzeugt!"
    )
    exit(1)

# 2. Die echten C++ Messwerte aus der CSV-Datei einlesen
path_x, path_y, path_theta, states = [], [], [], []


with open(csv_path, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        path_x.append(float(row['x']))
        path_y.append(float(row['y']))
        path_theta.append(float(row['theta']))
        states.append(int(row["state"]))

goal_x, goal_y = 4.0, 0.0

kisten = [
    {
        "min_x": 0.8,
        "max_x": 1.4,
        "min_y": -0.3,
        "max_y": 0.3,
        "label": "Kiste 1",
    },
    {
        "min_x": 2.2,
        "max_x": 2.8,
        "min_y": -0.1,
        "max_y": 0.5,
        "label": "Kiste 2",
    },
]

# 3. Grafik erstellen
fig, ax = plt.subplots(figsize=(12, 6))

# 4. Beide Kisten-Hindernisse dynamisch zeichnen

for i, kiste in enumerate(kisten):
    breite = kiste["max_x"] - kiste["min_x"]
    hoehe = kiste["max_y"] - kiste["min_y"]
    rechteck = patches.Rectangle(
        (kiste["min_x"], kiste["min_y"]),
        breite,
        hoehe,
        linewidth=2,
        edgecolor="red",
        facecolor="red",
        alpha=0.3,
        label="Hindernisse (slalom)" if i == 0 else "",
    )
    ax.add_patch(rechteck)

# 2. Start (0.0, 0.0) und Ziel (4.0, 0.0) einzeichnen
ax.plot(0, 0, 'go', markersize=10, label='Start (0,0)')
ax.plot(goal_x, goal_y, 'b*', markersize=15, label='Ziel ({goal_x}, {goal_y})')

# 3. Fahrspur nach Zustanden farblich zeichnen
for i in range(len(path_x) - 1):
    farbe = "royalblue" if states[i] == 0 else "darkorange"
    ax.plot(
        path_x[i : i + 2],
        path_y[i : i + 2],
        color=farbe,
        linewidth=2.5,
        alpha = 0.9,
    )

ax.plot([], [], color="royalblue", linewidth=2.5, label="Status: GOAL_SEEKING")
ax.plot([], [], color="darkorange", linewidth=2.5, label="Status: WALL_FOLLOWING")

# 4. Orientierungspfeile (Ausrichtung der Roboter-Nase)
step = 5
u = [math.cos(t) * 0.08 for t in path_theta[::step]]    
v = [math.sin(t) * 0.08 for t in path_theta[::step]] 
ax.quiver(
    path_x[::step],
    path_y[::step],
    u, 
    v, 
    color='darkblue',
    scale=1,
    scale_units='x',
    width=0.005,
)

# 5. Legende Orientierungspfeile
ax.plot(
    [],
    [],
    color='darkblue',
    marker='>',
    linestyle='None',
    label='Blickrichtung / Orientierung (Nase)',
)

# 6. Design & Beschriftung
ax.set_title(
    "Mecanum-Slalom: Doppelter Statuswechsel und Punktlandung am Ziel",
    fontsize=12,
    fontweight="bold",
)
ax.set_xlabel('X-Postion [Meter]')
ax.set_ylabel('Y-Postion [Meter]')
ax.set_aspect('equal')
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='upper right', fontsize=8, framealpha=0.8)

# 7. Bild speichern
output_dir = "../images"
if not os.path.exists(output_dir):
    os.makedirs(output_dir, exist_ok=True)

output_img = os.path.join(output_dir, "ausweich_trajektorie_slalom.png") 
if not os.path.exists("../images"):
    os.makedirs("images", exist_ok=True)
    output_img = "images/ausweich_trajektorie_slalom.png"

plt.savefig(output_img, dpi=150, bbox_inches='tight')
print("Grafik erfolgreich im Ordner images gespeichert: {output_img}")

# Automatische Öffnen:
subprocess.run(['code', output_img], check=False)

    