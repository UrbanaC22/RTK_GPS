#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np

INPUT_FILE = "gpsdata_ros.txt"
SKIP_MINUTES = 0
EARTH_RADIUS = 6378137.0


# ===============================
# TIME PARSER
# ===============================

def parse_time(timestr):
    """hhmmss.sss → seconds"""
    if timestr == "":
        return None

    h = int(timestr[0:2])
    m = int(timestr[2:4])
    s = float(timestr[4:])
    return h*3600 + m*60 + s


# ===============================
# NMEA PARSER
# ===============================

def nmea_to_decimal(coord, direction):
    if coord == "":
        return None

    value = float(coord)
    deg = int(value / 100)
    minutes = value - deg * 100
    dec = deg + minutes / 60.0

    if direction in ["S", "W"]:
        dec *= -1

    return dec


def parse_gga(line):
    if not line.startswith("$GNGGA"):
        return None

    p = line.split(",")

    if len(p) < 10:
        return None

    t = parse_time(p[1])
    lat = nmea_to_decimal(p[2], p[3])
    lon = nmea_to_decimal(p[4], p[5])
    alt = float(p[9]) if p[9] else None

    if None in (t, lat, lon, alt):
        return None

    return t, lat, lon, alt


# ===============================
# GEO → METERS
# ===============================

def latlon_to_xy(lat0, lon0, lat, lon):
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)

    x = dlon * EARTH_RADIUS * math.cos(math.radians(lat0))
    y = dlat * EARTH_RADIUS

    return x, y


# ===============================
# LOAD DATA
# ===============================

times, lats, lons, alts = [], [], [], []

with open(INPUT_FILE, "r", errors="ignore") as f:
    for line in f:
        parsed = parse_gga(line.strip())
        if parsed:
            times.append(parsed[0])
            lats.append(parsed[1])
            lons.append(parsed[2])
            alts.append(parsed[3])

print(f"Total samples loaded: {len(times)}")

# ===============================
# SKIP FIRST N MINUTES
# ===============================

start_time = times[0]
cutoff = start_time + SKIP_MINUTES * 60

filtered = [
    (t, la, lo, al)
    for t, la, lo, al in zip(times, lats, lons, alts)
    if t >= cutoff
]

print(f"Samples after skipping {SKIP_MINUTES} minutes: {len(filtered)}")

times, lats, lons, alts = zip(*filtered)

# ===============================
# DRIFT CALCULATION
# ===============================

lat0, lon0, alt0 = lats[0], lons[0], alts[0]

xs, ys, distances, alt_drift = [], [], [], []

for lat, lon, alt in zip(lats, lons, alts):

    x, y = latlon_to_xy(lat0, lon0, lat, lon)
    d = math.sqrt(x**2 + y**2)

    xs.append(x)
    ys.append(y)
    distances.append(d)
    alt_drift.append(alt - alt0)

xs = np.array(xs)
ys = np.array(ys)
distances = np.array(distances)

# ===============================
# ACCURACY METRICS
# ===============================

cep50 = np.percentile(distances, 50)
cep95 = np.percentile(distances, 95)
rms = np.sqrt(np.mean(distances**2))

print("\n===== Accuracy Metrics (After Warmup) =====")
print(f"CEP50 : {cep50:.3f} m")
print(f"CEP95 : {cep95:.3f} m")
print(f"RMS   : {rms:.3f} m")
print(f"Max   : {np.max(distances):.3f} m")

# ===============================
# PLOTS
# ===============================

fig = plt.figure(figsize=(15,5))

# XY + Accuracy Circle
ax1 = fig.add_subplot(1,3,1)
ax1.scatter(xs, ys, s=5)

circle50 = plt.Circle((0,0), cep50, fill=False)
circle95 = plt.Circle((0,0), cep95, fill=False)

ax1.add_patch(circle50)
ax1.add_patch(circle95)

ax1.set_title("2D Accuracy Circle (Stable Period)")
ax1.set_xlabel("East (m)")
ax1.set_ylabel("North (m)")
ax1.axis('equal')
ax1.grid(True)

# Horizontal drift
ax2 = fig.add_subplot(1,3,2)
ax2.plot(distances)
ax2.set_title("Horizontal Drift")
ax2.grid(True)

# Altitude drift
ax3 = fig.add_subplot(1,3,3)
ax3.plot(alt_drift)
ax3.set_title("Altitude Drift")
ax3.grid(True)

plt.tight_layout()
plt.show()
