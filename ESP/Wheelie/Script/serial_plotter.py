"""
Real-time serial plotter for Wheelie robot.
Parses the monitor output format:
  x1: ..., x2: ..., x3: ..., x4: ..., Pitch_Angle: ..., Voltage: ...,
  Left_Velocity: ..., Right_Velocity: ..., Motor_L_Angle: ..., Motor_R_Angle: ...

Usage:
  python3 serial_plotter.py              # auto-detect port, plot defaults
  python3 serial_plotter.py /dev/ttyUSB0
  python3 serial_plotter.py /dev/ttyUSB0 Pitch_Angle Voltage x3
"""

import sys
import re
import glob
import collections
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Config ---
BAUD = 115200
WINDOW = 200          # number of samples shown on screen
PORT = None           # set below

ALL_KEYS = ["x1", "x2", "x3", "x4",
            "Pitch_Angle", "Voltage", "VL", "VR",
            "Left_Velocity", "Right_Velocity",
            "Motor_L_Angle", "Motor_R_Angle",
            "yaw_angle", "yaw_rate", "Temp"]

DEFAULT_PLOT = ["Pitch_Angle", "Voltage", "yaw_angle"]  # change to taste

# --- Argument parsing ---
args = sys.argv[1:]
if args and not args[0].startswith("/dev/"):
    plot_keys = args
elif len(args) >= 2:
    PORT = args[0]
    plot_keys = args[1:]
elif len(args) == 1:
    PORT = args[0]
    plot_keys = DEFAULT_PLOT
else:
    plot_keys = DEFAULT_PLOT

plot_keys = [k for k in plot_keys if k in ALL_KEYS]
if not plot_keys:
    plot_keys = DEFAULT_PLOT

# --- Auto-detect serial port ---
def find_port():
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    if candidates:
        return candidates[0]
    raise RuntimeError("No serial port found. Pass port as first argument.")

port = PORT or find_port()
print(f"Opening {port} at {BAUD} baud")
print(f"Plotting: {plot_keys}  |  Ctrl-C or close window to stop")

ser = serial.Serial(port, BAUD, timeout=1)

# --- Data buffers ---
buffers = {k: collections.deque([0.0] * WINDOW, maxlen=WINDOW) for k in plot_keys}

# --- Parse one line ---
PATTERN = re.compile(r"([\w]+):\s*([-\d.]+)")

def parse_line(line):
    return {m.group(1): float(m.group(2)) for m in PATTERN.finditer(line)}

# --- Plot setup ---
fig, axes = plt.subplots(len(plot_keys), 1, figsize=(10, 2.5 * len(plot_keys)), sharex=True)
if len(plot_keys) == 1:
    axes = [axes]

lines = []
for ax, key in zip(axes, plot_keys):
    (ln,) = ax.plot(range(WINDOW), list(buffers[key]), linewidth=1)
    ax.set_ylabel(key, fontsize=9)
    ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax.grid(True, linewidth=0.4)
    lines.append(ln)

axes[-1].set_xlabel("samples")
fig.suptitle("Wheelie Serial Plotter", fontsize=11)
fig.tight_layout()

# --- Animation update ---
def update(_):
    while ser.in_waiting:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue
        data = parse_line(raw)
        for key in plot_keys:
            if key in data:
                buffers[key].append(data[key])

    for ln, key, ax in zip(lines, plot_keys, axes):
        y = list(buffers[key])
        ln.set_ydata(y)
        mn, mx = min(y), max(y)
        pad = max(abs(mx - mn) * 0.1, 0.1)
        ax.set_ylim(mn - pad, mx + pad)

    return lines

ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print("Serial port closed.")
