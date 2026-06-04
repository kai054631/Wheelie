"""
Real-time WebSocket plotter for Wheelie robot.
Connects to the ESP32 AP at ws://192.168.4.1/ws (same data as serial).
Requires: pip install websocket-client matplotlib

Usage:
  python3 ws_plotter.py                                     # all figure groups
  python3 ws_plotter.py Pitch_Rad Voltage x3 x4            # single custom figure
  python3 ws_plotter.py 192.168.4.1 Pitch_Rad x3           # custom IP + keys
"""

import sys
import re
import threading
import collections
import websocket
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Config ---
DEFAULT_IP = "192.168.4.1"
WINDOW     = 200

ALL_KEYS = [
    "meter_error", "x1", "x2", "x3", "x4",
    "cur_pos", "tgt_pos", "pos_sp",
    "cur_vel", "tgt_vel", "vel_ff",
    "cur_pitch", "tgt_pitch",
    "cur_gyro", "tgt_gyro",
    "Pitch_Rad", "Voltage", "VL", "VR",
    "yaw_rad", "yaw_rate", "yaw_e", "yaw_mpu",
    "Left_Velocity", "Right_Velocity",
    "Temp",
]

# Logical groupings shown when no keys are specified on the command line
FIGURE_GROUPS = [
    ("State Tracking — Position", ["cur_pos", "tgt_pos", "pos_sp", "x1"]),
    ("State Tracking — Velocity", ["cur_vel", "vel_ff", "x2"]),
    ("State Tracking — Pitch",    ["cur_pitch", "tgt_pitch", "x3"]),
    ("LQR Errors",                ["x1", "x2", "x3", "x4"]),
    ("Motor Voltages",            ["Voltage", "VL", "VR"]),
    ("Pitch & Yaw",               ["Pitch_Rad", "yaw_rad", "yaw_rate"]),
    ("Yaw Debug",                 ["yaw_e", "yaw_mpu"]),
    ("Wheel Feedback",            ["Left_Velocity", "Right_Velocity"]),
    ("Diagnostics",               ["Temp"]),
]

# --- Argument parsing ---
args = sys.argv[1:]
ip = DEFAULT_IP
custom_keys = []

if args:
    if "." in args[0] and not args[0].startswith("x") and args[0] not in ALL_KEYS:
        ip = args[0]
        args = args[1:]
    custom_keys = [k for k in args if k in ALL_KEYS]

url = f"ws://{ip}/ws"
print(f"Connecting to {url}")

active_keys = custom_keys if custom_keys else ALL_KEYS
buffers = {k: collections.deque([0.0] * WINDOW, maxlen=WINDOW) for k in active_keys}
PATTERN = re.compile(r"([\w]+):\s*([-\d.]+)")

# --- WebSocket thread ---
def on_message(ws_app, message):
    data = {m.group(1): float(m.group(2)) for m in PATTERN.finditer(message)}
    for key in active_keys:
        if key in data:
            buffers[key].append(data[key])

def on_error(ws_app, error):
    print(f"WebSocket error: {error}")

def on_close(ws_app, code, msg):
    print("WebSocket closed.")

def on_open(ws_app):
    print("Connected.")

ws_app = websocket.WebSocketApp(
    url,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close,
    on_open=on_open,
)
ws_thread = threading.Thread(target=ws_app.run_forever, kwargs={"reconnect": 3}, daemon=True)
ws_thread.start()

# --- Build figures ---
# Each entry: (fig, lines, axes, keys)
figures = []
colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

def build_figure(title, keys):
    fig, axes = plt.subplots(len(keys), 1, figsize=(10, 2.2 * len(keys)), sharex=True)
    if len(keys) == 1:
        axes = [axes]
    lines = []
    for i, (ax, key) in enumerate(zip(axes, keys)):
        (ln,) = ax.plot(range(WINDOW), list(buffers[key]),
                        linewidth=1, color=colors[i % len(colors)])
        ax.set_ylabel(key, fontsize=9)
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.grid(True, linewidth=0.4)
        lines.append(ln)
    axes[-1].set_xlabel("samples")
    fig.suptitle(f"Wheelie — {title}  |  {url}", fontsize=11)
    fig.tight_layout()
    return fig, lines, axes

if custom_keys:
    print(f"Plotting: {custom_keys}  |  Close window to stop")
    fig, lines, axes = build_figure("Custom", custom_keys)
    figures.append((fig, lines, axes, custom_keys))
else:
    for group_title, group_keys in FIGURE_GROUPS:
        keys = [k for k in group_keys if k in active_keys]
        if not keys:
            continue
        fig, lines, axes = build_figure(group_title, keys)
        figures.append((fig, lines, axes, keys))
    print(f"Opened {len(figures)} figure windows  |  Close any to stop")

# --- Animation ---
def make_update(lines, keys, axes):
    def update(_):
        for ln, key, ax in zip(lines, keys, axes):
            y = list(buffers[key])
            ln.set_ydata(y)
            mn, mx = min(y), max(y)
            pad = max(abs(mx - mn) * 0.1, 0.1)
            ax.set_ylim(mn - pad, mx + pad)
        return lines
    return update

anis = []
for fig, lines, axes, keys in figures:
    ani = animation.FuncAnimation(
        fig, make_update(lines, keys, axes),
        interval=50, blit=False, cache_frame_data=False
    )
    anis.append(ani)

try:
    plt.show()
finally:
    ws_app.close()
    print("Done.")
