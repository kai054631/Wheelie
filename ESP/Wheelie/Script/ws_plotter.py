"""
Real-time WebSocket plotter for Wheelie robot.
Connects to the ESP32 AP at ws://192.168.4.1/ws (same data as serial).
Requires: pip install websocket-client matplotlib

Usage:
  python3 ws_plotter.py                                     # default variables
  python3 ws_plotter.py Pitch_Angle Voltage x3 x4
  python3 ws_plotter.py 192.168.4.1 Pitch_Angle x3         # custom IP
"""

import sys
import re
import threading
import collections
import websocket
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Config ---
DEFAULT_IP   = "192.168.4.1"
WINDOW       = 200
DEFAULT_PLOT = ["Pitch_Angle", "Voltage", "x3"]

ALL_KEYS = ["x1", "x2", "x3", "x4",
            "Pitch_Angle", "Voltage",
            "Left_Velocity", "Right_Velocity",
            "Motor_L_Angle", "Motor_R_Angle"]

# --- Argument parsing ---
args = sys.argv[1:]
ip = DEFAULT_IP
plot_keys = DEFAULT_PLOT

if args:
    # first arg is IP if it contains dots, otherwise treat as key name
    if "." in args[0] and not args[0].startswith("x") and args[0] not in ALL_KEYS:
        ip = args[0]
        args = args[1:]
    plot_keys = [k for k in args if k in ALL_KEYS] or DEFAULT_PLOT

url = f"ws://{ip}/ws"
print(f"Connecting to {url}")
print(f"Plotting: {plot_keys}  |  Close window to stop")

# --- Shared data buffers (thread-safe via deque) ---
buffers = {k: collections.deque([0.0] * WINDOW, maxlen=WINDOW) for k in plot_keys}
PATTERN = re.compile(r"([\w]+):\s*([-\d.]+)")

# --- WebSocket thread ---
def on_message(ws_app, message):
    data = {m.group(1): float(m.group(2)) for m in PATTERN.finditer(message)}
    for key in plot_keys:
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
fig.suptitle(f"Wheelie WebSocket Plotter  ({url})", fontsize=11)
fig.tight_layout()

def update(_):
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
    ws_app.close()
    print("Done.")
