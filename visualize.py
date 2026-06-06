import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import re
import threading
import time

PORT = "/dev/cu.usbmodem11201"
BAUD = 115200
MAX_DIST = 1.0  # meters, matches firmware threshold

# shared state
grid = np.full((3, 3), np.nan)
grid_lock = threading.Lock()

LABELS = [
    ["ch0\nTL", "ch1\nT",  "ch2\nTR"],
    ["ch3\nL",  "center",  "ch4\nR"],
    ["ch5\nBL", "ch6\nB",  "ch7\nBR"],
]

def parse_serial():
    global grid
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    buf = []
    collecting = False

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if line == "grid (m):":
            collecting = True
            buf = []
            continue

        if collecting:
            buf.append(line)
            if len(buf) == 3:
                collecting = False
                new_grid = np.full((3, 3), np.nan)
                ok = True
                for r, row_str in enumerate(buf):
                    tokens = row_str.split()
                    c_out = 0
                    for tok in tokens:
                        if c_out >= 3:
                            break
                        if tok == "--":
                            c_out += 1  # center stays NaN
                        else:
                            try:
                                new_grid[r][c_out] = float(tok)
                                c_out += 1
                            except ValueError:
                                ok = False
                if ok:
                    with grid_lock:
                        grid = new_grid

def main():
    t = threading.Thread(target=parse_serial, daemon=True)
    t.start()

    fig, ax = plt.subplots(figsize=(6, 5))
    plt.title("ToF Depth Heatmap (0-1m)", fontsize=14)

    cmap = plt.cm.RdYlGn_r  # green=far, red=near
    im = ax.imshow(np.zeros((3, 3)), vmin=0, vmax=MAX_DIST,
                   cmap=cmap, interpolation="nearest")
    cbar = fig.colorbar(im, ax=ax, label="Distance (m)")

    texts = [[None]*3 for _ in range(3)]
    for r in range(3):
        for c in range(3):
            texts[r][c] = ax.text(c, r, "", ha="center", va="center",
                                  fontsize=11, color="black", fontweight="bold")

    ax.set_xticks([])
    ax.set_yticks([])

    # draw grid lines
    for i in range(4):
        ax.axhline(i - 0.5, color="white", linewidth=2)
        ax.axvline(i - 0.5, color="white", linewidth=2)

    def update(frame):
        with grid_lock:
            g = grid.copy()

        display = np.where(np.isnan(g), 0, g)
        im.set_data(display)

        for r in range(3):
            for c in range(3):
                label = LABELS[r][c]
                if r == 1 and c == 1:
                    texts[r][c].set_text(label)
                    texts[r][c].set_fontsize(9)
                elif np.isnan(g[r][c]):
                    texts[r][c].set_text(f"{label}\n---")
                else:
                    val = g[r][c]
                    texts[r][c].set_text(f"{label}\n{val:.2f}m")
                    texts[r][c].set_color("white" if val < 0.5 else "black")

        fig.canvas.draw_idle()

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
