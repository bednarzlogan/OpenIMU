import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

# Load files
waypoints_path = r"waypoints_20250618_210836.csv"
ground_truth_path = r"simulated_logs\ground_truth_2025_06_18_21_54_51.csv"
waypoints = pd.read_csv(waypoints_path, header=None, names=["x", "y"])
ground_truth = pd.read_csv(ground_truth_path)

# Convert and drop bad rows
waypoints = waypoints.apply(pd.to_numeric, errors='coerce').dropna()
ground_truth = ground_truth.apply(pd.to_numeric, errors='coerce').dropna()

# Subsample
ground_truth = ground_truth.iloc[::4].reset_index(drop=True)

# Setup plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.plot(waypoints["x"], waypoints["y"], "k--", label="Path")
robot_marker, = ax.plot([], [], "ro", markersize=8, label="Robot")
trace_line, = ax.plot([], [], "b-", lw=1, label="Trace")
ax.legend()
ax.set_xlim(waypoints["x"].min() - 1, waypoints["x"].max() + 1)
ax.set_ylim(waypoints["y"].min() - 1, waypoints["y"].max() + 1)
ax.set_aspect("equal")

trace_xs, trace_ys = [], []

def update(i):
    if i >= len(ground_truth):
        return robot_marker, trace_line

    x = ground_truth.iloc[i]["x"]
    y = ground_truth.iloc[i]["y"]

    trace_xs.append(x)
    trace_ys.append(y)

    robot_marker.set_data([x], [y])  # Wrap in list for sequence
    trace_line.set_data(trace_xs, trace_ys)
    return robot_marker, trace_line


Writer = animation.writers['ffmpeg']
writer = Writer(fps=20, metadata=dict(artist='Me'), bitrate=1800)
ani = FuncAnimation(fig, update, frames=len(ground_truth), interval=50)
ani.save(filename="simulation_playback.mp4", writer=writer)
