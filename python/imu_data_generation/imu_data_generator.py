import os
from datetime import datetime
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from scipy.interpolate import CubicSpline

# ----- parameters -----
m            = 5.0       # robot mass (kg)
Ichassis     = 0.5       # chassis yaw inertia (kg·m²)
Iw           = 0.01      # each wheel inertia (kg·m²)
r            = 0.1       # wheel radius (m)
L            = 0.3       # half‑axle width (m)
mu           = 0.8       # coefficient of static friction
N            = m * 9.81  # normal force per wheel (approx)
Ld           = 1.0       # look‑ahead distance for pure pursuit (m)
dt           = 0.05      # time step (s)

last_idx = 0  # for look‑ahead bookkeeping
last_timestamp = 0 # populates output log

# pure‑pursuit gains / speeds
desired_speed  = 0.5     # forward speed along path (m/s)
Kpv, Kiv       = 1.0, 0.1
Kpw, Kiw       = 2.0, 0.1

# ----- state variables -----
state = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,   # heading
    "v": 0.0,       # forward speed
    "yaw": 0.0      # yaw rate
}

# integral accumulators for PI
int_ev  = 0.0
int_ew  = 0.0

# store look‑ahead points for trace
pursuit_xs, pursuit_ys = [], []

# set the target for the log output
time_now: str = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
target_log_name: str = f"generator_test_{time_now}.csv"
target_log_path = os.path.join("simulated_logs", target_log_name)
if not os.path.exists("simulated_logs"):
    os.mkdir("simulated_logs")

with open(target_log_path, 'a') as log:
    log.write("timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z\n")

# set the ground truth output
truth_log_name = f"ground_truth_{time_now}.csv"
ground_truth_target = os.path.join("simulated_logs", truth_log_name)

with open(ground_truth_target, 'w') as log:
    log.write("timestamp,x,y,z,vx,vy,vz,phi,theta,psi,b_ax,b_ay,b_az,b_gx,b_gy,b_gz\n")


def smooth_path(path, num_points=200):
    """
    take a list of (x,y) waypoints and return a smoothed path
    of length num_points using a natural (or periodic) cubic spline.
    """
    pts = np.array(path)               # shape (N,2)
    n  = len(pts)

    # parameterize original points by u in [0,1]
    u  = np.linspace(0, 1, n)

    # build splines for x(u) and y(u)
    # use 'periodic' if your path loops; else leave bc_type default
    cs_x = CubicSpline(u, pts[:,0], bc_type='not-a-knot')
    cs_y = CubicSpline(u, pts[:,1], bc_type='not-a-knot')

    # sample a finer, smooth curve
    uf     = np.linspace(0, 1, num_points)
    smooth = list(zip(cs_x(uf), cs_y(uf)))
    return smooth


def find_lookahead_point(path, state, Ld, last_idx):
    # pull the state info and pack it into an array
    x, y = state["x"], state["y"]
    grid_position = np.array([x, y])

    # check the next path point using last indx
    for i in range(last_idx, len(path)-1):
        # find the points that intersect the route segment
        segment_start = np.array(path[i])
        segment_end = np.array(path[i+1]) 
        path_segment = segment_end - segment_start
        cur_offset = segment_start - grid_position

        # point along a path segment can be described via
        # X(t) = A + t * (B - A); 0 <= t <= 1 
        # where A is segment_start and B is segment_end
        # we're searching for the points where norm2(X(t) - [x; y]) = Ld**2

        # solution is in quadratic form
        a = np.dot(path_segment, path_segment)
        b = 2 * np.dot(path_segment, cur_offset)
        c = np.dot(cur_offset, cur_offset) - Ld**2

        # solve the quadratic formula, checking for no solution
        disc = b*b - 4*a*c
        if disc < 0:
            continue
        sqrt_disc = np.sqrt(disc)

        # find the closer intersection
        candidates = []
        for t in [(-b + sqrt_disc) / (2 * a), (-b - sqrt_disc) / (2 * a)]:
            if 0 <= t <= 1:
                point = segment_start + t * path_segment
                xb, _ = to_body_frame(point, state)
                if xb > 0:  # point is in front of robot
                    candidates.append((xb, point, i))

        if candidates:
            # pick the candidate that's most forward in body x direction
            _, best_point, best_i = max(candidates, key=lambda tup: tup[0])
            return best_point, best_i

    return np.array(path[-1]), last_idx


def to_body_frame(pt, state):
    # transform global point pt into robot body frame
    dx = pt[0] - state["x"]
    dy = pt[1] - state["y"]
    c = np.cos(-state["theta"])
    s = np.sin(-state["theta"])
    xb = c*dx - s*dy
    yb = s*dx + c*dy
    return xb, yb


def setup_plot(path) -> Tuple[Figure, Axes, Circle]:
    # ----- plotting setup -----
    # get x-y lims
    east_west = [p[0] for p in path]
    north_south = [p[1] for p in path]

    x_min, x_max = min(east_west), max(east_west)
    y_min, y_max = min(north_south), max(north_south)

    # is it square, taller, wider? we want some visual feedback of that
    x_span = x_max - x_min
    y_span = y_max - y_min
    aspect_ratio = x_span / y_span if y_span != 0 else 1.0

    plot_tuple = plt.subplots(figsize=(8, 8 / aspect_ratio))
    fig: Figure = plot_tuple[0]
    ax: Axes = plot_tuple[1]

    ax.set_xlim(x_min*1.1, x_max*1.1)
    ax.set_ylim(y_min*1.1, y_max*1.1)
    ax.set_aspect("equal")

    # initial pursuit circle
    circle = plt.Circle((0,0), Ld, fill=False, color="gray", linestyle="--")
    ax.add_patch(circle)

    return fig, ax, circle

def main(path_to_waypoints: str) -> int:
    # make and smooth path
    if path_to_waypoints:
        try:
            with open(path_to_waypoints, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]
                path = [tuple(map(float, line.split(',')[:2])) for line in lines]
        except Exception as e:
            print(f"Failed to load waypoints from {path_to_waypoints}: {e}")
            return 1
    else:
        path = [(0,0), (5,0), (5,5), (0,5), (0,0)]

    smooth_path_pts = smooth_path(path, num_points=len(path) * 3)
    path = smooth_path_pts

    # setup plotting space
    fig, ax, circle = setup_plot(path)

    # make figure frame
    robot_marker, = ax.plot([], [], "ro", markersize=8)
    path_line,    = ax.plot([], [], "k--", lw=1)
    pursuit_line, = ax.plot([], [], "b-", lw=1)  # pure pursuit trace
    goal_marker,  = ax.plot([], [], "gx", markersize=8)  # look‑ahead point

    def goal_reached() -> bool:
        goal_idx = len(path) - 1
        goal_point = np.array(path[goal_idx])
        robot_pos = np.array([state["x"], state["y"]])
        progress_ratio = last_idx / goal_idx

        if progress_ratio > 0.8:
            dist_to_goal = np.linalg.norm(robot_pos - goal_point)
            return dist_to_goal < 0.2
        return False

    # step through the simulation
    def update() -> None:
        global int_ev, int_ew, last_idx, last_timestamp

        # 1) pure‑pursuit - find look‑ahead
        goal_pt, last_idx = find_lookahead_point(path, state, Ld, last_idx)

        # update the look ahead circle
        circle.center = (state["x"], state["y"])

        # record pursuit trace for drawing intersection marker
        pursuit_xs.append(goal_pt[0])
        pursuit_ys.append(goal_pt[1])

        # compute curvature & desired yaw rate
        xg, yg  = to_body_frame(goal_pt, state)
        kappa   = 2*yg / (Ld**2)  # pure pursuit method
        vd      = desired_speed
        wd      = vd * kappa

        # compute errors & PI -> a_cmd, alpha_cmd
        ev = vd - state["v"]
        ew = wd - state["yaw"]
        int_ev  += ev * dt
        int_ew  += ew * dt

        a_cmd   = Kpv*ev + Kiv*int_ev
        alpha_cmd   = Kpw*ew + Kiw*int_ew

        # dynamics inversion -> wheel torques M_L, M_R
        denom_a = m - 2*Iw/(r**2)
        denom_w = Ichassis - 2*Iw*(L**2)/(r**2)

        Msum   = r * denom_a * a_cmd
        Mdif   = r * denom_w * (alpha_cmd / L)

        M_L = (Msum - Mdif) / 2
        M_R = (Msum + Mdif) / 2

        # low‑level dynamics - chassis accelerations
        a    = ((M_L + M_R)/r) / denom_a
        dotw = (((M_R - M_L)/r)*L) / denom_w

        # 5) integrate state (euler in body frame)
        state["v"]     += a      * dt
        state["yaw"]   += dotw   * dt
        state["x"]     += state["v"] * np.cos(state["theta"]) * dt
        state["y"]     += state["v"] * np.sin(state["theta"]) * dt
        state["theta"] += state["yaw"]               * dt

        # calculate any centripetal acceleration
        a_lat = state["v"] * state["yaw"]  # centripetal acceleration

        # write info into log target
        with open(target_log_path, 'a') as log:
            # format the row as:
            # timestamp, gyro_x, gyro_y, gyro_z, accel_x, ...
            row_formatted_str: str = f"{last_timestamp},0,0,{state["yaw"]},{a},{a_lat},0\n"
            log.write(row_formatted_str)
            last_timestamp += dt

        with open(ground_truth_target, 'a') as log:
            # move to ENU and log out 
            vx = state["v"] * np.cos(state["theta"])
            vy = state["v"] * np.sin(state["theta"])
            vz = 0.0

            # assume flat-ground motion — no roll or pitch
            phi, theta = 0.0, 0.0
            psi = state["theta"]  # yaw

            # biases not known in truth
            b_ax = b_ay = b_az = 0.0
            b_gx = b_gy = b_gz = 0.0

            # formatted as: timestamp, x, y, z, roll, pitch, yaw, etc... see above globals
            row_truth_str = (
                f"{last_timestamp}, "
                f"{state['x']}, {state['y']}, 0, "
                f"{vx}, {vy}, {vz}, "
                f"{phi}, {theta}, {psi}, "
                f"{b_ax}, {b_ay}, {b_az}, "
                f"{b_gx}, {b_gy}, {b_gz}\n"
            )
            
            log.write(row_truth_str)

        # update plot objects -- deprecated? was only used for animation
        robot_marker.set_data([state["x"]], [state["y"]])
        path_line.set_data(*zip(*path))
        pursuit_line.set_data(pursuit_xs, pursuit_ys)
        goal_marker.set_data([goal_pt[0]], [goal_pt[1]])
    
    while not goal_reached():
        update()

if __name__ == "__main__":
    path_to_data = "waypoints_20250618_210836.csv"
    return_code: int = main(path_to_data)
    print(f"Program exited with code: {return_code}")