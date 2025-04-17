import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

# simple example path: list of (x,y) waypoints
path = [(0,0), (5,0), (5,5), (0,5), (0,0)]

# store look‑ahead points for trace
pursuit_xs, pursuit_ys = [], []

def find_lookahead_point(path, state, Ld, last_idx):
    x, y = state["x"], state["y"]
    P = np.array([x, y])
    for i in range(last_idx, len(path)-1):
        A = np.array(path[i])
        B = np.array(path[i+1])
        d = B - A
        f = A - P

        a = np.dot(d, d)
        b = 2 * np.dot(d, f)
        c = np.dot(f, f) - Ld**2

        disc = b*b - 4*a*c
        if disc < 0:
            continue
        sqrt_disc = np.sqrt(disc)
        for t in sorted([(-b + sqrt_disc)/(2*a), (-b - sqrt_disc)/(2*a)]):
            if 0 <= t <= 1:
                return A + t*d, i
    # if no intersection, return last point
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

def update(frame):
    global int_ev, int_ew, last_idx

    # 1) pure‑pursuit → find look‑ahead
    goal_pt, last_idx = find_lookahead_point(path, state, Ld, last_idx)

    # move the existing circle
    circle.center = (state["x"], state["y"])

    # record pursuit trace
    pursuit_xs.append(goal_pt[0])
    pursuit_ys.append(goal_pt[1])

    # compute curvature & desired yaw rate
    xg, yg  = to_body_frame(goal_pt, state)
    kappa   = 2*yg / (Ld**2)
    vd      = desired_speed
    wd      = vd * kappa

    # 2) compute errors & PI → a_cmd, α_cmd
    ev = vd - state["v"]
    ew = wd - state["yaw"]
    int_ev  += ev * dt
    int_ew  += ew * dt

    a_cmd   = Kpv*ev + Kiv*int_ev
    α_cmd   = Kpw*ew + Kiw*int_ew

    # 3) dynamics inversion → wheel torques M_L, M_R
    denom_a = m - 2*Iw/(r**2)
    denom_w = Ichassis - 2*Iw*(L**2)/(r**2)

    Msum   = r * denom_a * a_cmd
    Mdif   = r * denom_w * (α_cmd / L)

    M_L = (Msum - Mdif) / 2
    M_R = (Msum + Mdif) / 2

    # 4) low‑level dynamics → chassis accelerations
    a    = ((M_L + M_R)/r) / denom_a
    dotw = (((M_R - M_L)/r)*L) / denom_w

    # 5) integrate state (euler in body frame)
    state["v"]     += a      * dt
    state["yaw"]   += dotw   * dt
    state["x"]     += state["v"] * np.cos(state["theta"]) * dt
    state["y"]     += state["v"] * np.sin(state["theta"]) * dt
    state["theta"] += state["yaw"]               * dt

    # update plot objects
    robot_marker.set_data([state["x"]], [state["y"]])
    path_line.set_data(*zip(*path))
    pursuit_line.set_data(pursuit_xs, pursuit_ys)
    goal_marker.set_data([goal_pt[0]], [goal_pt[1]])
    return robot_marker, path_line, pursuit_line, goal_marker

# ----- plotting setup -----
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 6)
ax.set_aspect("equal")

# initial pursuit circle
circle = plt.Circle((0,0), Ld, fill=False, color="gray", linestyle="--")
ax.add_patch(circle)

# plot handles
robot_marker, = ax.plot([], [], "ro", markersize=8)
path_line,    = ax.plot([], [], "k--", lw=1)
pursuit_line, = ax.plot([], [], "b-", lw=1)  # pure pursuit trace
goal_marker,  = ax.plot([], [], "gx", markersize=8)  # look‑ahead point

ani = FuncAnimation(fig, update, frames=500, interval=50)
plt.show()

