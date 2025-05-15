from utils.rotation_matrices import R_ENU_BODY_np, Qbe_inv_np 
import numpy as np
from typing import List
import matplotlib.pyplot as plt

class Params:
    def __init__(self, 
                 g_n: np.ndarray, 
                 tau_a: float, 
                 tau_g: float, 
                 nominal_dt: float):
        self.g_n = g_n
        self.tau_a = tau_a
        self.tau_g = tau_g
        self.nominal_dt = nominal_dt

def f_cont(x: np.ndarray, u: np.ndarray, params: Params) -> np.ndarray:
    # unpack state
    _    = x[0:3]
    v    = x[3:6]
    E    = x[6:9]     # Euler angles (phi,theta,psi)
    b_a  = x[9:12]
    b_g  = x[12:15]

    # unpack input
    omega_ib = u[0:3]      # body‐frame angular rates
    f_b      = u[3:6]      # specific‐force measurements

    # rotation and mapping matrices, evaluated at E
    R = R_ENU_BODY_np(*E)      # 3×3
    Q = Qbe_inv_np(*E)        # 3×3

    # true dynamics
    r_dot    = v
    v_dot    = R @ (f_b - b_a) + params.g_n
    E_dot    = Q @ (omega_ib - b_g)
    b_a_dot  = -1/params.tau_a * b_a
    b_g_dot  = -1/params.tau_g * b_g

    return np.hstack([r_dot, v_dot, E_dot, b_a_dot, b_g_dot])

def rk4_step(f_cont, x, u, dt, params):
    # classical fourth‐order Runge–Kutta method for discrete model
    k1 = f_cont(x,              u, params)
    k2 = f_cont(x + 0.5*dt*k1,  u, params)
    k3 = f_cont(x + 0.5*dt*k2,  u, params)
    k4 = f_cont(x +     dt*k3,  u, params)
    return x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

def main(csv_path: str, params: Params) -> List[np.ndarray]:
    """
    Here, we implement a UKF model for the IMU measurements, which will be ported to 
    C++
    """
    # basic model ignores earth's rotation for short missions. This creates:
    # r_dot = v_n
    # v_dot = R_ENU_BODY * f_b + g_n
    # E_dot = Q_be_inv * omega_ib

    # x_0: [r(3), v(3), E(3), b_a(3), b_g(3)]
    x = np.zeros(15)
    state_history: List[np.ndarray] = []
    prev_time = None

    # open the parsing look using a generator
    try:
        with open(csv_path, 'r') as f:
            next(f)  # skip the header
            for line in f:
                # pull apart the measurement
                parts = line.strip().split(",")
                t     = float(parts[0]) * 1e-3
                gyro  = np.array([float(p) for p in parts[1:4]]) # should already be rad/s
                accel = np.array([float(p) for p in parts[4:7]]) # should already be in m/s^2

                # build u = [f_b, omega] to match f_cont’s signature
                u: np.ndarray  = np.hstack([gyro, accel])

                if prev_time is None:
                    dt = params.nominal_dt  # your sample_rate
                else:
                    dt = t - prev_time
                prev_time = t
                
                # make the continuous time state matrix using the known points
                # and propagate state using rk4 method
                x = rk4_step(f_cont=f_cont, x=x, u=u, dt=dt, params=params)

                # add to state history for analysis
                state_history.append(x.copy())          
    except Exception as e:
        print(f"Got an exception parsing log: {e}")

    return np.array(state_history)

if __name__ == "__main__":
    # define system config
    params: Params = Params(
        g_n=np.array([0.0, 0.0, -9.81]),
        tau_a=3600,
        tau_g=3600,
        nominal_dt=0.05
    )

    history = main("t0.csv", params)

    # extract X and Y only
    xs = history[:, 0]
    ys = history[:, 1]

    # plot trajectory in the XY plane
    plt.figure()
    plt.plot(xs, ys, '-', linewidth=2, label='trajectory')
    plt.scatter(xs[0], ys[0], color='green', s=50, label='start')
    plt.scatter(xs[-1], ys[-1], color='red',   s=50, label='end')

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('2D IMU Model Trajectory (XY Plane)')
    plt.legend()
    plt.axis('equal')   # keep aspect ratio so distances aren’t distorted
    plt.grid(True)
    plt.show()