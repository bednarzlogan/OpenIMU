# to browse for log file
import os
from tkinter import filedialog
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import cholesky
from utils.rotation_matrices import Qbe_inv_np, R_ENU_BODY_np


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
    
class UKF:
    def __init__(self, n, f_cont, Q, alpha=1e-3, beta=2, kappa=0):
        self.n = n  # state dimension
        self.f_cont = f_cont  # continuous-time dynamics function
        self.Q = Q  # process noise covariance matrix

        # UKF scaling parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.gamma = np.sqrt(n + self.lambda_)

        # sigma point weights
        self.Wm = np.full(2 * n + 1, 0.5 / (n + self.lambda_))
        self.Wc = np.copy(self.Wm)
        self.Wm[0] = self.lambda_ / (n + self.lambda_)
        self.Wc[0] = self.Wm[0] + (1 - alpha**2 + beta)

    def generate_sigma_points(self, mu, P):
        S = cholesky((self.n + self.lambda_) * P)

        # set the minimum number of sigma points
        sigma_points = np.zeros((2 * self.n + 1, self.n)) 
        sigma_points[0] = mu  # mean

        # place sigma points from optimal sample spread
        for i in range(self.n):
            sigma_points[i + 1] = mu + S[i]
            sigma_points[self.n + i + 1] = mu - S[i]
        return sigma_points

    def predict(self, mu, P, u, dt, params, rk4_step):
        # use the nomlinear dynamics to propagate the sigma points into predicted
        # states
        sigma_points = self.generate_sigma_points(mu, P)
        propagated_sigma = np.zeros_like(sigma_points)

        # propagate each sigma point through the nonlinear dynamics using rk4
        for i, sp in enumerate(sigma_points):
            propagated_sigma[i] = rk4_step(self.f_cont, sp, u, dt, params)

        # compute predicted mean and covariance
        mu_pred = np.zeros(self.n)
        for i in range(2 * self.n + 1):
            mu_pred += self.Wm[i] * propagated_sigma[i]

        P_pred = self.Q.copy() # covariance update is of the form var(x) + Q
        for i in range(2 * self.n + 1):
            # "measure" the covariance based on deviations in the sigma points from the mean
            dx = propagated_sigma[i] - mu_pred
            P_pred += self.Wc[i] * np.outer(dx, dx)

        return mu_pred, P_pred, propagated_sigma

    def update(self, mu_pred, P_pred, propagated_sigma, z, h_func, R):
        # apply measurement model to predicted sigma points
        z_sigma = np.array([h_func(sp) for sp in propagated_sigma])

        # predict measurement mean
        z_pred = np.zeros(z_sigma.shape[1])
        for i in range(2 * self.n + 1):
            z_pred += self.Wm[i] * z_sigma[i]

        # compute innovation covariance and cross-covariance
        S = R.copy()
        P_xz = np.zeros((self.n, z.shape[0]))
        for i in range(2 * self.n + 1):
            dz = z_sigma[i] - z_pred
            dx = propagated_sigma[i] - mu_pred
            S += self.Wc[i] * np.outer(dz, dz)
            P_xz += self.Wc[i] * np.outer(dx, dz)

        # Kalman gain and update
        K = P_xz @ np.linalg.inv(S)
        mu = mu_pred + K @ (z - z_pred)
        P = P_pred - K @ S @ K.T

        return mu, P

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

def main(csv_path: str, ground_truth_path: str, params: Params) -> List[np.ndarray]:
    """
    Here, we implement a UKF model for the IMU measurements, which will be ported to 
    C++
    """
    # basic model ignores earth's rotation for short missions. This creates:
    # r_dot = v_n
    # v_dot = R_ENU_BODY * f_b + g_n
    # E_dot = Q_be_inv * omega_ib

    # book-keeping! 
    state_history: List[np.ndarray] = []
    prev_time = None

    # basic config params for now
    Q = np.eye(15) * 1e-3  # process noise

    # dummy sensor noise and basic model
    pos_std = 1.0
    vel_std = 0.1
    R = np.diag([pos_std**2] * 3 + [vel_std**2] * 3)
    def h_func(x):
        return x[0:6]  # extract [position (3), velocity (3)]


    # define the UKF and initial conditions
    ukf = UKF(n=15, f_cont=f_cont, Q=Q)

    # Initialize state and covariance
    mu = np.zeros(15)
    P = np.eye(15) * 0.1

    ground_truth_data: List[str] = []
    with open(ground_truth_path, 'r') as t_log:
        # read in data except header
        ground_truth_data: List[str] =  t_log.readlines()[1:]  # may get out of hand for large logs

    # open the parsing look using a generator
    try:
        line_number: int = 0
        with open(csv_path, 'r') as f:
            next(f)  # skip the header
            for line in f:
                # pull apart the measurement
                parts = line.strip().split(",")
                t     = float(parts[0])
                gyro  = np.array([float(p) for p in parts[1:4]]) # should already be rad/s
                accel = np.array([float(p) for p in parts[4:7]]) # should already be in m/s^2

                # build u = [f_b, omega] to match f_cont’s signature
                u: np.ndarray  = np.hstack([gyro, accel])

                if prev_time is None:
                    dt = params.nominal_dt  # your sample_rate
                else:
                    dt = t - prev_time
                prev_time = t

                # get truth data
                truth_data: List[str] = ground_truth_data[line_number].strip().split(",")
                z_true = np.array([float(datum) for datum in truth_data[1:7]]) # position and velocity states
                z = z_true + np.random.multivariate_normal(np.zeros(6), R)

                # make the continuous time state matrix using the known points
                # and propagate state using rk4 method
                mu, P, sigma_pts = ukf.predict(mu, P, u=u, dt=dt, params=params, rk4_step=rk4_step)
                mu, P = ukf.update(mu, P, sigma_pts, z=z, h_func=h_func, R=R)

                # add to state history for analysis
                state_history.append(mu.copy())
                line_number += 1
       
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

    # select the IMU measurement log
    target_log = filedialog.askopenfilename(title="Select IMU Measurement Log")

    # try to infer matching ground truth log
    log_dir, log_file = os.path.split(target_log)
    base_name = os.path.splitext(log_file)[0]
    suffix = "_".join(base_name.split("_")[-6:])  # pull timestamp suffix
    candidate_truth = os.path.join(log_dir, f"ground_truth_{suffix}.csv")

    if not os.path.exists(candidate_truth):
        print(f"Could not find matching ground truth file for: {target_log}")
        candidate_truth = filedialog.askopenfilename(title="Select Ground Truth Log")

    # run the filter
    history = main(target_log, candidate_truth, params)

    # extract filtered XY
    xs_est = history[:, 0]
    ys_est = history[:, 1]

    # load ground truth and extract XY
    ground_truth = np.loadtxt(candidate_truth, delimiter=",", skiprows=1)
    xs_gt = ground_truth[:, 1]  # x
    ys_gt = ground_truth[:, 2]  # y

    # make RMSE info
    # Extract position components
    x_est_pos = history[:, 0:3]
    x_gt_pos = ground_truth[:, 1:4]  # x, y, z

    # Compute RMSE per component (vectorized)
    rmse = np.sqrt((x_est_pos - x_gt_pos) ** 2)

    # Time vector for x-axis (use timestamps from ground truth)
    timestamps = ground_truth[:, 0]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [3, 1]})
    fig.suptitle('UKF Trajectory and RMSE')

    # --- top: Trajectory plot ---
    ax1.plot(xs_est, ys_est, '-', linewidth=2, label='UKF estimate')
    ax1.plot(xs_gt, ys_gt, '--', linewidth=2, label='Ground truth')

    ax1.scatter(xs_est[0], ys_est[0], color='green', s=50, label='Start (UKF)')
    ax1.scatter(xs_est[-1], ys_est[-1], color='red', s=50, label='End (UKF)')
    ax1.scatter(xs_gt[0], ys_gt[0], color='lime', marker='x', s=50, label='Start (Truth)')
    ax1.scatter(xs_gt[-1], ys_gt[-1], color='maroon', marker='x', s=50, label='End (Truth)')

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Trajectory (XY Plane)')
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend()

    # --- bottom: RMSE plot ---
    ax2.plot(timestamps, rmse[:, 0], label='X RMSE [m]')
    ax2.plot(timestamps, rmse[:, 1], label='Y RMSE [m]')
    ax2.plot(timestamps, rmse[:, 2], label='Z RMSE [m]')

    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('RMSE [m]')
    ax2.set_title('Position RMSE Over Time')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

    print("Program execution finished!")