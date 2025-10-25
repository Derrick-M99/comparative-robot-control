import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

l1, l2 = 1.0, 1.0

def desired_trajectory(t):
    return np.array([1.0 + 0.1 * np.cos(t), 1.0 + 0.1 * np.sin(t)])

def forward_kinematics(q):
    theta1, theta2 = q
    joint = np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    end_effector = joint + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])
    return end_effector

def inverse_kinematics(x, y):
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    k1 = l1 + l2 * cos_theta2
    k2 = l2 * sin_theta2
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    return theta1, theta2

t_vals = np.linspace(0, 10, 500)
theta1_desired, theta2_desired = [], []
for t in t_vals:
    x, y = desired_trajectory(t)
    th1, th2 = inverse_kinematics(x, y)
    theta1_desired.append(th1)
    theta2_desired.append(th2)
theta1_desired = np.array(theta1_desired)
theta2_desired = np.array(theta2_desired)

def simulate_pid(Kp, Kd, Ki, label, apply_disturbance=False):
    def dynamics(t, state):
        theta = state[:2]
        dtheta = state[2:4]
        integ = state[4:6]

        idx = np.searchsorted(t_vals, t)
        th_d = np.array([theta1_desired[idx], theta2_desired[idx]])
        dth_d = np.gradient([theta1_desired, theta2_desired], axis=1)[:, idx]

        error = th_d - theta
        derror = dth_d - dtheta

        tau = Kp * error + Kd * derror + Ki * integ
        if apply_disturbance and 4 <= t <= 6:
            tau += np.array([2.0, 0.0])

        ddtheta = tau
        return np.concatenate((dtheta, ddtheta, error))

    x0 = [np.pi/4, np.pi/4, 0, 0, 0, 0]
    sol = solve_ivp(dynamics, [0, 10], x0, t_eval=t_vals)
    q = sol.y[:2].T
    time = sol.t
    x_actual = np.array([forward_kinematics(qi) for qi in q])
    x_desired = np.array([desired_trajectory(t) for t in time])
    error = np.linalg.norm(x_actual - x_desired, axis=1)

    disturbance_mask = (time >= 4) & (time <= 6)
    post_force = (time >= 6)
    threshold = 0.01
    settle_idx = np.where((error < threshold) & post_force)[0]
    settle_time = time[settle_idx[0]] - 6 if settle_idx.size > 0 else None
    max_err = np.max(error[disturbance_mask]) if apply_disturbance else np.max(error)
    mean_err = np.mean(error)
    final_err = np.linalg.norm(x_actual[-1] - x_desired[-1])

    tag = "Disturbance" if apply_disturbance else "Original"
    print(f"\n--- {label} PID Control ({tag}) ---")
    print(f"Final error: {final_err:.4f}")
    print(f"Mean error: {mean_err:.4f}")
    print(f"Max error: {max_err:.4f}")
    if apply_disturbance:
        print(f"Settle time after disturbance: {settle_time:.2f} s" if settle_time else "Did not settle.")

    return time, x_actual, x_desired, error


def plot_results(label, tag, time, x_actual, x_desired, error, disturbed):
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 3, 1)
    plt.plot(x_actual[:, 0], x_actual[:, 1], label="Actual")
    plt.plot(x_desired[:, 0], x_desired[:, 1], '--', label="Desired")
    plt.title(f"{label} - Path ({tag})")
    plt.xlabel("X (m)"); plt.ylabel("Y (m)"); plt.legend(); plt.grid(True)

    plt.subplot(1, 3, 2)
    plt.plot(time, x_actual[:, 0], label="X actual")
    plt.plot(time, x_desired[:, 0], '--', label="X desired")
    plt.plot(time, x_actual[:, 1], label="Y actual")
    plt.plot(time, x_desired[:, 1], '--', label="Y desired")
    if disturbed: plt.axvspan(4, 6, color='red', alpha=0.2)
    plt.title(f"{label} - X/Y Position ({tag})")
    plt.xlabel("Time (s)"); plt.ylabel("Position (m)")
    plt.legend(); plt.grid(True)

    plt.subplot(1, 3, 3)
    plt.plot(time, error, label="Tracking Error")
    if disturbed: plt.axvspan(4, 6, color='red', alpha=0.2)
    plt.title(f"{label} - Error ({tag})")
    plt.xlabel("Time (s)"); plt.ylabel("Error (m)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

pid_settings = {
    "Soft": (np.array([50, 50]), np.array([10, 10]), np.array([5, 5])),
    "Balanced": (np.array([100, 100]), np.array([20, 20]), np.array([10, 10])),
    "Aggressive": (np.array([200, 200]), np.array([10, 10]), np.array([20, 20]))
}

for label, (Kp, Kd, Ki) in pid_settings.items():
    for disturbed in [False, True]:
        tag = "With Disturbance" if disturbed else "Original"
        t, x_act, x_des, err = simulate_pid(Kp, Kd, Ki, label, apply_disturbance=disturbed)
        plot_results(f"{label} PID", tag, t, x_act, x_des, err, disturbed)

