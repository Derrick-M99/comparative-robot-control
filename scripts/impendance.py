import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


l1, l2 = 1.0, 1.0


def desired_trajectory(t):
    xd = np.array([1.0 + 0.1 * np.cos(t), 1.0 + 0.1 * np.sin(t)])
    dxd = np.array([-0.1 * np.sin(t), 0.1 * np.cos(t)])
    return xd, dxd

def forward_kinematics(q):
    theta1, theta2 = q
    joint = np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    end_eff = joint + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])
    return end_eff

def jacobian(q):
    theta1, theta2 = q
    j11 = -l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2)
    j12 = -l2 * np.sin(theta1 + theta2)
    j21 = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    j22 = l2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])


def simulate(K_e, D_e, label, apply_disturbance=False):
    def dynamics(t, state):
        q = state[:2]
        dq = state[2:]
        xd, dxd = desired_trajectory(t)
        x = forward_kinematics(q)
        dx = jacobian(q) @ dq
        e = xd - x
        de = dxd - dx
        F = K_e @ e + D_e @ de
        if apply_disturbance and 4 <= t <= 6:
            F += np.array([5.0, 0.0])  
        tau = jacobian(q).T @ F
        ddq = tau
        return np.concatenate((dq, ddq))

    x0 = [np.pi/4, np.pi/4, 0, 0]
    t_eval = np.linspace(0, 10, 500)
    sol = solve_ivp(dynamics, [0, 10], x0, t_eval=t_eval)
    time = sol.t
    q = sol.y[:2].T
    x_actual = np.array([forward_kinematics(qq) for qq in q])
    x_desired = np.array([desired_trajectory(t)[0] for t in time])
    error = np.linalg.norm(x_actual - x_desired, axis=1)

    
    disturbance_mask = (time >= 4) & (time <= 6)
    post_force = (time >= 6)
    threshold = 0.01
    settle_idx = np.where((error < threshold) & post_force)[0]
    settle_time = time[settle_idx[0]] - 6 if settle_idx.size > 0 else None
    max_err = np.max(error[disturbance_mask]) if apply_disturbance else np.max(error)
    mean_err = np.mean(error)
    final_err = np.linalg.norm(x_actual[-1] - x_desired[-1])

    # Print metrics
    tag = "Disturbance" if apply_disturbance else "Original"
    print(f"\n--- {label} Control ({tag}) ---")
    print(f"Final error: {final_err:.4f}")
    print(f"Mean error: {mean_err:.4f}")
    print(f"Max error: {max_err:.4f}")
    if apply_disturbance:
        print(f"Settle time after disturbance: {settle_time:.2f} s" if settle_time else "Did not settle.")

    return time, x_actual, x_desired, error

# Plotting helper
def plot_results(label, tag, time, x_actual, x_desired, error, disturbed):
    plt.figure(figsize=(12, 4))

    # Path
    plt.subplot(1, 3, 1)
    plt.plot(x_actual[:, 0], x_actual[:, 1], label="Actual")
    plt.plot(x_desired[:, 0], x_desired[:, 1], '--', label="Desired")
    plt.title(f"{label} - Path ({tag})")
    plt.xlabel("X (m)"); plt.ylabel("Y (m)"); plt.legend(); plt.grid(True)

    # Position
    plt.subplot(1, 3, 2)
    plt.plot(time, x_actual[:, 0], label="X actual")
    plt.plot(time, x_desired[:, 0], '--', label="X desired")
    plt.plot(time, x_actual[:, 1], label="Y actual")
    plt.plot(time, x_desired[:, 1], '--', label="Y desired")
    if disturbed: plt.axvspan(4, 6, color='red', alpha=0.2, label="Disturbance")
    plt.title(f"{label} - X/Y Position ({tag})")
    plt.xlabel("Time (s)"); plt.ylabel("Position (m)")
    plt.legend(); plt.grid(True)

    # Error
    plt.subplot(1, 3, 3)
    plt.plot(time, error, label="Tracking Error")
    if disturbed: plt.axvspan(4, 6, color='red', alpha=0.2)
    plt.title(f"{label} - Error ({tag})")
    plt.xlabel("Time (s)"); plt.ylabel("Error (m)")
    plt.grid(True)

    plt.tight_layout()
    plt.show()


settings = {
    "Soft": (np.diag([20, 20]), np.diag([5, 5])),
    "Balanced": (np.diag([100, 100]), np.diag([20, 20])),
    "Aggressive": (np.diag([300, 300]), np.diag([5, 5]))
}


for label, (K, D) in settings.items():
    for disturbed in [False, True]:
        tag = "With Disturbance" if disturbed else "Original"
        t, x_act, x_des, err = simulate(K, D, label, apply_disturbance=disturbed)
        plot_results(label, tag, t, x_act, x_des, err, disturbed)

