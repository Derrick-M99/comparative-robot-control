import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

l1, l2 = 1.0, 1.0
m1, m2 = 1.0, 1.0
g = 9.81

def inertia_matrix(q):
    theta1, theta2 = q
    I11 = m1 * (l1**2) / 3 + m2 * (l1**2 + (l2**2) / 3 + l1 * l2 * np.cos(theta2))
    I12 = m2 * ((l2**2) / 3 + 0.5 * l1 * l2 * np.cos(theta2))
    I21 = I12
    I22 = m2 * (l2**2) / 3
    return np.array([[I11, I12], [I21, I22]])

def forward_kinematics(q):
    theta1, theta2 = q
    joint = np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    end = joint + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])
    return joint, end

def simulate_open_loop(initial_state, apply_external_force=False):
    def dynamics(t, state):
        q = state[:2]
        dq = state[2:]

        if apply_external_force:
            if 4 <= t <= 6:
                tau = np.array([10.0, -5.0])  
            else:
                tau = np.array([2.0, -1.0])   
        else:
            tau = np.array([0.0, 0.0])

        M = inertia_matrix(q)
        ddq = np.linalg.inv(M) @ tau
        return np.concatenate((dq, ddq))

    t_span = (0, 10)
    t_eval = np.linspace(t_span[0], t_span[1], 500)
    sol = solve_ivp(dynamics, t_span, initial_state, t_eval=t_eval)
    return sol.t, sol.y

def print_metrics(t, y, label):
    theta1, theta2 = y[0], y[1]
    dtheta1, dtheta2 = y[2], y[3]
    
    drift1 = theta1[-1] - theta1[0]
    drift2 = theta2[-1] - theta2[0]
    
    max_vel1 = np.max(np.abs(dtheta1))
    max_vel2 = np.max(np.abs(dtheta2))
    
    KE = 0.5 * (dtheta1**2 + dtheta2**2)
    total_energy = np.trapz(KE, t)
    max_energy = np.max(KE)

    start_pos = forward_kinematics([theta1[0], theta2[0]])[1]
    end_pos = forward_kinematics([theta1[-1], theta2[-1]])[1]
    ee_drift = np.linalg.norm(end_pos - start_pos)

    print(f"\n--- {label} ---")
    print(f"Joint 1 Drift: {drift1:.4f} rad, Joint 2 Drift: {drift2:.4f} rad")
    print(f"Max Joint Velocities: dθ1 = {max_vel1:.4f}, dθ2 = {max_vel2:.4f}")
    print(f"Total Kinetic Energy: {total_energy:.4f}, Max Energy: {max_energy:.4f}")
    print(f"End-Effector Drift: {ee_drift:.4f} m")

initial_disturbed = [np.pi/6, np.pi/4, 0.5, -0.3]
initial_rest = [0.0, 0.0, 0.0, 0.0]

t1, y1 = simulate_open_loop(initial_disturbed, apply_external_force=False)
t2, y2 = simulate_open_loop(initial_rest, apply_external_force=True)

print_metrics(t1, y1, "Initial Disturbance")
print_metrics(t2, y2, "External Force Disturbance")

fig, axs = plt.subplots(2, 2, figsize=(14, 8))

axs[0, 0].plot(t1, y1[0], label='θ1 (Joint 1)')
axs[0, 0].plot(t1, y1[1], label='θ2 (Joint 2)')
axs[0, 0].set_title('Joint Angles - Initial Disturbance')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Angle (rad)')
axs[0, 0].grid(True)
axs[0, 0].legend()

axs[0, 1].plot(t1, y1[2], label='dθ1/dt (Joint 1)')
axs[0, 1].plot(t1, y1[3], label='dθ2/dt (Joint 2)')
axs[0, 1].set_title('Joint Velocities - Initial Disturbance')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Velocity (rad/s)')
axs[0, 1].grid(True)
axs[0, 1].legend()

axs[1, 0].plot(t2, y2[0], label='θ1 (Joint 1)')
axs[1, 0].plot(t2, y2[1], label='θ2 (Joint 2)')
axs[1, 0].axvspan(4, 6, color='red', alpha=0.2, label='External Disturbance')
axs[1, 0].set_title('Joint Angles - External Force Disturbance')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Angle (rad)')
axs[1, 0].grid(True)
axs[1, 0].legend()

axs[1, 1].plot(t2, y2[2], label='dθ1/dt (Joint 1)')
axs[1, 1].plot(t2, y2[3], label='dθ2/dt (Joint 2)')
axs[1, 1].axvspan(4, 6, color='red', alpha=0.2)
axs[1, 1].set_title('Joint Velocities - External Force Disturbance')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Velocity (rad/s)')
axs[1, 1].grid(True)
axs[1, 1].legend()

plt.tight_layout()
plt.show()
