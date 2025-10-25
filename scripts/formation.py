import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Constants
dt = 0.1
T = 30
N = int(T / dt)
k1, k2 = 2.0, 3.0
convergence_threshold = 0.5

# Trajectory (circular path) 
def desired_trajectory(t):
    x_d = 5 * np.cos(0.1 * t)
    y_d = 5 * np.sin(0.1 * t)
    return x_d, y_d

# Formations (spaced to avoid overlap)
line_formation = [
    np.array([-2.0, 0.0]),
    np.array([0.0, 0.0]),
    np.array([2.0, 0.0])
]

triangle_formation = [
    np.array([0.0, 2.0]),
    np.array([1.5, -1.5]),
    np.array([-1.5, -1.5])
]

# Simulation Function
def run_simulation(add_noise=False):
    robots = np.array([
        [0.0, 0.0, np.pi / 4],
        [2.0, 2.0, -np.pi / 4],
        [-2.0, 2.0, np.pi / 2]
    ])

    follower_activation_time = 3
    formation_switch_times = [10, 18, 24]

    history = [[] for _ in range(3)]
    error_log = []
    error_r2, error_r3 = [], []
    offset_error_log = []
    distance_log = []
    lyapunov_log = []

    convergence_time = None
    convergence_time_r2 = None
    convergence_time_r3 = None
    overshoot = 0

    for step in range(N):
        t = step * dt

        if len([s for s in formation_switch_times if t >= s]) % 2 == 0:
            formation = line_formation
        else:
            formation = triangle_formation

        x_d, y_d = desired_trajectory(t)

        # Leader control
        ex, ey = x_d - robots[0, 0], y_d - robots[0, 1]
        dist = np.hypot(ex, ey)
        angle_to_target = np.arctan2(ey, ex)
        v = k1 * dist
        omega = k2 * ((angle_to_target - robots[0, 2] + np.pi) % (2 * np.pi) - np.pi)
        robots[0, 0] += v * np.cos(robots[0, 2]) * dt
        robots[0, 1] += v * np.sin(robots[0, 2]) * dt
        robots[0, 2] += omega * dt

        if t >= follower_activation_time:
            for i in range(1, 3):
                offset = formation[i] - formation[0]
                dx = robots[0, 0] + offset[0]
                dy = robots[0, 1] + offset[1]
                ex = dx - robots[i, 0]
                ey = dy - robots[i, 1]
                dist = np.hypot(ex, ey)

                if i == 1:
                    error_r2.append(dist)
                    if convergence_time_r2 is None and dist < convergence_threshold:
                        convergence_time_r2 = t
                if i == 2:
                    error_r3.append(dist)
                    if convergence_time_r3 is None and dist < convergence_threshold:
                        convergence_time_r3 = t

                error_log.append(dist)
                overshoot = max(overshoot, dist)
                lyapunov_log.append(0.5 * (ex**2 + ey**2))

                angle_to_target = np.arctan2(ey, ex)
                v = k1 * dist
                omega = k2 * ((angle_to_target - robots[i, 2] + np.pi) % (2 * np.pi) - np.pi)
                robots[i, 0] += v * np.cos(robots[i, 2]) * dt
                robots[i, 1] += v * np.sin(robots[i, 2]) * dt
                robots[i, 2] += omega * dt

                if add_noise:
                    robots[i, 0] += np.random.normal(0, 0.05)
                    robots[i, 1] += np.random.normal(0, 0.05)
                    robots[i, 2] += np.random.normal(0, 0.01)

                actual_offset = robots[i, :2] - robots[0, :2]
                desired_offset = offset
                offset_error = np.linalg.norm(actual_offset - desired_offset)
                offset_error_log.append(offset_error)

        if convergence_time is None and convergence_time_r2 and convergence_time_r3:
            convergence_time = max(convergence_time_r2, convergence_time_r3)

        d12 = np.linalg.norm(robots[0, :2] - robots[1, :2])
        d13 = np.linalg.norm(robots[0, :2] - robots[2, :2])
        d23 = np.linalg.norm(robots[1, :2] - robots[2, :2])
        distance_log.append((d12, d13, d23))

        for i in range(3):
            history[i].append(robots[i, :2].copy())

    return {
        "error_log": np.array(error_log),
        "error_r2": np.array(error_r2),
        "error_r3": np.array(error_r3),
        "offset_error_log": np.array(offset_error_log),
        "distance_log": np.array(distance_log),
        "lyapunov_log": np.array(lyapunov_log),
        "avg_error": np.mean(error_log),
        "max_error": np.max(error_log),
        "overshoot": overshoot,
        "convergence_time": convergence_time,
        "convergence_time_r2": convergence_time_r2,
        "convergence_time_r3": convergence_time_r3,
        "final_formation": "Triangle" if len(formation_switch_times) % 2 == 1 else "Line",
        "stability_check": np.std(distance_log[-50:], axis=0).max(),
        "history": [np.array(h) for h in history]
    }

# Constants
dt = 0.1
T = 30
N = int(T / dt)
k1, k2 = 2.0, 3.0
convergence_threshold = 0.5

# Trajectory (circular path) 
def desired_trajectory(t):
    x_d = 5 * np.cos(0.1 * t)
    y_d = 5 * np.sin(0.1 * t)
    return x_d, y_d

# Formations (adjusted for better spacing)
line_formation = [
    np.array([-2.0, 0.0]),
    np.array([0.0, 0.0]),
    np.array([2.0, 0.0])
]

triangle_formation = [
    np.array([0.0, 2.0]),
    np.array([1.5, -1.5]),
    np.array([-1.5, -1.5])
]

# Simulation Function 
def run_simulation(add_noise=False):
    robots = np.array([
        [0.0, 0.0, np.pi / 4],
        [2.0, 2.0, -np.pi / 4],
        [-2.0, 2.0, np.pi / 2]
    ])

    follower_activation_time = 3
    formation_switch_times = [10, 18, 24]

    history = [[] for _ in range(3)]
    error_log = []
    error_r2, error_r3 = [], []
    offset_error_log = []
    distance_log = []
    lyapunov_log = []

    convergence_time = None
    convergence_time_r2 = None
    convergence_time_r3 = None
    overshoot = 0

    for step in range(N):
        t = step * dt

        if len([s for s in formation_switch_times if t >= s]) % 2 == 0:
            formation = line_formation
        else:
            formation = triangle_formation

        x_d, y_d = desired_trajectory(t)

        # Leader control
        ex, ey = x_d - robots[0, 0], y_d - robots[0, 1]
        dist = np.hypot(ex, ey)
        angle_to_target = np.arctan2(ey, ex)
        v = k1 * dist
        omega = k2 * ((angle_to_target - robots[0, 2] + np.pi) % (2 * np.pi) - np.pi)
        robots[0, 0] += v * np.cos(robots[0, 2]) * dt
        robots[0, 1] += v * np.sin(robots[0, 2]) * dt
        robots[0, 2] += omega * dt

        if t >= follower_activation_time:
            for i in range(1, 3):
                offset = formation[i] - formation[0]
                dx = robots[0, 0] + offset[0]
                dy = robots[0, 1] + offset[1]
                ex = dx - robots[i, 0]
                ey = dy - robots[i, 1]
                dist = np.hypot(ex, ey)

                if i == 1:
                    error_r2.append(dist)
                    if convergence_time_r2 is None and dist < convergence_threshold:
                        convergence_time_r2 = t
                if i == 2:
                    error_r3.append(dist)
                    if convergence_time_r3 is None and dist < convergence_threshold:
                        convergence_time_r3 = t

                error_log.append(dist)
                overshoot = max(overshoot, dist)
                lyapunov_log.append(0.5 * (ex**2 + ey**2))

                angle_to_target = np.arctan2(ey, ex)
                v = k1 * dist
                omega = k2 * ((angle_to_target - robots[i, 2] + np.pi) % (2 * np.pi) - np.pi)
                robots[i, 0] += v * np.cos(robots[i, 2]) * dt
                robots[i, 1] += v * np.sin(robots[i, 2]) * dt
                robots[i, 2] += omega * dt

                if add_noise:
                    robots[i, 0] += np.random.normal(0, 0.05)
                    robots[i, 1] += np.random.normal(0, 0.05)
                    robots[i, 2] += np.random.normal(0, 0.01)

                actual_offset = robots[i, :2] - robots[0, :2]
                desired_offset = offset
                offset_error = np.linalg.norm(actual_offset - desired_offset)
                offset_error_log.append(offset_error)

        if convergence_time is None and convergence_time_r2 and convergence_time_r3:
            convergence_time = max(convergence_time_r2, convergence_time_r3)

        d12 = np.linalg.norm(robots[0, :2] - robots[1, :2])
        d13 = np.linalg.norm(robots[0, :2] - robots[2, :2])
        d23 = np.linalg.norm(robots[1, :2] - robots[2, :2])
        distance_log.append((d12, d13, d23))

        for i in range(3):
            history[i].append(robots[i, :2].copy())

    return {
        "error_log": np.array(error_log),
        "error_r2": np.array(error_r2),
        "error_r3": np.array(error_r3),
        "offset_error_log": np.array(offset_error_log),
        "distance_log": np.array(distance_log),
        "lyapunov_log": np.array(lyapunov_log),
        "avg_error": np.mean(error_log),
        "max_error": np.max(error_log),
        "overshoot": overshoot,
        "convergence_time": convergence_time,
        "convergence_time_r2": convergence_time_r2,
        "convergence_time_r3": convergence_time_r3,
        "final_formation": "Triangle" if len(formation_switch_times) % 2 == 1 else "Line",
        "stability_check": np.std(distance_log[-50:], axis=0).max(),
        "history": [np.array(h) for h in history]
    }

# Run and Analyze 
result_clean = run_simulation(add_noise=False)
result_noisy = run_simulation(add_noise=True)

def print_summary(name, result):
    print(f"\n=== {name} ===")
    print(f"Average tracking error: {result['avg_error']:.3f}")
    print(f"Maximum tracking error: {result['max_error']:.3f}")
    print(f"Overshoot: {result['overshoot']:.3f}")
    if result['convergence_time']:
        print(f"System converged in {result['convergence_time']:.2f} seconds")
    else:
        print("System did not fully converge")
    print(f"Final formation: {result['final_formation']}")
    print(f"Stability check: {result['stability_check']:.4f}")

print_summary("Clean Simulation", result_clean)
print_summary("Noisy Simulation", result_noisy)

# Plotting
def plot_metrics(result, label, switch_times=[10, 18, 24]):
    time = np.arange(len(result["distance_log"])) * dt
    lyap_time = np.arange(len(result["lyapunov_log"])) * dt

    # Tracking Error
    plt.figure(figsize=(10, 4))
    plt.plot(np.arange(len(result["error_log"])) * dt, result["error_log"], label="Tracking Error")
    for switch in switch_times:
        plt.axvline(x=switch, color='red', linestyle='--', label="Formation Change" if switch == switch_times[0] else "")
    plt.title(f"Tracking Error - {label}")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Inter-Robot Distances
    d12, d13, d23 = result["distance_log"].T
    plt.figure(figsize=(10, 4))
    plt.plot(time, d12, label="R1-R2")
    plt.plot(time, d13, label="R1-R3")
    plt.plot(time, d23, label="R2-R3")
    for switch in switch_times:
        plt.axvline(x=switch, color='red', linestyle='--')
    plt.title(f"Inter-Robot Distances - {label}")
    plt.xlabel("Time (s)")
    plt.ylabel("Distance")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Lyapunov Function
    plt.figure(figsize=(10, 4))
    plt.plot(lyap_time, result["lyapunov_log"], label="Lyapunov V(t)")
    for switch in switch_times:
        plt.axvline(x=switch, color='red', linestyle='--')
    plt.title(f"Lyapunov Function - {label}")
    plt.xlabel("Time (s)")
    plt.ylabel("V(t)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# Individual Errors
def plot_individual_errors(result, label):
    time_r2 = np.arange(len(result["error_r2"])) * dt
    time_r3 = np.arange(len(result["error_r3"])) * dt

    plt.figure(figsize=(10, 4))
    plt.plot(time_r2, result["error_r2"], label="R2")
    plt.plot(time_r3, result["error_r3"], label="R3")
    plt.axhline(y=convergence_threshold, color='gray', linestyle='--', label='Threshold')
    plt.title(f"Individual Tracking Errors - {label}")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def animate_robots(history, title):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    lines = [ax.plot([], [], 'o-')[0] for _ in range(3)]
    trails = [ax.plot([], [], '--')[0] for _ in range(3)]
    labels = [ax.text(0, 0, '', fontsize=8) for _ in range(3)]

    def init():
        for line in lines + trails:
            line.set_data([], [])
        for label in labels:
            label.set_text('')
        return lines + trails + labels

    def update(frame):
        for i in range(3):
            x, y = history[i][frame]
            lines[i].set_data([x], [y])
            trails[i].set_data(history[i][:frame+1, 0], history[i][:frame+1, 1])
            labels[i].set_position((x + 0.2, y + 0.2))
            labels[i].set_text(f"R{i+1}")
        return lines + trails + labels

    ani = animation.FuncAnimation(fig, update, frames=len(history[0]),
                                  init_func=init, blit=True, interval=100, repeat=False)
    plt.tight_layout()
    plt.show()

import matplotlib.animation as animation

def save_animation(history, filename="simulation.gif", title="Simulation"):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)

    lines = [ax.plot([], [], 'o-')[0] for _ in range(3)]
    trails = [ax.plot([], [], '--')[0] for _ in range(3)]
    labels = [ax.text(0, 0, '', fontsize=8) for _ in range(3)]

    def init():
        for line in lines + trails:
            line.set_data([], [])
        for label in labels:
            label.set_text('')
        return lines + trails + labels

    def update(frame):
        for i in range(3):
            x, y = history[i][frame]
            lines[i].set_data([x], [y])
            trails[i].set_data(history[i][:frame+1, 0], history[i][:frame+1, 1])
            labels[i].set_position((x + 0.2, y + 0.2))
            labels[i].set_text(f"R{i+1}")
        return lines + trails + labels

    ani = animation.FuncAnimation(fig, update, frames=len(history[0]),
                                  init_func=init, blit=True, interval=100, repeat=False)
    ani.save(filename, writer='pillow')

save_animation(result_clean["history"], "clean_simulation.gif", "Clean Simulation")
save_animation(result_noisy["history"], "noisy_simulation.gif", "Noisy Simulation")


plot_metrics(result_clean, "Clean")
plot_individual_errors(result_clean, "Clean")
animate_robots(result_clean["history"], "Clean Simulation")

plot_metrics(result_noisy, "Noisy")
plot_individual_errors(result_noisy, "Noisy")
animate_robots(result_noisy["history"], "Noisy Simulation")
