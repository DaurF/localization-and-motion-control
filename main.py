import numpy as np
import matplotlib.pyplot as plt

time_step = 0.05
total_time = 25
time_vector = np.arange(0, total_time + time_step, time_step)

position_x, position_y, orientation = 0, 0, 0

targets = np.array([[1, 3], [3, 1], [5, 3]])
current_target = 0

distance_gain = 0.8
angle_gain = 1.5

obstacle_position = np.array([2.5, 2])
obstacle_effective_radius = 0.75

trajectory_x, trajectory_y = np.zeros(len(time_vector)), np.zeros(len(time_vector))
trajectory_index = 0

for time in time_vector:
    if current_target < len(targets[0]):
        target = targets[:, current_target]

        error_distance = np.sqrt((target[0] - position_x) ** 2 + (target[1] - position_y) ** 2)
        error_angle = np.arctan2(target[1] - position_y, target[0] - position_x) - orientation

        obstacle_distance = np.sqrt((obstacle_position[0] - position_x) ** 2 + (obstacle_position[1] - position_y) ** 2)

        if obstacle_distance < obstacle_effective_radius:
            error_angle += np.pi / 2
            velocity = distance_gain * error_distance * 0.3
        else:
            velocity = distance_gain * error_distance

        angular_velocity = angle_gain * error_angle

        position_x = position_x + velocity * np.cos(orientation) * time_step
        position_y = position_y + velocity * np.sin(orientation) * time_step
        orientation = orientation + angular_velocity * time_step

        trajectory_x[trajectory_index] = position_x
        trajectory_y[trajectory_index] = position_y
        trajectory_index += 1

        if error_distance < 0.15:
            current_target += 1

trajectory_x, trajectory_y = trajectory_x[:trajectory_index], trajectory_y[:trajectory_index]

plt.plot(trajectory_x, trajectory_y, '-o', label='Path')
plt.plot(targets[0], targets[1], 'gx', markersize=10, linewidth=2, label='Waypoints')
plt.plot(obstacle_position[0], obstacle_position[1], 'kx', markersize=12, linewidth=2, label='Obstacle')

angle = np.linspace(0, 2 * np.pi, 100)
obstacle_x = obstacle_effective_radius * np.cos(angle) + obstacle_position[0]
obstacle_y = obstacle_effective_radius * np.sin(angle) + obstacle_position[1]
plt.plot(obstacle_x, obstacle_y, 'k--', label='Obstacle Area')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Navigational Path with Dynamic Obstacle Avoidance')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
