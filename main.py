import numpy as np
import matplotlib.pyplot as plt

# Adjusted Simulation parameters
time_step = 0.05  # Smaller time step for finer simulation
total_time = 25  # Extended simulation time
time_vector = np.arange(0, total_time + time_step, time_step)

# Initial robot state remains the same
position_x, position_y, orientation = 0, 0, 0

# Updated Target points with different locations
targets = np.array([[1, 3], [3, 1], [5, 3]])  # Updated target points
current_target = 0  # Using zero-based indexing

# Control parameters are kept the same for consistency
distance_gain = 0.8
angle_gain = 1.5

# Updated Obstacle parameters for a different scenario
obstacle_position = np.array([2.5, 2])  # Changed obstacle location
obstacle_effective_radius = 0.75  # Increased obstacle effective radius

# Allocate memory for trajectory with the same method
trajectory_x, trajectory_y = np.zeros(len(time_vector)), np.zeros(len(time_vector))
trajectory_index = 0

for time in time_vector:
    if current_target < len(targets[0]):
        target = targets[:, current_target]

        # Compute errors to the target
        error_distance = np.sqrt((target[0] - position_x) ** 2 + (target[1] - position_y) ** 2)
        error_angle = np.arctan2(target[1] - position_y, target[0] - position_x) - orientation

        # Compute distance to the obstacle
        obstacle_distance = np.sqrt((obstacle_position[0] - position_x) ** 2 + (obstacle_position[1] - position_y) ** 2)

        # Obstacle avoidance logic remains the same
        if obstacle_distance < obstacle_effective_radius:
            error_angle += np.pi / 2
            velocity = distance_gain * error_distance * 0.3
        else:
            velocity = distance_gain * error_distance

        angular_velocity = angle_gain * error_angle

        # Update the state with the same logic
        position_x = position_x + velocity * np.cos(orientation) * time_step
        position_y = position_y + velocity * np.sin(orientation) * time_step
        orientation = orientation + angular_velocity * time_step

        # Add the current position to the trajectory
        trajectory_x[trajectory_index] = position_x
        trajectory_y[trajectory_index] = position_y
        trajectory_index += 1

        # Check if the target is reached
        if error_distance < 0.15:
            current_target += 1

# Trim trajectory arrays to the actual size used
trajectory_x, trajectory_y = trajectory_x[:trajectory_index], trajectory_y[:trajectory_index]

# Visualization of the trajectory and the obstacle with the same plotting logic
plt.plot(trajectory_x, trajectory_y, '-o', label='Path')
plt.plot(targets[0], targets[1], 'gx', markersize=10, linewidth=2, label='Waypoints')
plt.plot(obstacle_position[0], obstacle_position[1], 'kx', markersize=12, linewidth=2, label='Obstacle')

# Visualize the obstacle's influence area with updated parameters
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
