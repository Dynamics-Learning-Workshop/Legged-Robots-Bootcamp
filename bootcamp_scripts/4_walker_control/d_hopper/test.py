import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
body_radius = 0.2  # radius of the body (circular)
leg_length = 1.0  # length of the leg
foot_position = np.array([0.3, 0])  # position of the foot (x, y)
ground_y = 0  # y-coordinate for the ground

# Time steps for the animation (simulating vertical bounce)
t = np.linspace(0, 2 * np.pi, 100)
# print(t.size)
# print(t)
# exit()
y_positions = leg_length + body_radius + 0.5 * np.sin(t)  # Varying y-positions (bouncing effect)

# Create figure and axis
fig, ax = plt.subplots()

# Plot the ground
ax.plot([-1, 1], [ground_y, ground_y], color='black', linewidth=2)

# Initialize body and leg objects
body_circle = plt.Circle((0, y_positions[0]), body_radius, color='red', fill=True)
foot_circle = plt.Circle((y_positions[0], 0), 0.05, color='green', fill=True)
leg_line, = ax.plot([0, foot_position[0]], [y_positions[0], foot_position[1]], color='blue', linewidth=3)

ax.add_patch(body_circle)
ax.add_patch(foot_circle)

# Set plot limits and aspect ratio
ax.set_xlim(-1, 3.5)
ax.set_ylim(-0.2, 3.5)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])

# Function to update the frame
def update(frame):
    # Update body position
    body_circle.center = (0, y_positions[frame])
    
    # Update leg position (line from body to foot)
    leg_line.set_data([0, foot_position[0]], [y_positions[frame], foot_position[1]])
    
    foot_circle.center = (y_positions[frame],0)

    return body_circle, leg_line, foot_circle

# Create animation
ani = FuncAnimation(fig, update, frames=len(t), interval=50, blit=True)

plt.title("Single-Leg Hopper Animation")
plt.show()
