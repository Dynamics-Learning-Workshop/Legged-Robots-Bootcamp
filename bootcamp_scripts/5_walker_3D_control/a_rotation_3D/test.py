import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def animate_line(start, end):
    # Create a figure and a 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set the limits for the plot
    ax.set_xlim([min(start[0], end[0]) - 1, max(start[0], end[0]) + 1])
    ax.set_ylim([min(start[1], end[1]) - 1, max(start[1], end[1]) + 1])
    ax.set_zlim([min(start[2], end[2]) - 1, max(start[2], end[2]) + 1])

    # Add labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create a line object
    line, = ax.plot([], [], [], color='b', linewidth=2)

    # Function to initialize the background of the animation
    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        return line,

    # Function to update the line for each frame
    def update(frame):
        # Calculate the current position along the line
        t = frame / num_frames  # Normalize the frame to [0, 1]
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        z = start[2] + t * (end[2] - start[2])

        # Update the line data
        line.set_data([start[0], x], [start[1], y])
        line.set_3d_properties([start[2], z])
        return line,

    # Number of frames for the animation
    num_frames = 100

    # Create the animation
    ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=50)

    # Show the animation
    plt.show()

# Example usage
start_point = [0, 0, 0]  # Starting point of the line
end_point = [3, 4, 5]    # Ending point of the line

animate_line(start_point, end_point)
