import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class Simulation3D:
    def __init__(self, sim_object, x_states, sim_info):
        self.sim_object = sim_object
        self.x_states = x_states  # x_states[0] for x, x_states[1] for y, x_states[2] for z
        self.sim_info = sim_info

        # Initialize figure and 3D axis using subplots with 'projection=3d'
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': '3d'})

        if self.sim_object == 'ball':
            # Initial position of the ball
            self.ball, = self.ax.plot([self.x_states[0][0]], [self.x_states[1][0]], [self.x_states[2][0]], 'o', color='red', markersize=10)
            
            # Draw ground (a plane at z = ground level)
            x_range = np.linspace(min(self.x_states[0]) - 2.0, max(self.x_states[0]) + 2.0, 100)
            y_range = np.linspace(min(self.x_states[1]) - 2.0, max(self.x_states[1]) + 2.0, 100)
            X, Y = np.meshgrid(x_range, y_range)
            Z = np.full_like(X, self.sim_info['ground'])
            self.ax.plot_surface(X, Y, Z, color='gray', alpha=0.5)
            
            # Set plot limits and labels
            self.ax.set_xlim(min(self.x_states[0]) - 2.0, max(self.x_states[0]) + 2.0)
            self.ax.set_ylim(min(self.x_states[1]) - 2.0, max(self.x_states[1]) + 2.0)
            self.ax.set_zlim(min(self.x_states[2]) - 2.0, max(self.x_states[2]) + 2.0)
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')

        # Animate the ball
        self.ani = FuncAnimation(self.fig, self.update, frames=len(x_states[0]), interval=50, blit=True)
        
    def update(self, frame):
        if self.sim_object == 'ball':
            # Update ball's position
            self.ball.set_data([self.x_states[0][frame]], [self.x_states[1][frame]])
            self.ball.set_3d_properties([self.x_states[2][frame]])  # Set z position

        return self.ball,

# Example usage
x_states = [
    np.linspace(0, 10, 100),  # X positions
    np.linspace(0, 5, 100),   # Y positions
    np.sin(np.linspace(0, 2*np.pi, 100))  # Z positions (ball bounces)
]

sim_info = {'ground': 0}  # Ground level at z = 0

sim = Simulation3D('ball', x_states, sim_info)
plt.show()
