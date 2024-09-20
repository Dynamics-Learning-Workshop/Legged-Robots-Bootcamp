from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

class RobotUtils:
    def __init__(self):
        pass
    
    def rot2D(self, psi):
        return np.array(
            [[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]]
        )
        
    def homo2D(self, psi, trans):
        homo2D_mat = np.zeros((3, 3))
        homo2D_mat[:2, :2] = self.rot2D(psi)
        homo2D_mat[0:2, 2] = trans
        homo2D_mat[2, 0:3] = np.array([0,0,1])
        
        return homo2D_mat
    
    def rot3D(self, phi, theta, psi):
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])
    
        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
    
        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])
    
        return np.dot(R_z, np.dot(R_y, R_x))
    

class Integrator(RobotUtils):
    def __init__(self):
        super().__init__()
        self.ball = plt.Circle((0, 0), 0.2, color='red', fill=True)
        self.foot_circle = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.leg = None

        self.x_states = []
        
        self.sim_object = 'ball'
        self.sim_object = 'hopper'
        self.sim_object = 'walker'
    
    def rk4(self, func, x, h):
        k1 = func(x) * h
        k2 = func(x + k1 / 2) * h
        k3 = func(x + k2 / 2) * h  
        k4 = func(x + k3) * h
        
        return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    
    def euler_forward(self, func, x, h):
        return x + h * func(x)
        
    def anime(
        self, 
        t, 
        x_states, 
        ground=0, 
        ms = 10,
        mission='lala', 
        sim_object='ball', 
        save=False, 
        save_name='obj_sim'
        ):
        fig, ax = plt.subplots()
        
        if sim_object == 'ball':
            self.x_states = x_states
            
            self.ball = plt.Circle((x_states[0][0], x_states[1][0]), 0.2, color='red', fill=True)
            ax.add_patch(self.ball)
            
        elif sim_object == 'hopper':
            self.x_states = x_states
            
            self.ball = plt.Circle((x_states[0][0], x_states[1][0]), 0.1, color='red', fill=True)

            self.leg, = ax.plot(
                [x_states[0][0], x_states[2][0]], 
                [x_states[1][0], x_states[3][0]], 
                color='blue', 
                linewidth=3)
            # body_x x_states[0]
            # body_y x_states[1]
            # leg_x x_states[2]
            # leg_g x_states[3]
            
            self.foot_circle = plt.Circle(
                (x_states[2][0], x_states[3][0]), 
                0.05, color='green', fill=True
            )
            
            ax.add_patch(self.ball)
            ax.add_patch(self.foot_circle)
        elif sim_object == 'walker':
            initial_state = x_states
            # leg0_theta x_states[0]
            # leg1_theta x_states[1]
            # leg_x x_states[2]
            # leg_g x_states[3]
            pass
        else:
            print("GOT ERROR CHOOSING OBJECT")
            exit()
        
        ax.set_aspect('equal')
        
        ax.plot(
            [
                min(np.min(x_states[0]), np.min(x_states[2])), 
                max(np.max(x_states[0]), np.max(x_states[2]))
            ], [ground, ground], color='black', linewidth=2)
        self.sim_object = sim_object
        
        ani = FuncAnimation(fig, self.update, frames=len(t), interval=ms, blit=True)
        if save:
            ani.save('./viz/'+ save_name + '.mp4', writer='ffmpeg', fps=1000/ms)  # Match to real-time playback speed
            
        plt.title(mission)
        plt.show()
        
    
    def update(self, frame):
        if self.sim_object == 'ball':
            self.ball.center = (self.x_states[0][frame], self.x_states[1][frame])
            
            return (self.ball,)
        elif self.sim_object == 'hopper':
            self.ball.center = (self.x_states[0][frame], self.x_states[1][frame])
            self.leg.set_data(
                [self.x_states[0][frame], self.x_states[2][frame]], 
                [self.x_states[1][frame], self.x_states[3][frame]]
                )
            self.foot_circle.center = (self.x_states[2][frame], self.x_states[3][frame])

            return (self.ball, self.leg, self.foot_circle,)
        else:
            exit()
        
