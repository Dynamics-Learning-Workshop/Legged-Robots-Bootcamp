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
    
        return R_z @ R_y @ R_x
    
    def rad_2_pi_range(self, angle):
        while np.abs(angle) > np.pi:
            if angle > 0:
                angle = angle - 2*np.pi
            else:
                angle = angle + 2*np.pi
            
        return angle

class Integrator(RobotUtils):
    def __init__(self):
        super().__init__()
        self.ball = plt.Circle((0, 0), 0.2, color='red', fill=True)
        self.foot_circle = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.leg = None
        
        self.head = plt.Circle((0, 0), 0.2, color='black', fill=True)
        self.leg1 = None
        self.leg2 = None
        self.hand1 = None
        self.hand2 = None
        self.neck = None

        self.foot1 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        self.foot2 = plt.Circle((0, 0), 0.05, color='green', fill=True)
        
        self.x_states = []
        
        self.sim_object = 'ball'
        self.sim_object = 'hopper'
        self.sim_object = 'walker'
        self.sim_info = {'ground':0, 'slope_angle':0, 'leg_l':0}
        
        self.fig = None
        self.ax = None
    
    def rk4(self, func, x, h):
        k1 = func(x) * h
        k2 = func(x + k1 / 2) * h
        k3 = func(x + k2 / 2) * h  
        k4 = func(x + k3) * h
        
        return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    
    def rk4_ctrl(self, func, x, u, h):
        k1 = func(x,u) * h
        k2 = func(x + k1 / 2, u) * h
        k3 = func(x + k2 / 2, u) * h  
        k4 = func(x + k3, u) * h
        
        return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    
    def euler_forward(self, func, x, h):
        return x + h * func(x)
        
    def anime_init(self):
        self.fig, self.ax = plt.subplots()
    
        if self.sim_object == 'ball':
            # draw ball
            self.ball = plt.Circle((self.x_states[0][0], self.x_states[1][0]), 0.2, color='red', fill=True)
            self.ax.add_patch(self.ball)
            
            # draw ground
            self.ax.plot(
                [
                    min(self.x_states[0]) - 2.0, max(self.x_states[0]) + 2.0 
                ], 
                [
                    self.sim_info['ground'], 
                    self.sim_info['ground']
                ], 
                color='black', linewidth=2
            )
            
            self.ax.plot(
                [
                    min(self.x_states[0]) - 2.0, min(self.x_states[0]) - 2.0 
                ], 
                [
                    min(self.x_states[1]) - 2.0, max(self.x_states[1]) + 2.0 
                ], 
                color='black', linewidth=2
            )
            
        elif self.sim_object == 'hopper':
            # draw hopper
            self.ball = plt.Circle((self.x_states[0][0], self.x_states[1][0]), 0.1, color='red', fill=True)

            self.leg, = self.ax.plot(
                [self.x_states[0][0], self.x_states[2][0]], 
                [self.x_states[1][0], self.x_states[3][0]], 
                color='blue', 
                linewidth=3)
            # body_x x_states[0]
            # body_y x_states[1]
            # leg_x x_states[2]
            # leg_g x_states[3]
            
            self.foot_circle = plt.Circle(
                (self.x_states[2][0], self.x_states[3][0]), 
                0.05, color='green', fill=True
            )
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot_circle)
            
            # draw ground
            self.ax.plot(
                [
                    min(np.min(self.x_states[0]), np.min(self.x_states[2])) - 2, 
                    max(np.max(self.x_states[0]), np.max(self.x_states[2])) + 2
                ], [self.sim_info['ground'], self.sim_info['ground']], color='black', linewidth=2)
            
        elif self.sim_object == 'walker':
            initial_state = [
                self.x_states[0][0], 
                self.x_states[1][0], 
                self.x_states[2][0], 
                self.x_states[3][0]
            ]
            # leg1_theta x_states[0]
            # leg2_theta x_states[1]
            # leg_xc x_states[2]
            # leg_yc x_states[3] in {G}
            
            # transform from {G} to {I}
            foot_on_ground, hip, foot_in_air = self.draw_walker(x=initial_state)
            
            # draw walker
            self.ball = plt.Circle((hip[0], hip[1]), 0.2, color='red', fill=True)
            self.head = plt.Circle((hip[0], hip[1]+0.4), 0.12, color='black', fill=True)
            self.foot1 = plt.Circle((foot_on_ground[0], foot_on_ground[1]), 0.05, color='green', fill=True)
            self.foot2 = plt.Circle((foot_in_air[0], foot_in_air[1]), 0.05, color='green', fill=True)
            self.leg1, = self.ax.plot(
                [hip[0], foot_on_ground[0]], 
                [hip[1], foot_on_ground[1]], 
                color='cyan', 
                linewidth=4)
            self.leg2, = self.ax.plot(
                [hip[0], foot_in_air[0]], 
                [hip[1], foot_in_air[1]], 
                color='pink', 
                linewidth=4)
            self.hand1, = self.ax.plot(
                [hip[0], hip[0]-0.6], 
                [hip[1], hip[1]+0.1], 
                color='orange', 
                linewidth=2
            )
            
            self.hand2, = self.ax.plot(
                [hip[0], hip[0]+0.6], 
                [hip[1], hip[1]+0.1], 
                color='orange', 
                linewidth=2
            )
            
            self.neck, = self.ax.plot(
                [hip[0], hip[0]], 
                [hip[1], hip[1]+0.6], 
                color='black', 
                linewidth=4)
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.foot2)
            self.ax.add_patch(self.head)
            
            # draw ground (slope)
            min_x = np.min(self.x_states[2]) - 2
            max_x = np.max(self.x_states[2]) + 2
            
            T_G_2_I = self.homo2D(
                psi=-self.sim_info['slope_angle'], 
                trans=np.zeros([2])
            )
            
            min_xy_I = T_G_2_I @ np.array([min_x, self.sim_info['ground'], 1])
            max_xy_I = T_G_2_I @ np.array([max_x, self.sim_info['ground'], 1])
            
            self.ax.plot(
                [min_xy_I[0], max_xy_I[0]], 
                [min_xy_I[1], max_xy_I[1]], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'double_pendulum':
            initial_state = [
                self.x_states[0][0], 
                self.x_states[1][0]
            ]
            fixation, hinge, end_effector = self.draw_double_pendulum(x=initial_state)
            
            # draw points
            self.foot1 = plt.Circle((fixation[0], fixation[1]), 0.05, color='green', fill=True)
            self.foot2 = plt.Circle((hinge[0], hinge[1]), 0.05, color='green', fill=True)
            self.ball = plt.Circle((end_effector[0], end_effector[1]), 0.05, color='green', fill=True)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.foot2)
            self.ax.add_patch(self.ball)
            
            # draw arms
            self.leg1, = self.ax.plot(
                [fixation[0], hinge[0]], 
                [fixation[1], hinge[1]], 
                color='cyan', 
                linewidth=4)
            self.leg2, = self.ax.plot(
                [hinge[0], end_effector[0]], 
                [hinge[1], end_effector[1]], 
                color='pink', 
                linewidth=4)
            
            # draw plane cross
            self.ax.plot(
                [-2, 2], 
                [0, 0], 
                color='black', linewidth=2
            )
            self.ax.plot(
                [0, 0], 
                [-2, 2], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'single_pendulum':
            initial_state = self.x_states[0][0]
            
            fixation, end_effector = self.draw_single_pendulum(x=initial_state)
            
            # draw points
            self.foot1 = plt.Circle((fixation[0], fixation[1]), 0.05, color='green', fill=True)
            self.ball = plt.Circle((end_effector[0], end_effector[1]), 0.05, color='green', fill=True)
            self.ax.add_patch(self.foot1)
            self.ax.add_patch(self.ball)
            
            # draw arms
            self.leg1, = self.ax.plot(
                [fixation[0], end_effector[0]], 
                [fixation[1], end_effector[1]], 
                color='cyan', 
                linewidth=4)
            
            # draw plane cross
            self.ax.plot(
                [-2, 2], 
                [0, 0], 
                color='black', linewidth=2
            )
            self.ax.plot(
                [0, 0], 
                [-2, 2], 
                color='black', linewidth=2
            )
        
        elif self.sim_object == 'spring_mass_damper':
            # draw hopper
            self.ball = plt.Circle((self.x_states[0][0], self.sim_info['ball_radius']), self.sim_info['ball_radius'], color='red', fill=True)

            self.leg, = self.ax.plot(
                [self.x_states[0][0], self.sim_info['wall']], 
                [self.sim_info['ball_radius'], self.sim_info['ball_radius']], 
                color='blue', 
                linewidth=3)
            # ball_x x_states[0]
            
            self.foot_circle = plt.Circle(
                (self.sim_info['wall'], self.sim_info['ball_radius']), 
                0.05, color='green', fill=True
            )
            
            self.ax.add_patch(self.ball)
            self.ax.add_patch(self.foot_circle)
            
            # draw ground
            self.ax.plot(
                [
                    min(np.min(self.x_states[0][0]), self.sim_info['wall']) - 0.5, 
                    max(np.max(self.x_states[0][0]), self.sim_info['wall']) + 0.5
                ], [self.sim_info['ground'], self.sim_info['ground']], color='black', linewidth=2)
            self.ax.plot(
                [self.sim_info['wall'], self.sim_info['wall']], 
                [1.5, -0.0], color='black', linewidth=2)
            
        else:
            print("GOT ERROR CHOOSING OBJECT")
            exit()
            
        self.ax.set_aspect('equal')
            
    def anime(
        self, 
        t, 
        x_states, 
        ms = 10,
        mission='lala', 
        sim_object='ball', 
        sim_info={}, 
        save=False, 
        save_name='obj_sim'
        ):
        
        self.sim_object = sim_object
        self.x_states = x_states
        self.sim_info = sim_info
        self.anime_init()    
            
    
        ani = FuncAnimation(self.fig, self.update, frames=len(t), interval=ms, blit=True)
        if save:
            ani.save( save_name + '.mp4', writer='ffmpeg', fps=1000/ms)  # Match to real-time playback speed
            
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
        elif self.sim_object == 'walker':
            
            current_state = [
                self.x_states[0][frame], 
                self.x_states[1][frame], 
                self.x_states[2][frame],
                self.x_states[3][frame]
            ]
            foot_on_ground, hip, foot_in_air = self.draw_walker(x=current_state)
            
            if self.x_states[4][frame] == 1:
                self.leg1.set_data(
                    [hip[0], foot_on_ground[0]], 
                    [hip[1], foot_on_ground[1]]
                )
                self.leg2.set_data(
                    [hip[0], foot_in_air[0]], 
                    [hip[1], foot_in_air[1]]
                )
            else:
                self.leg1.set_data(
                    [hip[0], foot_in_air[0]], 
                    [hip[1], foot_in_air[1]]
                )
                self.leg2.set_data(
                    [hip[0], foot_on_ground[0]], 
                    [hip[1], foot_on_ground[1]]                    
                )
            self.hand1.set_data(
                    [hip[0], hip[0]-0.6], 
                    [hip[1], hip[1]+0.1]                    
                )
            self.hand2.set_data(
                    [hip[0], hip[0]+0.6], 
                    [hip[1], hip[1]+0.1]                    
                )
            
            # draw walker
            self.ball.center = (hip[0], hip[1])
            self.head.center = (hip[0], hip[1]+0.4)
            self.neck.set_data(
                [hip[0], hip[0]],
                [hip[1], hip[1]+0.4]
            )
            self.foot1.center = (foot_on_ground[0], foot_on_ground[1])
            self.foot2.center = (foot_in_air[0], foot_in_air[1])
            
            
            return (self.ball, self.foot1, self.foot2, self.leg1, self.leg2, self.hand1, self.hand2, self.head, self.neck)
                    
        elif self.sim_object == 'double_pendulum':
            current_state = [
                self.x_states[0][frame], 
                self.x_states[1][frame]
            ]
            fixation, hinge, end_effector = self.draw_double_pendulum(x=current_state)
            self.foot1.center = (fixation[0], fixation[1])
            self.foot2.center = (hinge[0], hinge[1])
            self.ball.center = (end_effector[0], end_effector[1])
            self.leg1.set_data(
                    [fixation[0], hinge[0]], 
                    [fixation[1], hinge[1]]                    
                )
            self.leg2.set_data(
                    [hinge[0], end_effector[0]], 
                    [hinge[1], end_effector[1]]                    
                )
            return (self.foot1, self.foot2, self.ball, self.leg1, self.leg2)
        
        elif self.sim_object == 'single_pendulum':
            current_state = self.x_states[0][frame]
            
            fixation, end_effector = self.draw_single_pendulum(x=current_state)
            self.foot1.center = (fixation[0], fixation[1])
            self.ball.center = (end_effector[0], end_effector[1])
            self.leg1.set_data(
                    [fixation[0], end_effector[0]], 
                    [fixation[1], end_effector[1]]                    
                )
            
            return (self.foot1, self.ball, self.leg1,)
        
        elif self.sim_object == 'spring_mass_damper':
            self.ball.center = (self.x_states[0][frame], self.sim_info['ball_radius'])
            self.leg.set_data(
                [self.x_states[0][frame], self.sim_info['wall']], 
                [self.sim_info['ball_radius'], self.sim_info['ball_radius']]
                )
            # self.foot_circle.center = (self.x_states[2][frame], self.x_states[3][frame])

            return (self.ball, self.leg,)
        
        else:
            exit()
            
    def draw_walker(self, x):
        # draw the walker in inertial frame {I}
        # q = [q0, q1, x0, x1] (in ground frame {G})
        T_G_2_I = self.homo2D(
            psi=-self.sim_info['slope_angle'], 
            trans=np.zeros([2])
        )
        T_B1_2_G = self.homo2D(
            psi=np.pi/2+x[0], 
            trans=np.array([x[2],0])
        )
        T_B2_2_B1 = self.homo2D(
            psi=np.pi + x[1], 
            trans=np.array([self.sim_info['leg_l'],0])
        )
        
        # foot on ground in {I}
        foot_on_ground = T_G_2_I @ np.array([x[2], 0, 1])
        
        # hip in {I}
        hip_G = T_B1_2_G @ np.array([self.sim_info['leg_l'], 0, 1])
        hip = T_G_2_I @ hip_G
        
        # foot in air in {I}
        foot_in_air_B1 = T_B2_2_B1 @ np.array([self.sim_info['leg_l'], 0, 1])
        foot_in_air_G = T_B1_2_G @ foot_in_air_B1
        foot_in_air = T_G_2_I @ foot_in_air_G
        
        return foot_on_ground[0:2], hip[0:2], foot_in_air[0:2]
    
    def draw_double_pendulum(self, x):
        # draw the double pendulum in inertial frame {I}
        # q = [q0, q1] (in inertial frame {I})
        T_B1_2_I = self.homo2D(
            psi=x[0], 
            trans=np.array([0,0])
        )
        T_B2_2_B1 = self.homo2D(
            psi=x[1], 
            trans=np.array([self.sim_info['l1'],0])
        )
        
        # foot on ground in {I}
        fixation = np.array([0,0])
        
        # hip in {I}
        hinge = T_B1_2_I @ np.array([self.sim_info['l1'], 0, 1])
        
        # foot in air in {I}
        end_effector_B1 = T_B2_2_B1 @ np.array([self.sim_info['l2'], 0, 1])
        end_effector = T_B1_2_I @ end_effector_B1
        
        return fixation, hinge[0:2], end_effector[0:2]
    
    def draw_single_pendulum(self, x):
        # draw the double pendulum in inertial frame {I}
        # q = [q0, q1] (in inertial frame {I})
        T_B1_2_I = self.homo2D(
            psi=x, 
            trans=np.array([0,0])
        )
        
        # foot on ground in {I}
        fixation = np.array([0,0])
        
        # foot in air in {I}
        end_effector = T_B1_2_I  @ np.array([self.sim_info['l1'], 0, 1])
        
        return fixation, end_effector[0:2]
        