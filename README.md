# Bootcamp for Legged Robots
## Introduction
This is a learning bootcamp of mine for legged robotics, i.e., bipeds and quadrupeds. In which, we have three main camps:
1. $\texttt{3D walker}$: Naive simulator from scratch, which consists the development of:
    - [Hopper](/bootcamp_scripts/1_hopper_dynamics/README.md)
    - [Hopper Control](/bootcamp_scripts/2_hopper_control/README.md)
    - [Passive Walker](/bootcamp_scripts/3_passive_walker/README.md)
    - [Walker Control](/bootcamp_scripts/4_walker_control/README.md)
    - [3D Walker Control](/bootcamp_scripts/5_walker_3D_control/README.md)

    Goal:
    <p align="center">
      <img src="/viz/walker3D_control_partition_gif.gif" alt="Walker 3D Control Partition" style="width: 100%;"/>
    </p>

    via:
    <table align="center">
      <tr>
        <td align="center">
          <img src="/viz/bounce_3D.gif" alt="Projectile" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/raibert_hopper_200.gif" alt="Raibert Hopper" width="300"/>
        </td>
      </tr>
      <tr>
        <td align="center">
          <img src="/viz/double_pendulum_cartesian_control.gif" alt="Double Pendulum" width="300"/>
        </td>
        <td align="center">
          <img src="/viz/passive_walker_control_partition.gif" alt="Passive Walker" width="300"/>
        </td>
      </tr>
    </table>

    

    
2.  
## Reference
Based on 
- UIC course (legged robotics) by Pranav Bhounsule, 
  - I did my own implementation in Python
- Open project by Boston Cleek, 
  - I took reference from the code structure
  - and designed my own controller that can
    - stand
    - walk
    - gallop
- Simulation platform developed by UniTree Gazebo.
  - my implementation is based in Gazebo

```
@misc{pranav,
  title        = {Legged Robotics},
  author       = {Pranav Bhounsule},
  year         = 2021,
  note         = {\url{https://pab47.github.io/legs.html} [Accessed: 17/Sep/2024]}
}

@misc{qcontrol,
  title        = {Quadruped Control},
  author       = {Boston Cleek},
  year         = 2021,
  note         = {\url{https://github.com/bostoncleek/quadruped_control} [Accessed: 17/Sep/2024]}
}

@misc{unitree,
  title        = {Unitree Ros},
  author       = {Unitree},
  year         = 2024,
  note         = {\url{https://github.com/unitreerobotics/unitree_ros} [Accessed: 17/Sep/2024]}
}