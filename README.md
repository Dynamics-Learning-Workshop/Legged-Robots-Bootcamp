# Bootcamp for Legged Robots
## Introduction
This is a learning bootcamp of mine for legged robotics, i.e., bipeds and quadrupeds. In which, we have three main camps:
1.  Naive simulator from scratch, which consists the development of:
    - [Hopper](/bootcamp_scripts/1_hopper_dynamics/README.md)
    - [Hopper Control](/bootcamp_scripts/2_hopper_control/README.md)
    - [Passive Walker](/bootcamp_scripts/3_passive_walker/README.md)
    - [Walker Control](/bootcamp_scripts/4_walker_control/README.md)
    - [3D Walker Control](/bootcamp_scripts/5_walker_3D_control/README.md)
    [Download the video](/viz/projectile.mp4)
    [Download the video](/viz/raibert_hopper_200.mp4)
    [Download the video](/viz/double_pendulum_cartesian_control.mp4)
    [Download the video](/viz/passive_walker_control_partition.mp4)
    [Download the video](/viz/walker3D_control_partition.mp4)
    [Download the video](/viz/bounce_3D.mp4)
    <video width="320" height="240" controls>
      <source src="/viz/bounce_3D.mp4" type="video/mp4">
      Your browser does not support the video tag.
    </video>
    
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