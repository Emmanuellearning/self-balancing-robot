# Self-Balancing Robot — Inverted Pendulum Dynamics & Control

Two-wheeled self-balancing robot built around inverted pendulum dynamics, featuring a ROS 2 Jazzy + Gazebo physics simulation, MATLAB/Simulink control design (dual-loop PID and full state-space pole placement), and derivation notes.

## Repository Structure

```text
self-balancing-robot/
├── docs/
│   └── state_space_notes/               # Handwritten state-space derivation notes (JPEG)
└── self_balancing_sim/                  # ROS 2 Jazzy + Gazebo simulation & MATLAB scripts
    ├── Inverted_Pendulum_Dynamics_and_PID_Equations.pdf
    ├── balance_test.m                   # Inner-loop pitch PID simulation
    ├── full_cascade_sim.m               # Cascaded position PD + pitch PID simulation
    ├── matrix_statespace_sim.m          # State-space (A, B, K) pole placement & response
    ├── compare_before_after.m
    ├── plot_poles.m
    ├── simulate_pid.m
    ├── theory_inner_loop.m
    ├── generate_full_simulink.m
    ├── generate_simulink_inner_loop.m
    ├── generate_simulink_model.m
    ├── launch/
    ├── urdf/
    ├── worlds/
    └── self_balancing_sim/              # ROS 2 Python nodes
```

## Quick Start (ROS 2 Jazzy)

```bash
colcon build --symlink-install --packages-select self_balancing_sim
source install/setup.bash
ros2 launch self_balancing_sim sim.launch.py
```

See [self_balancing_sim/README.md](self_balancing_sim/README.md) for full ROS 2 package details.
