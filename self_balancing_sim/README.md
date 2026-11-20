# Self-Balancing Robot Simulation (PID Control)

A 2D two-wheeled self-balancing robot (inverted pendulum) simulation in ROS 2 Jazzy and Gazebo.

---

## Package Layout

```text
self_balancing_sim/
├── package.xml
├── setup.py
├── setup.cfg
├── urdf/
│   └── self_balancing_robot.urdf.xacro   # Robot URDF (chassis, wheels, IMU, DiffDrive)
├── worlds/
│   └── empty.world                       # Gazebo world
├── launch/
│   └── sim.launch.py                     # Main launch file
└── self_balancing_sim/
    ├── __init__.py
    ├── pid_gain_solver.py                # Simple script for parameter and gain calculation
    └── pid_controller_node.py            # Main PID controller node (100 Hz)
```

---

## How It Works

The robot uses a simple **dual-loop PID controller**:

1. **Inner Loop (Pitch Balance)**:
   - Reads the pitch angle $\theta$ and angular velocity $\dot{\theta}$ from the IMU.
   - When the robot leans forward ($\theta > 0$), it drives the wheels forward to catch itself:
     $$u = K_p (\theta - \theta_{\text{target}}) + K_i \int (\theta - \theta_{\text{target}}) dt + K_d \dot{\theta}$$

2. **Outer Loop (Position Holding)**:
   - Reads position $x$ and velocity $v$ from odometry.
   - Adjusts the target pitch angle to stay in place:
     $$\theta_{\text{target}} = -(K_{p,\text{pos}} (x - x_{\text{target}}) + K_{d,\text{pos}} v)$$

---

## How to Build & Run

### 1. Build
```bash
cd ~/Dynamics_project
colcon build --symlink-install --packages-select self_balancing_sim
source install/setup.bash
```

### 2. Run the Parameter Calculation Script
```bash
ros2 run self_balancing_sim pid_gain_solver
```

### 3. Launch Simulation
```bash
ros2 launch self_balancing_sim sim.launch.py
```
