
# Inverted Pendulum Optimal Controller Comparison

This repository presents a comparative study of four optimal control strategies applied to an inverted pendulum system simulated in Gazebo. The controllers evaluated include:

- **Linear Quadratic Regulator (LQR)**
- **Linear Model Predictive Control (LMPC)**
- **Nonlinear Model Predictive Control (NMPC)**
- **Learning-Based Model Predictive Control (LBMPC)**

The implementation leverages the following technologies:

- **Operating System:** Ubuntu 20.04 LTS
- **Robot Operating System (ROS):** ROS Noetic Ninjemys
- **Optimization Framework:** [CasADi](https://web.casadi.org/)
- **MPC Toolbox:** [HILO-MPC](https://github.com/hilo-mpc/hilo-mpc)
- **Control Library:** Python Control Systems Library (`python-control`)


## Installation and Setup

### Prerequisites

Ensure the following software is installed on your system:

- Ubuntu 20.04 LTS
- ROS Noetic Ninjemys
- Gazebo (compatible with ROS Noetic)
- Python 3.8 or higher
- [CasADi](https://web.casadi.org/get/)
- [HILO-MPC](https://github.com/hilo-mpc/hilo-mpc)
- Python Control Systems Library (`python-control`)

### Installation Steps

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/abuibaid/Inverted-Pendulum-optimal-controller-comparision.git
   cd Inverted-Pendulum-optimal-controller-comparision
   ```

2. **Set Up the ROS Workspace:**

   ```bash
   cd catkin_cart_pole
   catkin_make
   source devel/setup.bash
   ```

3. **Install Python Dependencies:**

   It is recommended to use a virtual environment:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install casadi hilo-mpc control
   ```

## Running the Simulation

1. **Launch the Gazebo Simulation:**

   ```bash
   roslaunch robot_launch launch_simulation.launch
   ```

2. **Execute the Desired Controller:**

   for LQR:

   ```bash
   cd Inverted-Pendulum-optimal-controller-comparision/cart_pole_control/LQR
   python closed_loop.py
   ```
  for linear, nonlinear and learning MPC:

   ```bash
   cd Inverted-Pendulum-optimal-controller-comparision/cart_pole_control/MPC
   python closed_loop.py
   ```
