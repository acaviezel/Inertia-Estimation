# Inertia Estimation for Franka Emika FR3 Robot

This code environment provides tools to estimate the inertial parameters (mass, center of mass, and inertia matrix) of a payload for the Franka Emika FR3 Robot. The parameters are estimated by analyzing the difference in measured joint torques while executing the same trajectory twice—once with the payload and once without.

---

## Prerequisites

Ensure the following are installed before using this tool:

- **ROS 2 Humble**
- **Libfranka 0.13.0** or newer
- **franka_ros2 v0.13.1** or newer
- **Cartesian Impedance Controller**
  - Installation details can be found here: [Cartesian Impedance Control Repository](https://github.com/acaviezel/cartesian_impedance_control)

---

## Installation

1. Clone this repository into the src directory of your workspace:

   ```bash
   cd franka_ros2_ws/src
   git clone https://github.com/acaviezel/Inertia-Estimation.git
   ```
2. Clone the message package into your workspace (if not already done):
  ```bash
   cd franka_ros2_ws/src
   git clone https://github.com/acaviezel/messages_fr3.git
   ```
3. Change the following things in your impedance controller for the inertia estimation:
     --**Control Law:**
     For the inertia estimation, a simple PD-controller is sufficient. Activated the respective controll law and comment out the impedance control law. Check lines **390 - 400** in the .cpp file
     --**Launch File:**
     If you want to estimate the inertial parameters of a new gripper, set the default value in the gripper launch argument to **false**. If you want to estimate the parameters of an attached object to the Franka Hand, keep the value at **true**. 
   
## Procedure

Once everything is set up, the inertial parameters can be estimated as follows:

1. Run the file `inertia_estimation.py` twice:
   - **First Run:** Without the payload.
   - **Second Run:** With the payload attached.
   
   Update the names of the generated JSON file accordingly (on **line 71** of `inertia_estimation.py`). This file will send a trajectory to the controller and log all relevant data needed for the estimation process.

2. Load the generated JSON files from Step 1 into `parameter_estimation.py`:
   - Specify the file paths on **lines 350 and 351** of `parameter_estimation.py`.
   - Run `parameter_estimation.py`. The estimated inertial parameters will be printed in the terminal.
