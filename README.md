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
2. Create the following two custom message types (Jacobian, Transformation Matrix)

    Jacobian:
    ```bash
    float64[42] jacobian  # A flat array representing J
    float64[42] d_jacobian # A flat array representing dJ
    ```
    Transformation Matrix:
    ```bash
    float64[16] transformation_matrix
   ```
## Procedure

Once everything is set up, the inertial parameters can be estimated as follows:

1. Run the file `inertia_estimation.py` twice:
   - **First Run:** Without the payload.
   - **Second Run:** With the payload attached.
   
   Update the names of the generated JSON file accordingly (on **line 71** of `inertia_estimation.py`). This file will send a trajectory to the controller and log all relevant data needed for the estimation process.

2. Load the generated JSON files from Step 1 into `parameter_estimation.py`:
   - Specify the file paths on **lines 350 and 351** of `parameter_estimation.py`.
   - Run `parameter_estimation.py`. The estimated inertial parameters will be printed in the terminal.
