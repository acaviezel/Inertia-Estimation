import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState
from messages_fr3.msg import Jacobian, Acceleration, TransformationMatrix
import numpy as np
import json
from datetime import datetime
import time




class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        self.start_time = time.monotonic()
        # Fixed positions for joints 1, 2, 3, and 4
        self.fixed_positions = {
            "A1": np.radians(0),
            "A2": np.radians(-45),
            "A3": np.radians(0),
            "A4": np.radians(-135)
        }

        # Initialize the rest of the joints (5, 6, 7) to neutral positions
        self.neutral_position = [
            self.fixed_positions["A1"], 
            self.fixed_positions["A2"], 
            self.fixed_positions["A3"], 
            self.fixed_positions["A4"], 
            np.radians(0),  
            np.radians(90),  
            np.radians(45)   
        ]

        # Parameters for trajectory and timing
        self.pause_time = 5                 # Initial 5-second pause for logging
        self.motion_time = 50               # 50seconds total for joint motions
        self.total_trajectory_time = self.pause_time + self.motion_time + 1.5 +3 # Add 1.5 seconds for small pauses
        self.timer_period = 0.01           # 100Hz frequency (5ms intervals)
        
        # Timer for executing the trajectory and logging
        self.timer = self.create_timer(self.timer_period, self.execute_trajectory_and_log)
        

        # Generate trajectory for joints 5, 6, and 7
        self.trajectory_points = self.generate_trajectory()

        # Publishers and subscribers
        self.q_d_publisher = self.create_publisher(JointState, '/franka_robot_state_broadcaster/desired_joint_state', 10)
        self.subscription = self.create_subscription(FrankaRobotState, '/franka_robot_state_broadcaster/robot_state', self.robot_state_callback, 10)
        
        self.jacobian_subscription = self.create_subscription(Jacobian, 'jacobian', self.jacobian_callback, 10)
        self.transformation_subscription = self.create_subscription(TransformationMatrix, 'transformation_matrix', self.transformation_callback, 10)
        # Initialize Jacobian and dJ matrices
        self.jacobian_translational = np.zeros((3, 7))
        self.jacobian_rotational = np.zeros((3, 7))
        self.dJ_translational = np.zeros((3, 7))
        self.dJ_rotational = np.zeros((3, 7))

        #initialize transformation matrix
        self.transformation_matrix = np.zeros((4, 4))
        # Initialize variables for logging
        self.measured_joint_state = None
    
        

        # Create log file
        timestamp = datetime.now().strftime("%Y_%m_%d_%H%M")
        self.log_file = f"inertia_estimation_no_object{timestamp}.json"
        self.get_logger().info(f"Logging to: {self.log_file}")

    def generate_trajectory(self):
        """ Generate the joint trajectory for each joint with a 5-second initial pause at the neutral position """
        trajectory_points = []
        joint_time = self.motion_time / 4  # Each joint moves for one-seventh of the total motion time

        # Define time points for each joint's movement after the initial pause
        time_steps = np.linspace(0, joint_time, int(joint_time / self.timer_period))

        # Add a 5-second pause at the neutral position
        
        small_pause = int(0.5 / self.timer_period)  # Half-second pause for smoother motion
        for i in range( int(5 / self.timer_period)):
            trajectory_points.append(self.neutral_position.copy())

        # Define sinusoidal trajectories for respective joints
        joint_2_positions = np.radians(-45) + np.radians(45) * np.sin(4 * np.pi * time_steps / joint_time)
        joint_5_positions = np.radians(100) * np.sin(4 * np.pi * time_steps / joint_time)
        joint_6_positions = np.radians(90) + np.radians(50) * np.sin(4 * np.pi * time_steps / joint_time)
        joint_7_positions = np.radians(45) + np.radians(90) * np.sin(4 * np.pi * time_steps / joint_time)

        # Populate trajectory points for each joint's sequence
        # Each joint's trajectory is a sinusoidal motion
        
        for i in range(len(time_steps)):
            q_desired = self.neutral_position.copy()
            q_desired[1] = joint_2_positions[i]
            trajectory_points.append(q_desired.copy())
        
        for i in range(small_pause):
            trajectory_points.append(self.neutral_position.copy())
        
        for i in range(len(time_steps)):
            q_desired = self.neutral_position.copy()
            q_desired[4] = joint_5_positions[i]
            trajectory_points.append(q_desired.copy())
 
        for i in range(small_pause):
            trajectory_points.append(self.neutral_position.copy())
 
        for i in range(len(time_steps)):
            q_desired = self.neutral_position.copy()
            q_desired[5] = joint_6_positions[i]
            trajectory_points.append(q_desired.copy())

        for i in range(small_pause):
            trajectory_points.append(self.neutral_position.copy())

        for i in range(len(time_steps)):
            q_desired = self.neutral_position.copy()
            q_desired[6] = joint_7_positions[i]
            trajectory_points.append(q_desired.copy())

        for i in range(3):
            trajectory_points.append(self.neutral_position.copy())
        
        return trajectory_points

    def execute_trajectory_and_log(self):
        """ Execute the trajectory and log data in sync at each timer event """
        # Calculate elapsed time based on the initial start time
        elapsed_time = time.monotonic() - self.start_time

        # Check if within the total duration (pause + motion)
        if elapsed_time < self.total_trajectory_time:
            # Calculate the current trajectory index based on elapsed time, capped by trajectory length
            current_index = min(int(elapsed_time / self.timer_period), len(self.trajectory_points) - 1)
            target_q_d = self.trajectory_points[current_index]

            # Publish desired joint positions
            msg = JointState()
            msg.position = target_q_d
            self.q_d_publisher.publish(msg)
            self.get_logger().debug(f'Sent trajectory Point {current_index+1}: q_d = {target_q_d}')

            # Log the data at each timer event
            if elapsed_time > 1.0:  # Skip the first second for smoother motion
                adjusted_timestamp = elapsed_time
                self.log_data(adjusted_timestamp)
            
        else:
            self.get_logger().info("Trajectory complete.")
            self.timer.cancel()  # Stop the timer after completing trajectory

    def robot_state_callback(self, msg: FrankaRobotState):
        """ Callback to receive the robot's state """
        self.measured_joint_state = msg.measured_joint_state
        self.tau_external = msg.tau_ext_hat_filtered.effort
        

    def jacobian_callback(self, msg: Jacobian):
        """ Callback to receive the Jacobian data """
        try:
            if len(msg.jacobian) != 42 or len(msg.d_jacobian) != 42:
                self.get_logger().error("Jacobian or d_jacobian does not contain 42 elements!")
                return

            self.jacobian = np.array(msg.jacobian).reshape(6, 7, order='F')
            self.jacobian_translational = self.jacobian[:3, :]
            self.jacobian_rotational = self.jacobian[3:, :]

            self.dJ = np.array(msg.d_jacobian).reshape(6, 7, order='F')
            self.dJ_translational = self.dJ[:3, :]
            self.dJ_rotational = self.dJ[3:, :]

        except Exception as e:
            self.get_logger().error(f"Error processing Jacobian data: {str(e)}")

    def transformation_callback(self, msg: TransformationMatrix):
        """ Callback to receive the transformation matrix data """
        self.transformation_matrix = np.array(msg.transformation_matrix).reshape(4, 4, order='F')
    # In your log_data function
    def log_data(self, adjusted_timestamp):
        """ Log the data with the adjusted timestamp """
        if self.measured_joint_state is None:
            self.get_logger().warn("Measured joint state not available. Skipping log entry.")
            return

        data_to_log = {
            "measured_qd": list(self.measured_joint_state.velocity),
            "measured_tau": list(self.measured_joint_state.effort),
            "J": self.jacobian.tolist(),
            "dJ": self.dJ.tolist(),
            "transformation_matrix": self.transformation_matrix.tolist(),
            "timestamp": adjusted_timestamp,  # Adjusted timestamp after pause
        }

        with open(self.log_file, 'a') as f:
            f.write(json.dumps(data_to_log) + '\n')

        self.get_logger().debug(f"Logged data at timestamp: {adjusted_timestamp:.2f} seconds")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
