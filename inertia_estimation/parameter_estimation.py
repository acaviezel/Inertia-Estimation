import numpy as np
import json
import sympy as sp
from numpy.linalg import pinv
from scipy.signal import butter, filtfilt
from scipy.optimize import lsq_linear
import matplotlib.pyplot as plt
from scipy.optimize import minimize

class InertiaEstimator:
    def __init__(self):
        self.tau_with_object = []  # Measured joint torques with object
        self.tau_without_object = []  # Measured joint torques without object
        self.jacobians_with_object = []  # Jacobians with object
        self.dq = []  # Joint velocities
        self.ddq = []  # Joint accelerations
        self.g_ee_all = []  # Gravity vector in the end-effector frame
        self.vel_ee_all = []  # Velocity vector in the end-effector frame
        self.a_ee_all = []  # Acceleration vector in the end-effector frame
        self.sampling_rate = 100  # Sampling rate in Hz
    def butter_lowpass_filter(self, data, cutoff, fs, order):
        """
        Apply a Butterworth low-pass filter to the data.
        
        Parameters:
        - data (array): Array of data to filter.
        - cutoff (float): Cutoff frequency in Hz.
        - fs (int): Sampling rate in Hz.
        - order (int): Order of the filter.
        
        Returns:
        - Filtered data as a NumPy array.
        """
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low')
        filtered_data = filtfilt(b, a, data, axis=0)  # Apply along the correct axis
        return filtered_data

    def damped_pseudo_inverse(self, K, damping_factor=0.01):
        """ Perform a damped pseudo-inverse calculation """
        
        # Ensure K is a 2D matrix
        if K.ndim != 2:
            raise ValueError(f"Expected K to be 2D, but got {K.ndim}D array.")

        # Perform Singular Value Decomposition
        U, singular_values, Vt = np.linalg.svd(K, full_matrices=False)

        # Apply damping to singular values
        singular_values_damped = singular_values / (singular_values ** 2 + damping_factor ** 2)
        
        # Create a diagonal matrix from the damped singular values
        S_damped = np.diag(singular_values_damped)
        
        # Pseudo-inverse formula: V * S_damped * U^T
        K_pseudo_inverse = Vt.T @ S_damped @ U.T
        
        return K_pseudo_inverse
    
    def load_data_with_object(self, log_file_with_object):
        """ Load the data from the log file for the object case """
        with open(log_file_with_object, 'r') as f_with_obj:
            data_points_with_obj = [json.loads(line) for line in f_with_obj]

        # Collect data in lists for filtering
        self.dq_all = []
        tau_all = []  # To store tau_with_object for filtering
        self.transfo = []
        for data_with_obj in data_points_with_obj:
            tau_measured = np.array(data_with_obj['measured_tau'])
            tau_all.append(tau_measured)  # Store in separate list for filtering
            self.dq_all.append(np.array(data_with_obj['measured_qd']))
            self.transfo.append(np.array(data_with_obj['transformation_matrix']))
            self.jacobians_with_object.append({
                'J': np.array(data_with_obj['J']),
                'dJ': np.array(data_with_obj['dJ'])
            })

        # Convert lists to arrays for filtering
        self.dq = np.vstack(self.dq_all)
        self.tau_with_object = np.vstack(tau_all)  # Convert to array for filtering

        # Apply filtering to the entire dataset
        self.dq_filtered = self.butter_lowpass_filter(self.dq, cutoff=0.5, fs=self.sampling_rate, order=2)  # Filter velocities
        self.tau_with_object = self.butter_lowpass_filter(self.tau_with_object, cutoff=1, fs=self.sampling_rate, order=2)  # Filter torques

    def load_data_without_object(self, log_file_without_object):
        """ Load the data from the log file for the no-object case """
        with open(log_file_without_object, 'r') as f_without_obj:
            data_points_without_obj = [json.loads(line) for line in f_without_obj]

        tau_all = []  # To store tau_without_object for filtering
        for data_without_obj in data_points_without_obj:
            tau_measured = np.array(data_without_obj['measured_tau'])
            tau_all.append(tau_measured)  # Store in separate list for filtering
            
        # Convert to array for filtering
        self.tau_without_object = np.vstack(tau_all)
        
        # Apply filtering to torques
        self.tau_without_object = self.butter_lowpass_filter(self.tau_without_object, cutoff=1, fs=self.sampling_rate, order=2)

    def construct_K_matrix(self, jacobian_dict,v_ee, a_ee, T_Flange):
        """ Construct the full K matrix using the loaded Jacobian data, joint velocities, and gravity """  
        # Extract relevant Jacobians and their derivatives
        J = jacobian_dict['J']  # Translational Jacobian
        #Compute the inverse of the transformation matrix
        
        R_load = T_Flange[:3, :3]
        g_base = np.array([0, 0, -9.81])  # Gravity vector in the base frame
        # Transform the gravity vector into the end-effector frame
        #Compute gravity vector in end-effector frame
        g_EE = R_load.T @ g_base
        g_x = g_EE[0]
        g_y = g_EE[1]
        g_z = g_EE[2]
        self.g_ee_all.append(g_EE) 
        # Filtered data is already passed in for the current configuration
        # Extract components directly
        omega_x = v_ee[3]
        omega_y = v_ee[4]
        omega_z = v_ee[5]
        a_x = a_ee[0]
        a_y = a_ee[1]
        a_z = a_ee[2]
        omega_dot_x = a_ee[3]
        omega_dot_y = a_ee[4]
        omega_dot_z = a_ee[5]

        # Construct Matrix A
        A_1 = np.zeros((6, 4))
        A_1[0, :] = [a_x - g_x, -(omega_y)**2 - (omega_z)**2, -omega_dot_z + omega_x * omega_y, omega_dot_y + omega_x * omega_z]
        A_1[1, :] = [a_y - g_y, omega_dot_z + omega_x * omega_y, -(omega_x)**2 - (omega_z)**2, -omega_dot_x + omega_y * omega_z]
        A_1[2, :] = [a_z - g_z, -omega_dot_y + omega_x * omega_z, omega_dot_x + omega_y * omega_z, -(omega_x)**2 - (omega_y)**2]
        A_1[3, :] = [0, 0, a_z - g_z , g_y -a_y]
        A_1[4, :] = [0, g_z - a_z, 0, a_x - g_x]
        A_1[5, :] = [0, a_y - g_y, g_x -a_x , 0]

        A_2 = np.zeros((6, 6))
        A_2[3, :] = [omega_dot_x, omega_dot_y - omega_x * omega_z, omega_dot_z + omega_x * omega_y, -omega_y * omega_z,
                     (omega_y)**2 - (omega_z)**2, omega_y * omega_z]
        A_2[4, :] = [omega_x * omega_z, omega_dot_x + omega_y * omega_z, (omega_z)**2 - (omega_x)**2, omega_dot_y,
                     omega_dot_z - omega_x * omega_y, -omega_x * omega_z]
        A_2[5, :] = [-omega_x * omega_y, (omega_x)**2 - (omega_y)**2, omega_dot_x - omega_y * omega_z, omega_x * omega_y,
                     omega_dot_y + omega_x * omega_z, omega_dot_z]

        # Combine A_1 and A_2 to form the full A matrix 6x10
        A = np.hstack((A_1, A_2))

        # Calculate K matrix = J.T * A
        K_matrix = J.T @ A
        return K_matrix

    def estimate_phi(self):
        """Estimate inertial parameters using least squares with optimization constraints"""
        tau_diff = []
        K_matrices = []

        num_configs = min(len(self.tau_with_object), len(self.tau_without_object))
        
        for i in range(num_configs):
            jacobian_dict = self.jacobians_with_object[i]
            dq_filt = self.dq_filtered[i] if i > 0 else np.zeros(7)
            ddq_filt = (dq_filt - self.dq_filtered[i - 1]) / 0.01 if i > 0 else np.zeros(7)
            

            J = jacobian_dict['J']  # Translational Jacobian
            vel = J @ dq_filt
            accel = J @ ddq_filt

            self.vel_ee_all.append(vel)
            self.a_ee_all.append(accel)
        
        for i in range(num_configs):
            tau_P = self.tau_with_object[i]
            tau_0_P = self.tau_without_object[i]
            jacobian_dict = self.jacobians_with_object[i]
            transfo = self.transfo[i]
            v_ee = self.vel_ee_all[i]
            a_ee = self.a_ee_all[i]

            K_matrix = self.construct_K_matrix(jacobian_dict, v_ee, a_ee, transfo)
            tau_diff.append(tau_P - tau_0_P)
            K_matrices.append(K_matrix)

        # Combine tau_diff and K_matrices into single matrices
        tau_diff = np.vstack(tau_diff).reshape(-1, 1)
        K_combined = np.vstack(K_matrices)

        # Compute initial phi estimation with pseudo-inverse
        K_pseudo_inverse = self.damped_pseudo_inverse(K_combined)
        phi_initial = K_pseudo_inverse @ tau_diff

        # Define the objective function to minimize the difference from initial estimate
        def objective(phi):
            return np.sum((K_combined @ phi - tau_diff.flatten()) ** 2)

        # Define constraints for phi[4], phi[7], and phi[9] to be greater than zero
        constraints = [
            {'type': 'ineq', 'fun': lambda phi: phi[4] - 1e-10},  # phi[4] > 0
            {'type': 'ineq', 'fun': lambda phi: phi[7] - 1e-10},  # phi[7] > 0
            {'type': 'ineq', 'fun': lambda phi: phi[9] - 1e-10}   # phi[9] > 0
        ]

        # Run the optimization starting from the initial estimate
        result = minimize(objective, phi_initial.flatten(), constraints=constraints)

        # Check if optimization was successful and return the result
        if result.success:
            phi_estimated = result.x
        else:
            print("Optimization failed:", result.message)
            phi_estimated = phi_initial.flatten()  # Fallback to initial estimate if optimization fails

        return phi_estimated

    def run_inertia_estimation(self, log_file_with_object, log_file_without_object):
        """ Run inertia estimation using the data with the object and construct K matrices """
        self.load_data_with_object(log_file_with_object)
        self.load_data_without_object(log_file_without_object)
        phi = self.estimate_phi()
        #self.plot_measured_torques()
        #self.plot_end_effector_accelerations()
        #self.plot_end_effector_velocities()
        #self.plot_end_effector_accelerations_angular()
        phi[1] = phi[1]/phi[0]
        phi[2] = phi[2]/phi[0]
        phi[3] = phi[3]/phi[0]
        
        print("Estimated inertial parameters (mass, r_com, Inertia_Matrix):")
        print(f"mass: {phi[0]:.4f},")
        print(f"center_of_mass: [{phi[1]:.4f}, {phi[2]:.4f}, {phi[3]:.4f}],")
        print(f"load_inertia: [{phi[4]}, {phi[5]}, {phi[6]}, {phi[5]} , {phi[7]}, {phi[8]}, {phi[6]},{phi[8]}, {phi[9]}]")


        

    def plot_measured_torques(self):
        """ Plot the measured torques for the configurations with and without the object """
        plt.figure(figsize=(12, 6))
        plt.plot(self.tau_with_object, label='With Object')
        plt.plot(self.tau_without_object, label='Without Object')
        plt.xlabel('Time Step')
        plt.ylabel('Torque (Nm)')
        plt.title('Measured Torques with and without Object')
        plt.legend()
        plt.show()

    # plot dq_filtered
    def plot_joint_velocities(self):
        """ Plot the filtered joint velocities and accelerations """
        plt.figure(figsize=(12, 6))
        plt.plot(self.dq_filtered, label='Joint Velocities')
        plt.xlabel('Time Step')
        plt.ylabel('Value')
        plt.title('Filtered Joint Velocities')
        plt.legend()
        plt.show()

    def plot_joint_accelerations(self):
        """ Plot the filtered joint accelerations """
        plt.figure(figsize=(12, 6))
        plt.plot(self.ddq_filt_all, label='Joint Accelerations')
        plt.xlabel('Time Step')
        plt.ylabel('Value')
        plt.title('Filtered Joint Accelerations')
        plt.legend()
        plt.show()
        
    
    
    
    def plot_end_effector_accelerations(self):
        """ Plot the acceleration vector in the end-effector frame over all configurations """
        a_ee_all = np.vstack(self.a_ee_all)
        plt.figure(figsize=(10, 6))
        plt.plot(a_ee_all[:, 0], label='a_x in EE frame')
        plt.plot(a_ee_all[:, 1], label='a_y in EE frame')
        plt.plot(a_ee_all[:, 2], label='a_z in EE frame')
        plt.xlabel('Time Step')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Acceleration Vector in End-Effector Frame Over Configurations')
        plt.legend()
        plt.grid(True)
        plt.show()
    
    def plot_end_effector_accelerations_angular(self):
        """ Plot the angular acceleration vector in the end-effector frame over all configurations """
        a_ee_all = np.vstack(self.a_ee_all)
        plt.figure(figsize=(10, 6))
        plt.plot(a_ee_all[:, 3], label='alpha_x in EE frame')
        plt.plot(a_ee_all[:, 4], label='alpha_y in EE frame')
        plt.plot(a_ee_all[:, 5], label='alpha_z in EE frame')
        plt.xlabel('Time Step')
        plt.ylabel('Angular Acceleration (rad/s²)')
        plt.title('Angular Acceleration Vector in End-Effector Frame Over Configurations')
        plt.legend()
        plt.grid(True)
        plt.show()
    def plot_end_effector_velocities(self):
        """ Plot the velocity vector in the end-effector frame over all configurations """
        vel_ee_all = np.vstack(self.vel_ee_all)
        plt.figure(figsize=(10, 6))
        plt.plot(vel_ee_all[:, 0], label='v_x in EE frame')
        plt.plot(vel_ee_all[:, 1], label='v_y in EE frame')
        plt.plot(vel_ee_all[:, 2], label='v_z in EE frame')
        plt.xlabel('Time Step')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity Vector in End-Effector Frame Over Configurations')
        plt.legend()
        plt.grid(True)
        plt.show()
        
    def plot_joint_positions(self):
        """ Plot the joint positions for the configurations with and without the object """
        plt.figure(figsize=(12, 6))
        plt.plot(self.q_obj, label='With Object')
        plt.plot(self.q_no_obj, label='Without Object')
        plt.xlabel('Time Step')
        plt.ylabel('Joint Position (rad)')
        plt.title('Joint Positions with and without Object')
        plt.legend()
        plt.show()

    def plot_gravity_vector(self):
        """ Plot the gravity vector components in the end-effector frame over all configurations """
        g_ee_all = np.array(self.g_ee_all)
        
        # Calculate the magnitude of the gravity vector in the end-effector frame for each configuration
        g_magnitude = np.linalg.norm(g_ee_all, axis=1)

        plt.figure(figsize=(10, 6))
        plt.plot(g_ee_all[:, 0], label='g_x in EE frame')
        plt.plot(g_ee_all[:, 1], label='g_y in EE frame')
        plt.plot(g_ee_all[:, 2], label='g_z in EE frame')
        plt.plot(g_magnitude, label='|g| in EE frame', linestyle='--', color='black')  # Plot the magnitude

        plt.xlabel("Configuration Index")
        plt.ylabel("Gravity Component (m/s²)")
        plt.title("Gravity Vector Components in End-Effector Frame Over Configurations")
        plt.legend()
        plt.grid(True)
        plt.show()



def main():
    # Insert paths to your log files for the respective cases
    log_file_with_object = "/home/andri/inertia_estimation_cases/ball/inertia_estimation_ball2024_11_07_1044.json"
    log_file_without_object = "/home/andri/inertia_estimation_cases/ball/inertia_estimation_no_object2024_11_07_1045.json"

    estimator = InertiaEstimator()
    estimator.run_inertia_estimation(log_file_with_object, log_file_without_object)

if __name__ == "__main__":
    main()
