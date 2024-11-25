import numpy as np
from scipy.optimize import minimize

class InverseKinematicsso100:
    def __init__(self):
        """
        Initializes the robotic arm with a given number of joints.

        Parameters:
            joint_count (int): Number of joints in the robotic arm.
            zyx Tait-Bryan angles
            assume joint angles to be 0 when in middle position
        """
        self.joint_angles = np.zeros(5)  # generalized coordinates

        # Define link vectors and initial transformations between joints
        self.vec_1 = np.array([0.0729, 0.0, 0.0306]) * 1000
        self.vec_2 = np.array([0.0, 0.028, 0.11257]) * 1000
        self.vec_3 = np.array([0.0, -0.0052, 0.1349]) * 1000
        self.vec_4 = np.array([0.0, 0.0, 0.0424]) * 1000
        self.vec_5 = np.array([0.0, 0.0, 0.0545]) * 1000

    def rotation_matrix_zyx(self, angles):
        """
        Computes the rotation matrix given ZYX Tait-Bryan angles.

        Parameters:
            angles (np.array): Array of ZYX angles [z, y, x].

        Returns:
            np.array: The corresponding 3x3 rotation matrix.
        """
        z, y, x = angles
        Rz = np.array([[np.cos(z), -np.sin(z), 0],
                       [np.sin(z),  np.cos(z), 0],
                       [0,          0,         1]])
        Ry = np.array([[np.cos(y),  0, np.sin(y)],
                       [0,          1,          0],
                       [-np.sin(y), 0, np.cos(y)]])
        Rx = np.array([[1, 0,          0         ],
                       [0, np.cos(x), -np.sin(x)],
                       [0, np.sin(x),  np.cos(x)]])
        return Rz @ Ry @ Rx

    def forward_kinematics_(self, joint_angles = None): 
        """
        Calculate the forward kinematics to determine the end-effector position and orientation given joint angles.

        Parameters:
            joint_angles (np.array): Array of joint angles.

        Returns:
            tuple: The position and orientation (rotation matrix) of the end-effector.
        """
        debug = False

        if joint_angles is not None:
            self.joint_angles = joint_angles
        # Homogeneous transformation matrices for each joint
        T_1_2 = np.eye(4)
        T_1_2[:3, :3] = self.rotation_matrix_zyx([1/2  * np.pi, self.joint_angles[0], 1/2  * np.pi])
        T_1_2[:3, 3] = np.array([self.vec_1[0], 
                                 -self.vec_1[2] * np.sin(self.joint_angles[0]), 
                                 self.vec_1[2] * np.cos(self.joint_angles[0])])

        T_2_3 = np.eye(4)
        T_2_3[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[1] - np.pi / 2])
        T_2_3[:3, 3] = np.array([0, 
                                 self.vec_2[0] * np.cos(self.joint_angles[1]) - self.vec_2[2] * np.sin(self.joint_angles[1]), 
                                 self.vec_2[1] * np.sin(self.joint_angles[1]) + self.vec_2[2] * np.cos(self.joint_angles[1])])

        T_3_4 = np.eye(4)
        T_3_4[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[2]])
        T_3_4[:3, 3] = np.array([0, 
                                 -self.vec_3[2] * np.sin(self.joint_angles[2]) + self.vec_3[1] * np.cos(self.joint_angles[2]), 
                                 self.vec_3[2] * np.cos(self.joint_angles[2]) + self.vec_3[1] * np.sin(self.joint_angles[2])])

        T_4_5_1 = np.eye(4)
        T_4_5_1[:3, :3] = self.rotation_matrix_zyx([0, -np.pi/2, 0])

        T_4_5_2 = np.eye(4)
        T_4_5_2[:3, :3] = self.rotation_matrix_zyx([self.joint_angles[3], 0, 0])
        T_4_5_2[:3, 3] = np.array([self.vec_4[2] * np.cos(self.joint_angles[3]),
                                   self.vec_4[2] * np.sin(self.joint_angles[3]),
                                   0])

        T_4_5 = T_4_5_1 @ T_4_5_2

        T_5_EE = np.eye(4)
        T_5_EE[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[4]])
        T_5_EE[:3, 3] = self.vec_5

        # Compute the final transformation matrix from the base to the end-effector
        T_final = T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_EE

        # Extract end-effector position and orientation
        end_effector_position = T_final[:3, 3]
        end_effector_orientation = T_final[:3, :3]

        # convert orientation into euler angles, zyx
        z = np.arctan2(T_final[1, 0], T_final[0, 0])
        y = np.arctan2(-T_final[2, 0], np.sqrt(T_final[2, 1]**2 + T_final[2, 2]**2))
        x = np.arctan2(T_final[2, 1], T_final[2, 2])
        end_effector_orientation = np.array([z, y, x]) * 180 / np.pi

        if debug:
            # print all transformations
            print("T_1_2:", np.round(T_1_2, 4))
            print("T_2_3:", np.round(T_2_3, 4))
            print("T_3_4:", np.round(T_3_4, 4))
            print("T_4_5_1:", np.round(T_4_5_1, 4))
            print("T_4_5_2:", np.round(T_4_5_2, 4))
            print("T_5_EE:", np.round(T_5_EE, 4))
            print("T_final:", np.round(T_final, 4))
            print("Joint angles:", np.round(self.joint_angles * 180 / np.pi, 4))
            
            print("Frame 2: pos:", np.round(T_1_2[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_1_2[1, 0], T_1_2[0, 0]),
                    np.arctan2(-T_1_2[2, 0], np.sqrt(T_1_2[2, 1]**2 + T_1_2[2, 2]**2)),
                    np.arctan2(T_1_2[2, 1], T_1_2[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_2_3 = T_1_2 @ T_2_3
            print("Frame 3: pos:", np.round(T_2_3[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_2_3[1, 0], T_2_3[0, 0]),
                    np.arctan2(-T_2_3[2, 0], np.sqrt(T_2_3[2, 1]**2 + T_2_3[2, 2]**2)),
                    np.arctan2(T_2_3[2, 1], T_2_3[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_3_4 = T_2_3 @ T_3_4
            print("Frame 4: pos:", np.round(T_3_4[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_3_4[1, 0], T_3_4[0, 0]),
                    np.arctan2(-T_3_4[2, 0], np.sqrt(T_3_4[2, 1]**2 + T_3_4[2, 2]**2)),
                    np.arctan2(T_3_4[2, 1], T_3_4[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_4_5_1 = T_3_4 @ T_4_5_1
            print("Frame 5_1: pos:", np.round(T_4_5_1[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_4_5_1[1, 0], T_4_5_1[0, 0]),
                    np.arctan2(-T_4_5_1[2, 0], np.sqrt(T_4_5_1[2, 1]**2 + T_4_5_1[2, 2])),
                    np.arctan2(T_4_5_1[2, 1], T_4_5_1[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_4_5_2 = T_4_5_1 @ T_4_5_2
            print("Frame 5_2: pos:", np.round(T_4_5_2[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_4_5_2[1, 0], T_4_5_2[0, 0]),
                    np.arctan2(-T_4_5_2[2, 0], np.sqrt(T_4_5_2[2, 1]**2 + T_4_5_2[2, 2])),
                    np.arctan2(T_4_5_2[2, 1], T_4_5_2[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_5_EE = T_4_5_2 @ T_5_EE
            print("Frame EE: pos:", np.round(T_5_EE[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_5_EE[1, 0], T_5_EE[0, 0]),
                    np.arctan2(-T_5_EE[2, 0], np.sqrt(T_5_EE[2, 1]**2 + T_5_EE[2, 2])),
                    np.arctan2(T_5_EE[2, 1], T_5_EE[2, 2])
                ]) * 180 / np.pi, 4))

        return end_effector_position, end_effector_orientation
    
    

    # def inverse_kinematics_(self, target_position, target_orientation):
    #     """
    #     Solves the inverse kinematics problem to find the joint angles that reach the desired target position.

    #     Parameters:
    #         target_position (np.array): Desired position of the end-effector.

    #     Returns:
    #         np.array: The joint angles that achieve the desired end-effector position.
    #     """
    #     # Objective function to minimize: distance between current EE position and target position
    #     def objective_function(joint_angles):
    #         current_position, current_oreientation = self.forward_kinematics(joint_angles)
    #         error = np.linalg.norm(current_position - target_position + current_oreientation - target_orientation)
    #         return error

    #     # Initial guess for the joint angles (current angles)
    #     initial_guess = self.joint_angles
    #     # initial_guess = np.array([0, 0, 0, 0, 0])

    #     # Use a nonlinear optimizer to minimize the error
    #     result = minimize(objective_function, initial_guess, method='BFGS')

    #     # Check if the optimization was successful
    #     if result.success:
    #         optimized_joint_angles = result.x
    #     else:
    #         raise ValueError("Inverse kinematics optimization did not converge")

    #     # # convert the joint angles to the valid range [-3/2 pi, 3/2 pi]
    #     # valid_range_min = -1.5 * np.pi
    #     # valid_range_max = 1.5 * np.pi
    #     # optimized_joint_angles = np.mod(optimized_joint_angles + np.pi, 2 * np.pi) - np.pi
    #     # optimized_joint_angles = np.clip(optimized_joint_angles, valid_range_min, valid_range_max)

    #     print("target_position", target_position)
    #     print("position_from_ik", self.forward_kinematics_(optimized_joint_angles)[0])

    #     return optimized_joint_angles

    from scipy.optimize import minimize

    def inverse_kinematics_(self, target_position, target_orientation):
        """
        Solves the inverse kinematics problem to find the joint angles that reach the desired target position and orientation.

        Parameters:
            target_position (np.array): Desired position of the end-effector.
            target_orientation (np.array): Desired orientation of the end-effector in ZYX Euler angles.

        Returns:
            np.array: The joint angles that achieve the desired end-effector position and orientation.
        """
        # Objective function to minimize: sum of position and orientation errors
        def objective_function(joint_angles):
            current_position, current_orientation = self.forward_kinematics(joint_angles)

            # Calculate the position error
            position_error = np.linalg.norm(current_position - target_position)

            # Calculate the orientation error (assuming orientation is given in Euler angles)
            orientation_error = np.linalg.norm(current_orientation - target_orientation)

            # Combine errors with weighting factor
            orientation_weight = 0.1  # You can adjust this weight based on the importance of orientation
            total_error = position_error + orientation_weight * orientation_error

            return total_error

        # Initial guess for the joint angles (current angles)
        initial_guess = self.joint_angles
        # initial_guess = np.array([0, 0, 0, 0, 0])

        # Define joint limits (assuming [-3/2 * pi, 3/2 * pi] for each joint)
        bounds = [(-1.0 * np.pi, 1.0 * np.pi) for _ in range(len(self.joint_angles))]

        # Use a nonlinear optimizer to minimize the error
        result = minimize(objective_function, initial_guess, method='SLSQP', bounds=bounds)

        # Check if the optimization was successful
        if result.success:
            optimized_joint_angles = result.x
        else:
            raise ValueError("Inverse kinematics optimization did not converge")

        # Print the results for debugging purposes
        print("Target Position:", target_position)
        print("Achieved Position:", self.forward_kinematics_(optimized_joint_angles)[0])

        return optimized_joint_angles

    
    def conversion_from_actuator_to_generalized(self, actuator_positions):
        """
        Converts the actuator positions to the generalized coordinates.

        Parameters:
            actuator_angles (np.array): Array of actuator angles.
        Returns:
            np.array: The corresponding generalized coordinates.

            This is somwhat, what I defined to be zero (this are the motor positions, when they are actually in the middle position)
            leader_pos {'main': tensor([ -6.3281,  89.3848,  89.2969,  -1.4062, -87.9785,  95.1396])}
            goal_pos tensor([ -6.3281,  89.3848,  89.2969,  -1.4062, -87.9785,  95.1396])

        """
        self.joint_angles[0] = actuator_positions[0] * np.pi / 180
        self.joint_angles[1] = actuator_positions[1] * np.pi / 180  - np.pi / 2
        self.joint_angles[2] = actuator_positions[2] * np.pi / 180  - np.pi / 2
        self.joint_angles[3] = actuator_positions[3] * np.pi / 180
        self.joint_angles[4] = actuator_positions[4] * np.pi / 180  + np.pi / 2
        # self.joint_angles[5] = actuator_positions[5] * np.pi / 180 - np.pi / 2
    
    def conversion_from_generalized_to_actuator(self, generalized_coordinates):
        """
        Converts the generalized coordinates to the actuator positions.

        Parameters:
            generalized_coordinates (np.array): Array of generalized coordinates.

        Returns:
            np.array: The corresponding actuator positions.
        """
        actuator_positions = np.zeros(6)
        actuator_positions[0] = generalized_coordinates[0] * 180 / np.pi
        actuator_positions[1] = (generalized_coordinates[1] + np.pi / 2) * 180 / np.pi
        actuator_positions[2] = (generalized_coordinates[2] + np.pi / 2) * 180 / np.pi
        actuator_positions[3] = generalized_coordinates[3] * 180 / np.pi
        actuator_positions[4] = (generalized_coordinates[4] - np.pi / 2) * 180 / np.pi
        # actuator_positions[5] = (generalized_coordinates[5] + np.pi / 2) * 180 / np.pi

        # convert the joint angles to the valid range [-3/2 pi, 3/2 pi]
        valid_range_min = -1.5 * np.pi
        valid_range_max = 1.5 * np.pi
        actuator_positions = np.mod(actuator_positions + 180, 360) - 180
        actuator_positions = np.clip(actuator_positions, valid_range_min * 180 / np.pi, valid_range_max * 180 / np.pi)

        return actuator_positions
    
    def forward_kinematics(self, joint_angles):
        """
        Calculate the forward kinematics to determine the end-effector position and orientation given joint angles.

        Parameters:
            joint_angles (np.array): Motor positions.

        Returns:
            tuple: The position and orientation (rotation matrix) of the end-effector.
        """
        self.conversion_from_actuator_to_generalized(joint_angles)  # write joint angels in class variable
        return self.forward_kinematics_()
    
    def inverse_kinematics(self, target_position, target_orientation):
        """
        Solves the inverse kinematics problem to find the joint angles that reach the desired target position.

        Parameters:
            target_position (np.array): Desired position of the end-effector.

        Returns:
            np.array: The joint angles that achieve the desired end-effector position.
        """
        return self.conversion_from_generalized_to_actuator(self.inverse_kinematics_(target_position, target_orientation))

# Example usage
if __name__ == "__main__":
    ik_solver = InverseKinematicsso100()
    target_pos = np.array([0.1, 0.1, 0.3])  # Example target position
    try:
        joint_angles_solution = ik_solver.inverse_kinematics(target_pos)
        print("Joint angles to reach the target position:", joint_angles_solution)
    except ValueError as e:
        print(e)
