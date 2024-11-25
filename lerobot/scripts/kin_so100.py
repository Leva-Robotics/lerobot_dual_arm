import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
from numba import njit

class InverseKinematicsso100:
    def __init__(self):
        """
        Initializes the robotic arm with a given number of joints.
        """
        # self.joint_angles = np.zeros(5)  # generalized coordinates
        self.joint_angles = [-0.03681554, 1.71345651, -1.49102962, -0.79460204,  1.57386434, 1/4 * np.pi]  # generalized coordinates

        # Define link vectors and initial transformations between joints
        self.vec_0 = np.array([0.0, 0.0, 0.05]) * 1000
        self.vec_1 = np.array([0.0729, 0.0, 0.0306]) * 1000
        self.vec_2 = np.array([0.0, 0.028, 0.11257]) * 1000
        self.vec_3 = np.array([0.0, -0.0052, 0.1349]) * 1000
        self.vec_4 = np.array([0.0, 0.0, 0.0424]) * 1000
        self.vec_5 = np.array([0.0, 0.0, 0.0545]) * 1000

    # def rotation_matrix_zyx(self, angles):
    #     """
    #     Computes the rotation matrix given ZYX Tait-Bryan angles.
    #     """
    #     z, y, x = angles
    #     Rz = np.array([[np.cos(z), -np.sin(z), 0],
    #                    [np.sin(z),  np.cos(z), 0],
    #                    [0,          0,         1]])
    #     Ry = np.array([[np.cos(y),  0, np.sin(y)],
    #                    [0,          1,          0],
    #                    [-np.sin(y), 0, np.cos(y)]])
    #     Rx = np.array([[1, 0,          0         ],
    #                    [0, np.cos(x), -np.sin(x)],
    #                    [0, np.sin(x),  np.cos(x)]])
    #     return Rz @ Ry @ Rx

    def rotation_matrix_zyx(self, angles):
        """
        Computes the rotation matrix given ZYX Tait-Bryan angles using optimized operations.
        """
        z, y, x = angles

        cz = np.cos(z)
        sz = np.sin(z)
        cy = np.cos(y)
        sy = np.sin(y)
        cx = np.cos(x)
        sx = np.sin(x)

        # Directly compute the rotation matrix without intermediate matrices
        R = np.empty((3, 3), dtype=np.float64)
        R[0, 0] = cz * cy
        R[0, 1] = cz * sy * sx - sz * cx
        R[0, 2] = cz * sy * cx + sz * sx

        R[1, 0] = sz * cy
        R[1, 1] = sz * sy * sx + cz * cx
        R[1, 2] = sz * sy * cx - cz * sx

        R[2, 0] = -sy
        R[2, 1] = cy * sx
        R[2, 2] = cy * cx

        return R

    # @njit
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

         # print joint positions
        # print("Angles:", self.joint_angles)

        T_I_1 = np.eye(4)
        T_I_1[:3, :3] = self.rotation_matrix_zyx([-1/2 * np.pi + self.joint_angles[0], -1/2 * np.pi, 0])
        T_I_1[:3, 3] = self.vec_0

        T_1_2 = np.eye(4)
        T_1_2[:3, :3] = self.rotation_matrix_zyx([1/2  * np.pi, 0, 1/2  * np.pi + self.joint_angles[1]])
        # T_1_2[:3, 3] = np.array([self.vec_1[0], 
        #                          -self.vec_1[2] * np.sin(self.joint_angles[0]), 
        #                          self.vec_1[2] * np.cos(self.joint_angles[0])])
        T_1_2[:3, 3] = self.vec_1

        T_2_3 = np.eye(4)
        T_2_3[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[2] - np.pi / 2])
        # T_2_3[:3, 3] = np.array([0, 
        #                          self.vec_2[0] * np.cos(self.joint_angles[1]) - self.vec_2[2] * np.sin(self.joint_angles[1]), 
        #                          self.vec_2[1] * np.sin(self.joint_angles[1]) + self.vec_2[2] * np.cos(self.joint_angles[1])])
        T_2_3[:3, 3] = self.vec_2

        T_3_4 = np.eye(4)
        T_3_4[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[3]])
        # T_3_4[:3, 3] = np.array([0, 
        #                          -self.vec_3[2] * np.sin(self.joint_angles[2]) + self.vec_3[1] * np.cos(self.joint_angles[2]), 
        #                          self.vec_3[2] * np.cos(self.joint_angles[2]) + self.vec_3[1] * np.sin(self.joint_angles[2])])
        T_3_4[:3, 3] = self.vec_3

        T_4_5 = np.eye(4)
        T_4_5[:3, :3] = self.rotation_matrix_zyx([0, -np.pi/2, self.joint_angles[4]])
        T_4_5[:3, 3] = self.vec_4

        # T_4_5_1 = np.eye(4)
        # T_4_5_1[:3, :3] = self.rotation_matrix_zyx([0, -np.pi/2, 0])
        # T_4_5_1[:3, 3] = self.vec_4

        # T_4_5_2 = np.eye(4)
        # T_4_5_2[:3, :3] = self.rotation_matrix_zyx([self.joint_angles[3], 0, 0])
        # # T_4_5_2[:3, 3] = np.array([self.vec_4[2] * np.cos(self.joint_angles[3]),
        # #                            self.vec_4[2] * np.sin(self.joint_angles[3]),
        # #                            0])
        # T_4_5_2[:3, 3] = self.vec_4

        # T_4_5 = T_4_5_1 @ T_4_5_2

        T_5_EE = np.eye(4)
        # T_5_EE[:3, :3] = self.rotation_matrix_zyx([0, 0, self.joint_angles[4]])
        T_5_EE[:3, 3] = self.vec_5

        # Compute the final transformation matrix from the base to the end-effector
        T_final = T_I_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_EE

        # Extract end-effector position and orientation
        end_effector_position = T_final[:3, 3]
        end_effector_orientation = T_final[:3, :3]

        # convert orientation into euler angles, zyx
        z = np.arctan2(T_final[1, 0], T_final[0, 0])
        y = np.arctan2(-T_final[2, 0], np.sqrt(T_final[2, 1]**2 + T_final[2, 2]**2))
        x = np.arctan2(T_final[2, 1], T_final[2, 2])
        end_effector_orientation = np.array([z, y, x]) * 180 / np.pi

        if debug:
            # # print all transformations
            # print("T_1_2:", np.round(T_1_2, 4))
            # print("T_2_3:", np.round(T_2_3, 4))
            # print("T_3_4:", np.round(T_3_4, 4))
            # print("T_4_5_1:", np.round(T_4_5_1, 4))
            # print("T_4_5_2:", np.round(T_4_5_2, 4))
            print("T_5_EE:", np.round(T_5_EE, 4))
            # print("T_final:", np.round(T_final, 4))
            # print("Joint angles:", np.round(self.joint_angles * 180 / np.pi, 4))"
            
            print("angles:", np.round(self.joint_angles * 180 / np.pi, 4))

            print("Frame 1: pos:", np.round(T_I_1[:3, 3], 4),
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_I_1[1, 0], T_I_1[0, 0]),
                    np.arctan2(-T_I_1[2, 0], np.sqrt(T_I_1[2, 1]**2 + T_I_1[2, 2]**2)),
                    np.arctan2(T_I_1[2, 1], T_I_1[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_1_2 = T_I_1 @ T_1_2
            
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

            T_4_5 = T_3_4 @ T_4_5
            print("Frame 5: pos:", np.round(T_4_5[:3, 3], 4),
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_4_5[1, 0], T_4_5[0, 0]),
                    np.arctan2(-T_4_5[2, 0], np.sqrt(T_4_5[2, 1]**2 + T_4_5[2, 2]**2)),
                    np.arctan2(T_4_5[2, 1], T_4_5[2, 2])
                ]) * 180 / np.pi, 4))
            
            T_5_EE = T_4_5 @ T_5_EE
            print("Frame EE: pos:", np.round(T_5_EE[:3, 3], 4), 
                ", rot (zyx):", np.round(np.array([
                    np.arctan2(T_5_EE[1, 0], T_5_EE[0, 0]),
                    np.arctan2(-T_5_EE[2, 0], np.sqrt(T_5_EE[2, 1]**2 + T_5_EE[2, 2])),
                    np.arctan2(T_5_EE[2, 1], T_5_EE[2, 2])
                ]) * 180 / np.pi, 4))

        return end_effector_position, end_effector_orientation
    

    def inverse_kinematics_(self, target_position, target_orientation):
        """
        Solves the inverse kinematics problem to find the joint angles that reach the desired target position and orientation.
        """
        
        def objective_function(joint_angles):
            current_position, current_orientation = self.forward_kinematics_(joint_angles)
            error = np.concatenate((current_position - target_position, 0.1 * (current_orientation - target_orientation)))
            return np.linalg.norm(error)

        initial_guess = self.joint_angles
        bounds = [(-3/5 * np.pi, 3/5 * np.pi) for _ in range(len(self.joint_angles))]
        # result = minimize(objective_function, initial_guess, method='SLSQP', bounds=bounds)
        result = minimize(
            objective_function,
            initial_guess,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-4}
        )

        if result.success:
            optimized_joint_angles = result.x
        else:
            raise ValueError("Inverse kinematics optimization did not converge")
        
        # print target and result
        # print("target_pos", target_position, "target_orientation", target_orientation)
        # print("result_pos", self.forward_kinematics_(result.x))

        # print("initial_joint_angles", initial_guess)
        # print("optimized_joint_angles", optimized_joint_angles)

        # append a pi/4 entry 
        optimized_joint_angles = np.append(optimized_joint_angles, np.pi / 4)
        self.joint_angles = optimized_joint_angles
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
        # print("befor_conversion", actuator_positions)
        self.joint_angles[0] = (actuator_positions[0] * np.pi / 180) * (-1)
        self.joint_angles[1] = actuator_positions[1] * np.pi / 180  - np.pi / 2
        self.joint_angles[2] = (actuator_positions[2] * np.pi / 180  - np.pi / 2) * (-1)
        self.joint_angles[3] = (actuator_positions[3] * np.pi / 180) * (-1)
        self.joint_angles[4] = actuator_positions[4] * np.pi / 180  + np.pi / 2
        # self.joint_angles[5] = actuator_positions[5] * np.pi / 180
        # print("after_conversion", self.joint_angles)
    
    def conversion_from_generalized_to_actuator(self, generalized_positions):
        """
        Converts the generalized coordinates to the actuator positions.

        Parameters:
            generalized_positions (np.array): Array of generalized coordinates.
        Returns:
            np.array: The corresponding actuator positions.
        """
        actuator_positions = np.zeros(6)
        actuator_positions[0] = (generalized_positions[0] * (-1) * 180 / np.pi)
        actuator_positions[1] = (generalized_positions[1] + np.pi / 2) * 180 / np.pi
        actuator_positions[2] = (generalized_positions[2] * (-1) + np.pi / 2) * 180 / np.pi
        actuator_positions[3] = (generalized_positions[3] * (-1)) * 180 / np.pi
        actuator_positions[4] = (generalized_positions[4] - np.pi / 2) * 180 / np.pi
        actuator_positions[5] = generalized_positions[5] * 180 / np.pi
        return actuator_positions
    
    def forward_kinematics(self, joint_angles):
        self.conversion_from_actuator_to_generalized(joint_angles)
        return self.forward_kinematics_()
    
    def inverse_kinematics(self, target_position, target_orientation):
        return self.conversion_from_generalized_to_actuator(self.inverse_kinematics_(target_position, target_orientation))

# Example usage
if __name__ == "__main__":
    ik_solver = InverseKinematicsso100()
    target_pos = np.array([0.1, 0.1, 0.3])
    target_orientation = R.from_euler('zyx', [0.1, 0.2, 0.3]).as_quat()
    try:
        joint_angles_solution = ik_solver.inverse_kinematics(target_pos, target_orientation)
        print("Joint angles to reach the target position:", joint_angles_solution)
    except ValueError as e:
        print(e)
