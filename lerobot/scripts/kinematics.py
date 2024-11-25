import numpy as np
from scipy.optimize import minimize

class InverseKinematicsso100:
    def __init__(self, joint_count):
        """
        Initializes the robotic arm with a given number of joints.

        Parameters:
            joint_count (int): Number of joints in the robotic arm.
            zyx Tait_Bryan angles
            assume joint angeles to be 0 when in middle position
        """
        self.joint_count = 6    # sixed joint is the gripper

        self.joint_angles = np.zeros(joint_count)   # generalized coordinates

        # joint_1_to_joint_2 
        self.vec_1 = np.array([0.07, 0, 0.05])  # translatorial vector from joint 1 to joint 2, when joint at 0
        self.rot_1_to_2 = np.array([1/2  * np.pi, self.joint_angles[0], 1/2  * np.pi])
        self.trans_1_to_2 = np.array([self.vec_1[0], -self.vec_1[2] * np.sin(self.joint_angles[0]), self.vec_1[2] * np.cos(self.joint_angles[0])]) # in one frame

        # joint_2_to_joint_3
        self.vec_2 = np.array([0, 0.02, 0.15])  
        self.rot_2_to_3 = np.array([0, 0, self.joint_angles[1] - np.pi/2])  # zyx Tait_Bryan angles order of rotations in angle -> first number is roatation around z ...
        self.trans_2_to_3 = np.array([0, self.vec_2[0] * np.cos(self.joint_angles[1]) - self.vec_2[2] * np.sin(self.joint_angles[1]), self.vec_2[1] * np.sin(self.joint_angles[1]) + self.vec_2[2] * np.cos(self.joint_angles[1])]) 

        # joint_3_to_joint_4
        self.vec_3 = np.array([0.0, 0.0, 0.18])
        self.rot_3_to_4 = np.array([0, 0, self.joint_angles[2]])
        self.trans_3_to_4 = np.array([0, - self.vec_3[2] * np.sin(self.joint_angles[2]) , self.vec_3[2] * np.cos(self.joint_angles[2])])

        # joint_4_to_joint_5
        self.vec_4 = np.array([0.0, 0.0, 0.05])
        self.rot_4_to_5_1 = np.array([0.0, -1/2 * np.py, 0.0])  # Gimbal lock, use second rotation to depic the motor rotation
        self.rot_4_to_5_2 = np.array([self.joint_angles[3], 0.0, 0.0])  # both rotations follwo the zyx Tait_Bryan angles [z_angel, y_angle, x_angle]

        # joint_5_to_EE
        self.vec_5 = np.array([0.0, 0.0, 0.1])
        self.rot_5_to_EE = np.array([0.0, 0.0, self.joint_angles[4]])

        # joint_6 is the gripper

        # Assume all links are of length 1 for simplicity; can be customized

    def set_link_lengths(self, link_lengths):
        """
        Set the lengths of each link in the robot arm.

        Parameters:
            link_lengths (list or np.array): Lengths of each link.
        """
        assert len(link_lengths) == self.joint_count, "Number of link lengths must match joint count."
        self.link_lengths = np.array(link_lengths)

    def forward_kinematics(self, joint_angles):
        """
        Computes the end-effector position based on given joint angles.

        Parameters:
            joint_angles (list or np.array): Angles of each joint.

        Returns:
            np.array: The x, y, z position of the end-effector.
        """
        x, y = 0, 0
        total_angle = 0
        for i in range(self.joint_count):
            total_angle += joint_angles[i]
            x += self.link_lengths[i] * np.cos(total_angle)
            y += self.link_lengths[i] * np.sin(total_angle)
        return np.array([x, y])

    def inverse_kinematics(self, target_position):
        """
        Computes the joint angles to reach the target position.

        Parameters:
            target_position (list or np.array): The target x, y position of the end-effector.

        Returns:
            np.array: The joint angles required to reach the target position.
        """
        def objective_function(joint_angles):
            current_position = self.forward_kinematics(joint_angles)
            return np.linalg.norm(current_position - target_position)

        result = minimize(objective_function, self.joint_angles, bounds=[(-np.pi, np.pi)]*self.joint_count)
        if result.success:
            self.joint_angles = result.x
            return self.joint_angles
        else:
            raise ValueError("Inverse kinematics optimization failed to find a solution.")

    def get_motor_commands(self):
        """
        Converts the joint angles to motor position commands (assumed linear scaling).

        Returns:
            list: Motor position commands for each joint.
        """
        motor_commands = self.joint_angles  # Assuming motor commands directly match joint angles for simplicity
        return motor_commands

# Example usage
if __name__ == "__main__":
    # Create an instance of the IK class with 3 joints
    arm = InverseKinematics(joint_count=3)
    arm.set_link_lengths([1.0, 1.0, 1.0])

    # Define a target position for the end-effector
    target = [1.5, 1.5]

    # Calculate joint angles to reach the target position
    try:
        joint_angles = arm.inverse_kinematics(target)
        print("Joint Angles:", joint_angles)
        motor_commands = arm.get_motor_commands()
        print("Motor Commands:", motor_commands)
    except ValueError as e:
        print("Error:", e)





import numpy as np
from scipy.optimize import minimize

class InverseKinematicsso100:
    def __init__(self, joint_count):
        """
        Initializes the robotic arm with a given number of joints.

        Parameters:
            joint_count (int): Number of joints in the robotic arm.
            zyx Tait-Bryan angles
            assume joint angles to be 0 when in middle position
        """
        self.joint_count = joint_count
        self.joint_angles = np.zeros(joint_count)  # generalized coordinates

        # Define link vectors and initial transformations between joints
        self.vec_1 = np.array([0.07, 0, 0.05])
        self.vec_2 = np.array([0, 0.02, 0.15])
        self.vec_3 = np.array([0.0, 0.0, 0.18])
        self.vec_4 = np.array([0.0, 0.0, 0.05])
        self.vec_5 = np.array([0.0, 0.0, 0.1])

    def forward_kinematics(self, joint_angles):
        """
        Calculate the forward kinematics to determine the end-effector position given joint angles.

        Parameters:
            joint_angles (np.array): Array of joint angles.

        Returns:
            np.array: The position of the end-effector.
        """
        # Update joint angles
        self.joint_angles = joint_angles

        # Transformation from joint 1 to end-effector (EE)
        trans_1_to_2 = np.array([0, -self.vec_1[2] * np.sin(self.joint_angles[0]), self.vec_1[2] * np.cos(self.joint_angles[0])])
        trans_2_to_3 = np.array([0, self.vec_2[0] * np.cos(self.joint_angles[1]) - self.vec_2[2] * np.sin(self.joint_angles[1]),
                                              self.vec_2[1] * np.sin(self.joint_angles[1]) + self.vec_2[2] * np.cos(self.joint_angles[1])])
        trans_3_to_4 = self.vec_3 + np.array([0, -self.vec_3[2] * np.sin(self.joint_angles[2]), self.vec_3[2] * np.cos(self.joint_angles[2])])
        trans_4_to_5 = self.vec_4  # Simplified for gimbal lock scenario
        trans_5_to_EE = self.vec_5

        # Calculate the final end-effector position by summing the transformations
        end_effector_position = trans_1_to_2 + trans_2_to_3 + trans_3_to_4 + trans_4_to_5 + trans_5_to_EE

        return end_effector_position

    def inverse_kinematics(self, target_position):
        """
        Solves the inverse kinematics problem to find the joint angles that reach the desired target position.

        Parameters:
            target_position (np.array): Desired position of the end-effector.

        Returns:
            np.array: The joint angles that achieve the desired end-effector position.
        """
        # Objective function to minimize: distance between current EE position and target position
        def objective_function(joint_angles):
            current_position = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(current_position - target_position)
            return error

        # Initial guess for the joint angles (current angles)
        initial_guess = self.joint_angles

        # Use a nonlinear optimizer to minimize the error
        result = minimize(objective_function, initial_guess, method='BFGS')

        # Check if the optimization was successful
        if result.success:
            optimized_joint_angles = result.x
        else:
            raise ValueError("Inverse kinematics optimization did not converge")

        return optimized_joint_angles

# Example usage
if __name__ == "__main__":
    ik_solver = InverseKinematicsso100(joint_count=6)
    target_pos = np.array([0.1, 0.1, 0.3])  # Example target position
    try:
        joint_angles_solution = ik_solver.inverse_kinematics(target_pos)
        print("Joint angles to reach the target position:", joint_angles_solution)
    except ValueError as e:
        print(e)
