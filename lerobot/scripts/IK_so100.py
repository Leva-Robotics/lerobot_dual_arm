from ikpy.chain import Chain
import numpy as np
import torch

from ikpy.link import OriginLink, URDFLink

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
# from ikpy.utils.plot import plot_chain

class So100Kin:
    def __init__(self):
        """
        Initializes the RobotArmKinematics class by loading the robot's URDF file.
        
        :param urdf_file_path: Path to the URDF file of the robot arm.
        """
        # # self.urdf_file_path = "/local/home/kjanssen/lerobot/lerobot/common/robot_devices/robots/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/urdf/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf"
        # self.urdf_file_path = "/local/home/kjanssen/lerobot/lerobot/common/robot_devices/robots/URDF/SO_5DOF_ARM100_05d.SLDASM/urdf/SO_5DOF_ARM100_05d.SLDASM.urdf"
        
        # # Load the robot chain with specified base and end-effector links
        # self.robot_chain = Chain.from_urdf_file(
        #     self.urdf_file_path,
        #     base_elements=['Base'],             # Specify the base link from your URDF
        #     # last_link_elements=['Moving Jaw'],  # Specify the end-effector link from your URDF
        #     base_element_type='link'
        # )
        # self.robot_chain.active_links_mask[0] = False
        # self.robot_chain.active_links_mask[-1] = False
        
        # Define the chain
        self.robot_chain = Chain(name='so100_arm', links=[
            OriginLink(),  # Base link

            # Joint 1: Shoulder
            URDFLink(
                name="shoulder",
                origin_translation=[0.0729, 0.0, 0.0306],  # vec_1 in meters
                origin_orientation=[0, np.pi/2, 0],        # rpy: [roll, pitch, yaw] in radians
                rotation=[0, 1, 0],                        # Rotate around Y-axis
            ),

            # Joint 2: Elbow
            URDFLink(
                name="elbow",
                origin_translation=[0.0, 0.028, 0.11257],  # vec_2 in meters
                origin_orientation=[0, 0, -np.pi/2],       # rpy
                rotation=[0, 0, 1],                        # Rotate around Z-axis
            ),

            # Joint 3: Wrist Pitch
            URDFLink(
                name="wrist_pitch",
                origin_translation=[0.0, -0.0052, 0.1349], # vec_3 in meters
                origin_orientation=[0, 0, 0],              # rpy
                rotation=[0, 0, 1],                        # Rotate around Z-axis
            ),

            # Joint 4: Wrist Yaw
            URDFLink(
                name="wrist_yaw",
                origin_translation=[0.0, 0.0, 0.0424],    # vec_4 in meters
                origin_orientation=[0, -np.pi/2, 0],      # rpy
                rotation=[1, 0, 0],                        # Rotate around X-axis
            ),

            # Joint 5: End Effector
            URDFLink(
                name="end_effector",
                origin_translation=[0.0, 0.0, 0.0545],    # vec_5 in meters
                origin_orientation=[0, 0, 0],              # rpy
                rotation=[0, 0, 1],                        # Rotate around Z-axis (if applicable)
            ),
        ])

        # Optionally, you can set joint limits here if they are not defined in the URDF
        # For example:
        # for link in self.robot_chain.links:
        #     if link.name == 'some_joint_name':
        #         link.bounds = (-np.pi, np.pi)

    def inverse_kinematics(self, target_position, initial_position=None):
        """
        Computes the inverse kinematics for a given target position.
        
        :param target_position: A list or array of the target [x, y, z] coordinates.
        :param initial_position: (Optional) Initial guess for the joint angles.
        :return: An array of joint angles that achieve the target position.
        """
        joint_angles = self.robot_chain.inverse_kinematics(
            target_position,
            initial_position=initial_position
        )
        return joint_angles

    def forward_kinematics(self, joint_angles):
        """
        Computes the forward kinematics for a given set of joint angles.
        Also visualizes the robot arm configuration.

        :param joint_angles: A list or array of joint angles.
        :return: A 4x4 transformation matrix representing the end-effector pose.
        """

        # Ensure joint_angles is a list
        if isinstance(joint_angles, torch.Tensor):
            joint_angles = joint_angles.tolist()
        # joint_angles = [0] + joint_angles  # Prepend zero for the fixed base link

        # Compute the forward kinematics
        end_effector_frame = self.robot_chain.forward_kinematics(joint_angles)

        # Print end-effector position and orientation
        position = end_effector_frame[:3, 3]
        rotation_matrix = end_effector_frame[:3, :3]
        roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

        print("End-Effector Position (meters): x={:.4f}, y={:.4f}, z={:.4f}".format(*position))
        print("End-Effector Orientation (degrees): roll={:.2f}, pitch={:.2f}, yaw={:.2f}".format(
            np.degrees(roll), np.degrees(pitch), np.degrees(yaw)))

        # Visualize the robot arm configuration
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        self.robot_chain.plot(joint_angles, ax, target=None, show=False)

        # Set plot limits (adjust these based on your robot's dimensions)
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(0, 0.8)

        # Label axes
        ax.set_xlabel('X axis (meters)')
        ax.set_ylabel('Y axis (meters)')
        ax.set_zlabel('Z axis (meters)')

        # Show the plot
        plt.show()

        return end_effector_frame
    
    def forward_kinematics_(self, joint_angles):
        """
        Computes the forward kinematics for a given set of joint angles.
        Prints the position (x, y, z) and orientation (Euler angles) of every frame relative to the base frame.
        Also visualizes the robot arm configuration.

        :param joint_angles: A list or array of joint angles (in radians).
        :return: A list of transformation matrices representing each frame's pose relative to the base frame.
        """
        # Ensure joint_angles is a list
        if isinstance(joint_angles, torch.Tensor):
            joint_angles = joint_angles.tolist()

        joint_angles = self.conversion_from_actuator_to_generalized(joint_angles)
        
        # Prepend zero for the fixed base link if necessary
        # Uncomment the following line if your chain requires it
        # joint_angles = [0] + joint_angles  
        
        # Compute the forward kinematics for the entire chain
        # ikpy's forward_kinematics method can provide all intermediate frames if full_kinematics=True
        # However, depending on the ikpy version, this might not be supported. If not, compute manually.
        try:
            # Attempt to get full kinematics (list of transformation matrices)
            frames = self.robot_chain.forward_kinematics(joint_angles, full_kinematics=True)
        except TypeError:
            # If full_kinematics is not supported, compute manually
            frames = []
            cumulative_transform = np.eye(4)
            for i, link in enumerate(self.robot_chain.links):
                # Get joint angle for the current link (skip OriginLink which has no joint)
                if i == 0:
                    # OriginLink
                    joint_angle = 0
                else:
                    if i-1 < len(joint_angles):
                        joint_angle = joint_angles[i-1]
                    else:
                        joint_angle = 0  # Default to 0 if not enough joint angles provided
                
                # Get the transformation matrix for the current link
                transform = link.forward_kinematics(joint_angle)
                cumulative_transform = cumulative_transform @ transform
                frames.append(cumulative_transform.copy())
        
        # If full_kinematics=True worked, 'frames' is a list of transformation matrices
        # Otherwise, 'frames' has been manually computed
        
        # Iterate through each frame and print position and orientation
        for idx, frame in enumerate(frames):
            position = frame[:3, 3]
            rotation_matrix = frame[:3, :3]
            roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
            
            print(f"Frame {idx}:")
            print(f"  Position (meters): x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}")
            print(f"  Orientation (degrees): roll={np.degrees(roll):.2f}, pitch={np.degrees(pitch):.2f}, yaw={np.degrees(yaw):.2f}")
            print()
        
        # Print end-effector details (last frame)
        end_effector_frame = frames[-1]
        end_position = end_effector_frame[:3, 3]
        end_rotation_matrix = end_effector_frame[:3, :3]
        end_roll, end_pitch, end_yaw = rotation_matrix_to_euler_angles(end_rotation_matrix)
        
        print("End-Effector Position (meters): x={:.4f}, y={:.4f}, z={:.4f}".format(*end_position))
        print("End-Effector Orientation (degrees): roll={:.2f}, pitch={:.2f}, yaw={:.2f}".format(
            np.degrees(end_roll), np.degrees(end_pitch), np.degrees(end_yaw)))
        
        # Visualize the robot arm configuration
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # plot_chain(self.robot_chain, joint_angles, ax, target=None, show=False)
        
        # Set plot limits (adjust these based on your robot's dimensions)
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(0, 0.8)
        
        # Label axes
        ax.set_xlabel('X axis (meters)')
        ax.set_ylabel('Y axis (meters)')
        ax.set_zlabel('Z axis (meters)')
        
        # Show the plot
        # plt.show()
        
        return frames  # Return all frames for further processing if needed


    def get_end_effector_position(self, joint_angles):
        """
        Extracts the end-effector position from the forward kinematics.
        
        :param joint_angles: A list or array of joint angles.
        :return: A list of [x, y, z] coordinates of the end-effector.
        """
        end_effector_frame = self.forward_kinematics(joint_angles)
        position = end_effector_frame[:3, 3]
        return position.tolist()

    def get_joint_limits(self):
        """
        Retrieves the joint limits of the robot arm.
        
        :return: A list of tuples representing the (lower, upper) bounds for each joint.
        """
        joint_limits = [link.bounds for link in self.robot_chain.links[1:-1]]
        return joint_limits

    # def visualize(self, joint_angles):
    #     """
    #     Visualizes the robot arm using matplotlib.
        
    #     :param joint_angles: A list or array of joint angles.
    #     """


    #     fig, ax = plot_chain(self.robot_chain, joint_angles, show=False)
    #     ax.set_xlim(-0.5, 0.5)
    #     ax.set_ylim(-0.5, 0.5)
    #     plt.show()

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
        joint_angles = np.zeros(6)
        joint_angles[0] = actuator_positions[0] * np.pi / 180
        joint_angles[1] = actuator_positions[1] * np.pi / 180  - np.pi / 2
        joint_angles[2] = actuator_positions[2] * np.pi / 180  - np.pi / 2
        joint_angles[3] = actuator_positions[3] * np.pi / 180
        joint_angles[4] = actuator_positions[4] * np.pi / 180  + np.pi / 2
        joint_angles[5] = actuator_positions[5] * np.pi / 180
        return joint_angles
        # self.joint_angles[5] = actuator_positions[5] * np.pi / 180 - np.pi / 2

def rotation_matrix_to_euler_angles(R):
    """
    Converts a rotation matrix to Euler angles (roll, pitch, yaw).
    Assumes the rotation matrix is in the form of R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    import numpy as np

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock case
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw