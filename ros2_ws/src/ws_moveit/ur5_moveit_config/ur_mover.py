"""
Description:
Helper class to move UR using MoveIt. Is supposed to make using moveit easier
by providing a simple movement function for the endeffector.

Features:
    -Set absolute position/rotation for endeffector
    -Set relative position/rotation for endeffector 
    -Get endeffector position/rotation 

Usage: 
    -Import file into your skript, create class object and use its methods
    -In order to function the "tf" topic must exist (created by starting UR drivers. See readme "Usage")
    -Should be run in nix terminal using ros2-rolling (check with "echo $ROS_DISTRO"
)

Changelog:
    -09.08.2024, Philip Hotz: Initial
    23.09.2025, Valentyn Petrichenko: Adding relative rotations about all axes, changing of UR_EE_pose Dataclass

TODO:
    -Function descriptions
"""


### IMPORTS ###
from pathlib import Path
import os
import time
from typing import Dict, List
from dataclasses import dataclass
import numpy as np

from moveit import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_link_constraint
from moveit.planning import PlanRequestParameters

import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@dataclass
class UR_EE_pose():
    """Class that stores UR endeffector position and rotation"""
    x: float = 0  # x-position [m]
    y: float = 0  # y-position [m]
    z: float = 0  # z-position [m]

    # Quaternion components 
    q1: float = 0  # x
    q2: float = 0  # y
    q3: float = 0  # z
    q4: float = 0  # w

    def __str__(self) -> str:
        return f"x={self.x}, y={self.y}, z={self.z}, q1={self.q1}, q2={self.q2}, q3={self.q3}, q4={self.q4}"

    def get_translation(self) -> List:
        """x/y/z as list."""
        return [self.x, self.y, self.z]

    def get_orientation(self) -> List:
        """Quat as list."""
        return [self.q1, self.q2, self.q3, self.q4]
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (in degrees) to a quaternion."""
        roll, pitch, yaw = np.radians(roll), np.radians(pitch), np.radians(yaw)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]
    
    @staticmethod
    def quaternion_multiply(q1, q2):
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return [w, x, y, z]

    def apply_relative_translation(self, rel_pose):
        """Apply relative translation to the current pose."""
        self.x += rel_pose.x
        self.y += rel_pose.y
        self.z += rel_pose.z

    def apply_relative_rotation(self, angle_x=0.0, angle_y=0.0, angle_z=0.0):
        """Apply relative rotation to the current pose based on Euler angles."""
        if angle_x or angle_y or angle_z:
            q_rel = UR_EE_pose.euler_to_quaternion(angle_x, angle_y, angle_z)
            q_current = [self.q4, self.q1, self.q2, self.q3]  # [w, x, y, z]
            q_new = UR_EE_pose.quaternion_multiply(q_current, q_rel)
            self.q4, self.q1, self.q2, self.q3 = q_new  # Update the quaternion
    
class Simple_Moveit(Node):
    def __init__(self) -> None:
        super().__init__("ur_mover")
        self.logger = get_logger("Simple MoveIt Logger")

        # Instantiate MoveItPy instance and get planning component
        config= self.configsbuilder()
        config['publish_robot_description'] =  True
        config['publish_robot_description_semantic']  = True
        
        self.ur = MoveItPy(node_name="moveit_py", config_dict = config)
        self.ur_manipulator = self.ur.get_planning_component("ur_manipulator")
        self.logger.info("MoveItPy instance created!")

        ### Create UR joint state listener ###
        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'wrist_3_link') 
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    def move_absolute(self, pose: UR_EE_pose, cartesian_pose_tolerance=0.001, orientation_tolerance=0.001) -> None:
        """
        Moves UR endeffector to an absolute position in its coordinate frame
        Input:
            -position: [x, y, z] (position in [m])
            -rotation: [q0, q1, q2, q3] Rotation in quaternion representation
            -cartesian_pose_tolerance: (float) tolerance in m for the planning process
            -orientation_tolerance: (float) tolerance for orientation. Possibly radian for euler angles? Not documented.
        """
        self.get_logger().info(f"Moving to absolute pose: {pose}")
        #print(f"Moving to absolute pose: {pose}")
        # set plan start state to current state
        self.ur_manipulator.set_start_state_to_current_state()
              
        # we use a link constraint instead of a pose msg because it allows to set tolerances.
        # see https://github.com/moveit/moveit2/blob/94e84a17d8551b43fa4011a6cc9c7fbd1ea6c70d/moveit_py/src/moveit/moveit_core/kinematic_constraints/utils.cpp#L46
        link_constraint = construct_link_constraint("tool0",
                                                    "base_link",
                                                    cartesian_position=pose.get_translation(),
                                                    cartesian_position_tolerance=0.001,
                                                    orientation=pose.get_orientation(),
                                                    orientation_tolerance=0.001
                                                    )
        
        self.ur_manipulator.set_goal_state(motion_plan_constraints=[link_constraint])
        
        # plan to goal
        self.plan_and_execute(self.ur, self.ur_manipulator, self.logger, sleep_time=0.1)
        # self.logger.info("Done!")
        print("Done!")


    def move_relative(self, rel_pose: UR_EE_pose = UR_EE_pose(), angle_x=0.0, angle_y=0.0, angle_z=0.0) -> UR_EE_pose:
        """Moves UR endeffector relative to current position with optional rotation.
        
        Note that translation is taken from rel_pose, and the relative orientation taken from the optional angle_* args."""

        self.get_logger().info(f"Starting relative move: {rel_pose}, angles: x={angle_x}, y={angle_y}, z={angle_z}")

        # Get the current pose
        pose = self.get_EE_pose()
        self.get_logger().info(f"Start pose: {pose}")

        # Apply relative translation
        pose.apply_relative_translation(rel_pose)

        # Apply relative rotation
        pose.apply_relative_rotation(angle_x, angle_y, angle_z)

        # Move to the new pose
        self.move_absolute(pose)
        return pose


    def get_EE_pose(self) -> UR_EE_pose:
        """Get and return UR endeffector pose"""
        # Frame names used to compute transformations
        from_frame_rel = 'tool0'
        to_frame_rel = 'base_link'

        # Hacky way to work through callback queue since node is not always spun 
        # Do not make too small or result of lookup_transform will be inprecise (@10 iterations up to 0.07 offset on single move)
        # Gets waaaay worse on chained movements
        for i in range(100):
            rclpy.spin_once(self)

        # Look up for the transformation between target_frame and turtle2 frames
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        pose = UR_EE_pose()
        pose.x  = trans._transform._translation._x
        pose.y  = trans._transform._translation._y
        pose.z  = trans._transform._translation._z
        pose.q1 = trans._transform.rotation._x
        pose.q2 = trans._transform.rotation._y
        pose.q3 = trans._transform.rotation._z
        pose.q4 = trans._transform.rotation._w
        return pose
    
    def configsbuilder(self) -> Dict:
        """Helper method to init moveitpy"""
        print("##################################")
        yaml_file = os.path.dirname(__file__)+"/config/ur_moveit_config.yaml"
        print(yaml_file)
        print("##################################")
        moveit_configs = ( MoveItConfigsBuilder("ur") 
                        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name":"ur10", "ur_type": "ur10"})
                        .moveit_cpp(yaml_file)
                        )
        return moveit_configs.to_dict()
        
    def plan_and_execute(self,robot,planning_component,logger,sleep_time=0.0):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        plan_result = planning_component.plan(PlanRequestParameters(robot, "pilz_lin"))

        
        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])  #, blocking=True
        else:
            logger.error("Planning failed")

        time.sleep(sleep_time)

def main():
    """Usage example"""
    rclpy.init()
    #rclpy.logging.set_logger_level("Simple MoveIt Logger", 30)
    rclpy.logging.set_logger_level("moveit_3851786301.moveit.plugins.simple_controller_manager", 30)
    rclpy.logging.set_logger_level("moveit_1681096291.moveit.plugins.simple_controller_manager", 30)
    URmoveIt = Simple_Moveit()

    # EXAMPLE 1: set absolute pose
    if 1:
        pose1 = UR_EE_pose()
        pose1.x  = 0.0
        pose1.y  = 0.3
        pose1.z  = 0.6
        
        # Enter rotation as axis-angle
        if False:
            # Axis angle definition
            rx = 1
            ry = 0
            rz = 0
            rw = np.pi*4/2
            # Axis-Angle to Quaternion transformation
            pose1.q1 = np.cos(rw/2)
            pose1.q2 = rz * np.sin(rw/2)
            pose1.q3 = ry * np.sin(rw/2)
            pose1.q4 = rx * np.sin(rw/2)

        # Enter rotation as quaternion
        else:
            pose1.q1 = 1.0
            pose1.q2 = 0.0
            pose1.q3 = 0.0
            pose1.q4 = 0.0

        URmoveIt.move_absolute(pose1)
        print(URmoveIt.get_EE_pose())
        
    
    # EXAMPLE 2: get pose
    if 0:
        pose2 = URmoveIt.get_EE_pose()
        print("######################################")
        print(pose2)
        print("######################################")

    # EXAMPLE 3: move to relative pose
    if 0:
        pose3 = UR_EE_pose(z=0.1)
        URmoveIt.move_relative(pose3)

    # EXAMPLE 4: move in a box shape with absolute movements 
    if 0:
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.5, y=0, z=0.5, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.5, y=0, z=0.6, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.6, y=0, z=0.6, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.6, y=0.1, z=0.6, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.6, y=0.1, z=0.5, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.5, y=0.1, z=0.5, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.5, y=0.0, z=0.5, q1=1, q2=0, q3=0, q4=0))
        print("#####################################################")
        print(URmoveIt.get_EE_pose())

    # EXAMPLE 5: move in a box shape with relative movements
    if 0:
        print("#####################################################")
        print(URmoveIt.get_EE_pose())
        URmoveIt.move_absolute(UR_EE_pose(x=0.5, y=0, z=0.5, q1=1, q2=0, q3=0, q4=0))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(z=0.1))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(x=0.1))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(y=0.1))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(z=-0.1))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(x=-0.1))
        time.sleep(1)
        URmoveIt.move_relative(UR_EE_pose(y=-0.1))

    # EXAMPLE 6: rotate the TCP relative to current position
    if 0:
        # Rotate around Z-axis
        URmoveIt.move_relative(angle_z=-45)

    if 0:
        # EXAMPLE: move relative with translation and rotation
        URmoveIt.move_relative(UR_EE_pose(z=0.1))
        URmoveIt.move_relative(UR_EE_pose(z=-0.1))
        URmoveIt.move_relative(UR_EE_pose(x=-0.1,y=-0.1), angle_y=-10, angle_x=-10)
        URmoveIt.move_relative(UR_EE_pose(x=0.1,y=0.1), angle_y=10, angle_x=10)
        
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
