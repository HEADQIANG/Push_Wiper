from enum import auto
from airdc.utils import StrEnum, zip
from airdc.common.utils.coordinate import CoordinateTools
from rclpy.node import Node
import rclpy
import rclpy.publisher
from std_msgs.msg import Float64MultiArray

# from control_msgs.msg import DynamicJointState, InterfaceValue
from rs_airbot_control.msg import JointState
from typing import Dict, List, Optional
from arm_control_py.mmk2_kdl import MMK2Kdl
import time


class Component(StrEnum):
    LEFT_ARM = auto()
    RIGHT_ARM = auto()
    HEAD = auto()
    SPINE = auto()
    LEFT_CAMERA = auto()
    RIGHT_CAMERA = auto()
    HEAD_CAMERA = auto()


class Ros2Backend:
    def __init__(self, node: Node):
        self._node = node
        self._cmd_puber: dict[Component, rclpy.publisher.Publisher] = {}
        self._joint_states: dict = {}
        self._arms = [Component.LEFT_ARM, Component.RIGHT_ARM]
        self._joint_names = [
            f"{comp}_joint{i}" for i in range(1, 8) for comp in self._arms
        ] + [
            "head_pitch_joint",
            "head_yaw_joint",
            "slide_joint",
        ]
        self._last_target_joint = {}
        self.joint_slices = {
            Component.LEFT_ARM: slice(0, 7),
            Component.RIGHT_ARM: slice(7, 14),
            Component.HEAD: slice(14, 16),
            Component.SPINE: slice(16, 17),
        }
        self._component_slices = {
            Component.SPINE: slice(0, 1),
            Component.LEFT_ARM: slice(1, 7),
            Component.RIGHT_ARM: slice(7, 13),
        }
        self.get_logger().info(f"Joint names: {self._joint_names}")
        for component in [
            Component.HEAD,
            Component.SPINE,
        ] + self._arms:
            self._cmd_puber[component] = self._node.create_publisher(
                Float64MultiArray,
                f"{component.value}_forward_position_controller/commands",
                10,
            )
        self._node.create_subscription(
            # DynamicJointState,
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
        )
        self._kdl = MMK2Kdl()
        # self._joint_states = {
        #     "position": [0.0] * len(self._joint_names),
        #     "velocity": [0.0] * len(self._joint_names),
        #     "effort": [0.0] * len(self._joint_names),
        # }
        while len(self._joint_states) < len(self._arms):
            self.get_logger().info("Waiting for joint states to be initialized. ")
            rclpy.spin_once(self._node)
            time.sleep(0.2)
        # assert len(self._joint_msg.name) == len(
        #     self._joint_msg.position
        # ), "Joint names and positions must have the same length."
        self._last_pose = {comp: self.get_pose(comp) for comp in self._arms}
        self._node.create_timer(0.01, self._publish_target)

    def get_logger(self):
        return self._node.get_logger().get_child(self.__class__.__name__)

    # def _joint_state_callback(self, msg: DynamicJointState):
    #     """
    #     Callback for joint state messages.
    #     """
    #     start = time.monotonic()
    #     values: List[InterfaceValue] = sort_index(
    #         self._joint_names, msg.joint_names, msg.interface_values
    #     )
    #     joint_states = defaultdict(list)
    #     for value in values:
    #         for name, value in zip(
    #             value.interface_names,
    #             value.values,
    #         ):
    #             joint_states[name].append(value)
    #     self._joint_states = {
    #         field: joint_states[field] for field in ["position", "velocity", "effort"]
    #     }
    #     self.get_logger().info(
    #         f"Joint states updated in {time.monotonic() - start:.3f} seconds."
    #     )

    def _joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages.
        """
        # start = time.monotonic()
        joint_states = {}
        for field in ["position", "velocity", "effort"]:
            # values = sort_index(self._joint_names, msg.name, getattr(msg, field))
            joint_states[field] = getattr(msg, field)
        self._joint_states = joint_states
        self._joint_msg = msg
        # self.get_logger().info(
        #     f"Joint states updated in {time.monotonic() - start:.3f} seconds."
        # )

    def get_joint_state(self) -> Dict[str, List[float]]:
        """
        Get the joint state for a specific component.
        """
        return self._joint_states

    def set_joint_state(self, comp: Component, target: List[float]):
        assert len(target) == 7, (
            f"Target state must have 7 elements, but got {len(target)}."
        )
        self._last_target_joint[comp] = target

    def get_pose(self, comp: Optional[Component] = None) -> tuple:
        """
        Get the pose of a specific component.
        """
        position = list(self.get_joint_state()["position"])
        # spine_pos = position[self.joint_slices[Component.SPINE]]
        spine_pos = [0.0]
        if comp is not None:
            index = comp.removesuffix("_arm")
            pos = spine_pos + position[self.joint_slices[comp]][:-1]
        else:
            index = None
            pos = (
                spine_pos
                + position[self.joint_slices[Component.LEFT_ARM]][:-1]
                + position[self.joint_slices[Component.RIGHT_ARM]][:-1]
            )
        left, right = self._kdl.forward_kinematics(pos, index)
        matrix = {
            Component.LEFT_ARM: left,
            Component.RIGHT_ARM: right,
        }
        if comp is None:
            return (
                CoordinateTools.to_pose(matrix[component]) for component in self._arms
            )
        return CoordinateTools.to_pose(matrix[comp])

    def set_pose(self, components: List[Component], targets: List[tuple]):
        """
        Set the pose of a specific component.
        """

        def get_key(comp: Component):
            return f"T_{comp.removesuffix('_arm')}"

        last_pose = {
            get_key(comp): CoordinateTools.to_transform_matrix(*pose)
            for comp, pose in self._last_pose.items()
        }
        last_pose.update(
            {
                get_key(component): CoordinateTools.to_transform_matrix(*target)
                for component, target in zip(components, targets)
            }
        )
        # from pprint import pprint

        # pprint(last_pose)
        position = list(self.get_joint_state()["position"])
        # spine_pos = position[self.joint_slices[Component.SPINE]]
        spine_pos = [0.0]
        joints = self._kdl.inverse_kinematics(
            **last_pose,
            ref_pos=spine_pos
            + position[self.joint_slices[Component.LEFT_ARM]][:-1]
            + position[self.joint_slices[Component.RIGHT_ARM]][:-1],
            target_height=spine_pos[0],
            force_calculate=True,
            use_clip=True,
        )[0]
        for component in components:
            arm_joint = joints[self._component_slices[component]]
            print(f"Setting joint for {component} to {arm_joint}.")
            target = arm_joint.tolist()
            if component in self._last_target_joint:
                target += self._last_target_joint[component][-1:]
            else:
                target += self._joint_states["position"][self.joint_slices[component]][
                    -1:
                ]
            self.set_joint_state(component, target)
        return joints

    def _publish_target(self):
        for comp in list(self._last_target_joint.keys()):
            self._cmd_puber[comp].publish(
                Float64MultiArray(data=self._last_target_joint[comp])
            )


if __name__ == "__main__":
    import rclpy
    from rclpy.node import Node
    import numpy as np
    from pprint import pprint
    from threading import Thread

    np.set_printoptions(precision=3, suppress=True)

    rclpy.init()
    node = Node("airbot_mmk_ros2", namespace="MK14QWZ024480005")
    backend = Ros2Backend(node)

    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    cnt = 100
    for i in range(cnt):
        # get current joint state
        joint_position = backend.get_joint_state()["position"]
        # for comp in backend._arms:
        #     print(f"{comp} Joint State: {joint_position[backend.joint_slices[comp]]}")
        joint_dict = {
            comp: np.array(joint_position[backend.joint_slices[comp]])
            for comp in backend._arms + [Component.HEAD, Component.SPINE]
        }
        print("**** Current Values ****")
        pprint(joint_dict)
        # get current pose
        start = time.monotonic()
        left_arm_pose = backend.get_pose(Component.LEFT_ARM)
        right_arm_pose = backend.get_pose(Component.RIGHT_ARM)
        pprint(
            {
                "Left Arm Pose": left_arm_pose,
                "Right Arm Pose": right_arm_pose,
            }
        )
        left_pose, right_pose = backend.get_pose()
        assert np.allclose(left_arm_pose, left_pose)
        assert np.allclose(right_arm_pose, right_pose)

        print("**** Target Values ****")
        # set target pose to current pose
        start = time.monotonic()
        # backend.set_pose([Component.LEFT_ARM], [left_pose])
        cal_joints = backend.set_pose(
            [Component.LEFT_ARM, Component.RIGHT_ARM], [left_pose, right_pose]
        )
        print(f"Set pose in {time.monotonic() - start:.3f} seconds. ")
        cal_joint_dict = {
            comp: cal_joints[backend._component_slices[comp]]
            for comp in backend._arms + [Component.SPINE]
        }
        for comp in backend._arms:
            assert np.allclose(joint_dict[comp][:-1], cal_joint_dict[comp]), (
                f"Joint state for {comp} does not match after setting pose. "
                f"Expected: {joint_dict[comp]}, Got: {cal_joint_dict[comp]}"
            )
        # Set a new pose for the left arm
        # new_left_arm_pose = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # backend.set_pose(Component.LEFT_ARM, new_left_arm_pose)
        if i < cnt - 1:
            input("Press Enter to continue to the next iteration or Ctrl+C to exit. ")
            # time.sleep(0.5)
    rclpy.shutdown()
