from pydantic import BaseModel, PositiveInt
from airdc.common.systems.basis import System, SystemMode
from airdc.utils import list_remove
from mmk2_types.types import (
    RobotComponents,
    ImageTypes,
    TopicNames,
    RobotComponentsGroup,
    ControllerTypes,
    JointNames,
)
from mmk2_types.grpc_msgs import (
    JointState,
    Time,
    TrajectoryParams,
    ForwardPositionParams,
    MoveServoParams,
    Pose3D,
    Twist3D,
    BaseControlParams,
)
from airbot_py.airbot_mmk2 import AirbotMMK2
from typing import Optional, List, Union, Dict
import time


class AIRBOTMMKConfig(BaseModel):
    ip: str = "192.168.11.200"
    port: PositiveInt = 50055
    name: Optional[str] = None
    domain_id: Optional[int] = None
    components: List[Union[str, RobotComponents]] = []
    default_action: Optional[List[float]] = None
    cameras: Dict[Union[str, RobotComponents], Dict[str, str]] = {}
    demonstrate: bool = True

    def model_post_init(self, context):
        for i, component in enumerate(self.components):
            if isinstance(component, str):
                self.components[i] = RobotComponents[component.upper()]
        for cam in list(self.cameras.keys()):
            if isinstance(cam, str):
                self.cameras[RobotComponents[cam.upper()]] = self.cameras.pop(cam)


class AIRBOTMMK(System):
    config: AIRBOTMMKConfig
    interface: AirbotMMK2

    def on_configure(self) -> bool:
        self._joint_components = list_remove(
            self.config.components, {RobotComponents.BASE}
        )
        self._use_base = RobotComponents.BASE in self.config.components
        if self.config.demonstrate:
            self._action_topics = {
                comp: TopicNames.tracking.format(component=comp.value)
                for comp in set(RobotComponentsGroup.ARMS) & set(self._joint_components)
            }
            self._action_topics.update(
                {
                    comp: TopicNames.controller_command.format(
                        controller=f"/{comp.value}_{ControllerTypes.FORWARD_POSITION.value}_controller"
                    )
                    for comp in set(RobotComponentsGroup.HEAD_SPINE)
                    & set(self._joint_components)
                }
            )
            if self._use_base:
                base_index = self.config.components.index(RobotComponents.BASE)
                if base_index != len(self.config.components) - 1:
                    self.get_logger().warning(
                        "BASE component should be the last in the components list."
                    )
                self._action_topics[RobotComponents.BASE] = TopicNames.velocity
            self.get_logger().info(f"Action topics: {self._action_topics}")
            self.interface.listen_to(self._action_topics.values())
        self.interface.enable_resources(self.config.cameras)
        # get the camera goal by the config
        self._cameras_goal = {}
        for cam, cfg in self.config.cameras.items():
            goal = [ImageTypes.COLOR]
            if (
                cfg["camera_type"] == "REALSENSE"
                and cfg.get("enable_depth", "false") == "true"
            ):
                if cfg.get("align_depth.enable", "false") == "true":
                    goal.append(ImageTypes.ALIGNED_DEPTH_TO_COLOR)
                else:
                    goal.append(ImageTypes.DEPTH)
            self._cameras_goal[cam] = goal
        self.get_logger().info(f"Camera goals: {self._cameras_goal}")
        self._check_joint_names(self.interface.get_robot_state().joint_state.name)
        jn = sum(len(JointNames[comp.name].value) for comp in self._joint_components)
        self._expected_dims = {jn + 2, jn + 3} if self._use_base else {jn}
        self.switch_mode(SystemMode.RESETTING)
        self._reset()
        self._logs = {}
        if self._cameras_goal:
            while not self._capture_images(False):
                self.get_logger().info("Waiting for valid images...")
                time.sleep(1)
        return True

    def get_info(self):
        return {}

    def _reset(self, sleep_time=0):
        if self.config.default_action is not None:
            self.send_action(self.config.default_action)
        else:
            self.get_logger().warning("No default action is set.")
        time.sleep(sleep_time)

    def send_action(self, action):
        if isinstance(action, dict):
            goal, param = self._action_dict_to_goal(action)
        else:
            goal, param = self._action_array_to_goal(action)
        if self.config.demonstrate:
            # TODO: since the arms and eefs are controlled by the teleop bag
            for comp in RobotComponentsGroup.ARMS_EEFS:
                goal.pop(comp, None)
        # pprint(goal)
        # pprint(param)
        self.interface.set_goal(goal, param)
        # set forward position goal so that action can always be listened
        if self.config.demonstrate and self._current_mode is SystemMode.RESETTING:
            goal.pop(RobotComponents.BASE, None)
            self.interface.set_goal(goal, ForwardPositionParams())

    def _get_js_param(self):
        return (
            TrajectoryParams()
            if self._current_mode is SystemMode.RESETTING
            else ForwardPositionParams()
        )

    def _action_dict_to_goal(self, obs: dict) -> List[float]:
        goal = {}
        param = {}
        js_param = self._get_js_param()
        for comp in self._joint_components:
            goal[comp] = JointState(
                position=obs[f"/mmk/action/{comp.value}/joint_state/position"]
            )
            param[comp] = js_param
        if self._use_base:
            linear = obs["/mmk/action/base/twist/linear"]
            angular = obs["/mmk/action/base/twist/angular"]
            goal[RobotComponents.BASE] = Twist3D(
                x=linear[0], y=linear[1], omega=angular[2]
            )
            param[RobotComponents.BASE] = BaseControlParams()
        return goal, param

    def _action_array_to_goal(self, action) -> Dict[RobotComponents, JointState]:
        if len(action) not in self._expected_dims:
            raise ValueError(
                f"Action dimension mismatch: expected {self._expected_dims}, got {len(action)}"
            )
        goal = {}
        param = {}
        j_cnt = 0
        js_param = self._get_js_param()
        for comp in self._joint_components:
            end = j_cnt + len(JointNames[comp.name].value)
            goal[comp] = JointState(position=action[j_cnt:end])
            param[comp] = js_param
            j_cnt = end
        if self._use_base:
            base_action = action[j_cnt:]
            if len(base_action) == 2:
                base_goal = Twist3D(x=base_action[0], omega=base_action[1])
            elif len(base_action) == 3:
                x, y, theta = base_action
                base_goal = Pose3D(x=x, y=y, theta=theta)
            else:
                raise ValueError(
                    f"Base action dimension mismatch: expected 2 or 3, got {len(base_action)}"
                )
            goal[RobotComponents.BASE] = base_goal
            param[RobotComponents.BASE] = BaseControlParams()
        return goal, param

    def on_switch_mode(self, mode: SystemMode):
        self._current_mode = mode
        return True

    def _get_low_dim(self):
        data = {}
        start = time.perf_counter()
        robot_state = self.interface.get_robot_state()
        self._logs["get_robot_state_dt_s"] = time.perf_counter() - start
        all_joints = robot_state.joint_state
        stamp = robot_state.joint_state.header.stamp
        t = self._to_time_ns(stamp)
        for comp in self.config.components:
            if comp == RobotComponents.BASE:
                base_pose = robot_state.base_state.pose
                base_vel = robot_state.base_state.velocity
                data[f"observation/{comp.value}/velocity"] = {
                    "t": t,
                    "data": [base_vel.x, base_vel.y, base_vel.omega],
                }
                data[f"observation/{comp.value}/pose"] = {
                    "t": t,
                    "data": [base_pose.x, base_pose.y, base_pose.theta],
                }
            else:
                self._set_js_field(data, comp, t, all_joints)
        if self.config.demonstrate:
            for comp in self.config.components:
                action_topic = self._action_topics[comp]
                listened_data = self.interface.get_listened(action_topic)
                if listened_data is None:
                    raise ValueError(
                        f"Action topic: {action_topic} is not listened yet, "
                        "please make sure the robot has entered the teleoperating sync mode"
                    )
                # self.get_logger().info(f"Processing component: {comp}, topic: {self._action_topics.get(comp)}")
                if comp in RobotComponentsGroup.ARMS:
                    arm_jn = JointNames[comp.name].value
                    comp_eef = comp.value + "_eef"
                    eef_jn = JointNames[RobotComponents(comp_eef).name].value
                    start = time.perf_counter()
                    js = listened_data
                    jq = self.interface.get_joint_values_by_names(js, arm_jn + eef_jn)
                    slices = {
                        comp.value: slice(0, len(arm_jn)),
                        comp_eef: slice(len(arm_jn), len(arm_jn) + len(eef_jn)),
                    }
                    for component in (comp.value, comp_eef):
                        self._set_js_fields(
                            data, component, t, jq[slices[component]], prefix="action"
                        )
                    self._logs[f"get_listened_{comp.value}_dt_s"] = (
                        time.perf_counter() - start
                    )
                elif comp in RobotComponentsGroup.HEAD_SPINE:
                    start = time.perf_counter()
                    if listened_data.data:
                        self._set_js_fields(
                            data,
                            comp.value,
                            t,
                            list(listened_data.data),
                            prefix="action",
                        )
                        self._logs[f"get_listened_{comp.value}_dt_s"] = (
                            time.perf_counter() - start
                        )
                    else:
                        self.get_logger().warning(
                            f"No data received for component: {comp}"
                        )
                elif comp is RobotComponents.BASE:
                    start = time.perf_counter()
                    self._set_twist_field(
                        data, comp.value, t, listened_data, prefix="action"
                    )
                    self._logs[f"get_listened_{comp.value}_dt_s"] = (
                        time.perf_counter() - start
                    )
                else:
                    raise ValueError(f"Unknown component in demonstrate mode: {comp}")
        return data

    def _set_js_field(
        self,
        data: dict,
        comp: RobotComponents,
        t: float,
        js: JointState,
        prefix: str = "observation",
    ):
        for field in {"position", "velocity", "effort"}:
            value = self.interface.get_joint_values_by_names(
                js, JointNames[comp.name].value, field
            )
            data[f"{prefix}/{comp.value}/joint_state/{field}"] = {"t": t, "data": value}

    def _set_js_fields(
        self,
        data: dict,
        comp: str,
        t: float,
        pos,
        vel=None,
        eff=None,
        prefix: str = "observation",
    ):
        for field, value in zip(("position", "velocity", "effort"), (pos, vel, eff)):
            if value is not None:
                data[f"{prefix}/{comp}/joint_state/{field}"] = {
                    "t": t,
                    "data": value,
                }

    def _set_twist_field(
        self,
        data: dict,
        comp: str,
        t: float,
        twist,
        prefix: str = "observation",
    ):
        for field, value in zip(("linear", "angular"), (twist.linear, twist.angular)):
            data[f"{prefix}/{comp}/twist/{field}"] = {"t": t, "data": value}

    def _capture_images(self, strict: bool = True) -> dict:
        images_obs = {}
        start = time.perf_counter()
        comp_images = self.interface.get_image(self._cameras_goal)
        self._logs["get_image_dt_s"] = time.perf_counter() - start
        for comp, images in comp_images.items():
            stamp = self._to_time_ns(images.stamp)
            for img_type, image in images.data.items():
                if image.shape[0] == 1:
                    desc = f"Image from {comp.value}/{img_type.value} is not valid"
                    if strict:
                        raise ValueError(desc)
                    else:
                        self.get_logger().warning(desc)
                        return {}
                suffix = (
                    "image_raw"
                    if img_type is not ImageTypes.DEPTH
                    else "image_rect_raw"
                )
                images_obs[f"{comp.value}/{img_type.value}/{suffix}"] = {
                    "t": stamp,
                    "data": image,
                }
        return images_obs

    def capture_observation(self, timeout: Optional[float] = None):
        """The returned observations do not have a batch dimension."""
        obs_act_dict = self._get_low_dim()
        obs_act_dict.update(self._capture_images())
        # self.get_logger().info("Time costs:\n" + pformat(self._logs))
        return obs_act_dict

    def _to_time_ns(self, stamp: Time) -> int:
        """Get the current time in nanoseconds."""
        return int(stamp.sec * 1e9 + stamp.nanosec)

    def _check_joint_names(self, joint_names: List[str]):
        required_joints = set()
        for component in self._joint_components:
            required_joints.update(JointNames[component.name].value)
        missing = required_joints - set(joint_names)
        if missing:
            raise ValueError(f"Missing required joints: {missing}")

    def shutdown(self) -> bool:
        self.interface.close()
        return True


if __name__ == "__main__":
    from airdc.common.visualizers.opencv import (
        OpenCVVisualizer,
        OpenCVVisualizerConfig,
    )
    from itertools import count

    viser = OpenCVVisualizer(OpenCVVisualizerConfig(ignore_info=True))
    mmk = AIRBOTMMK(
        AIRBOTMMKConfig(
            ip="192.168.11.200",
            # ip="172.25.12.57",
            components=RobotComponentsGroup.ARMS_EEFS + RobotComponentsGroup.HEAD_SPINE,
            cameras={
                RobotComponents.HEAD_CAMERA: {
                    "camera_type": "REALSENSE",
                    "rgb_camera.color_profile": "640,480,30",
                    "enable_depth": "false",
                },
                RobotComponents.LEFT_CAMERA: {
                    "camera_type": "USB",
                    "video_device": "/dev/left_camera",
                    "image_width": "640",
                    "image_height": "480",
                    "framerate": "25",
                },
                RobotComponents.RIGHT_CAMERA: {
                    "camera_type": "USB",
                    "video_device": "/dev/right_camera",
                    "image_width": "640",
                    "image_height": "480",
                    "framerate": "25",
                },
            },
            demonstrate=False,
            default_action=[
                # arms will not move when demonstrating
                # left_arm (6 joints)
                -0.233,
                -0.73,
                1.088,
                1.774,
                -1.1475,
                -0.1606,
                # right_arm (6 joints)
                0.2258,
                -0.6518,
                0.9543,
                -1.777,
                1.0615,
                0.3588,
                1.0,  # left_arm_eef (1 joint)
                1.0,  # right_arm_eef (1 joint)
                # head (2 joints)
                0.0,
                -0.0,
                0.0,  # spine (1 joint)
            ],
        )
    )
    assert mmk.configure()
    assert viser.configure()
    costs = []
    total_start = time.perf_counter()
    try:
        for times in count():
            start = time.perf_counter()
            obs = mmk.capture_observation()
            costs.append(time.perf_counter() - start)
            viser.update(obs, None)
            if viser.current_key in {27, ord("q")}:
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
        # print(f"Iteration {i} took {time.perf_counter() - start:.4f} seconds")
    print(f"Min: {min(costs)} Max: {max(costs)}")
    print(f"Total time taken: {time.perf_counter() - total_start:.4f} seconds")
    print(f"Average frequency: {(times / (time.perf_counter() - total_start)):.4f} Hz")
    print("Shutting down the robot...")
    mmk.shutdown()
    viser.shutdown()
