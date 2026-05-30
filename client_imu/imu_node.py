import time
from dataclasses import dataclass
from typing import Optional, Tuple

from PyQt5.QtCore import QObject, pyqtSignal
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .blackboard import ImuBlackboard
from .utils import apply_offsets_deg, quaternion_to_rpy_rad, rad_to_deg, wrap_angle_180


AngleTriple = Tuple[float, float, float]


@dataclass
class UiUpdate:
    status_text: str
    is_stale: bool
    raw_rpy_deg: Optional[AngleTriple]
    corrected_rpy_deg: Optional[AngleTriple]
    offsets_deg: AngleTriple


class ImuQtBridge(QObject):
    """Qt signal bridge so ROS-side code never touches widgets directly."""

    imu_updated = pyqtSignal(object)


class ImuNode(Node):
    """ROS2 node that stores latest IMU and publishes corrected RPY at a fixed rate."""

    def __init__(self, bridge: ImuQtBridge) -> None:
        super().__init__("client_imu")
        self._bridge = bridge
        self._blackboard = ImuBlackboard()

        self.declare_parameter("input_topic", "/imu")
        self.declare_parameter("output_topic", "/client_imu/imu_value")
        self.declare_parameter("publish_hz", 25.0)
        self.declare_parameter("angle_unit", "degree")
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("yaw_display_mode", "signed")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        if self._publish_hz <= 0.0:
            self.get_logger().warn("publish_hz must be > 0. Falling back to 25.0 Hz.")
            self._publish_hz = 25.0

        self._imu_subscription = self.create_subscription(
            Imu,
            self._input_topic,
            self._imu_callback,
            10,
        )
        self._publisher = self.create_publisher(Vector3, self._output_topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_hz, self._timer_callback)

        self.get_logger().info(
            f"client_imu started: input={self._input_topic}, "
            f"output={self._output_topic}, hz={self._publish_hz:.1f}"
        )

    def _imu_callback(self, imu_msg: Imu) -> None:
        """Store the latest IMU only. Heavy work stays in the timer callback."""
        self._blackboard.set_latest_imu(imu_msg)

    def _timer_callback(self) -> None:
        latest_imu, last_rx_time = self._blackboard.get_latest_imu()
        offsets = self._blackboard.get_offsets()
        offset_tuple = (offsets["roll"], offsets["pitch"], offsets["yaw"])

        if latest_imu is None or last_rx_time is None:
            self._emit_ui_update(
                UiUpdate(
                    status_text="No IMU data",
                    is_stale=True,
                    raw_rpy_deg=None,
                    corrected_rpy_deg=None,
                    offsets_deg=offset_tuple,
                )
            )
            return

        age_sec = time.monotonic() - last_rx_time
        orientation = latest_imu.orientation
        raw_rpy_rad = quaternion_to_rpy_rad(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        raw_rpy_deg = tuple(rad_to_deg(value) for value in raw_rpy_rad)
        raw_rpy_deg = tuple(wrap_angle_180(value) for value in raw_rpy_deg)

        corrected_rpy_deg = apply_offsets_deg(
            raw_rpy_deg[0],
            raw_rpy_deg[1],
            raw_rpy_deg[2],
            offsets["roll"],
            offsets["pitch"],
            offsets["yaw"],
        )

        self._blackboard.set_processed_rpy(raw_rpy_deg, corrected_rpy_deg)

        is_stale = age_sec > self._stale_timeout_sec
        status_text = (
            f"Stale IMU data ({age_sec:.2f}s old)"
            if is_stale
            else f"IMU OK ({self._publish_hz:.1f} Hz publish)"
        )

        self._emit_ui_update(
            UiUpdate(
                status_text=status_text,
                is_stale=is_stale,
                raw_rpy_deg=raw_rpy_deg,
                corrected_rpy_deg=corrected_rpy_deg,
                offsets_deg=offset_tuple,
            )
        )

        if is_stale:
            return

        vector_msg = Vector3()
        vector_msg.x = corrected_rpy_deg[0]
        vector_msg.y = corrected_rpy_deg[1]
        vector_msg.z = corrected_rpy_deg[2]
        self._publisher.publish(vector_msg)

    def set_yaw_reference(self, target_yaw_deg: float) -> None:
        raw_rpy_deg, _ = self._blackboard.get_processed_rpy()
        if raw_rpy_deg is None:
            self.get_logger().warn("Cannot set yaw reference before IMU data arrives.")
            return

        yaw_offset = wrap_angle_180(target_yaw_deg - raw_rpy_deg[2])
        self._blackboard.set_yaw_offset(yaw_offset)
        self.get_logger().info(
            f"Yaw reference updated: target={target_yaw_deg:.1f} deg, "
            f"offset={yaw_offset:.1f} deg"
        )

    def set_roll_pitch_current_as_zero(self) -> None:
        raw_rpy_deg, _ = self._blackboard.get_processed_rpy()
        if raw_rpy_deg is None:
            self.get_logger().warn("Cannot set roll/pitch reference before IMU data arrives.")
            return

        self._blackboard.set_roll_pitch_offsets(-raw_rpy_deg[0], -raw_rpy_deg[1])
        self.get_logger().info(
            "Roll/Pitch reference updated to current posture as zero."
        )

    def set_roll_current_as_zero(self) -> None:
        raw_rpy_deg, _ = self._blackboard.get_processed_rpy()
        if raw_rpy_deg is None:
            self.get_logger().warn("Cannot set roll reference before IMU data arrives.")
            return

        self._blackboard.set_roll_offset(-raw_rpy_deg[0])
        self.get_logger().info("Roll reference updated to current posture as zero.")

    def set_pitch_current_as_zero(self) -> None:
        raw_rpy_deg, _ = self._blackboard.get_processed_rpy()
        if raw_rpy_deg is None:
            self.get_logger().warn("Cannot set pitch reference before IMU data arrives.")
            return

        self._blackboard.set_pitch_offset(-raw_rpy_deg[1])
        self.get_logger().info("Pitch reference updated to current posture as zero.")

    def reset_offsets(self) -> None:
        self._blackboard.reset_offsets()
        self.get_logger().info("IMU offsets reset.")

    def _emit_ui_update(self, update: UiUpdate) -> None:
        self._bridge.imu_updated.emit(update)
