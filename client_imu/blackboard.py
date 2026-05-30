import threading
import time
from typing import Dict, Optional, Tuple

from sensor_msgs.msg import Imu


AngleTriple = Tuple[float, float, float]


class ImuBlackboard:
    """Thread-safe shared state for IMU data and UI-selected offsets."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._latest_imu: Optional[Imu] = None
        self._last_imu_monotonic: Optional[float] = None
        self._raw_rpy_deg: Optional[AngleTriple] = None
        self._corrected_rpy_deg: Optional[AngleTriple] = None
        self._offsets_deg: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def set_latest_imu(self, imu_msg: Imu) -> None:
        with self._lock:
            self._latest_imu = imu_msg
            self._last_imu_monotonic = time.monotonic()

    def get_latest_imu(self) -> Tuple[Optional[Imu], Optional[float]]:
        with self._lock:
            return self._latest_imu, self._last_imu_monotonic

    def get_offsets(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._offsets_deg)

    def set_offsets(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        with self._lock:
            self._offsets_deg["roll"] = roll_deg
            self._offsets_deg["pitch"] = pitch_deg
            self._offsets_deg["yaw"] = yaw_deg

    def set_roll_pitch_offsets(self, roll_deg: float, pitch_deg: float) -> None:
        with self._lock:
            self._offsets_deg["roll"] = roll_deg
            self._offsets_deg["pitch"] = pitch_deg

    def set_roll_offset(self, roll_deg: float) -> None:
        with self._lock:
            self._offsets_deg["roll"] = roll_deg

    def set_pitch_offset(self, pitch_deg: float) -> None:
        with self._lock:
            self._offsets_deg["pitch"] = pitch_deg

    def set_yaw_offset(self, yaw_deg: float) -> None:
        with self._lock:
            self._offsets_deg["yaw"] = yaw_deg

    def reset_offsets(self) -> None:
        self.set_offsets(0.0, 0.0, 0.0)

    def set_processed_rpy(
        self, raw_rpy_deg: AngleTriple, corrected_rpy_deg: AngleTriple
    ) -> None:
        with self._lock:
            self._raw_rpy_deg = raw_rpy_deg
            self._corrected_rpy_deg = corrected_rpy_deg

    def get_processed_rpy(self) -> Tuple[Optional[AngleTriple], Optional[AngleTriple]]:
        with self._lock:
            return self._raw_rpy_deg, self._corrected_rpy_deg
