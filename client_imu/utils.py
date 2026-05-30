import math
from typing import Tuple


AngleTriple = Tuple[float, float, float]


def quaternion_to_rpy_rad(x: float, y: float, z: float, w: float) -> AngleTriple:
    """Convert quaternion to roll, pitch, yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rad_to_deg(angle_rad: float) -> float:
    return math.degrees(angle_rad)


def wrap_angle_180(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0
    return wrapped


def wrap_angle_360(angle_deg: float) -> float:
    wrapped = angle_deg % 360.0
    if wrapped < 0.0:
        wrapped += 360.0
    return wrapped


def apply_offsets_deg(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    roll_offset_deg: float,
    pitch_offset_deg: float,
    yaw_offset_deg: float,
) -> AngleTriple:
    corrected_roll = wrap_angle_180(roll_deg + roll_offset_deg)
    corrected_pitch = wrap_angle_180(pitch_deg + pitch_offset_deg)
    corrected_yaw = wrap_angle_180(yaw_deg + yaw_offset_deg)
    return corrected_roll, corrected_pitch, corrected_yaw
