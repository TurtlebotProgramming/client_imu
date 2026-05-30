from typing import Tuple

from PyQt5.QtWidgets import QMainWindow

from .imu_node import ImuNode, UiUpdate
from .mainwindow_ui import Ui_MainWindow
from .utils import wrap_angle_360


AngleTriple = Tuple[float, float, float]


class MainWindow(QMainWindow):
    """Main Qt window for IMU monitoring and offset control."""

    def __init__(self, imu_node: ImuNode) -> None:
        super().__init__()
        self._imu_node = imu_node
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)

        self._configure_dials()
        self._connect_buttons()
        self._show_no_data_state("No IMU data")

    def _configure_dials(self) -> None:
        self._ui.yawDial.setMinimum(0)
        self._ui.yawDial.setMaximum(359)
        self._ui.yawDial.setWrapping(True)
        self._ui.yawDial.setNotchesVisible(True)
        self._ui.yawDial.setEnabled(False)

        self._ui.pitchSlider.setMinimum(-180)
        self._ui.pitchSlider.setMaximum(180)
        self._ui.pitchSlider.setTickInterval(30)
        self._ui.pitchSlider.setTickPosition(self._ui.pitchSlider.TicksRight)
        self._ui.pitchSlider.setEnabled(False)

        self._ui.rollSlider.setMinimum(-180)
        self._ui.rollSlider.setMaximum(180)
        self._ui.rollSlider.setTickInterval(30)
        self._ui.rollSlider.setTickPosition(self._ui.rollSlider.TicksBelow)
        self._ui.rollSlider.setEnabled(False)

    def _connect_buttons(self) -> None:
        self._ui.setYaw0Button.clicked.connect(lambda: self._imu_node.set_yaw_reference(0.0))
        self._ui.setYaw90Button.clicked.connect(lambda: self._imu_node.set_yaw_reference(90.0))
        self._ui.setYawNeg90Button.clicked.connect(
            lambda: self._imu_node.set_yaw_reference(-90.0)
        )
        self._ui.setYaw180Button.clicked.connect(
            lambda: self._imu_node.set_yaw_reference(180.0)
        )
        self._ui.setRollButton.clicked.connect(self._imu_node.set_roll_current_as_zero)
        self._ui.setPitchButton.clicked.connect(self._imu_node.set_pitch_current_as_zero)

    def handle_imu_update(self, update: UiUpdate) -> None:
        self._set_status(update.status_text, update.is_stale)

        if update.raw_rpy_deg is None or update.corrected_rpy_deg is None:
            self._show_no_data_state(update.status_text)
            return

        self._set_angle_labels(update.raw_rpy_deg, update.corrected_rpy_deg)
        self._set_offset_labels(update.offsets_deg)
        self._set_dials(update.corrected_rpy_deg)

    def _set_status(self, text: str, is_stale: bool) -> None:
        color = "#b00020" if is_stale else "#176b3a"
        self._ui.statusLabel.setText(text)
        self._ui.statusLabel.setStyleSheet(
            f"color: {color}; font-weight: 600; padding: 4px 8px;"
        )

    def _show_no_data_state(self, status_text: str) -> None:
        self._ui.rawRollValueLabel.setText("No IMU data")
        self._ui.rawPitchValueLabel.setText("No IMU data")
        self._ui.rawYawValueLabel.setText("No IMU data")
        self._ui.correctedRollValueLabel.setText("No IMU data")
        self._ui.correctedPitchValueLabel.setText("No IMU data")
        self._ui.correctedYawValueLabel.setText("No IMU data")
        self._ui.offsetValueLabel.setText("Roll 0.0 / Pitch 0.0 / Yaw 0.0")
        self._ui.statusLabel.setText(status_text)
        self._ui.yawDial.setValue(0)
        self._ui.pitchSlider.setValue(0)
        self._ui.rollSlider.setValue(0)

    def _set_angle_labels(
        self, raw_rpy_deg: AngleTriple, corrected_rpy_deg: AngleTriple
    ) -> None:
        self._ui.rawRollValueLabel.setText(self._format_deg(raw_rpy_deg[0]))
        self._ui.rawPitchValueLabel.setText(self._format_deg(raw_rpy_deg[1]))
        self._ui.rawYawValueLabel.setText(self._format_deg(raw_rpy_deg[2]))
        self._ui.correctedRollValueLabel.setText(self._format_deg(corrected_rpy_deg[0]))
        self._ui.correctedPitchValueLabel.setText(self._format_deg(corrected_rpy_deg[1]))
        self._ui.correctedYawValueLabel.setText(self._format_deg(corrected_rpy_deg[2]))

    def _set_offset_labels(self, offsets_deg: AngleTriple) -> None:
        self._ui.offsetValueLabel.setText(
            f"Roll {offsets_deg[0]:.1f} / Pitch {offsets_deg[1]:.1f} / Yaw {offsets_deg[2]:.1f}"
        )

    def _set_dials(self, corrected_rpy_deg: AngleTriple) -> None:
        self._ui.yawDial.setValue(self._yaw_dial_value(corrected_rpy_deg[2]))
        self._ui.pitchSlider.setValue(int(round(corrected_rpy_deg[1])))
        self._ui.rollSlider.setValue(int(round(corrected_rpy_deg[0])))

    def _dial_value(self, angle_deg: float) -> int:
        return int(round(wrap_angle_360(angle_deg))) % 360

    def _yaw_dial_value(self, angle_deg: float) -> int:
        # Shift the dial so 12 o'clock is 0 deg and 6 o'clock is 180 deg,
        # while keeping the visible yaw progression aligned with the topic value.
        return int(round(wrap_angle_360(180.0 - angle_deg))) % 360

    def _format_deg(self, angle_deg: float) -> str:
        return f"{angle_deg:+.1f} deg"
