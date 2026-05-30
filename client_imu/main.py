import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .imu_node import ImuNode, ImuQtBridge
from .ui import MainWindow


def main(args=None) -> int:
    rclpy.init(args=args)

    app = QApplication(sys.argv)
    bridge = ImuQtBridge()
    imu_node = ImuNode(bridge)
    executor = SingleThreadedExecutor()
    executor.add_node(imu_node)

    window = MainWindow(imu_node)
    bridge.imu_updated.connect(window.handle_imu_update)
    window.show()

    spin_timer = QTimer()
    spin_timer.setInterval(10)
    spin_timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.0))
    spin_timer.start()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        spin_timer.stop()
        executor.remove_node(imu_node)
        imu_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
