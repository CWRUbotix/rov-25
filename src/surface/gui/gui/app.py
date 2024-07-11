import atexit
import signal
from pathlib import Path

import qdarktheme
import rclpy.utilities
from ament_index_python.packages import get_package_share_directory
from PyQt6.QtWidgets import QApplication, QWidget
from rclpy.node import Node


class App(QWidget):
    """Main app window."""

    app = QApplication([])

    def __init__(self, node_name: str) -> None:
        if not rclpy.utilities.ok():
            rclpy.init()
        super().__init__()
        self.node = Node(node_name, parameter_overrides=[])

        self.theme_param = self.node.declare_parameter('theme', '')
        self.resize(1850, 720)

        atexit.register(clean_shutdown)

    def run_gui(self) -> None:
        # Kills with Control + C
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # Apply theme
        theme_param = self.theme_param.get_parameter_value().string_value
        theme_path = Path(get_package_share_directory('gui')) / 'styles' / (theme_param + '.qss')

        base_theme = 'dark' if theme_param == 'dark' else 'light'
        custom_styles = '\n'
        if theme_path.exists():
            with theme_path.open(encoding='utf-8') as theme_file:
                custom_styles += theme_file.read()

        qdarktheme.setup_theme(base_theme, additional_qss=custom_styles)

        # Delete node now that we've used it to get params
        self.node.destroy_node()

        self.show()

        # TODO: when the app closes it causes an error. Make not cause error?
        self.app.exec()


def clean_shutdown() -> None:
    if rclpy.utilities.ok():
        rclpy.shutdown()
