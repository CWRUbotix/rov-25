from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget
from rov_msgs.msg import IPAddress

from gui.gui_node import GUINode


class IPWidget(QWidget):
    signal = pyqtSignal(IPAddress)

    def __init__(self) -> None:
        super().__init__()

        self.signal.connect(self.refresh)
        GUINode().create_signal_subscription(IPAddress, 'ip_address', self.signal)

        ip_layout = QVBoxLayout()
        wired_str = f'Last known Pi Wired IP: {IPAddress.ETHERNET_ADDRESS__DEFAULT}'
        wireless_str = f'Last known Pi Wireless IP: {IPAddress.WIRELESS_ADDRESS__DEFAULT}'
        self.ethernet_label = QLabel(wired_str)
        self.wireless_label = QLabel(wireless_str)

        ip_layout.addWidget(self.ethernet_label)
        ip_layout.addWidget(self.wireless_label)
        self.setLayout(ip_layout)

    @pyqtSlot(IPAddress)
    def refresh(self, msg: IPAddress) -> None:
        self.ethernet_label.setText(f'Last known Pi Wired IP:  {msg.ethernet_address}')
        self.wireless_label.setText(f'Last known Pi Wireless IP: {msg.wireless_address}')
