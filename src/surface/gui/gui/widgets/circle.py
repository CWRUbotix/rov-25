from PyQt6.QtCore import QSize
from PyQt6.QtWidgets import QLabel, QWidget

from gui.styles.custom_styles import IndicatorMixin, WidgetState


class CircleIndicator(QLabel, IndicatorMixin):
    def __init__(self, parent: QWidget | None = None, radius: int = 50) -> None:
        super().__init__(parent)
        self.setFixedSize(QSize(2 * radius, 2 * radius))
        stylesheet = f'QLabel {{border-radius: {radius}px;}}'
        self.set_initial_stylesheet(stylesheet)
        self.set_state(WidgetState.INACTIVE)
