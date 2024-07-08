from typing import Optional

from gui.styles.custom_styles import IndicatorMixin
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QLabel, QWidget


class Circle(QLabel):
    def __init__(
        self,
        parent: Optional[QWidget] = None,
        radius: int = 50,
        color: Optional[QColor | Qt.GlobalColor] = None,
    ) -> None:
        super().__init__(parent)
        self.setFixedSize(QSize(2 * radius, 2 * radius))
        stylesheet = self.styleSheet()
        self.setStyleSheet(f'{stylesheet}border-radius: {radius}px;')

        if color:
            self.set_color(color)

    def set_color(self, color: QColor | Qt.GlobalColor) -> None:
        if isinstance(color, Qt.GlobalColor):
            color = QColor(color)
        style = (
            f'background-color: rgb({color.red()}, {color.green()}, {color.blue()});'
        )
        self.setStyleSheet(f'{self.styleSheet()}{style}')


class CircleIndicator(Circle, IndicatorMixin):
    def __init__(self, parent: Optional[QWidget] = None, radius: int = 50) -> None:
        super().__init__(parent, radius)
        self.set_inactive()
