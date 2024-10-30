from PyQt6.QtWidgets import QPushButton, QWidget


class IndicatorMixin(QWidget):
    # Stylesheet for when a component is running, enabled, or armed
    _ON_STYLESHEET = 'QWidget { background-color: limegreen; }'

    # Stylesheet for when a component is disabled, not running, or disarmed, but could be enabled
    # through this widget
    _OFF_STYLESHEET = 'QWidget { background-color: red; }'

    # Stylesheet for when a component is disabled, not expected to have any effect or perform its
    # function because of some external factor, either another widget or something external
    # to the gui. For example, a the arm button when the pi is not connected
    _INACTIVE_STYLESHEET = 'QWidget { background-color: silver; }'

    def set_initial_stylesheet(self) -> None:
        self._ORIGINAL_STYLESHEET = self.styleSheet()

    def set_on(self) -> None:
        self.setStyleSheet(self._ORIGINAL_STYLESHEET + self._ON_STYLESHEET)

    def set_off(self) -> None:
        self.setStyleSheet(self._ORIGINAL_STYLESHEET + self._OFF_STYLESHEET)

    def set_inactive(self) -> None:
        self.setStyleSheet(self._ORIGINAL_STYLESHEET + self._INACTIVE_STYLESHEET)

    def remove_state(self) -> None:
        self.setStyleSheet(self._ORIGINAL_STYLESHEET)


class ButtonIndicator(QPushButton, IndicatorMixin):
    def __init__(self, text: str = '') -> None:
        super().__init__(text)
        self.set_initial_stylesheet()
