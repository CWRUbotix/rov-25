from PyQt6.QtWidgets import QPushButton, QWidget


class IndicatorMixin(QWidget):
    _PROPERTY_NAME = 'widgetState'

    # A component is running, enabled, or armed
    _ON = 'on'

    # A component is disabled, not running, or disarmed, but could be enabled through this widget
    _OFF = 'off'

    # A component is disabled, not expected to have any effect or perform its function because of
    # some external factor, either another widget or something external to the gui
    # For example, a the arm button when the pi is not connected
    _INACTIVE = 'inactive'

    # Removes any state
    _NO_STATE = ''

    def set_on(self) -> None:
        self.setProperty(IndicatorMixin._PROPERTY_NAME, IndicatorMixin._ON)
        self._update_style()

    def set_off(self) -> None:
        self.setProperty(IndicatorMixin._PROPERTY_NAME, IndicatorMixin._OFF)
        self._update_style()

    def set_inactive(self) -> None:
        self.setProperty(IndicatorMixin._PROPERTY_NAME, IndicatorMixin._INACTIVE)
        self._update_style()

    def remove_state(self) -> None:
        self.setProperty(IndicatorMixin._PROPERTY_NAME, IndicatorMixin._NO_STATE)
        self._update_style()

    def _update_style(self) -> None:
        style = self.style()
        if style is not None:
            style.polish(self)


class ButtonIndicator(QPushButton, IndicatorMixin):
    pass
