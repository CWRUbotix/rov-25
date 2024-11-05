from PyQt6.QtWidgets import QPushButton, QWidget
from enum import Enum

class WidgetState(Enum):
    ON = 1
    OFF = 2
    INACTIVE = 3
    NONE = 4

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

    def set_initial_stylesheet(self, extra_styles: str = '') -> None:
        """
        Stores the original stylesheet of the widget so changing state does not remove styles

        Parameters
        ----------
        extra_styles : str, optional
            an additional stylesheet that will be constant for the widget, by default ''
        """
        self._original_stylesheet = self.styleSheet() + extra_styles
        self.current_state = WidgetState.NONE

    def set_state(self, new_state: WidgetState) -> None:
        """
        Sets a new state for the widget

        Parameters
        ----------
        new_state : WidgetState
            The new state for the widget
        """
        self.current_state = new_state
        self.setStyleSheet(self._original_stylesheet + self.get_state_stylesheet())
        
    def get_state_stylesheet(self) -> str:
        """
        Returns the stylesheet according to the current state of the widget

        Returns
        -------
        str
            the stylesheet that corresponds to the current state of the widget
        """
        if self.current_state == WidgetState.ON:
            return self._ON_STYLESHEET
        elif self.current_state == WidgetState.OFF:
            return self._OFF_STYLESHEET
        elif self.current_state == WidgetState.INACTIVE:
            return self._INACTIVE_STYLESHEET
        else:
            return ""


class ButtonIndicator(QPushButton, IndicatorMixin):
    def __init__(self, text: str = '', extra_styles: str = '') -> None:
        super().__init__(text)
        self.set_initial_stylesheet(extra_styles)
