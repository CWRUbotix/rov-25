from enum import Enum
from typing import TypedDict, Final

from PyQt6.QtWidgets import QPushButton, QWidget


class WidgetState(Enum):
    ON = 1
    OFF = 2
    INACTIVE = 3
    NONE = 4

# class StylesheetDictionary(TypedDict):
#     WidgetState.ON: str
#     WidgetState.OFF: str
#     WidgetState.INACTIVE: str
#     WidgetState.NONE: str


class IndicatorMixin(QWidget):
    # The stylesheets that correspond to each widget state
    _STYLESHEETS: Final[dict] = {
        # Stylesheet for when a component is running, enabled, or armed
        WidgetState.ON: 'QWidget { background-color: limegreen; }',
        # Stylesheet for when a component is disabled, not running, or disarmed, but could be
        # enabled through this widget
        WidgetState.OFF: 'QWidget { background-color: red; }',
        # Stylesheet for when a component is disabled, not expected to have any effect or perform
        # its function because of some external factor, either another widget or something
        # external to the gui. For example, a the arm button when the pi is not connected
        WidgetState.INACTIVE: 'QWidget { background-color: silver; }',
        WidgetState.NONE: '',
    }

    def set_initial_stylesheet(self, extra_styles: str = '') -> None:
        """
        Store the original stylesheet of the widget so changing state does not remove styles.

        Parameters
        ----------
        extra_styles : str, optional
            an additional stylesheet that will be constant for the widget, by default ''
        """
        self._original_stylesheet = self.styleSheet() + extra_styles
        self.current_state = WidgetState.NONE

    def set_state(self, new_state: WidgetState) -> None:
        """
        Set a new state for the widget.

        Parameters
        ----------
        new_state : WidgetState
            The new state for the widget
        """
        self.current_state = new_state
        self.setStyleSheet(self._original_stylesheet + self._STYLESHEETS[self.current_state])


class ButtonIndicator(QPushButton, IndicatorMixin):
    def __init__(self, text: str = '', extra_styles: str = '') -> None:
        super().__init__(text)
        self.set_initial_stylesheet(extra_styles)
