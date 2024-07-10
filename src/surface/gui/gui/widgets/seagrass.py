from collections.abc import Callable

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from gui.widgets.video_widget import CameraDescription, CameraType, PauseableVideoWidget


class SeagrassWidget(QWidget):
    BUTTON_WIDTH = 120

    def __init__(self) -> None:
        super().__init__()

        root_layout = QHBoxLayout(self)

        self.after_grid = SeagrassGrid(self.update_result_text)
        self.before_grid = SeagrassGrid(self.update_result_text, self.after_grid.set_button)
        # Before layout
        before_layout = QVBoxLayout()

        before_btns_layout = QHBoxLayout()

        set_all_green = QPushButton('Set All Green')
        set_all_green.setMaximumWidth(self.BUTTON_WIDTH)
        set_all_green.clicked.connect(lambda: self.before_grid.reset_grid(True))

        set_all_white = QPushButton('Set All White')
        set_all_white.setMaximumWidth(self.BUTTON_WIDTH)
        set_all_white.clicked.connect(lambda: self.before_grid.reset_grid(False))

        before_btns_layout.addWidget(set_all_green)
        before_btns_layout.addWidget(set_all_white)

        before_layout.addWidget(QLabel('Before'), alignment=Qt.AlignmentFlag.AlignCenter)
        before_layout.addLayout(before_btns_layout)
        before_layout.addWidget(self.before_grid.frame)

        before_layout.addStretch()

        bottom_cam_description = CameraDescription(
            CameraType.ETHERNET, 'bottom_cam/image_raw', 'Bottom Camera', 640, 370
        )
        # Bottom cam
        self.bottom_cam = PauseableVideoWidget(bottom_cam_description)

        # After layout
        after_layout = QVBoxLayout()
        after_bttn_layout = QHBoxLayout()

        match_before = QPushButton('Match Before')
        match_before.setMaximumWidth(120)
        match_before.clicked.connect(self.before_grid.update_connected_grid)

        after_bttn_layout.addWidget(match_before)

        after_layout.addWidget(QLabel('After'), alignment=Qt.AlignmentFlag.AlignCenter)
        after_layout.addLayout(after_bttn_layout)
        after_layout.addWidget(self.after_grid.frame)

        after_layout.addStretch()

        # Result layout
        result_widget = QWidget()
        result_layout = QVBoxLayout()

        result_widget.setLayout(result_layout)

        self.before_label = QLabel('Before: ')
        self.after_label = QLabel('After: ')
        self.diff_label = QLabel()

        result_layout.addWidget(self.before_label)
        result_layout.addWidget(self.after_label)
        result_layout.addWidget(self.diff_label)

        # Add all sections to main layout
        root_layout.addLayout(before_layout, 1)
        root_layout.addWidget(self.bottom_cam, 1, alignment=Qt.AlignmentFlag.AlignTop)
        root_layout.addLayout(after_layout, 1)
        root_layout.addWidget(result_widget, 2)

        result_layout.addStretch()

        self.show()

    def update_result_text(self) -> None:
        before_num: int = self.before_grid.get_num_recovered()
        after_num: int = self.after_grid.get_num_recovered()

        diff: int = abs(after_num - before_num)

        if after_num > before_num:
            result = f'{diff} went from white to green'
        elif after_num < before_num:
            result = f'{diff} went from green to white'
        else:
            result = 'There was no change'

        self.before_label.setText(f'Before: {before_num} green')
        self.after_label.setText(f'After: {after_num} green')

        self.diff_label.setText(result)


class SeagrassGrid(QWidget):
    def __init__(
        self,
        update_result_text: Callable[[], None],
        set_other_button: Callable[[int, bool], None] | None = None,
    ) -> None:
        super().__init__()

        self.set_other_button: Callable[[int, bool], None] | None = set_other_button

        self.setMaximumWidth(200)

        grid_layout = QGridLayout()
        grid_layout.setSpacing(0)
        grid_layout.setContentsMargins(0, 0, 0, 0)

        self.setLayout(grid_layout)

        self.frame = QFrame()
        self.frame.setLayout(grid_layout)
        self.frame.setStyleSheet('border: 1px solid gray')

        self.all_buttons: list[SeagrassButton] = []
        N = 8
        button_id = 0

        for row in range(N):
            for col in range(N):
                seagrass_button: SeagrassButton = SeagrassButton(
                    button_id, 50, update_result_text, self.set_other_button
                )
                self.all_buttons.append(seagrass_button)

                grid_layout.addWidget(seagrass_button, row, col)

                button_id += 1

    def reset_grid(self, recovered: bool) -> None:
        for button in self.all_buttons:
            button.set_color(recovered)

    def get_num_recovered(self) -> int:
        num_recovered: int = 0

        for button in self.all_buttons:
            if button.recovered:
                num_recovered += 1

        return num_recovered

    def update_connected_grid(self) -> None:
        if self.set_other_button is None:
            raise ValueError('self.set_other_button has been called on after grid')

        for button in self.all_buttons:
            self.set_other_button(button.button_id, button.recovered)

    def set_button(self, button_id: int, recovered: bool) -> None:
        button = self.all_buttons[button_id]
        button.set_color(recovered)


class SeagrassButton(QPushButton):
    def __init__(
        self,
        button_id: int,
        size: int,
        update_text: Callable[[], None],
        set_other_button: Callable[[int, bool], None] | None = None,
    ) -> None:
        super(SeagrassButton, self).__init__()

        self.button_id: int = button_id
        self.setFixedSize(size, size)

        self.recovered = True
        self.setStyleSheet('border: 1px solid gray; background-color : green')

        self.update_text = update_text
        self.set_other_button = set_other_button

        self.clicked.connect(self.toggle_button_color)

    def toggle_button_color(self) -> None:
        self.recovered = not self.recovered

        self.set_color(self.recovered)

    def set_color(self, recovered: bool) -> None:
        self.recovered = recovered

        if recovered:
            color = 'green'
        else:
            color = 'white'

        self.setStyleSheet(f'border: 1px solid gray; background-color :{color}')

        # Update other button
        if self.set_other_button is not None:
            self.set_other_button(self.button_id, self.recovered)

        self.update_text()
