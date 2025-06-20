from PyQt6.QtWidgets import QVBoxLayout, QWidget

from gui.widgets.dna import DNA


class DNATab(QWidget):
    def __init__(self) -> None:
        super().__init__()

        root_layout = QVBoxLayout()
        root_layout.addWidget(DNA())
        self.setLayout(root_layout)
