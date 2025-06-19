from pathlib import Path

from PyQt6.QtWidgets import (
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

START_YEAR = 2016
END_YEAR = 2025

DNA_NEEDLES = [
    ['Bighead Carp','AACTTAAATAAACAGATTATTCCACTAACAATTGATTCTCAAATTTATTACTGAATTATTAACTAAAATCTAACTCAAGTATATTATTAAAGTAAGAGACCACCTACTTATTTATATTAAGGTATTATATTCATGATAAGATCAAGGACAATAACAGTGGGGGTGGCGCAAAATGAACTATTACTTGCATCTGGTTTGGAATCTCACGGACATGGCTACAAAATTCCACCCCCGTTACATTATAACTGGCATATGGTTAAATGATGTGAGTACATACTCCTCATTAACCCCACATGCCGAGCATTCTTTTAT'],
    ['Silver Carp','CCTGAGAAAAGAGTTGTTCCACTATAATTGGTTCTCAAATATTTCCTTGAAATATTAACTTCTATTTAATTTAACTATATTAATGTAGTAAGAAACCACCTACTGGTTTATATTAAGGTATTCTATTCATGATAAGATCAGGGACAATAATCGTGGGGGTGGCGCAGAATGAACTATTACTTGCATTTGGC'],
    ['Grass Carp','GAGTTTCTGACTTCTACCCCCTTCTTTCCTCCTACTATTAGCCTCTTCTGGTGTTGAGGCCGGAGCTGGAACAGGGTGAACAG'],
    ['Black Carp','ACACCACGTTCTTTGACCCAGCAGGCGGAGGAGACCCAATCCTATATCAACACCTGTTCTGATTCTTCGGCCACCCAGAAGTTTACATTCTTATTTTACCCGGGTTTGGGATCATTTCAC']
]

class DNA(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.root_layout = QHBoxLayout()

        input_layout = QVBoxLayout()

        file_picker_layout = QHBoxLayout()

        self.sample = QLineEdit()

        form = QFormLayout()
        form.addRow('Sample', self.sample)

        file_browse = QPushButton('Browse')
        file_browse.clicked.connect(self.open_file_dialog)
        self.filename = QLineEdit()

        show_button = QPushButton('Search Database', None)

        show_button.clicked.connect(self.display_result)

        self.setLayout(self.root_layout)

        input_layout.addLayout(form)

        file_picker_layout.addWidget(self.filename)
        file_picker_layout.addWidget(file_browse)
        input_layout.addLayout(file_picker_layout)

        input_layout.addWidget(show_button)
        input_layout.addStretch()

        self.root_layout.addLayout(input_layout)

        self.results: list[QLabel] | None = None

    def open_file_dialog(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            caption='Select a File',
            filter='Text files (*.txt)'
        )
        if filename:
            path = Path(filename)
            self.text_file = str(path)
            self.filename.setText(self.text_file)

    def display_result(self) -> None:
        self.root_layout.removeWidget(self.results)
        if self.filename.text():
            self.results = []
            with Path.open(self.filename.text(), 'r') as samples_file:
                samples = samples_file.readlines()
                for i, sample in enumerate(samples):
                    self.results.append(str(i+1) + ': ' + QLabel(self.search(sample)))
        else:
            self.results = [QLabel(self.search(self.sample.text()))]

        for result in self.results:
            self.root_layout.addWidget(result)

    def search(self, samples: list[str]) -> str:
        for (name, substr) in DNA_NEEDLES:
            if samples.find(substr) != -1:
                return name

        return 'No match'


