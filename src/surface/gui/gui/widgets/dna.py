from pathlib import Path

from pypdf import PdfReader
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

        self.input_layout = QVBoxLayout()
        self.output_layout = QVBoxLayout()

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

        self.input_layout.addLayout(form)

        file_picker_layout.addWidget(self.filename)
        file_picker_layout.addWidget(file_browse)
        self.input_layout.addLayout(file_picker_layout)

        self.input_layout.addWidget(show_button)
        self.input_layout.addStretch()

        self.root_layout.addLayout(self.input_layout)
        self.root_layout.addLayout(self.output_layout)

        self.results: list[QLabel] = []

    def open_file_dialog(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            caption='Select a File',
            filter='PDF files (*.pdf)'
        )
        if filename:
            path = Path(filename)
            self.text_file = str(path)
            self.filename.setText(self.text_file)

    def display_result(self) -> None:
        for result in self.results:
            self.output_layout.removeWidget(result)

        if self.filename.text():
            self.results = []
            samples = self.find_samples("/home/bunando/Downloads/EX PN RN Sample eDNA profile 1.pdf")
            for i, sample in enumerate(samples):
                self.results.append(QLabel(str(i+1) + ': ' + self.search(sample)))
        else:
            self.results = [QLabel(self.search(self.sample.text()))]

        for result in self.results:
            self.output_layout.addWidget(result)

    def search(self, sample: str) -> str:
        for (name, substr) in DNA_NEEDLES:
            if sample.find(substr) != -1:
                return name

        return 'No match'

    def find_samples(self, pdf_file: str) -> list[str]:
        reader = PdfReader(pdf_file)
        samples = []
        for page in reader.pages:
            text = page.extract_text()
            last_index = 0
            while text.find('Unknown Sample',last_index) != -1:
                current_index = text.find('Unknown Sample', last_index) # Roughly identify location of next sample
                sample_index = text.find('\n', current_index) + 1 # Find start of sample
                sample_index_end = text.find(' \n', sample_index) # Find end of sample
                samples.append(text[sample_index:sample_index_end].replace('\n', ''))
                last_index = sample_index_end
        return samples
