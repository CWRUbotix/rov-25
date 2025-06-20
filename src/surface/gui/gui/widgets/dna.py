import re
from pathlib import Path

from pypdf import PdfReader
from PyQt6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

DNA_NEEDLES = [
    [
        'Bighead Carp',
        'AACTTAAATAAACAGATTATTCCACTAACAATTGATTCTCAAATTTATTACTGAATTATTAACTAAAATCTAACTCAAGTATATTATTAAAGTAAGAGACCACCTACTTATTTATATTAAGGTATTATATTCATGATAAGATCAAGGACAATAACAGTGGGGGTGGCGCAAAATGAACTATTACTTGCATCTGGTTTGGAATCTCACGGACATGGCTACAAAATTCCACCCCCGTTACATTATAACTGGCATATGGTTAAATGATGTGAGTACATACTCCTCATTAACCCCACATGCCGAGCATTCTTTTAT',
    ],
    [
        'Silver Carp',
        'CCTGAGAAAAGAGTTGTTCCACTATAATTGGTTCTCAAATATTTCCTTGAAATATTAACTTCTATTTAATTTAACTATATTAATGTAGTAAGAAACCACCTACTGGTTTATATTAAGGTATTCTATTCATGATAAGATCAGGGACAATAATCGTGGGGGTGGCGCAGAATGAACTATTACTTGCATTTGGC',
    ],
    [
        'Grass Carp',
        'GAGTTTCTGACTTCTACCCCCTTCTTTCCTCCTACTATTAGCCTCTTCTGGTGTTGAGGCCGGAGCTGGAACAGGGTGAACAG',
    ],
    [
        'Black Carp',
        'ACACCACGTTCTTTGACCCAGCAGGCGGAGGAGACCCAATCCTATATCAACACCTGTTCTGATTCTTCGGCCACCCAGAAGTTTACATTCTTATTTTACCCGGGTTTGGGATCATTTCAC',
    ],
]


class DNA(QWidget):
    def __init__(self) -> None:
        super().__init__()

        self.root_layout = QVBoxLayout()
        self.base_layout = QHBoxLayout()

        self.input_layout = QVBoxLayout()
        self.auto_output_layout = QVBoxLayout()
        self.manual_output_layout = QVBoxLayout()

        file_picker_layout = QHBoxLayout()

        self.sample = QPlainTextEdit()

        file_browse = QPushButton('Browse')
        file_browse.clicked.connect(self.open_file_dialog)
        self.filename = QLineEdit()

        show_button = QPushButton('Search Database', None)

        show_button.clicked.connect(self.display_result)

        self.check_text = QTextEdit()

        self.input_layout.addWidget(QLabel('Select PDF File: '))
        file_picker_layout.addWidget(self.filename)
        file_picker_layout.addWidget(file_browse)
        self.input_layout.addLayout(file_picker_layout)

        self.input_layout.addWidget(QLabel('Or enter sample directly: '))
        self.input_layout.addWidget(self.sample)

        self.input_layout.addWidget(show_button)

        self.base_layout.addLayout(self.input_layout)

        self.base_layout.addLayout(self.auto_output_layout)
        self.base_layout.addLayout(self.manual_output_layout)
        self.root_layout.addLayout(self.base_layout)
        self.root_layout.addWidget(QLabel('Check DNA Samples:'))
        self.root_layout.addWidget(self.check_text)

        self.manual_result: QLabel | None = None
        self.auto_results: list[QLabel] = []

        self.setLayout(self.root_layout)

    def open_file_dialog(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            caption='Select a File', filter='PDF files (*.pdf)'
        )
        if filename:
            path = Path(filename)
            self.text_file = str(path)
            self.filename.setText(self.text_file)

    def find_samples(self, pdf_file: str) -> list[str]:
        reader = PdfReader(pdf_file)
        samples = []
        self.check_text.setText('')
        for page in reader.pages:
            text = page.extract_text()
            last_index = 0
            while text.find('Unknown Sample', last_index) != -1:
                current_index = text.find(
                    'Unknown Sample', last_index
                )  # Roughly identify location of next sample
                sample_index = text.find('\n', current_index) + 1  # Find start of sample
                sample_index_end = text.find(' \n', sample_index)  # Find end of sample
                sample = text[sample_index:sample_index_end].replace('\n', '')
                samples.append(sample)
                self.check_sample(sample, len(samples))
                last_index = sample_index_end
        return samples

    def check_sample(self, sample: str, sample_num: int) -> None:
        # Check if sample has characters not corresponding to a DNA base
        if sample.replace('A', '').replace('C', '').replace('G', '').replace('T', '') != '':
            self.check_text.setText(
                'Sample '
                + str(sample_num)
                + ' has characters other than ACGT\n\n'
                + self.check_text.toPlainText()
            )

        # Display all samples for manual checking
        self.check_text.setText(
            self.check_text.toPlainText() + ('Sample ' + str(sample_num) + ': ' + sample) + '\n\n'
        )

    def display_result(self) -> None:
        for result in self.auto_results:
            self.auto_output_layout.removeWidget(result)
        self.manual_output_layout.removeWidget(self.manual_result)

        if self.filename.text():
            self.auto_results = []
            samples = self.find_samples(self.filename.text())
            for i, sample in enumerate(samples):
                self.auto_results.append(QLabel(str(i + 1) + ': ' + self.search(sample)))
        if self.sample.toPlainText():
            sample = self.sample.toPlainText()
            sample = re.sub('[^ACGT]', '', sample) # Remove every character except ACGT
            self.manual_result = QLabel('Manual: ' + self.search(sample))

        for result in self.auto_results:
            self.auto_output_layout.addWidget(result)
        self.manual_output_layout.addWidget(self.manual_result)

    def search(self, sample: str) -> str:
        for name, substr in DNA_NEEDLES:
            if sample.find(substr) != -1:
                return name

        return 'No match'
