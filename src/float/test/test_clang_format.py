"""Test clang-format on this module."""

import os
import shutil
from typing import Literal

import pytest
from ament_clang_format.main import main


INO_EXTENSION = ".ino"
SKETCHES = ["float_transceiver", "surface_transceiver"]
SRC = "src"
INCLUDE = "include"


@pytest.mark.linter
def test_clang_format() -> None:
    """Tests clang-format on this module."""

    regular_cpp = [
        *[os.path.join(os.getcwd(), folder) for folder in [SRC, INCLUDE]],
        *[os.path.join(os.getcwd(), folder, folder + INO_EXTENSION) for folder in SKETCHES]
    ]

    config_file = os.path.join(os.getcwd(), '.clang-format.yaml')

    error_codes: list[Literal[0, 1]] = []

    for file in regular_cpp:
        error_code = main(argv=['paths', file, '--config', config_file])
        error_codes.append(error_code)

    # Done so code fails "slow"
    for code in error_codes:
        assert code == 0, 'Found code style errors / warnings'
