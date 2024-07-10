"""Test mypy on this module."""

from pathlib import Path

import pytest
from ament_mypy.main import main


@pytest.mark.mypy()
@pytest.mark.linter()
def test_mypy() -> None:
    """Tests mypy on this module."""
    path = Path(Path.cwd()) / '..' / '..' / '..' / 'pyproject.toml'
    error_code = main(argv=['--config', path])
    assert error_code == 0, 'Found code style errors / warnings'
