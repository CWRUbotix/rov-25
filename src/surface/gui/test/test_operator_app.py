import pytest
from gui.operator_app import OperatorApp
from pytestqt.qtbot import QtBot


@pytest.fixture()
def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for OperatorApp instantiation."""
    app = OperatorApp()
    app.show()
    qtbot.addWiget(app)
