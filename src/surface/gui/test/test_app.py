import pytest
from gui.app import App
from pytestqt.qtbot import QtBot


@pytest.fixture()
def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for App instantiation."""
    app = App('test')
    app.show()
    qtbot.addWiget(app)
