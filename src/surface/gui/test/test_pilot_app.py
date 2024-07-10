from gui.pilot_app import PilotApp
from pytestqt.qtbot import QtBot


def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for PilotApp instantiation."""
    app = PilotApp()
    app.show()
    qtbot.addWiget(app)
