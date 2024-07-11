from gui.app import App
from pytestqt.qtbot import QtBot


def test_app_instantiation(qtbot: QtBot) -> None:
    """Unit test for App instantiation."""
    app = App('test')
    app.show()
    qtbot.addWidget(app)
