import sys

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

from .main_window import FiveBarSerialGUI


def run_app() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("SCARA_UI")
    app.setOrganizationName("SCARA Course Design")
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei", 9))

    window = FiveBarSerialGUI()
    window.show()
    return app.exec()
