import signal
import sys

from PyQt5.QtWidgets import QApplication

from .main_window import GuiSignals, MainWindow
from .ros_adapter import RosAdapter


def main():
    app = QApplication(sys.argv)
    signals = GuiSignals()
    ros = RosAdapter(
        status_callback=signals.log,
        feedback_callback=signals.nav_feedback,
        result_callback=signals.nav_result,
        mission_status_callback=signals.mission_status,
    )
    window = MainWindow(ros, signals)
    app.aboutToQuit.connect(window.shutdown)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())
    window.show()
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        window.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
