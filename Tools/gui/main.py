# -*- coding: utf-8 -*-
"""dart_final_test_app 可视化上位机 (输入=滑条命令, 输出=STAT 文本)。

运行:
    python main.py

依赖: PyQt5, pyserial (见 requirements.txt)。
打包: powershell -File build_exe.ps1
"""

from __future__ import annotations

import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402
except ImportError:
    sys.stderr.write(
        "PyQt5 未安装。请先:  python -m pip install -r requirements.txt\n")
    raise SystemExit(1)

from gui.main_window import MainWindow  # noqa: E402


def _excepthook(exc_type, exc_value, exc_tb) -> None:
    text = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
    sys.stderr.write(text)
    try:
        QMessageBox.critical(None, "Unexpected error", text)
    except Exception:  # noqa: BLE001
        pass


def main() -> int:
    sys.excepthook = _excepthook
    app = QApplication(sys.argv)
    app.setApplicationName("Dart Test Console")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
