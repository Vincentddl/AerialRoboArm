"""Windows 无控制台启动入口；启动异常写入本目录供排查。"""
from __future__ import annotations

import os
from pathlib import Path
import traceback

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)

try:
    from main import main
    main()
except Exception:
    (ROOT / "startup_error.log").write_text(traceback.format_exc(), encoding="utf-8")
    raise
