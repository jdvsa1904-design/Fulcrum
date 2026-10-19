import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PORT = 8790
URL = f"http://127.0.0.1:{PORT}"
EDGE = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]


def up():
    try:
        with urllib.request.urlopen(URL + "/api/bridge/alive", timeout=1) as r:
            return r.status == 200
    except Exception:
        return False


def start_server():
    log = open(ROOT / "exports" / "server.log", "a", encoding="utf-8")
    python = Path(sys.executable).with_name("python.exe")
    subprocess.Popen(
        [str(python if python.exists() else sys.executable), str(ROOT / "server.py"), str(PORT)],
        cwd=str(ROOT), stdout=log, stderr=log, stdin=subprocess.DEVNULL,
        creationflags=subprocess.CREATE_NO_WINDOW | subprocess.DETACHED_PROCESS,
    )
    for _ in range(100):
        if up():
            return True
        time.sleep(0.1)
    return False


def open_window():
    edge = next((p for p in EDGE if os.path.exists(p)), None)
    profile = ROOT / ".appprofile"
    if edge:
        subprocess.Popen([edge, f"--app={URL}/studio.html", f"--user-data-dir={profile}", "--window-size=1600,1000",
                          "--disable-background-timer-throttling", "--disable-renderer-backgrounding",
                          "--disable-backgrounding-occluded-windows"])
    else:
        os.startfile(URL + "/studio.html")


if not up():
    start_server()
open_window()
