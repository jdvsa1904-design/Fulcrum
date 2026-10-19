import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULTS = {
    "taste": "",
    "samples": "{root}/samples",
    "blender": "",
    "exports": "{root}/exports",
    "imports": "{root}/imports",
    "port": 8790,
}


def documents():
    try:
        import ctypes
        import uuid
        guid = (ctypes.c_byte * 16).from_buffer_copy(uuid.UUID("FDD39AD0-238F-46AF-ADB4-6C85480369C7").bytes_le)
        ptr = ctypes.c_wchar_p()
        if ctypes.windll.shell32.SHGetKnownFolderPath(ctypes.byref(guid), 0, None, ctypes.byref(ptr)) == 0:
            path = ptr.value
            ctypes.windll.ole32.CoTaskMemFree(ptr)
            return path
    except Exception:
        pass
    return str(Path.home() / "Documents")


def user_file():
    return Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming") / "Fulcrum" / "fulcrum.json"


def read(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return {}


def raw():
    cfg = dict(DEFAULTS)
    cfg.update(read(ROOT / "fulcrum.json"))
    cfg.update(read(user_file()))
    return cfg


def expand(value):
    if not value:
        return None
    s = str(value).replace("{root}", str(ROOT)).replace("{documents}", documents())
    return Path(os.path.expandvars(os.path.expanduser(s))).resolve()


def find_blender():
    found = []
    for base in {os.environ.get("ProgramFiles"), os.environ.get("ProgramW6432"), r"C:\Program Files"}:
        if base:
            found += Path(base, "Blender Foundation").glob("*/blender.exe")
    def ver(p):
        return [int(x) if x.isdigit() else 0 for x in p.parent.name.replace("Blender", "").strip().split(".")]
    return max(found, key=ver) if found else None


def blender(cfg):
    p = expand(cfg.get("blender"))
    if p and p.is_file():
        return p
    return find_blender() or p


def load():
    cfg = raw()
    return {
        "taste": expand(cfg.get("taste")),
        "samples": expand(cfg.get("samples")),
        "blender": blender(cfg),
        "exports": expand(cfg.get("exports")) or ROOT / "exports",
        "imports": expand(cfg.get("imports")) or ROOT / "imports",
        "port": int(cfg.get("port") or 8790),
    }
