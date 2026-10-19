import base64
import hashlib
import io
import json
import shutil
import sys
import tarfile
import threading
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VENDOR = ROOT / "vendor"
THREE = "0.169.0"
TASKS = "0.10.21"
MODELS = ("lite", "full", "heavy")
PACKAGES = {
    "three": {
        "tarball": f"https://registry.npmjs.org/three/-/three-{THREE}.tgz",
        "integrity": "sha512-Ed906MA3dR4TS5riErd4QBsRGPcx+HBDX2O5yYE5GqJeFQTPU+M56Va/f/Oph9X7uZo3W3o4l2ZhBZ6f6qUv0w==",
        "keep": ("build/three.module.js", "build/three.module.min.js", "examples/jsm/", "LICENSE"),
        "dest": "three",
        "probe": "three/build/three.module.js",
        "cdn": f"https://cdn.jsdelivr.net/npm/three@{THREE}/",
    },
    "tasks-vision": {
        "tarball": f"https://registry.npmjs.org/@mediapipe/tasks-vision/-/tasks-vision-{TASKS}.tgz",
        "integrity": "sha512-TuhKH+credq4zLksGbYrnvJ1aLIWMc5r0UHwzxzql4BHECJwIAoBR61ZrqwGOW6ZmSBIzU1t4VtKj8hbxFaKeA==",
        "keep": ("vision_bundle.mjs", "wasm/", "README.md"),
        "dest": "mediapipe/tasks-vision",
        "probe": "mediapipe/tasks-vision/wasm/vision_wasm_internal.wasm",
        "cdn": f"https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@{TASKS}/",
    },
}
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_{k}/float16/latest/pose_landmarker_{k}.task"
LOCK = threading.Lock()
STATE = {"running": False, "error": None, "log": []}


def model_path(kind):
    return VENDOR / "mediapipe" / "models" / f"pose_landmarker_{kind}.task"


def cdn_url(rel):
    rel = rel.lstrip("/")
    for p in PACKAGES.values():
        if rel.startswith(p["dest"] + "/"):
            return p["cdn"] + rel[len(p["dest"]) + 1:]
    for k in MODELS:
        if rel == f"mediapipe/models/pose_landmarker_{k}.task":
            return MODEL_URL.format(k=k)
    return None


def status():
    out = {name: (VENDOR / p["probe"]).is_file() for name, p in PACKAGES.items()}
    out["models"] = [k for k in MODELS if model_path(k).is_file()]
    out["running"] = STATE["running"]
    out["error"] = STATE["error"]
    return out


def fetch(url, log):
    log(f"downloading {url}")
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def check_integrity(data, integrity):
    algo, want = integrity.split("-", 1)
    got = base64.b64encode(hashlib.new(algo, data).digest()).decode()
    if got != want:
        raise ValueError(f"integrity mismatch ({algo})")


def install_package(name, log):
    p = PACKAGES[name]
    data = fetch(p["tarball"], log)
    check_integrity(data, p["integrity"])
    dest = VENDOR / p["dest"]
    tmp = dest.with_name(dest.name + ".tmp")
    shutil.rmtree(tmp, ignore_errors=True)
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        for m in tar.getmembers():
            if not m.isfile() or not m.name.startswith("package/"):
                continue
            rel = m.name[len("package/"):]
            if ".." in Path(rel).parts or not any(rel == k or (k.endswith("/") and rel.startswith(k)) for k in p["keep"]):
                continue
            out = tmp / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(tar.extractfile(m).read())
    shutil.rmtree(dest, ignore_errors=True)
    tmp.rename(dest)
    log(f"installed {name} into {dest}")


def install_model(kind, log):
    data = fetch(MODEL_URL.format(k=kind), log)
    if len(data) < 1_000_000:
        raise ValueError(f"pose model {kind} is only {len(data)} bytes")
    out = model_path(kind)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".part")
    tmp.write_bytes(data)
    tmp.replace(out)
    log(f"installed pose model {kind}")


def ensure(models=MODELS, log=None):
    def say(msg):
        STATE["log"] = (STATE["log"] + [msg])[-20:]
        (log or print)(msg)
    with LOCK:
        STATE["running"], STATE["error"] = True, None
        try:
            for name, p in PACKAGES.items():
                if not (VENDOR / p["probe"]).is_file():
                    install_package(name, say)
            for k in models:
                if not model_path(k).is_file():
                    install_model(k, say)
        except Exception as e:
            STATE["error"] = f"{type(e).__name__}: {e}"
            say(f"vendor download failed: {STATE['error']}")
        finally:
            STATE["running"] = False
    return status()


def ensure_background(log=None):
    t = threading.Thread(target=ensure, kwargs={"log": log}, daemon=True)
    t.start()
    return t


if __name__ == "__main__":
    if "--check" in sys.argv:
        print(json.dumps(status(), indent=1))
        sys.exit(0)
    res = ensure()
    print(json.dumps(res, indent=1))
    sys.exit(1 if res["error"] else 0)
