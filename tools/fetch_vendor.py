import io
import shutil
import sys
import tarfile
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
VENDOR = ROOT / "vendor"
THREE = "0.169.0"
VISION = "0.10.21"
NPM = "https://registry.npmjs.org"
MODELS = {
    "lite": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task",
    "full": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task",
    "heavy": "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task",
}
THREE_FILES = ["build/three.module.js"]
THREE_DIRS = ["examples/jsm/controls/", "examples/jsm/exporters/", "examples/jsm/loaders/", "examples/jsm/utils/", "examples/jsm/libs/", "examples/jsm/curves/"]


def get(url):
    print("  fetching", url, flush=True)
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def extract(tgz, dest, keep):
    with tarfile.open(fileobj=io.BytesIO(tgz), mode="r:gz") as tf:
        for m in tf.getmembers():
            if not m.isfile():
                continue
            rel = m.name.split("/", 1)[1]
            if not keep(rel) or ".." in rel:
                continue
            out = dest / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(tf.extractfile(m).read())


def missing(models=("full",)):
    need = [VENDOR / "three" / "build" / "three.module.js", VENDOR / "three" / "examples" / "jsm" / "exporters" / "GLTFExporter.js",
            VENDOR / "tasks-vision" / "vision_bundle.mjs", VENDOR / "tasks-vision" / "wasm" / "vision_wasm_internal.wasm"]
    need += [VENDOR / "models" / f"pose_landmarker_{m}.task" for m in models]
    return [p for p in need if not p.is_file()]


def fetch(models=("full",), force=False):
    VENDOR.mkdir(exist_ok=True)
    three = VENDOR / "three"
    if force or not (three / "examples" / "jsm" / "exporters" / "GLTFExporter.js").is_file():
        shutil.rmtree(three, ignore_errors=True)
        extract(get(f"{NPM}/three/-/three-{THREE}.tgz"), three, lambda r: r in THREE_FILES or any(r.startswith(d) for d in THREE_DIRS))
    vision = VENDOR / "tasks-vision"
    if force or not (vision / "wasm" / "vision_wasm_internal.wasm").is_file():
        shutil.rmtree(vision, ignore_errors=True)
        extract(get(f"{NPM}/@mediapipe/tasks-vision/-/tasks-vision-{VISION}.tgz"), vision, lambda r: r == "vision_bundle.mjs" or r.startswith("wasm/"))
    (VENDOR / "models").mkdir(exist_ok=True)
    for m in models:
        p = VENDOR / "models" / f"pose_landmarker_{m}.task"
        if force or not p.is_file():
            data = get(MODELS[m])
            p.write_bytes(data)
    return missing(models)


def ensure(models=("full",)):
    if not missing(models):
        return True
    print("Fulcrum: downloading runtime libraries and pose models into vendor/ (first run only)...", flush=True)
    try:
        left = fetch(models)
    except Exception as e:
        print("Fulcrum: vendor download failed:", e, flush=True)
        return False
    if left:
        print("Fulcrum: still missing", *left, flush=True)
    return not left


if __name__ == "__main__":
    args = sys.argv[1:]
    models = [a for a in args if a in MODELS] or ["lite", "full", "heavy"]
    left = fetch(models, force="--force" in args)
    print("missing:", left if left else "nothing")
    sys.exit(1 if left else 0)
