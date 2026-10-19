import hashlib
import json
import shutil
import subprocess
import sys
import threading
import time
import uuid
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from fulcrum_config import load as load_config
import fetch_vendor
CONFIG = load_config()
EXPORTS = CONFIG["exports"]
IMPORTS = CONFIG["imports"]
SAMPLES = CONFIG["samples"] or IMPORTS
BLENDER = CONFIG["blender"] or Path("blender.exe")
WORK = ROOT / "work" / "mocap"
FFMPEG = shutil.which("ffmpeg")
FFPROBE = shutil.which("ffprobe")
RATES = (23.976, 24.0, 25.0, 29.97, 30.0, 48.0, 50.0, 59.94, 60.0, 90.0, 100.0, 119.88, 120.0, 240.0)


def ratio(text):
    try:
        a, b = str(text).split("/") if "/" in str(text) else (text, 1)
        return float(a) / float(b) if float(b) else 0.0
    except (TypeError, ValueError):
        return 0.0


def probe_video(path):
    if not FFPROBE:
        return {}
    r = subprocess.run([FFPROBE, "-v", "error", "-select_streams", "v:0", "-show_entries",
                        "stream=codec_name,width,height,avg_frame_rate,r_frame_rate,nb_frames,duration:format=duration",
                        "-of", "json", str(path)], capture_output=True, text=True, timeout=60)
    try:
        info = json.loads(r.stdout)
    except ValueError:
        return {}
    st = (info.get("streams") or [{}])[0]
    fps = ratio(st.get("avg_frame_rate")) or ratio(st.get("r_frame_rate"))
    near = min(RATES, key=lambda x: abs(x - fps)) if fps else 0.0
    fps = near if fps and abs(near - fps) < 0.02 * fps else round(fps, 3)
    dur = ratio(st.get("duration")) or ratio((info.get("format") or {}).get("duration"))
    frames = int(st.get("nb_frames") or 0) or (round(dur * fps) if dur and fps else 0)
    return {"codec": st.get("codec_name"), "width": st.get("width"), "height": st.get("height"), "fps": fps, "duration": dur, "frames": frames}


def transcode(raw, name):
    WORK.mkdir(parents=True, exist_ok=True)
    stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in Path(name).stem)[:40] or "video"
    tag = hashlib.sha1(raw).hexdigest()[:10]
    src = WORK / f"{stem}-{tag}{Path(name).suffix.lower() or '.mp4'}"
    out = WORK / f"{stem}-{tag}.webm"
    if not out.is_file():
        src.write_bytes(raw)
        info = probe_video(src)
        fps = info.get("fps") or 30.0
        scale = "scale='if(gt(iw,ih),min(1280,iw),-2)':'if(gt(iw,ih),-2,min(1280,ih))'"
        cmd = [FFMPEG, "-y", "-v", "error", "-i", str(src), "-an", "-vf", scale, "-r", str(fps), "-c:v", "libvpx-vp9",
               "-deadline", "realtime", "-cpu-used", "8", "-row-mt", "1", "-b:v", "0", "-crf", "32", "-pix_fmt", "yuv420p", str(out.with_suffix(".part.webm"))]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
        if r.returncode != 0:
            return {"ok": False, "error": (r.stderr or "ffmpeg failed")[-2000:]}
        out.with_suffix(".part.webm").replace(out)
        src.unlink(missing_ok=True)
    info = probe_video(out)
    return {"ok": True, "url": "/work/mocap/" + out.name, **info}


BRIDGE = {"queue": [], "results": {}, "cv": threading.Condition(), "seen": 0.0, "client": None}


def bridge_call(cmd, args, timeout):
    if time.time() - BRIDGE["seen"] > 30:
        return {"ok": False, "error": "Fulcrum studio page is not open. Open http://localhost:8790/studio.html in a browser."}
    job = {"id": uuid.uuid4().hex, "cmd": cmd, "args": args}
    with BRIDGE["cv"]:
        BRIDGE["queue"].append(job)
        BRIDGE["cv"].notify_all()
        end = time.time() + timeout
        while job["id"] not in BRIDGE["results"]:
            left = end - time.time()
            if left <= 0:
                if job in BRIDGE["queue"]:
                    BRIDGE["queue"].remove(job)
                return {"ok": False, "error": "timed out after %ss waiting for the page" % timeout}
            BRIDGE["cv"].wait(left)
        return BRIDGE["results"].pop(job["id"])


def bridge_next(wait, client=None):
    if BRIDGE["client"] is not None and BRIDGE["client"] != client:
        return {"stale": True}
    BRIDGE["seen"] = time.time()
    with BRIDGE["cv"]:
        end = time.time() + wait
        while not BRIDGE["queue"]:
            left = end - time.time()
            if left <= 0:
                return {}
            BRIDGE["cv"].wait(left)
            BRIDGE["seen"] = time.time()
            if BRIDGE["client"] is not None and BRIDGE["client"] != client:
                return {"stale": True}
        return BRIDGE["queue"].pop(0)


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css",
                      ".html": "text/html", ".json": "application/json", ".wasm": "application/wasm", ".task": "application/octet-stream",
                      ".glb": "model/gltf-binary", ".gltf": "model/gltf+json", ".webm": "video/webm", ".mp4": "video/mp4", ".bvh": "text/plain",
                      ".fbx": "application/octet-stream", ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg"}

    def __init__(self, *a, **k):
        super().__init__(*a, directory=str(ROOT), **k)

    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = urlsplit(self.path).path
        if route.startswith("/vendor/"):
            rel = unquote(route[len("/vendor/"):])
            if ".." not in Path(rel).parts and not (fetch_vendor.VENDOR / rel).is_file():
                url = fetch_vendor.cdn_url(rel)
                if url:
                    self.send_response(302)
                    self.send_header("Location", url)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
            return super().do_GET()
        if route == "/api/mocap/status":
            return self.send_json({"ffmpeg": bool(FFMPEG and FFPROBE), "blender": BLENDER.is_file(), "vendor": fetch_vendor.status()})
        if self.path.split('?')[0] == "/api/bridge/next":
            client = self.path.split('client=', 1)[1] if 'client=' in self.path else None
            return self.send_json(bridge_next(20, client))
        if self.path == "/api/bridge/alive":
            return self.send_json({"page_seen_s_ago": round(time.time() - BRIDGE["seen"], 1) if BRIDGE["seen"] else None})
        if self.path == "/api/samples":
            files = sorted(p.name for p in SAMPLES.glob("*.fbx")) if SAMPLES.exists() else []
            extra = sorted(p.name for p in IMPORTS.glob("*.fbx") if p.name not in files) if IMPORTS.exists() else []
            return self.send_json({"dir": str(SAMPLES), "imports": str(IMPORTS), "files": files + extra})
        if self.path.startswith("/samples/"):
            p = SAMPLES / Path(unquote(self.path[len("/samples/"):])).name
            if not p.is_file():
                p = IMPORTS / p.name
            if not p.is_file():
                return self.send_json({"error": "not found"}, 404)
            data = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith("/exports/"):
            p = EXPORTS / Path(unquote(self.path[len("/exports/"):])).name
            if not p.is_file():
                return self.send_json({"error": "not found"}, 404)
            data = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n)
        if self.path.startswith("/api/glb-to-fbx"):
            if not BLENDER.is_file():
                return self.send_json({"ok": False, "error": "FBX export needs Blender. Install it from blender.org (or set \"blender\" in %APPDATA%\\Fulcrum\\fulcrum.json), then restart Fulcrum. The .glb export works without it."}, 503)
            q = parse_qs(urlsplit(self.path).query)
            name = "".join(c if c.isalnum() or c in "-_" else "_" for c in Path((q.get("name") or ["motion"])[0]).stem)[:60] or "motion"
            fps = (q.get("fps") or ["30"])[0]
            EXPORTS.mkdir(parents=True, exist_ok=True)
            glb, fbx = EXPORTS / f"{name}.glb", EXPORTS / f"{name}.fbx"
            glb.write_bytes(raw)
            cmd = [str(BLENDER), "-b", "--factory-startup", "-P", str(ROOT / "tools" / "glb_to_fbx.py"), "--", str(glb), str(fbx), str(fps)]
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
            except (OSError, subprocess.TimeoutExpired) as error:
                return self.send_json({"ok": False, "error": str(error)}, 500)
            log = [l for l in (r.stdout + r.stderr).splitlines() if l.startswith("FULCRUM")]
            ok = r.returncode == 0 and fbx.is_file()
            return self.send_json({"ok": ok, "fbx": f"/exports/{fbx.name}" if ok else None, "path": str(fbx), "log": log or (r.stdout + r.stderr)[-3000:].splitlines(), "error": None if ok else "Blender could not convert the file."})
        if self.path.startswith("/api/transcode"):
            if not (FFMPEG and FFPROBE):
                return self.send_json({"ok": False, "error": "This video format needs ffmpeg to convert it. Install ffmpeg (ffmpeg.org) and add it to PATH, or convert the video to MP4 (H.264) or WebM first."}, 503)
            name = (parse_qs(urlsplit(self.path).query).get("name") or ["video.mp4"])[0]
            try:
                return self.send_json(transcode(raw, name))
            except (OSError, subprocess.TimeoutExpired) as error:
                return self.send_json({"ok": False, "error": str(error)}, 500)
        if self.path == "/api/bridge/connect":
            req = json.loads(raw)
            with BRIDGE["cv"]:
                BRIDGE["client"] = str(req["client"])
                BRIDGE["seen"] = time.time()
                BRIDGE["cv"].notify_all()
            return self.send_json({"ok": True})
        if self.path == "/api/cascadeur-model":
            req = json.loads(raw)
            kind = req.pop("kind", "pose")
            scripts = {"pose": "cascadeur_pose_backend.py", "inbetween": "cascadeur_inbetween_backend.py"}
            if kind not in scripts:
                return self.send_json({"ok": False, "error": "Unknown model kind"}, 400)
            runtime = ROOT / ".venv-ml" / "Scripts" / "python.exe"
            if not runtime.is_file():
                return self.send_json({"ok": False, "error": "Local ML runtime is unavailable"}, 503)
            if kind == "inbetween":
                req["model_path"] = r"C:\re_work\fulcrum_cascadeur\inbetween\archive_0\model.pt"
            try:
                result = subprocess.run([str(runtime), str(ROOT / "tools" / scripts[kind])], input=json.dumps(req), capture_output=True, text=True, timeout=180)
                data = json.loads(result.stdout)
                data["ok"] = result.returncode == 0 and not data.get("error")
                return self.send_json(data, 200 if data["ok"] else 422)
            except (subprocess.TimeoutExpired, ValueError) as error:
                return self.send_json({"ok": False, "error": str(error)}, 500)
        if self.path == "/api/export-fbx":
            req = json.loads(raw)
            if not BLENDER.is_file():
                return self.send_json({"ok": False, "fbx": None, "error": "Blender not found. FBX export needs Blender: install it from blender.org or set \"blender\" in %APPDATA%\\Fulcrum\\fulcrum.json, then restart Fulcrum.", "log": ["FULCRUM Blender not found at " + str(BLENDER)]})
            EXPORTS.mkdir(parents=True, exist_ok=True)
            name = Path(req["name"]).stem
            skel = EXPORTS / Path(req["skeleton"]).name
            jpath = EXPORTS / f"{name}.frames.json"
            fpath = EXPORTS / f"{name}.fbx"
            jpath.write_text(json.dumps(req))
            cmd = [str(BLENDER), "-b", "--factory-startup", "-P", str(ROOT / "tools" / "frames_to_fbx.py"), "--", str(skel), str(jpath), str(fpath)]
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
            log = [l for l in (r.stdout + r.stderr).splitlines() if l.startswith("FULCRUM")]
            ok = r.returncode == 0 and fpath.exists()
            return self.send_json({"ok": ok, "fbx": f"/exports/{fpath.name}" if ok else None, "path": str(fpath), "log": log or (r.stdout + r.stderr)[-3000:].splitlines()})
        if self.path == "/api/bridge/call":
            req = json.loads(raw)
            return self.send_json(bridge_call(req["cmd"], req.get("args") or {}, float(req.get("timeout", 300))))
        if self.path == "/api/bridge/result":
            res = json.loads(raw)
            with BRIDGE["cv"]:
                BRIDGE["results"][res["id"]] = res
                BRIDGE["cv"].notify_all()
            return self.send_json({"ok": True})
        if self.path.startswith("/api/save/"):
            EXPORTS.mkdir(parents=True, exist_ok=True)
            p = EXPORTS / Path(unquote(self.path[len("/api/save/"):])).name
            p.write_bytes(raw)
            return self.send_json({"ok": True, "path": str(p)})
        return self.send_json({"error": "unknown"}, 404)

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else CONFIG["port"]
    fetch_vendor.ensure_background(log=lambda m: print("vendor:", m, flush=True))
    print(f"Fulcrum on http://localhost:{port}  (studio: http://localhost:{port}/studio.html)", flush=True)
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
