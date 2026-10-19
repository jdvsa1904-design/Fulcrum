import base64
import json
import os
import urllib.request
from pathlib import Path

from mcp.server.mcpserver import MCPServer, Image

BASE = os.environ.get("FULCRUM_URL", "http://127.0.0.1:8790")
SHOTS = Path(__file__).resolve().parents[1] / "exports" / "shots"

mcp = MCPServer("fulcrum")


def call(cmd, timeout=300, **args):
    body = json.dumps({"cmd": cmd, "args": args, "timeout": timeout}).encode()
    req = urllib.request.Request(BASE + "/api/bridge/call", data=body, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout + 10) as r:
            res = json.loads(r.read().decode())
    except Exception as e:
        raise RuntimeError(f"Fulcrum server not reachable at {BASE} ({e}). Start it with: python C:/Projects/Fulcrum/server.py 8790, then open {BASE}/studio.html in a browser.")
    if not res.get("ok"):
        raise RuntimeError(res.get("error", "unknown error"))
    return res["result"]


@mcp.tool()
def status() -> dict:
    """Current take, fixes, locks, pins, fidelity gate and physics flags."""
    return call("status")


@mcp.tool()
def load_frt() -> dict:
    """Load the Checkpoint soldier and the three FRT shots (clears fixes)."""
    return call("load_frt")


@mcp.tool()
def guided_motion(action: str, options: dict | None = None) -> dict:
    """Guided motion workflow: inbetween, suggest_pose, cascadeur_pose, next_pose, physics, secondary create previews; accept/reject resolve a preview; weapon_grip captures an attached prop and two-hand grip; undo/redo, validate, bookmark. Select a frame range first. cascadeur_pose requires a selected joint that was actually moved, plus explicit options.poseSnap (six numbers) and options.poseSpine; these are not native Cascadeur defaults."""
    return call("guided_motion", action=action, options=options or {})


@mcp.tool()
def motion_quality() -> dict:
    """Per-frame support polygons and balance margins, required foot/hand/weapon residuals in cm, bone-length integrity and ground penetration. Required constraints over 3cm block export unless overridden."""
    return call("motion_quality")


@mcp.tool()
def cascadeur_model(request: dict) -> dict:
    """Run the recovered local Cascadeur TorchScript models with explicit canonical43 tensors. kind=pose modes metadata/init_dc/pose/auto_unlock; kind=inbetween requires positions[K,43,3],frames[K],tpose[43,3],style (numeric label). Ordinary Fulcrum skeletons require the control-rig adapter. Returns model output without automatically modifying animation."""
    req = urllib.request.Request(BASE + "/api/cascadeur-model", data=json.dumps(request).encode(), headers={"Content-Type":"application/json"})
    with urllib.request.urlopen(req, timeout=190) as response:
        return json.loads(response.read().decode())


@mcp.tool()
def open_project(name: str) -> dict:
    """Open a saved project from C:/Projects/Fulcrum/exports by file name, e.g. FRT_fulcrum_v6.fulcrum.json."""
    return call("open_project", name=name)


@mcp.tool()
def save_project(name: str) -> dict:
    """Save the current project to exports/<name>.fulcrum.json."""
    return call("save_project", name=name)


@mcp.tool()
def add_fix(fix: dict) -> dict:
    """Append one fix and rebuild. Types: travel{a,b,speed m/s,ease,plant,reach|footsteps,balance,width,cycle,contact} (footsteps=True places Biped footprints: run contact 0.6 leg/speed, COM ahead of the stance foot by 0.9595 v + 0.0107 leg, half width 0.1 leg; balance = Biped balanceFactor), story{at,pre,post,curve,match}, lean{a,b,deg,ease}, smooth{a,b}, ground{a,b,side?}, momentum{a,b}, ballistic{a,b}, floor{a,b}|{whole:true}, level{side?}, roll{a,b,side}, seams{blend}, pull{pull,gravity}, despike{a,b}, filter{a,b,fc}, median{a,b,window}, plus any type app.js understands."""
    return call("add_fix", fix=fix)


@mcp.tool()
def set_fixes(fixes: list) -> dict:
    """Replace the whole fix list and rebuild."""
    return call("set_fixes", fixes=fixes)


@mcp.tool()
def cut(a: int, b: int, blend: int = 6, story: bool = False, curve: str = "ease", match: str = "gravity") -> dict:
    """Delete frames a..b from the take and remap every fix/lock/pin/layer key. Default: carry the pelvis across the join and smooth +-blend frames. story=True instead crossfades the outgoing clip (the source frames that were cut) into the incoming one over +-blend frames like a MotionBuilder Story transition: curve ease|linear|slow_in|fast_in, match gravity (horizontal hips match) | direct | none."""
    return call("cut", a=a, b=b, blend=blend, story=story, curve=curve, match=match)


@mcp.tool()
def insert(at: int, a: int, b: int, blend: int = 6, story: bool = False, curve: str = "ease", match: str = "gravity") -> dict:
    """Copy frames a..b of the current take and insert them before frame `at`; later fixes/locks shift. Default: pelvis carry plus smoothing at both joins. story=True crossfades each join with Story transitions (see cut)."""
    return call("insert", at=at, a=a, b=b, blend=blend, story=story, curve=curve, match=match)


@mcp.tool()
def import_fbx(name: str, mode: str = "hik", as_body: bool = False, replace: bool = True, realistic: float = 0.0, leg_roll: float = 0.0) -> dict:
    """Load an animated humanoid FBX by file name (from C:/Projects/Fulcrum/imports or the Checkpoint samples) and retarget it onto the loaded body. mode hik = HumanIK characterization + retarget (any naming, axis or scale), names = by bone name, auto = names when 80% of the core bones match. replace=False appends it as another shot. realistic/leg_roll are HIK RealisticKneeSolving and LegRoll. as_body=True loads it as the target character instead (any humanoid naming: Unreal, Mixamo, VRM, Rigify, Biped) and returns rigmap {confidence 0-1, missing canonical slots, aliased canonical->bone names, map}; fixes then resolve Unreal names through the map and exports keep the body's own bone names."""
    return call("import_fbx", timeout=600, name=name, mode=mode, as_body=as_body, replace=replace, realistic=realistic, leg_roll=leg_roll)


@mcp.tool()
def set_anim_layers(layers: list) -> dict:
    """Replace the Maya-style animation layers: [{name, weight 0-1, mute, override, passthrough, accumulation 'layer'|'component', tangent auto|autoease|automix|spline|clamped|plateau|linear|flat|step, bones: {bone: [{f, e:[x,y,z] degrees, tangent?}]}}]. Layers stack in order on top of the pose layer; additive keys are Euler offsets, override keys are local rotations (rotate order xyz)."""
    return call("set_anim_layers", layers=layers)


@mcp.tool()
def key_anim_layer(bone: str, f: int, layer: int = 0, euler: list | None = None, q: list | None = None) -> dict:
    """Set one key on an animation layer. Give euler [x,y,z] degrees as the layer value, or q [x,y,z,w] as the wanted local rotation and the layer value is solved through the layers below it."""
    return call("key_anim_layer", layer=layer, bone=bone, f=f, euler=euler, q=q)


@mcp.tool()
def sole_markers(frame: int | None = None, contact_size: float = 100) -> dict:
    """HumanIK sole contact markers (cm) for both feet at a frame. contact_size scales the drawn markers only; the floor solver always uses 100, as HumanIK 2016 does."""
    return call("sole_markers", f=frame, contact_size=contact_size)


@mcp.tool()
def remove_fix(index: int) -> dict:
    """Remove the fix at index (see status().fixes) and rebuild."""
    return call("remove_fix", index=index)


@mcp.tool()
def set_locks(locks: list) -> dict:
    """Replace foot locks: [{side:'l'|'r', a, b}], then rebuild."""
    return call("set_locks", locks=locks)


@mcp.tool()
def set_pins(pins: list) -> dict:
    """Replace hand pins: [{a,b,ref,off:16 floats}] grip pins or [{a,b,ref,world:16 floats}] world pins, then rebuild."""
    return call("set_pins", pins=pins)


@mcp.tool()
def set_layer(bone: str, keys: list) -> dict:
    """Set the additive pose-layer keys for one bone: [{f, q:[x,y,z,w]}] local delta quaternions; empty list clears it."""
    return call("set_layer", bone=bone, keys=keys)


@mcp.tool()
def select(a: int | None = None, b: int | None = None) -> dict:
    """Set the frame selection used by range tools (None clears it)."""
    return call("select", a=a, b=b)


@mcp.tool()
def retime(scale: float) -> dict:
    """Retime the currently selected frame range by a positive scale (greater than 1 makes it longer). Frames, shots, keys, fixes, locks and pins are remapped locally."""
    return call("retime", scale=scale)


@mcp.tool()
def auto_fix(max_rounds: int = 4) -> dict:
    """Run the Auto Fix loop: judge with Fulcrum's own checks (physics flags for foot slide, float and pose snaps; motion quality for feet below the floor, failed foot locks and hand pins, invalid transforms; sliding foot plants), plan one fix per blocker (ground, floor, despike, foot locks, level, smooth, travel toe/pitch), apply each through fixes/locks + rebuild and keep it only if blockers (then warnings) drop. Returns start/end blocker counts, kept and reverted changes and the final state."""
    return call("auto_fix", timeout=1800, max_rounds=max_rounds)


@mcp.tool()
def graph_editor(open: bool = True, bone: str | None = None, key_frame: int | None = None, layer: int | None = None) -> dict:
    """Open or close the graph editor panel bound to the active animation layer and the selected joint. bone selects a joint (canonical or body name), layer picks the active layer, key_frame keys the joint's current pose into the layer at that frame (creates a layer if none). Returns the curves shown."""
    return call("graph_editor", open=open, bone=bone, key_frame=key_frame, layer=layer)


@mcp.tool()
def pose(frames: list[int], bones: list[str] | None = None) -> dict:
    """World joint positions (cm, Y-up) for the given frames; default bones are pelvis, spine, head, hands, legs, feet."""
    return call("pose", frames=frames, bones=bones or [])


@mcp.tool()
def screenshot(frame: int | None = None, view: str = "side", width: int = 640, height: int = 640) -> list:
    """Render the soldier at a frame from a fixed camera (side, front, back, top, three_quarter) and return the image."""
    r = call("screenshot", f=frame, view=view, width=width, height=height)
    SHOTS.mkdir(parents=True, exist_ok=True)
    p = SHOTS / f"f{r['frame']:04d}_{view}.png"
    p.write_bytes(base64.b64decode(r["png"]))
    return [f"frame {r['frame']} view {view}: {p}", Image(path=str(p))]


@mcp.tool()
def export_fbx(name: str, override: bool = False) -> dict:
    """Export the current output to exports/<name>.fbx through Blender with the round-trip check. Blocked by motion-quality blockers (failed foot locks or hand pins, invalid transforms) unless override=True."""
    return call("export_fbx", timeout=900, name=name, override=override)


@mcp.tool()
def eval_js(code: str) -> object:
    """Run JavaScript inside the Fulcrum page for anything the other tools do not cover. The body runs as an async function with S (app state), THREE and api {rebuild, autoFix, showFrame, idx, firstIdx, wpos}; return a JSON-serialisable value."""
    return call("eval", code=code)


if __name__ == "__main__":
    mcp.run()
