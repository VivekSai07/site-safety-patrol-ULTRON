"""Local-only simulated B2 teleoperation and evidence capture.

Run this module inside the organizer kit image with its ``/opt/unitree-venv``
Python. It uses the official kit B2 model for kinematic placement only and
never creates a DDS, ROS, SDK, or hardware command path.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Protocol
import uuid

from .capture_session import CaptureSession
from .kit_sim import KitSimulation, encode_png


_ACTIONS = frozenset({"forward", "back", "turn_left", "turn_right"})
_CAPTURE_KINDS = frozenset({"OVERVIEW", "LABEL", "INSPECTION_STICKER", "DETAIL"})
_SESSION_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_MAX_INPUT_AGE_MS = 1500
_MIN_MOVE_INTERVAL_SECONDS = 0.10
_ADAPTIVE_MANIFEST_VERSION = "1.0.0"


class Simulation(Protocol):
    def step(self, action: str) -> bool: ...

    def pose(self) -> Any: ...

    def camera_metadata(self) -> dict[str, Any]: ...

    def set_view(self, view: str) -> None: ...

    def render_rgb(self) -> bytes: ...

    def frame_size(self) -> tuple[int, int]: ...


class TeleopController:
    """Thread-safe, stopped-by-default controller and append-only capture owner."""

    def __init__(self, simulation: Simulation, session_root: str | Path):
        self.simulation = simulation
        self.session_root = Path(session_root).resolve()
        self._lock = threading.RLock()
        self._simulation_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self.session: CaptureSession | None = None
        self.armed = False
        self.generation = 0
        self._last_sequence = -1
        self._last_motion_at = 0.0
        self.stop_reason = "NOT_ARMED"

    def start_session(
        self, *, ladder_id: str, session_id: str | None = None, operator_id: str | None = None
    ) -> dict[str, Any]:
        if not isinstance(ladder_id, str) or not ladder_id.strip():
            raise ValueError("select a non-empty ladder ID")
        if session_id is not None and (
            not isinstance(session_id, str) or not _SESSION_ID.fullmatch(session_id)
        ):
            raise ValueError("session ID must be a safe path-component identifier")
        if operator_id is not None and (
            not isinstance(operator_id, str) or not operator_id.strip()
        ):
            raise ValueError("operator ID must be empty or non-empty text")
        with self._capture_lock, self._lock:
            self.armed = False
            self.generation += 1
            if session_id:
                session_path = self.session_root / session_id
                if session_path.exists():
                    if session_path.resolve().parent != self.session_root:
                        raise ValueError("session directory must remain inside the selected output root")
                    session = CaptureSession.open(session_path)
                    if session.metadata["ladder_id"] != ladder_id:
                        raise ValueError("selected ladder does not match the existing session")
                    self.session = session
                else:
                    self.session = CaptureSession.create(
                        self.session_root,
                        ladder_id=ladder_id,
                        operator_id=operator_id,
                        session_id=session_id,
                    )
            else:
                self.session = CaptureSession.create(
                    self.session_root, ladder_id=ladder_id, operator_id=operator_id
                )
            self._write_adaptive_manifest(self.session)
            self.stop_reason = "SESSION_READY"
            return self.status()

    def arm(self, expected_generation: int | None = None) -> dict[str, Any]:
        with self._lock:
            if self.session is None:
                raise ValueError("create or select a capture session before enabling motion")
            if expected_generation is not None and expected_generation != self.generation:
                raise ValueError("stale enable request rejected; explicitly enable motion again")
            self.generation += 1
            self._last_sequence = -1
            self._last_motion_at = 0.0
            self.armed = True
            self.stop_reason = ""
            return self.status()

    def stop(self, reason: str = "OPERATOR_STOP") -> dict[str, Any]:
        with self._lock:
            self.generation += 1
            self.armed = False
            self.stop_reason = reason[:80] if isinstance(reason, str) else "OPERATOR_STOP"
            return self.status()

    def move(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("movement input must be an object")
        action = payload.get("action")
        sequence = payload.get("sequence")
        sent_at_ms = payload.get("sent_at_ms")
        generation = payload.get("generation")
        if action not in _ACTIONS:
            raise ValueError("movement action is not supported")
        for value, name in ((sequence, "sequence"), (sent_at_ms, "sent_at_ms"), (generation, "generation")):
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"{name} must be an integer")
        age_ms = int(time.time() * 1000) - sent_at_ms
        if age_ms < -250 or age_ms > _MAX_INPUT_AGE_MS:
            raise ValueError("stale or future-dated movement input rejected")
        with self._lock:
            if not self.armed or generation != self.generation:
                raise ValueError("teleoperation is stopped; explicitly enable motion again")
            if sequence <= self._last_sequence:
                raise ValueError("duplicate or out-of-order movement input rejected")
            now = time.monotonic()
            if now - self._last_motion_at < _MIN_MOVE_INTERVAL_SECONDS:
                raise ValueError("movement rate limit exceeded")
            if not self._simulation_lock.acquire(blocking=False):
                raise ValueError("simulation is busy rendering; retry the movement")
            self._last_sequence = sequence
            self._last_motion_at = now
            try:
                moved = self.simulation.step(action)
            finally:
                self._simulation_lock.release()
            return {
                "moved": bool(moved),
                "blocked": not bool(moved),
                "pose": self._pose_dict(),
                "camera": self.simulation.camera_metadata(),
                "generation": self.generation,
            }

    def capture(self, *, kind: str, roi: Any = None) -> dict[str, Any]:
        if kind not in _CAPTURE_KINDS:
            raise ValueError(f"capture kind must be one of {sorted(_CAPTURE_KINDS)}")
        with self._capture_lock:
            with self._lock:
                session = self.session
            if session is None:
                raise ValueError("create or select a capture session before capturing")
            with self._simulation_lock:
                width, height = self.simulation.frame_size()
                bounds = self._validate_roi(roi, width, height) if roi is not None else None
                if kind == "LABEL" and bounds is None:
                    raise ValueError("label capture requires a manually selected ROI")
                rgb = self.simulation.render_rgb()
                pose = self._pose_dict()
                camera = self.simulation.camera_metadata()
            image_bytes = encode_png(rgb, width, height)
            stage = session.path / f".capture-{uuid.uuid4().hex}.png"
            try:
                with stage.open("xb") as stream:
                    stream.write(image_bytes)
                evidence = session.add_image(
                    stage,
                    kind=kind,
                    pose=pose,
                    camera=camera,
                    provenance={
                        "method": "LOCAL_KINEMATIC_SIM_CAPTURE",
                        "producer": "ultron.teleop",
                        "inspection_oracle": False,
                        "walking_claim": False,
                        "fixtures": "SYNTHETIC_LADDER_LABELS_NOT_INSPECTION_DATA",
                    },
                )
                crop_record = None
                if bounds is not None:
                    crop_rgb = self._crop_rgb(rgb, width, bounds)
                    crop_width, crop_height = bounds[2] - bounds[0], bounds[3] - bounds[1]
                    crop_stage = session.path / f".crop-{uuid.uuid4().hex}.png"
                    try:
                        with crop_stage.open("xb") as stream:
                            stream.write(encode_png(crop_rgb, crop_width, crop_height))
                        crop_record = session.add_image(
                            crop_stage,
                            kind="DERIVED_CROP",
                            parent_evidence_id=evidence["evidence_id"],
                            pose=pose,
                            camera=camera,
                            provenance={
                                "method": "OPERATOR_SELECTED_ROI",
                                "producer": "ultron.teleop",
                                "roi_pixels": list(bounds),
                                "reader": "NONE",
                            },
                        )
                    finally:
                        crop_stage.unlink(missing_ok=True)
                self._write_adaptive_manifest(session)
                return {"evidence": evidence, "crop": crop_record}
            finally:
                stage.unlink(missing_ok=True)

    @staticmethod
    def _crop_rgb(rgb: bytes, width: int, bounds: tuple[int, int, int, int]) -> bytes:
        x1, y1, x2, y2 = bounds
        stride = width * 3
        return b"".join(
                rgb[y * stride + x1 * 3 : y * stride + x2 * 3]
                for y in range(y1, y2)
        )

    def _write_adaptive_manifest(self, session: CaptureSession) -> None:
        records = sorted(
            session.list_evidence(), key=lambda record: record["captured_at"]
        )
        crops_by_parent: dict[str, list[dict[str, Any]]] = {}
        for record in records:
            if record["kind"] != "DERIVED_CROP":
                continue
            metadata = record.get("provenance", {}).get("source_metadata", {})
            box = metadata.get("roi_pixels")
            if not isinstance(box, list) or len(box) != 4:
                continue
            crops_by_parent.setdefault(record["parent_evidence_id"], []).append(
                {
                    "evidence_id": record["evidence_id"],
                    "box": box,
                    "provenance": {
                        "method": "OPERATOR_SELECTED_ROI",
                        "producer": "ultron.teleop",
                        "capture_record": f"records/{record['evidence_id']}.json",
                        "untrusted": True,
                    },
                }
            )
        views = []
        for record in records:
            if record["kind"] == "DERIVED_CROP":
                continue
            views.append(
                {
                    "evidence_id": record["evidence_id"],
                    "ladder_id": record["ladder_id"],
                    "kind": record["kind"],
                    "path": record["relative_path"],
                    "visual_fields": [],
                    "provenance": {
                        "method": "LOCAL_KINEMATIC_SIM_CAPTURE",
                        "producer": "ultron.teleop",
                        "capture_record": f"records/{record['evidence_id']}.json",
                        "suggested_visual_field_targets_unreviewed": [],
                        "operator_review_required_before_assigning_visual_fields": True,
                        "untrusted": True,
                    },
                    "crops": crops_by_parent.get(record["evidence_id"], []),
                    "observations": [],
                    "date_observations": [],
                    "pose": record.get("pose"),
                    "camera": record.get("camera"),
                }
            )
        manifest_path = session.path / "adaptive_manifest.template.json"
        stage_path = session.path / f".adaptive-manifest-{uuid.uuid4().hex}.json"
        try:
            with stage_path.open("x", encoding="utf-8") as stream:
                json.dump(
                    {"schema_version": _ADAPTIVE_MANIFEST_VERSION, "views": views},
                    stream,
                    ensure_ascii=False,
                    indent=2,
                    allow_nan=False,
                )
                stream.write("\n")
            os.replace(stage_path, manifest_path)
        finally:
            stage_path.unlink(missing_ok=True)

    @staticmethod
    def _validate_roi(roi: Any, width: int, height: int) -> tuple[int, int, int, int]:
        if not isinstance(roi, dict):
            raise ValueError("ROI must be an object with integer x1,y1,x2,y2 coordinates")
        coords = [roi.get(name) for name in ("x1", "y1", "x2", "y2")]
        if any(not isinstance(value, int) or isinstance(value, bool) for value in coords):
            raise ValueError("ROI coordinates must be integers")
        x1, y1, x2, y2 = coords
        if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
            raise ValueError("ROI must be a non-empty rectangle inside the preview image")
        if x2 - x1 < 16 or y2 - y1 < 16:
            raise ValueError("ROI must be at least 16 by 16 pixels")
        return x1, y1, x2, y2

    def _pose_dict(self) -> dict[str, float]:
        pose = self.simulation.pose()
        if hasattr(pose, "as_dict"):
            return pose.as_dict()
        if isinstance(pose, dict):
            return dict(pose)
        raise ValueError("simulation adapter returned an unsupported pose")

    def status(self) -> dict[str, Any]:
        with self._lock:
            evidence_count = (
                sum(1 for _ in (self.session.path / "records").glob("*.json"))
                if self.session else 0
            )
            return {
                "mode": "SIMULATED KINEMATIC POSITIONING — NOT WALKING",
                "started": self.session is not None,
                "armed": self.armed,
                "generation": self.generation,
                "stop_reason": self.stop_reason,
                "pose": self._pose_dict(),
                "camera": self.simulation.camera_metadata(),
                "session_id": self.session.metadata["session_id"] if self.session else None,
                "ladder_id": self.session.metadata["ladder_id"] if self.session else None,
                "session_path": str(self.session.path) if self.session else None,
                "adaptive_manifest_template_path": (
                    str(self.session.path / "adaptive_manifest.template.json")
                    if self.session
                    else None
                ),
                "evidence_count": evidence_count,
            }

    def render_frame(self) -> bytes:
        with self._simulation_lock:
            width, height = self.simulation.frame_size()
            return encode_png(self.simulation.render_rgb(), width, height)

    def set_view(self, view: str) -> dict[str, Any]:
        with self._lock:
            if not self._simulation_lock.acquire(blocking=False):
                raise ValueError("simulation is busy rendering; retry view selection")
            try:
                self.simulation.set_view(view)
            finally:
                self._simulation_lock.release()
            return self.status()


_PAGE = r"""<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ULTRON simulated B2 capture</title>
<style>
body{font:16px system-ui,sans-serif;max-width:1100px;margin:1rem auto;padding:0 1rem;background:#111820;color:#eef3f6}
header,.panel{border:1px solid #53616a;border-radius:8px;padding:1rem;margin:.75rem 0;background:#1a252e}
.banner{font-weight:700;color:#ffd47a}.row{display:flex;gap:.75rem;align-items:center;flex-wrap:wrap}
label{display:flex;gap:.35rem;align-items:center}input{padding:.5rem;max-width:18rem}
button{padding:.65rem .9rem;border:0;border-radius:5px;background:#357a9b;color:white;font-weight:650}
button:disabled{opacity:.45}button.stop{background:#a53e3e}button.arm{background:#56864d}
#preview{display:block;width:min(100%,800px);height:auto;touch-action:none;user-select:none;cursor:crosshair}
#framebox{position:relative;width:min(100%,800px)}#selection{position:absolute;border:2px solid #ffdc65;background:#ffdc6529;display:none;pointer-events:none}
kbd{background:#33414c;padding:.15rem .35rem;border-radius:3px}#status{white-space:pre-wrap}
</style>
<header><div class="banner">SIMULATED TELEOP / KINEMATIC POSITIONING — NOT WALKING</div>
<p>Local capture preview only. Fixtures are synthetic. No OCR, inspection verdict, robot command channel, or hardware path.</p></header>
<section class="panel">
<div class="row"><label>Ladder ID <input id="ladder" value="LAD-001" maxlength="128"></label>
<label>Session ID (blank creates a unique session) <input id="session" maxlength="128"></label>
<label>Operator ID (optional) <input id="operator" maxlength="128"></label>
<button id="start">Create / select session</button></div>
<p id="sessioninfo">No session selected.</p>
<button class="arm" id="arm" disabled>Enable movement</button>
<button class="stop" id="stop">STOP / disarm</button>
</section>
<section class="panel">
<div class="row">
<button data-action="forward">↑ Forward (W)</button><button data-action="back">↓ Back (S)</button>
<button data-action="turn_left">↶ Turn left (A)</button><button data-action="turn_right">↷ Turn right (D)</button>
<button id="overviewView">Overview view</button><button id="inspectionView">Simulated inspection camera</button>
<button id="captureOverview">Capture overview</button><button id="captureLabel">Capture label ROI</button>
</div>
<p>Each key/button repeat advances one bounded kinematic increment; focus loss sends STOP.
One increment is at most 0.10 m or 5° and the server rate-limits movement to 10 inputs/s.
Collision contacts and the simulation workspace boundary reject a step.
Drag a rectangle in the preview for a label crop. Captures retain the full original image.
The session refreshes a generated adaptive-manifest template after each capture.
Copy it before editing; assign visual fields only after human review.</p>
<div id="framebox"><img id="preview" draggable="false" alt="Rendered MuJoCo tracking preview"><div id="selection"></div></div>
<p id="status"></p><p id="notice"></p>
</section>
<script>
const $=id=>document.getElementById(id), preview=$("preview"), selection=$("selection");
let seq=0, generation=0, active=null, timer=null, roi=null, drag=null;
let armed=false, stopping=false, moving=false, stopGeneration=-1;
async function api(path, body) {
  const response=await fetch(path,{method:body===undefined?"GET":"POST",
    headers:body===undefined?{}:{"Content-Type":"application/json"},
    body:body===undefined?undefined:JSON.stringify(body),cache:"no-store"});
  const data=await response.json(); if(!response.ok) throw new Error(data.error||"Request failed"); return data;
}
function showStatus(s) {
  if(s.generation<generation)return;
  generation=s.generation;
  if(!s.armed){armed=false;if(!stopping||s.generation>stopGeneration)stopping=false}
  else if(!stopping)armed=true;
  $("arm").disabled=!s.started||armed||stopping;
  document.querySelectorAll("[data-action]").forEach(button=>button.disabled=!armed||stopping);
  $("captureOverview").disabled=!s.started;
  $("captureLabel").disabled=!s.started;
  $("sessioninfo").textContent=s.started?`Session ${s.session_id} · Ladder ${s.ladder_id} · ${s.evidence_count} evidence items`:"No session selected.";
  $("status").textContent=`${s.mode}\nMotion: ${stopping?"STOP REQUESTED":armed?"ENABLED":"STOPPED"}${s.stop_reason?` (${s.stop_reason})`:""}\nPose: ${JSON.stringify(s.pose)}\nCamera: ${s.camera.type}\nSession: ${s.session_path||"not started"}\nAdaptive manifest template: ${s.adaptive_manifest_template_path||"not available"}`;
}
async function refresh(){try{const s=await api("/api/status");showStatus(s);if(s.started)preview.src="/api/frame?t="+Date.now()}catch(e){$("notice").textContent=e.message}}
function notice(text){$("notice").textContent=text}
async function sendMove(action){
  if(moving||!armed||stopping||active!==action)return;
  moving=true;
  try {const r=await api("/api/move",{action,sequence:seq++,sent_at_ms:Date.now(),generation});
    if(r.blocked)notice("Kinematic step rejected: collision or workspace boundary.");}
  catch(e){notice(e.message);
    if(e.message!=="simulation is busy rendering; retry the movement"&&e.message!=="movement rate limit exceeded"){
      clearHold();await refresh()
    }}
  finally{moving=false}
}
function hold(action){if(!armed||stopping)return;clearHold();active=action;sendMove(action);timer=setInterval(()=>active&&sendMove(active),140)}
function clearHold(){if(timer)clearInterval(timer);timer=null;active=null}
async function stop(reason="OPERATOR_STOP",beacon=false){
  armed=false;stopping=true;stopGeneration=generation;clearHold();
  document.querySelectorAll("[data-action]").forEach(button=>button.disabled=true);
  $("arm").disabled=true;
  if(beacon){navigator.sendBeacon("/api/stop",new Blob([JSON.stringify({reason})],{type:"application/json"}));return}
  try{showStatus(await api("/api/stop",{reason}))}catch(e){notice(e.message)}
}
document.querySelectorAll("[data-action]").forEach(button=>{
  const action=button.dataset.action;
  button.addEventListener("pointerdown",e=>{e.preventDefault();hold(action)});
  ["pointerup","pointercancel","pointerleave"].forEach(name=>button.addEventListener(name,clearHold));
});
window.addEventListener("keydown",e=>{
  if(e.key==="Escape"){e.preventDefault();stop("ESCAPE");return}
  if(e.target instanceof Element && e.target.closest("input,textarea,select,[contenteditable=true]"))return;
  const action={w:"forward",s:"back",a:"turn_left",d:"turn_right"}[e.key.toLowerCase()];
  if(action){e.preventDefault();if(!e.repeat)hold(action)}
});
window.addEventListener("keyup",e=>{if("wasd".includes(e.key.toLowerCase()))clearHold()});
window.addEventListener("blur",()=>stop("FOCUS_LOST",true));
document.addEventListener("visibilitychange",()=>{if(document.hidden)stop("PAGE_HIDDEN",true)});
$("start").onclick=async()=>{armed=false;stopping=true;stopGeneration=generation;clearHold();try{const s=await api("/api/session",{ladder_id:$("ladder").value,
  session_id:$("session").value||null,operator_id:$("operator").value||null});showStatus(s);notice("Session ready. Movement remains stopped until explicitly enabled.")}catch(e){notice(e.message);await stop("SESSION_SELECTION_FAILED")}};
$("arm").onclick=async()=>{try{showStatus(await api("/api/arm",{generation}));notice("Movement enabled; focus loss or STOP disarms it.")}catch(e){notice(e.message)}};
$("stop").onclick=()=>stop();
async function setView(view){try{showStatus(await api("/api/view",{view}));roi=null;selection.style.display="none";await refresh()}catch(e){notice(e.message)}}
$("overviewView").onclick=()=>setView("overview");
$("inspectionView").onclick=()=>setView("inspection");
function capture(kind){
  const body={kind};if(kind==="LABEL"){if(!roi){notice("Drag an ROI over the preview before label capture.");return}
    body.roi=roi}
  api("/api/capture",body).then(result=>{
    roi=null;selection.style.display="none";
    notice(`Saved ${result.evidence.evidence_id}${result.crop?` and linked ROI ${result.crop.evidence_id}`:""}; no OCR or inspection claim.`);
    refresh();
  }).catch(e=>notice(e.message));
}
$("captureOverview").onclick=()=>capture("OVERVIEW");
$("captureLabel").onclick=()=>capture("LABEL");
preview.addEventListener("pointerdown",e=>{
  const r=preview.getBoundingClientRect();drag={x:e.clientX-r.left,y:e.clientY-r.top,rect:r};
  preview.setPointerCapture(e.pointerId);selection.style.display="block";selection.style.left=`${drag.x}px`;selection.style.top=`${drag.y}px`;
  selection.style.width="0";selection.style.height="0";
});
preview.addEventListener("pointermove",e=>{
  if(!drag)return;const x=e.clientX-drag.rect.left,y=e.clientY-drag.rect.top;
  selection.style.left=`${Math.min(drag.x,x)}px`;selection.style.top=`${Math.min(drag.y,y)}px`;
  selection.style.width=`${Math.abs(x-drag.x)}px`;selection.style.height=`${Math.abs(y-drag.y)}px`;
});
preview.addEventListener("pointercancel",()=>{drag=null;selection.style.display="none"});
preview.addEventListener("pointerup",e=>{
  if(!drag)return;const x=e.clientX-drag.rect.left,y=e.clientY-drag.rect.top;
  const sx=preview.naturalWidth/drag.rect.width,sy=preview.naturalHeight/drag.rect.height;
  roi={x1:Math.round(Math.min(drag.x,x)*sx),y1:Math.round(Math.min(drag.y,y)*sy),
    x2:Math.round(Math.max(drag.x,x)*sx),y2:Math.round(Math.max(drag.y,y)*sy)};
  drag=null;notice(`Selected ROI ${JSON.stringify(roi)} pixels; crop remains linked to its full source.`);
});
setInterval(refresh,1000);refresh();
</script></html>"""


def make_handler(controller: TeleopController, port: int):
    class Handler(BaseHTTPRequestHandler):
        server_version = "ULTRON-Teleop/1"

        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def _local_origin(self) -> bool:
            host = self.headers.get("Host", "").split(":", 1)[0].strip("[]").lower()
            if host not in {"127.0.0.1", "localhost"}:
                return False
            origin = self.headers.get("Origin")
            return origin is None or origin in {
                f"http://127.0.0.1:{port}",
                f"http://localhost:{port}",
            }

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Ultron-Request-Thread", str(threading.get_ident()))
            self.send_header(
                "Content-Security-Policy",
                "default-src 'none'; img-src 'self'; style-src 'unsafe-inline'; "
                "script-src 'unsafe-inline'; connect-src 'self'",
            )
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, value: Any) -> None:
            body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self._send(status, body, "application/json; charset=utf-8")

        def _read_json(self) -> dict[str, Any]:
            length = self.headers.get("Content-Length")
            if length is None or not length.isdecimal() or int(length) > 8192:
                raise ValueError("request body must be valid JSON under 8 KiB")
            if self.headers.get_content_type() != "application/json":
                raise ValueError("request content type must be application/json")
            try:
                value = json.loads(self.rfile.read(int(length)))
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                raise ValueError("malformed JSON request") from exc
            if not isinstance(value, dict):
                raise ValueError("request body must be a JSON object")
            return value

        def do_GET(self) -> None:
            if not self._local_origin():
                self._json(403, {"error": "local browser requests only"})
                return
            if self.path == "/":
                self._send(200, _PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif self.path.startswith("/api/frame"):
                try:
                    image = controller.render_frame()
                    self._send(200, image, "image/png")
                except Exception as exc:
                    self._json(503, {"error": str(exc)})
            elif self.path == "/api/status":
                self._json(200, controller.status())
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self) -> None:
            if not self._local_origin():
                self._json(403, {"error": "local browser requests only"})
                return
            try:
                payload = self._read_json()
                if self.path == "/api/session":
                    result = controller.start_session(
                        ladder_id=payload.get("ladder_id"),
                        session_id=payload.get("session_id"),
                        operator_id=payload.get("operator_id"),
                    )
                elif self.path == "/api/arm":
                    generation = payload.get("generation")
                    if not isinstance(generation, int) or isinstance(generation, bool):
                        raise ValueError("enable generation must be an integer")
                    result = controller.arm(expected_generation=generation)
                elif self.path == "/api/stop":
                    result = controller.stop(payload.get("reason", "OPERATOR_STOP"))
                elif self.path == "/api/move":
                    result = controller.move(payload)
                elif self.path == "/api/capture":
                    result = controller.capture(
                        kind=payload.get("kind"), roi=payload.get("roi")
                    )
                elif self.path == "/api/view":
                    result = controller.set_view(payload.get("view"))
                else:
                    self._json(404, {"error": "not found"})
                    return
                self._json(200, result)
            except (ValueError, FileExistsError, FileNotFoundError) as exc:
                self._json(400, {"error": str(exc)})
            except Exception as exc:
                self._json(500, {"error": str(exc)})

    return Handler


def serve(
    controller: TeleopController, *, host: str = "127.0.0.1", port: int = 8765
) -> None:
    if host not in {"127.0.0.1", "0.0.0.0"}:
        raise ValueError("UI host must be 127.0.0.1 or 0.0.0.0")
    if host == "0.0.0.0" and os.environ.get("ULTRON_TELEOP_ISOLATED") != "1":
        raise ValueError(
            "non-loopback binding requires ULTRON_TELEOP_ISOLATED=1 and a private "
            "container network with a loopback-only published port"
        )
    if not 1024 <= port <= 65535:
        raise ValueError("UI port must be between 1024 and 65535")
    server = ThreadingHTTPServer((host, port), make_handler(controller, port))
    server.daemon_threads = True
    print(
        f"SIMULATED KINEMATIC TELEOP — not walking; local UI http://127.0.0.1:{port}/",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kit-root", type=Path, default=Path("/opt/unitree_mujoco"))
    parser.add_argument("--output", type=Path, default=Path("/sessions/teleop"))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--host", choices=("127.0.0.1", "0.0.0.0"), default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args(argv)
    try:
        sim = KitSimulation(
            args.kit_root,
            width=args.width,
            height=args.height,
        )
        controller = TeleopController(sim, args.output)
        serve(controller, host=args.host, port=args.port)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(2, f"ultron teleop: {exc}\n")
    finally:
        if "sim" in locals():
            sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
