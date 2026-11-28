"""Local, append-only evidence capture sessions for simulated inspection."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import mimetypes
from pathlib import Path
import re
import shutil
from typing import Any, Mapping
import uuid


SESSION_SCHEMA_VERSION = "1.0.0"
SESSION_MODE = "SIMULATION_KINEMATIC"
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".webp", ".tif", ".tiff", ".bmp"})
EVIDENCE_KINDS = frozenset(
    {"OVERVIEW", "LABEL", "INSPECTION_STICKER", "DETAIL", "DERIVED_CROP", "OTHER"}
)
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def _safe_id(value: str, label: str) -> str:
    if not isinstance(value, str) or not _SAFE_ID.fullmatch(value):
        raise ValueError(f"{label} must be a safe path-component identifier")
    return value


def _timestamp(value: str | None = None) -> str:
    if value is None:
        return datetime.now(timezone.utc).isoformat()
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("timestamps must include a timezone")
    return value


def _json_copy(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError("metadata must be finite JSON data") from exc


class CaptureSession:
    """A session directory that refuses replacement and preserves original files.

    Session metadata is created once. Each image and its JSON record receive a
    unique ID and are written with exclusive creation, so retries cannot
    silently replace earlier evidence.
    """

    def __init__(self, path: Path, metadata: dict[str, Any]):
        self.path = path
        self.metadata = metadata

    @classmethod
    def create(
        cls,
        root: str | Path,
        *,
        ladder_id: str,
        operator_id: str | None = None,
        session_id: str | None = None,
        created_at: str | None = None,
    ) -> "CaptureSession":
        if not isinstance(ladder_id, str) or not ladder_id.strip():
            raise ValueError("ladder_id must be a non-empty string")
        generated_id = session_id or (
            "session-"
            + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            + "-"
            + uuid.uuid4().hex[:12]
        )
        _safe_id(generated_id, "session_id")
        if operator_id is not None and (
            not isinstance(operator_id, str) or not operator_id.strip()
        ):
            raise ValueError("operator_id must be None or a non-empty string")
        session_path = Path(root) / generated_id
        session_path.parent.mkdir(parents=True, exist_ok=True)
        session_path.mkdir(exist_ok=False)
        try:
            (session_path / "evidence").mkdir()
            (session_path / "records").mkdir()
            metadata = {
                "schema_version": SESSION_SCHEMA_VERSION,
                "session_id": generated_id,
                "ladder_id": ladder_id,
                "operator_id": operator_id,
                "mode": SESSION_MODE,
                "created_at": _timestamp(created_at),
                "untrusted_inputs": True,
            }
            encoded = json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False)
            with (session_path / "session.json").open("x", encoding="utf-8") as stream:
                stream.write(encoded)
                stream.write("\n")
        except Exception:
            shutil.rmtree(session_path, ignore_errors=True)
            raise
        return cls(session_path, metadata)

    @classmethod
    def open(cls, path: str | Path) -> "CaptureSession":
        """Reopen an existing session for append-only evidence additions."""
        session_path = Path(path)
        metadata_path = session_path / "session.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if (
            metadata.get("schema_version") != SESSION_SCHEMA_VERSION
            or metadata.get("mode") != SESSION_MODE
            or metadata.get("untrusted_inputs") is not True
        ):
            raise ValueError("unsupported or invalid capture session")
        _safe_id(metadata.get("session_id"), "session_id")
        if not isinstance(metadata.get("ladder_id"), str) or not metadata["ladder_id"].strip():
            raise ValueError("session is missing its ladder association")
        return cls(session_path, metadata)

    def add_image(
        self,
        source: str | Path,
        *,
        kind: str = "OVERVIEW",
        evidence_id: str | None = None,
        captured_at: str | None = None,
        ladder_id: str | None = None,
        parent_evidence_id: str | None = None,
        pose: Mapping[str, Any] | None = None,
        camera: Mapping[str, Any] | None = None,
        provenance: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Copy an image into the session and record its hash and provenance.

        A derivative crop must reference an earlier evidence item. This method
        never stores an absolute source path and never merges ladder identities.
        """
        if not isinstance(kind, str) or kind not in EVIDENCE_KINDS:
            raise ValueError(f"kind must be one of {sorted(EVIDENCE_KINDS)}")
        if ladder_id is not None and (
            not isinstance(ladder_id, str) or not ladder_id.strip()
        ):
            raise ValueError("ladder_id must be None or a non-empty string")
        associated_ladder = (
            ladder_id if ladder_id is not None else self.metadata["ladder_id"]
        )
        if associated_ladder != self.metadata["ladder_id"]:
            raise ValueError("evidence ladder association must match the session")
        if kind == "DERIVED_CROP" and parent_evidence_id is None:
            raise ValueError("derived crops must identify their source evidence")
        if kind != "DERIVED_CROP" and parent_evidence_id is not None:
            raise ValueError("only derived crops may reference parent evidence")
        if parent_evidence_id is not None:
            _safe_id(parent_evidence_id, "parent_evidence_id")
            if not any(item["evidence_id"] == parent_evidence_id for item in self.list_evidence()):
                raise ValueError("parent evidence must already exist in this session")
        source_path = Path(source)
        if not source_path.is_file():
            raise FileNotFoundError(source_path)
        suffix = source_path.suffix.lower()
        if suffix not in IMAGE_SUFFIXES:
            raise ValueError(f"unsupported image suffix: {suffix or '(none)'}")
        new_id = evidence_id or ("ev-" + uuid.uuid4().hex)
        _safe_id(new_id, "evidence_id")
        payload_path = self.path / "evidence" / f"{new_id}{suffix}"
        record_path = self.path / "records" / f"{new_id}.json"
        if payload_path.exists() or record_path.exists():
            raise FileExistsError(f"evidence already exists: {new_id}")

        metadata_path: Path | None = None
        try:
            digest = hashlib.sha256()
            size = 0
            with source_path.open("rb") as input_stream, payload_path.open("xb") as output_stream:
                while chunk := input_stream.read(1024 * 1024):
                    digest.update(chunk)
                    size += len(chunk)
                    output_stream.write(chunk)
            if size == 0:
                raise ValueError("empty files are not valid image evidence")
            source_info = _json_copy(dict(provenance or {}))
            entry = {
                "evidence_id": new_id,
                "session_id": self.metadata["session_id"],
                "ladder_id": self.metadata["ladder_id"],
                "kind": kind,
                "captured_at": _timestamp(captured_at),
                "relative_path": f"evidence/{new_id}{suffix}",
                "source_filename": source_path.name,
                "sha256": digest.hexdigest(),
                "size_bytes": size,
                "media_type": mimetypes.guess_type("image" + suffix)[0]
                or "application/octet-stream",
                "parent_evidence_id": parent_evidence_id,
                "pose": _json_copy(dict(pose)) if pose is not None else None,
                "camera": _json_copy(dict(camera)) if camera is not None else None,
                "provenance": {
                    "method": "LOCAL_EVIDENCE_COPY",
                    "producer": "CaptureSession.add_image",
                    "untrusted": True,
                    "source_metadata": source_info,
                },
                "untrusted": True,
            }
            serialized = json.dumps(entry, ensure_ascii=False, indent=2, allow_nan=False)
            metadata_path = record_path
            with record_path.open("x", encoding="utf-8") as stream:
                stream.write(serialized)
                stream.write("\n")
        except Exception:
            if payload_path.exists():
                payload_path.unlink()
            if metadata_path is not None and metadata_path.exists():
                metadata_path.unlink()
            raise
        return _json_copy(entry)

    def list_evidence(self) -> list[dict[str, Any]]:
        """Return evidence records in stable order without altering stored files."""
        records = []
        for path in sorted((self.path / "records").glob("*.json")):
            record = json.loads(path.read_text(encoding="utf-8"))
            if record.get("session_id") != self.metadata["session_id"]:
                raise ValueError(f"record belongs to a different session: {path.name}")
            if record.get("ladder_id") != self.metadata["ladder_id"]:
                raise ValueError(f"record belongs to a different ladder: {path.name}")
            records.append(record)
        return records
