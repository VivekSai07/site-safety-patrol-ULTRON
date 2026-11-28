"""Local adaptive image selection and evidence-linked draft export.

View targets and fixture/manual observations are untrusted input. Selection
cannot establish completeness, physical condition, an inspection verdict, or
human sign-off.
"""

from __future__ import annotations

from html import escape
import json
from pathlib import Path
from typing import Any, Mapping

from .capture_session import CaptureSession, EVIDENCE_KINDS
from .checklist import (
    COVERAGE_STATES,
    DATE_COVERAGE_STATES,
    DATE_ROLES,
    add_label_date_observation,
    add_proposal,
    load_form_schema,
    new_draft,
    serialize_draft,
)


MANIFEST_VERSION = "1.0.0"
_INITIAL_KINDS = frozenset({"OVERVIEW", "LABEL"})
_IMAGE_KINDS = EVIDENCE_KINDS - {"DERIVED_CROP"}
_PROPOSAL_COVERAGE = COVERAGE_STATES - {"NOT_ASSESSED", "HUMAN_ONLY"}
_GAP_COVERAGE = frozenset({"NOT_ASSESSED", "OCCLUDED", "UNREADABLE", "CONFLICTING"})
_DATE_TARGET_ROLES = frozenset({"RECURRING_INSPECTION", "NEXT_INSPECTION"})


def _catalog(schema: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        item["id"]: item
        for item in schema["x-form-mapping"]["field_catalog"]
    }


def _require_object(value: Any, label: str, keys: set[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be an object")
    extra = set(value) - keys
    missing = keys - set(value)
    if extra or missing:
        raise ValueError(
            f"{label} has unsupported keys {sorted(extra)} or missing keys {sorted(missing)}"
        )
    return value


def _string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list) or any(
        not isinstance(item, str) or not item.strip() for item in value
    ):
        raise ValueError(f"{label} must be a list of non-empty strings")
    if len(set(value)) != len(value):
        raise ValueError(f"{label} entries must be unique")
    return value


def _provenance(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must identify method and producer")
    copied = dict(value)
    if not isinstance(copied.get("method"), str) or not copied["method"].strip():
        raise ValueError(f"{label}.method must be a non-empty string")
    if not isinstance(copied.get("producer"), str) or not copied["producer"].strip():
        raise ValueError(f"{label}.producer must be a non-empty string")
    if copied.get("untrusted", True) is not True:
        raise ValueError(f"{label} must remain explicitly untrusted")
    copied["untrusted"] = True
    try:
        return json.loads(json.dumps(copied, ensure_ascii=False, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain finite JSON data") from exc


def _validate_manifest(
    manifest: Mapping[str, Any],
    *,
    base_dir: Path,
    ladder_id: str,
    schema: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    root = _require_object(manifest, "manifest", {"schema_version", "views"})
    if root.get("schema_version") != MANIFEST_VERSION:
        raise ValueError(f"manifest schema_version must be {MANIFEST_VERSION}")
    raw_views = root.get("views")
    if not isinstance(raw_views, list) or not raw_views:
        raise ValueError("manifest.views must contain at least one source image")

    catalog = _catalog(schema)
    visual_fields = {
        field_id
        for field_id, definition in catalog.items()
        if not definition["human_only"]
        and str(definition.get("evidence_mode", "")).startswith("VISUAL")
    }
    views: list[dict[str, Any]] = []
    evidence_ids: set[str] = set()
    for index, raw in enumerate(raw_views):
        view = dict(
            _require_object(
                raw,
                f"views[{index}]",
                {
                    "evidence_id",
                    "ladder_id",
                    "kind",
                    "path",
                    "visual_fields",
                    "provenance",
                    "crops",
                    "observations",
                    "date_observations",
                    "pose",
                    "camera",
                },
            )
        )
        evidence_id = view["evidence_id"]
        if not isinstance(evidence_id, str) or not evidence_id.strip():
            raise ValueError(f"views[{index}].evidence_id must be non-empty")
        if evidence_id in evidence_ids:
            raise ValueError(f"duplicate evidence ID: {evidence_id}")
        evidence_ids.add(evidence_id)
        if view["ladder_id"] != ladder_id:
            raise ValueError(
                f"view {evidence_id} is associated with ladder {view['ladder_id']!r}, "
                f"not the requested ladder {ladder_id!r}"
            )
        if not isinstance(view["kind"], str) or view["kind"] not in _IMAGE_KINDS:
            raise ValueError(f"views[{index}].kind must be an original-image evidence kind")
        if not isinstance(view["path"], str) or not view["path"].strip():
            raise ValueError(f"views[{index}].path must be a non-empty local path")
        source = Path(view["path"])
        if not source.is_absolute():
            source = base_dir / source
        if not source.is_file():
            raise FileNotFoundError(source)
        view["source_path"] = source

        targets = _string_list(view["visual_fields"], f"views[{index}].visual_fields")
        unknown = set(targets) - visual_fields
        if unknown:
            raise ValueError(
                f"views[{index}] includes non-visual or unknown target fields: {sorted(unknown)}"
            )
        view["visual_fields"] = targets
        view["provenance"] = _provenance(view["provenance"], f"views[{index}].provenance")
        for optional in ("pose", "camera"):
            if view[optional] is not None and not isinstance(view[optional], Mapping):
                raise ValueError(f"views[{index}].{optional} must be an object or null")
            if view[optional] is not None:
                _provenance(
                    {"method": "MANIFEST_METADATA", "producer": "ultron.adaptive_draft",
                     "value": dict(view[optional])},
                    f"views[{index}].{optional}",
                )

        raw_crops = view["crops"]
        if not isinstance(raw_crops, list):
            raise ValueError(f"views[{index}].crops must be a list")
        crops = []
        for crop_index, raw_crop in enumerate(raw_crops):
            crop = dict(
                _require_object(
                    raw_crop,
                    f"views[{index}].crops[{crop_index}]",
                    {"evidence_id", "box", "provenance"},
                )
            )
            crop_id = crop["evidence_id"]
            if not isinstance(crop_id, str) or not crop_id.strip() or crop_id in evidence_ids:
                raise ValueError(f"invalid or duplicate crop evidence ID: {crop_id!r}")
            evidence_ids.add(crop_id)
            box = crop["box"]
            if (
                not isinstance(box, list)
                or len(box) != 4
                or any(not isinstance(point, int) or isinstance(point, bool) for point in box)
                or box[0] < 0
                or box[1] < 0
                or box[2] <= box[0]
                or box[3] <= box[1]
            ):
                raise ValueError(f"views[{index}].crops[{crop_index}].box must be [left, top, right, bottom]")
            crop["provenance"] = _provenance(
                crop["provenance"], f"views[{index}].crops[{crop_index}].provenance"
            )
            crops.append(crop)
        view["crops"] = crops
        view["crop_ids"] = {crop["evidence_id"] for crop in crops}

        for collection in ("observations", "date_observations"):
            if not isinstance(view[collection], list):
                raise ValueError(f"views[{index}].{collection} must be a list")
        for observation_index, observation in enumerate(view["observations"]):
            item = _require_object(
                observation,
                f"views[{index}].observations[{observation_index}]",
                {"field_id", "value", "coverage", "reason", "evidence_ids", "provenance"},
            )
            field_id = item["field_id"]
            if field_id not in targets:
                raise ValueError(
                    f"{field_id} must be declared in its view's visual_fields before assessment"
                )
            if (
                not isinstance(item["coverage"], str)
                or item["coverage"] not in _PROPOSAL_COVERAGE
            ):
                raise ValueError(f"invalid proposal coverage for {field_id}")
            if item["coverage"] == "OBSERVED" and item["value"] is None:
                raise ValueError(f"OBSERVED assessment for {field_id} needs a value")
            try:
                json.dumps(item["value"], ensure_ascii=False, allow_nan=False)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"assessment value for {field_id} must be finite JSON data") from exc
            if item["reason"] is not None and not isinstance(item["reason"], str):
                raise ValueError(f"reason for {field_id} must be a string or null")
            if item["coverage"] == "NOT_APPLICABLE" and not (
                isinstance(item["reason"], str) and item["reason"].strip()
            ):
                raise ValueError(f"NOT_APPLICABLE assessment for {field_id} needs a reason")
            refs = _string_list(item["evidence_ids"], f"evidence_ids for {field_id}")
            if not refs:
                refs = [view["evidence_id"]]
            allowed = {view["evidence_id"]} | view["crop_ids"]
            if set(refs) - allowed:
                raise ValueError(
                    f"assessment for {field_id} must reference this view or its linked crops"
                )
            _provenance(item["provenance"], f"provenance for {field_id}")

        for date_index, observation in enumerate(view["date_observations"]):
            item = _require_object(
                observation,
                f"views[{index}].date_observations[{date_index}]",
                {"date_role", "value", "coverage", "evidence_ids", "provenance"},
            )
            if not isinstance(item["date_role"], str) or item["date_role"] not in DATE_ROLES:
                raise ValueError(f"invalid date role: {item['date_role']}")
            if (
                not isinstance(item["coverage"], str)
                or item["coverage"] not in DATE_COVERAGE_STATES
            ):
                raise ValueError(f"invalid date coverage for {item['date_role']}")
            if item["coverage"] == "OBSERVED" and item["value"] is None:
                raise ValueError(f"OBSERVED date for {item['date_role']} needs a value")
            try:
                json.dumps(item["value"], ensure_ascii=False, allow_nan=False)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"date value for {item['date_role']} must be finite JSON data"
                ) from exc
            refs = _string_list(
                item["evidence_ids"], f"date evidence_ids for {item['date_role']}"
            )
            if not refs:
                refs = [view["evidence_id"]]
            allowed = {view["evidence_id"]} | view["crop_ids"]
            if set(refs) - allowed:
                raise ValueError(
                    f"date observation for {item['date_role']} must reference this view or its linked crops"
                )
            _provenance(item["provenance"], f"date provenance for {item['date_role']}")
        views.append(view)

    kinds = {view["kind"] for view in views}
    if not _INITIAL_KINDS <= kinds:
        raise ValueError("adaptive draft input needs at least one OVERVIEW and one LABEL image")
    return views, catalog


def _create_crop(session: CaptureSession, source_record: Mapping[str, Any], crop: Mapping[str, Any]) -> dict[str, Any]:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Install the simulation extra (Pillow) to create linked evidence crops") from exc

    original_path = session.path / source_record["relative_path"]
    crop_id = crop["evidence_id"]
    staging = session.path / f".crop-{crop_id}.png"
    try:
        with Image.open(original_path) as image:
            left, top, right, bottom = crop["box"]
            if right > image.width or bottom > image.height:
                raise ValueError(f"crop box exceeds source image dimensions: {crop_id}")
            image.crop((left, top, right, bottom)).save(staging, format="PNG")
        return session.add_image(
            staging,
            kind="DERIVED_CROP",
            evidence_id=crop_id,
            parent_evidence_id=source_record["evidence_id"],
            pose=source_record["pose"],
            camera=source_record["camera"],
            provenance={
                "method": "DERIVED_PIXEL_CROP",
                "producer": "ultron.adaptive_draft",
                "source_evidence_id": source_record["evidence_id"],
                "box_xyxy_original_pixels": crop["box"],
                "operator_provenance": crop["provenance"],
                "untrusted": True,
            },
        )
    finally:
        staging.unlink(missing_ok=True)


def _gap_fields(draft: Mapping[str, Any], catalog: Mapping[str, Mapping[str, Any]]) -> set[str]:
    return {
        field_id
        for field_id, field in draft["field_observations"].items()
        if not catalog[field_id]["human_only"] and field["coverage"] in _GAP_COVERAGE
    }


def _date_role_coverage(draft: Mapping[str, Any], date_role: str) -> str:
    observations = [
        item
        for item in draft["label_date_observations"]
        if item["date_role"] == date_role
    ]
    if not observations:
        return "NOT_ASSESSED"
    values = []
    for item in observations:
        value = item.get("value")
        if value is not None and not any(value == previous for previous in values):
            values.append(value)
    if len(values) > 1 or any(
        item.get("coverage") == "CONFLICTING" for item in observations
    ):
        return "CONFLICTING"
    return observations[-1].get("coverage", "NOT_ASSESSED")


def _pending_targets(
    draft: Mapping[str, Any],
    catalog: Mapping[str, Mapping[str, Any]],
) -> set[str]:
    pending = _gap_fields(draft, catalog)
    for field_id, definition in catalog.items():
        date_role = definition.get("date_role")
        if (
            date_role in _DATE_TARGET_ROLES
            and not definition["human_only"]
            and _date_role_coverage(draft, date_role) in {"NOT_ASSESSED", "UNREADABLE", "CONFLICTING"}
        ):
            pending.add(field_id)
    return pending


def _add_view_observations(
    draft: dict[str, Any],
    view: Mapping[str, Any],
    selected_evidence: set[str],
) -> None:
    for item in view["observations"]:
        evidence_ids = item["evidence_ids"] or [view["evidence_id"]]
        if set(evidence_ids) - selected_evidence:
            continue
        add_proposal(
            draft,
            item["field_id"],
            value=item["value"],
            coverage=item["coverage"],
            evidence_ids=evidence_ids,
            provenance=item["provenance"],
            reason=item["reason"],
        )
    for item in view["date_observations"]:
        evidence_ids = item["evidence_ids"] or [view["evidence_id"]]
        if set(evidence_ids) - selected_evidence:
            continue
        add_label_date_observation(
            draft,
            date_role=item["date_role"],
            value=item["value"],
            coverage=item["coverage"],
            evidence_ids=evidence_ids,
            provenance=item["provenance"],
        )


def _unresolved(
    draft: Mapping[str, Any],
    catalog: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    result = []
    for field_id, field in draft["field_observations"].items():
        definition = catalog[field_id]
        coverage = field["coverage"]
        if coverage in _GAP_COVERAGE or definition["human_only"]:
            reasons = [
                proposal["reason"]
                for proposal in field["proposals"]
                if proposal.get("reason")
            ]
            result.append(
                {
                    "field_id": field_id,
                    "label_de": definition["label_de"],
                    "source_page": definition["page"],
                    "coverage": coverage,
                    "human_only": definition["human_only"],
                    "reason": reasons[-1] if reasons else None,
                }
            )
    return result


def _unresolved_dates(draft: Mapping[str, Any]) -> list[dict[str, Any]]:
    result = []
    for date_role in sorted(DATE_ROLES):
        observations = [
            item
            for item in draft["label_date_observations"]
            if item["date_role"] == date_role
        ]
        coverage = _date_role_coverage(draft, date_role)
        if coverage != "OBSERVED":
            values = []
            evidence_ids = []
            for item in observations:
                value = item.get("value")
                if value is not None and not any(value == previous for previous in values):
                    values.append(value)
                for evidence_id in item.get("evidence_ids", []):
                    if evidence_id not in evidence_ids:
                        evidence_ids.append(evidence_id)
            result.append(
                {
                    "date_role": date_role,
                    "coverage": coverage,
                    "latest_value": observations[-1].get("value") if observations else None,
                    "conflicting_values": values if coverage == "CONFLICTING" else [],
                    "evidence_ids": evidence_ids,
                }
            )
    return result


def _selection_plan(
    views: list[dict[str, Any]],
    draft: dict[str, Any],
    catalog: Mapping[str, Mapping[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    selected: list[dict[str, Any]] = []
    selected_ids: set[str] = set()
    for kind in ("OVERVIEW", "LABEL"):
        view = next(item for item in views if item["kind"] == kind)
        selected.append(
            {
                "evidence_id": view["evidence_id"],
                "kind": view["kind"],
                "reason": f"Required label-first starting view: {kind.lower()}.",
                "operator_declared_targets": list(view["visual_fields"]),
            }
        )
        selected_ids.add(view["evidence_id"])

    for view in views:
        if view["evidence_id"] in selected_ids:
            _add_view_observations(
                draft,
                view,
                {view["evidence_id"]} | view["crop_ids"],
            )

    candidates = [
        view
        for view in views
        if view["evidence_id"] not in selected_ids and view["kind"] != "OTHER"
    ]
    while True:
        gaps = _pending_targets(draft, catalog)
        ranked = []
        for index, view in enumerate(candidates):
            contribution = sorted(gaps & set(view["visual_fields"]))
            if contribution:
                ranked.append((len(contribution), -index, contribution, view))
        if not ranked:
            break
        _, _, contribution, chosen = max(ranked, key=lambda row: (row[0], row[1]))
        candidates.remove(chosen)
        selected_ids.add(chosen["evidence_id"])
        selected.append(
            {
                "evidence_id": chosen["evidence_id"],
                "kind": chosen["kind"],
                "reason": "Selected to address currently unresolved visual fields; "
                "targets are operator-declared and do not establish visibility or condition.",
                "operator_declared_targets": list(chosen["visual_fields"]),
                "gap_fields_at_selection": contribution,
            }
        )
        _add_view_observations(
            draft,
            chosen,
            {chosen["evidence_id"]} | chosen["crop_ids"],
        )

    return selected, _unresolved(draft, catalog), _unresolved_dates(draft)


def _write_exclusive(path: Path, content: str) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(content)
        stream.write("\n")


def _review_html(
    draft: Mapping[str, Any],
    selection: Mapping[str, Any],
    evidence: list[Mapping[str, Any]],
    schema: Mapping[str, Any],
) -> str:
    if draft.get("form_id") != schema["properties"]["form_id"]["const"]:
        raise ValueError("selected catalog does not match the draft")
    notice = schema["x-form-mapping"].get("catalog_notice")
    catalog_notice = f'<p class="warning">{escape(notice)}</p>' if notice else ""
    chosen_ids = {
        item["evidence_id"] for item in selection["selected_views"]
    }
    evidence_paths = {entry["evidence_id"]: entry["relative_path"] for entry in evidence}

    def evidence_links(evidence_ids: list[str]) -> str:
        links = []
        for evidence_id in evidence_ids:
            path = evidence_paths.get(evidence_id)
            if path is None:
                links.append(escape(evidence_id))
                continue
            links.append(
                f'<a href="{escape(path, quote=True)}">{escape(evidence_id)}</a>'
            )
        return ", ".join(links)

    images = []
    for entry in evidence:
        selected = entry["evidence_id"] in chosen_ids
        relative = escape(entry["relative_path"], quote=True)
        title = escape(entry["evidence_id"])
        status = "Selected view" if selected else (
            f"Derived crop of {entry['parent_evidence_id']}"
            if entry["kind"] == "DERIVED_CROP"
            else "Retained source, not selected"
        )
        images.append(
            f'<figure><a href="{relative}"><img loading="lazy" src="{relative}" '
            f'alt="{title}"></a><figcaption>{title} · {escape(entry["kind"])} · '
            f'{escape(status)}</figcaption></figure>'
        )

    unresolved_rows = "".join(
        "<tr><td>"
        + escape(item["field_id"])
        + "</td><td>"
        + escape(item["label_de"])
        + f'</td><td>p. {item["source_page"]}</td><td>'
        + escape(item["coverage"])
        + "</td><td>"
        + ("Human-only task" if item["human_only"] else "Visual gap")
        + "</td><td>"
        + escape(item["reason"] or "No supported evidence recorded")
        + "</td></tr>"
        for item in selection["unresolved_fields"]
    )
    proposals = []
    for field_id, field in draft["field_observations"].items():
        for proposal in field["proposals"]:
            proposals.append(
                f"<li><strong>{escape(field_id)}</strong>: "
                f"{escape(proposal['coverage'])}; value "
                f"<code>{escape(json.dumps(proposal['value'], ensure_ascii=False))}</code>; "
                f"evidence {evidence_links(proposal['evidence_ids'])}; "
                f"provenance {escape(proposal['provenance']['method'])}/"
                f"{escape(proposal['provenance']['producer'])}"
                + (f"; {escape(proposal['reason'])}" if proposal["reason"] else "")
                + "</li>"
            )
    dates = []
    for item in draft["label_date_observations"]:
        dates.append(
            "<li><strong>"
            + escape(item["date_role"])
            + "</strong>: "
            + escape(item["coverage"])
            + "; value <code>"
            + escape(json.dumps(item["value"], ensure_ascii=False))
            + "</code>; evidence "
            + evidence_links(item["evidence_ids"])
            + "; provenance "
            + escape(item["provenance"]["method"])
            + "/"
            + escape(item["provenance"]["producer"])
            + "</li>"
        )
    unresolved_dates = "".join(
        "<tr><td>"
        + escape(item["date_role"])
        + "</td><td>"
        + escape(item["coverage"])
        + "</td><td>"
        + escape(
            json.dumps(item.get("conflicting_values", []), ensure_ascii=False)
        )
        + "</td><td>"
        + (evidence_links(item["evidence_ids"]) or "—")
        + "</td></tr>"
        for item in selection["unresolved_dates"]
    )
    return (
        "<!doctype html><html lang=\"en\"><meta charset=\"utf-8\">"
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        "<title>Evidence-linked draft review</title>"
        "<style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:0 1rem}"
        "figure{display:inline-block;width:min(30%,320px);vertical-align:top;margin:.6rem}"
        "img{width:100%;height:auto}table{border-collapse:collapse;width:100%}"
        "th,td{border:1px solid #999;padding:.4rem;text-align:left}code{overflow-wrap:anywhere}"
        ".warning{padding:1rem;background:#fff2cc}</style>"
        "<h1>Selected form · draft review</h1>"
        '<p class="warning"><strong>Untrusted draft only.</strong> Images and fixture/manual '
        "inputs do not establish a complete inspection, pass, safe use, or human sign-off. "
        "Tactile, functional, structural and context-only checks remain for a qualified human.</p>"
        + catalog_notice
        + f"<p>Form <code>{escape(draft['form_id'])}</code> · "
        f"session <code>{escape(draft['session_id'])}</code> · ladder "
        f"<code>{escape(draft['ladder_id'])}</code></p>"
        f"<p>Selected {len(selection['selected_views'])} original view(s); "
        f"retained {len(selection['unused_original_ids'])} unused original(s) and "
        f"{len(selection['crop_ids'])} linked crop(s). Crops are not independent views.</p>"
        "<h2>Retained evidence</h2><section>"
        + "".join(images)
        + "</section><h2>Unresolved fields</h2>"
        + (
            "<p>None of the visual gaps were left in this draft; this does not certify completeness.</p>"
            if not selection["unresolved_fields"]
            else "<table><thead><tr><th>Field</th><th>Catalog label</th><th>Catalog page</th>"
            "<th>Coverage</th><th>Review status</th><th>Unresolved reason</th>"
            "</tr></thead><tbody>"
            + unresolved_rows
            + "</tbody></table>"
        )
        + "<h2>Unresolved date roles</h2>"
        + (
            "<p>No unresolved date roles are recorded; review all distinct date types.</p>"
            if not selection["unresolved_dates"]
            else "<table><thead><tr><th>Date role</th><th>Coverage</th>"
            "<th>Conflicting readings</th><th>Evidence</th></tr></thead><tbody>"
            + unresolved_dates
            + "</tbody></table>"
        )
        + "<h2>Untrusted field proposals</h2><ul>"
        + "".join(proposals)
        + "</ul><h2>Separate date readings</h2><ul>"
        + "".join(dates)
        + "</ul><h2>Human review</h2><p>Required and pending. No verdict, "
        "signature, safe-use statement or completion is generated.</p>"
        "</html>"
    )


def build_adaptive_draft(
    manifest: Mapping[str, Any],
    *,
    sessions_root: str | Path,
    ladder_id: str,
    operator_id: str | None = None,
    session_id: str | None = None,
    base_dir: str | Path = ".",
) -> dict[str, Any]:
    """Capture manifest sources and create local JSON/HTML draft review artifacts.

    All input originals are copied to a new session before selection, so
    unused candidates remain available. ``visual_fields`` are operator-declared
    targets, not model findings or claims that a feature is visible.
    """
    if not isinstance(manifest, Mapping):
        raise ValueError("manifest must be an object")
    if not isinstance(ladder_id, str) or not ladder_id.strip():
        raise ValueError("ladder_id must be a non-empty string")
    schema = load_form_schema()
    views, catalog = _validate_manifest(
        manifest,
        base_dir=Path(base_dir).resolve(),
        ladder_id=ladder_id,
        schema=schema,
    )
    if load_form_schema() != schema:
        raise ValueError("selected catalog changed during draft construction")
    base_views = [view for view in views if view["kind"] in _INITIAL_KINDS]
    if not any(view["kind"] == "OVERVIEW" for view in base_views) or not any(
        view["kind"] == "LABEL" for view in base_views
    ):
        raise ValueError("adaptive draft input needs an OVERVIEW and LABEL")

    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Install the simulation extra (Pillow) to review source images") from exc
    for view in views:
        try:
            with Image.open(view["source_path"]) as image:
                image.verify()
                width, height = image.size
        except Exception as exc:
            raise ValueError(f"source is not a readable image: {view['source_path']}") from exc
        for crop in view["crops"]:
            left, top, right, bottom = crop["box"]
            if right > width or bottom > height:
                raise ValueError(f"crop box exceeds source image dimensions: {crop['evidence_id']}")

    if load_form_schema() != schema:
        raise ValueError("selected catalog changed during draft construction")
    session = CaptureSession.create(
        sessions_root,
        ladder_id=ladder_id,
        operator_id=operator_id,
        session_id=session_id,
    )
    draft = new_draft(
        session_id=session.metadata["session_id"],
        ladder_id=session.metadata["ladder_id"],
    )
    if (
        draft["form_id"] != schema["properties"]["form_id"]["const"]
        or load_form_schema() != schema
    ):
        raise ValueError("selected catalog changed during draft construction")

    source_records: dict[str, dict[str, Any]] = {}
    crops_by_view: dict[str, list[dict[str, Any]]] = {}
    for view in views:
        record = session.add_image(
            view["source_path"],
            kind=view["kind"],
            evidence_id=view["evidence_id"],
            ladder_id=ladder_id,
            pose=view["pose"],
            camera=view["camera"],
            provenance={
                "method": "LOCAL_MANIFEST_SOURCE",
                "producer": "ultron.adaptive_draft",
                "operator_declared_visual_fields": view["visual_fields"],
                "input_provenance": view["provenance"],
                "untrusted": True,
            },
        )
        source_records[view["evidence_id"]] = record
        crops_by_view[view["evidence_id"]] = [
            _create_crop(session, record, crop) for crop in view["crops"]
        ]

    selected, unresolved, unresolved_dates = _selection_plan(views, draft, catalog)
    serialized_draft = serialize_draft(draft)
    if load_form_schema() != schema:
        raise ValueError("selected catalog changed during draft construction")
    evidence = session.list_evidence()
    selected_ids = {item["evidence_id"] for item in selected}
    all_original_ids = [view["evidence_id"] for view in views]
    selection = {
        "schema_version": MANIFEST_VERSION,
        "selection_method": "OPERATOR_TAGGED_FIELD_TARGETS_AND_CONTRACT_GAPS",
        "selection_is_not_a_completeness_or_safety_claim": True,
        "session_id": session.metadata["session_id"],
        "ladder_id": session.metadata["ladder_id"],
        "selected_views": selected,
        "selected_original_view_count": len(selected),
        "unused_original_ids": [item for item in all_original_ids if item not in selected_ids],
        "all_original_ids": all_original_ids,
        "crop_ids": [
            entry["evidence_id"]
            for entry in evidence
            if entry["kind"] == "DERIVED_CROP"
        ],
        "unresolved_fields": unresolved,
        "unresolved_dates": unresolved_dates,
    }
    _write_exclusive(session.path / "draft.json", serialized_draft)
    _write_exclusive(
        session.path / "selection.json",
        json.dumps(selection, ensure_ascii=False, indent=2, allow_nan=False),
    )
    _write_exclusive(
        session.path / "review.html",
        _review_html(draft, selection, evidence, schema),
    )
    return {
        "session_id": session.metadata["session_id"],
        "ladder_id": session.metadata["ladder_id"],
        "session_path": str(session.path),
        "draft_json": str(session.path / "draft.json"),
        "selection_json": str(session.path / "selection.json"),
        "review_html": str(session.path / "review.html"),
        "selected_view_ids": [item["evidence_id"] for item in selected],
        "unresolved_field_ids": [item["field_id"] for item in unresolved],
    }


def build_adaptive_draft_from_file(
    manifest_path: str | Path,
    *,
    sessions_root: str | Path,
    ladder_id: str,
    operator_id: str | None = None,
    session_id: str | None = None,
) -> dict[str, Any]:
    """Load a local JSON manifest and build its isolated evidence session."""
    manifest_file = Path(manifest_path)
    try:
        manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read local JSON manifest: {manifest_file}") from exc
    return build_adaptive_draft(
        manifest,
        sessions_root=sessions_root,
        ladder_id=ladder_id,
        operator_id=operator_id,
        session_id=session_id,
        base_dir=manifest_file.resolve().parent,
    )
