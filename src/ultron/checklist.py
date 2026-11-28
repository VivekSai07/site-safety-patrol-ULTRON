"""Untrusted, evidence-linked drafts for a selected local form catalog.

This contract records proposals and human-only tasks. It deliberately has no
verdict, pass, safe-use, or sign-off operation.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
from importlib.resources import files
import json
import os
from pathlib import Path
from typing import Any, Mapping


SCHEMA_VERSION = "1.0.0"
# Retained for callers identifying legacy drafts; new drafts use the selected form.
FORM_ID = "nesseler_kontrollblatt_leitern_tritte"
GENERIC_FORM_ID = "ultron_synthetic_ladder_demo_v1"
SCHEMA_ASSET = ("assets", "schemas", "generic_ladder_draft_v1.json")
SCHEMA_ENV = "ULTRON_FORM_SCHEMA"
COVERAGE_STATES = frozenset(
    {
        "NOT_ASSESSED",
        "OBSERVED",
        "OCCLUDED",
        "UNREADABLE",
        "CONFLICTING",
        "NOT_APPLICABLE",
        "HUMAN_ONLY",
    }
)
DATE_COVERAGE_STATES = frozenset({"OBSERVED", "UNREADABLE", "CONFLICTING"})
DATE_ROLES = frozenset(
    {
        "MANUFACTURER_FINAL_CONTROL",
        "ACQUISITION",
        "RECURRING_INSPECTION",
        "NEXT_INSPECTION",
    }
)


def _validate_form_schema(schema: Any) -> None:
    if not isinstance(schema, dict) or schema.get("type") != "object":
        raise ValueError("form schema must be a JSON Schema object")
    properties = schema.get("properties")
    root_keys = {
        "schema_version",
        "form_id",
        "session_id",
        "ladder_id",
        "untrusted_inputs",
        "created_at",
        "field_observations",
        "inspection_history",
        "label_date_observations",
        "human_review",
    }
    if (
        not isinstance(properties, dict)
        or set(properties) != root_keys
        or any(not isinstance(value, dict) for value in properties.values())
        or not isinstance(schema.get("required"), list)
        or len(schema["required"]) != len(root_keys)
        or set(schema["required"]) != root_keys
        or schema.get("additionalProperties") is not False
    ):
        raise ValueError("form schema must describe the complete v1 draft contract")
    if properties["schema_version"].get("const") != SCHEMA_VERSION:
        raise ValueError("unsupported form schema version")
    _nonempty(properties["form_id"].get("const"), "form_id")
    if properties["untrusted_inputs"].get("const") is not True:
        raise ValueError("form schema must preserve untrusted inputs")

    review = properties["human_review"]
    review_fields = {"required", "status", "reviewer", "reviewed_at", "signature"}
    review_properties = review.get("properties")
    if (
        review.get("type") != "object"
        or not isinstance(review.get("required"), list)
        or set(review["required"]) != review_fields
        or not isinstance(review_properties, dict)
        or set(review_properties) != review_fields
        or review.get("additionalProperties") is not False
        or review_properties["required"] != {"const": True}
        or review_properties["status"] != {"const": "PENDING"}
        or any(review_properties[key] != {"type": "null"} for key in ("reviewer", "reviewed_at", "signature"))
    ):
        raise ValueError("form schema must require pending human review")

    mapping = schema.get("x-form-mapping")
    if not isinstance(mapping, dict):
        raise ValueError("form schema requires x-form-mapping")
    if "catalog_notice" in mapping:
        _nonempty(mapping["catalog_notice"], "catalog_notice")
    source_pages = mapping.get("source_pages")
    if type(source_pages) is not int or source_pages < 1:
        raise ValueError("source_pages must be a positive integer")
    headings = mapping.get("inspection_column_headings")
    if not isinstance(headings, list) or not headings:
        raise ValueError("inspection_column_headings must be a non-empty list")
    for heading in headings:
        _nonempty(heading, "inspection column heading")
    inspection_pages = mapping.get("inspection_pages")
    if inspection_pages is not None and (
        not isinstance(inspection_pages, list)
        or any(type(page) is not int or not 1 <= page <= source_pages for page in inspection_pages)
        or len(set(inspection_pages)) != len(inspection_pages)
    ):
        raise ValueError("inspection_pages must contain unique source page numbers")

    catalog = mapping.get("field_catalog")
    if not isinstance(catalog, list) or not catalog:
        raise ValueError("field_catalog must be a non-empty list")
    field_ids: set[str] = set()
    for definition in catalog:
        if not isinstance(definition, dict):
            raise ValueError("catalog fields must be objects")
        for key in ("id", "group", "label_de", "evidence_mode"):
            _nonempty(definition.get(key), f"catalog field {key}")
        field_id = definition["id"]
        if field_id in field_ids:
            raise ValueError(f"duplicate catalog field ID: {field_id}")
        field_ids.add(field_id)
        page = definition.get("page")
        if type(page) is not int or not 1 <= page <= source_pages:
            raise ValueError(f"invalid source page for catalog field: {field_id}")
        human_only = definition.get("human_only")
        if type(human_only) is not bool:
            raise ValueError(f"human_only must be a boolean for catalog field: {field_id}")
        expected_mode = "HUMAN" if human_only else "VISUAL"
        if not definition["evidence_mode"].startswith(expected_mode):
            raise ValueError(f"evidence mode conflicts with human_only for: {field_id}")
        if "date_role" in definition and definition["date_role"] not in DATE_ROLES:
            raise ValueError(f"invalid date role for catalog field: {field_id}")
        if "applicability" in definition:
            _nonempty(definition["applicability"], f"catalog field applicability: {field_id}")

    definitions = schema.get("$defs")
    definition_fields = {
        "field_observation": {"field_id", "source_page", "coverage", "proposals"},
        "proposal": {"value", "reason", "coverage", "evidence_ids", "provenance", "untrusted"},
        "inspection_record": {"inspection_id", "column_number", "source_heading", "inspection_date", "field_observations"},
    }
    if not isinstance(definitions, dict) or any(
        not isinstance(definitions.get(name), dict)
        or definitions[name].get("type") != "object"
        or not isinstance(definitions[name].get("properties"), dict)
        or not fields <= definitions[name]["properties"].keys()
        for name, fields in definition_fields.items()
    ):
        raise ValueError("form schema is missing required v1 definitions")

    def check_references(value: Any) -> None:
        if isinstance(value, dict):
            if "$ref" in value:
                reference = value["$ref"]
                prefix = "#/$defs/"
                if (
                    not isinstance(reference, str)
                    or not reference.startswith(prefix)
                    or "/" in reference[len(prefix):]
                    or not isinstance(definitions.get(reference[len(prefix):]), dict)
                ):
                    raise ValueError("form schema contains an unsupported or unresolved reference")
            for nested in value.values():
                check_references(nested)
        elif isinstance(value, list):
            for nested in value:
                check_references(nested)

    check_references(schema)
    fields = properties["field_observations"]
    fixed_fields = fields.get("properties")
    if fields.get("type") != "object":
        raise ValueError("field_observations must be an object")
    if fixed_fields is None:
        if fields.get("additionalProperties") != {"$ref": "#/$defs/field_observation"}:
            raise ValueError("form schema must describe catalog observations")
    elif (
        not isinstance(fixed_fields, dict)
        or set(fixed_fields) != field_ids
        or any(not isinstance(value, dict) for value in fixed_fields.values())
        or not isinstance(fields.get("required"), list)
        or set(fields["required"]) != field_ids
        or fields.get("additionalProperties") is not False
    ):
        raise ValueError("form schema fields must match the selected catalog")
    for key in ("inspection_history", "label_date_observations"):
        items = properties[key].get("items")
        if properties[key].get("type") != "array" or not isinstance(items, dict) or not isinstance(items.get("$ref"), str):
            raise ValueError(f"{key} must reference a v1 record definition")


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid JSON constant: {value}")


def _parse_form_schema(text: str) -> dict[str, Any]:
    try:
        schema = json.loads(
            text, object_pairs_hook=_unique_json_object, parse_constant=_reject_json_constant
        )
        _validate_form_schema(schema)
    except (TypeError, ValueError, RecursionError) as exc:
        raise ValueError(f"invalid form schema: {exc}") from exc
    return schema


def _bundled_form_schema() -> dict[str, Any]:
    asset = files("ultron")
    for part in SCHEMA_ASSET:
        asset = asset.joinpath(part)
    try:
        text = asset.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise ValueError("could not read the bundled synthetic form schema") from exc
    schema = _parse_form_schema(text)
    if schema["properties"]["form_id"]["const"] != GENERIC_FORM_ID:
        raise ValueError("bundled form schema must use the synthetic form ID")
    return schema


def _load_form_schema() -> dict[str, Any]:
    if SCHEMA_ENV not in os.environ:
        return _bundled_form_schema()
    configured = os.environ[SCHEMA_ENV]
    if not configured.strip():
        raise ValueError(f"{SCHEMA_ENV} must name a local schema file")
    try:
        path = Path(configured).expanduser()
        if not path.is_file():
            raise ValueError(f"{SCHEMA_ENV} must name an existing local schema file")
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError, RuntimeError) as exc:
        raise ValueError(f"could not read local form schema from {SCHEMA_ENV}") from exc
    schema = _parse_form_schema(text)
    if (
        schema["properties"]["form_id"]["const"] == GENERIC_FORM_ID
        and schema != _bundled_form_schema()
    ):
        raise ValueError("the synthetic form ID is reserved for the bundled example")
    return schema


def load_form_schema() -> dict[str, Any]:
    """Load the synthetic default, or an explicitly selected local v1 schema."""
    return deepcopy(_load_form_schema())


def _catalog(schema: Mapping[str, Any] | None = None) -> dict[str, dict[str, Any]]:
    if schema is None:
        schema = _load_form_schema()
    return {
        item["id"]: item for item in schema["x-form-mapping"]["field_catalog"]
    }


def _inspection_pages(mapping: Mapping[str, Any]) -> set[int]:
    return set(mapping.get("inspection_pages", (2, 3)))


def _utc_timestamp(value: str | None = None) -> str:
    if value is None:
        return datetime.now(timezone.utc).isoformat()
    if not isinstance(value, str):
        raise ValueError("timestamps must be ISO-formatted strings")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("timestamps must include a timezone")
    return value


def _json_copy(value: Any) -> Any:
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError("contract data must be finite JSON data") from exc


def _nonempty(value: str, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value


def new_draft(
    *,
    session_id: str,
    ladder_id: str,
    created_at: str | None = None,
) -> dict[str, Any]:
    """Create an incomplete draft with every source field explicitly present."""
    _nonempty(session_id, "session_id")
    _nonempty(ladder_id, "ladder_id")
    schema = _load_form_schema()
    catalog = _catalog(schema)
    return {
        "schema_version": SCHEMA_VERSION,
        "form_id": schema["properties"]["form_id"]["const"],
        "session_id": session_id,
        "ladder_id": ladder_id,
        "untrusted_inputs": True,
        "created_at": _utc_timestamp(created_at),
        "field_observations": {
            field_id: {
                "field_id": field_id,
                "source_page": definition["page"],
                "coverage": "HUMAN_ONLY" if definition["human_only"] else "NOT_ASSESSED",
                "proposals": [],
            }
            for field_id, definition in catalog.items()
        },
        "inspection_history": [],
        "label_date_observations": [],
        "human_review": {
            "required": True,
            "status": "PENDING",
            "reviewer": None,
            "reviewed_at": None,
            "signature": None,
        },
    }


def _proposals_conflict(proposals: list[Mapping[str, Any]]) -> bool:
    readings = [proposal["value"] for proposal in proposals if proposal["value"] is not None]
    return any(proposal["coverage"] == "CONFLICTING" for proposal in proposals) or any(
        reading != readings[0] for reading in readings[1:]
    )


def add_proposal(
    draft: dict[str, Any],
    field_id: str,
    *,
    value: Any,
    coverage: str,
    evidence_ids: list[str] | tuple[str, ...],
    provenance: Mapping[str, Any],
    reason: str | None = None,
) -> dict[str, Any]:
    """Append a proposal; conflicting coverage cannot be cleared by later readings."""
    schema = _load_form_schema()
    _validate_draft(draft, schema)
    catalog = _catalog(schema)
    if not isinstance(field_id, str) or field_id not in catalog or field_id not in draft.get(
        "field_observations", {}
    ):
        raise ValueError(f"unknown selected form field: {field_id}")
    if catalog[field_id]["human_only"] or draft["field_observations"][field_id]["coverage"] == "HUMAN_ONLY":
        raise ValueError(f"{field_id} is a human-only checklist task")
    if not isinstance(coverage, str) or coverage not in COVERAGE_STATES - {
        "NOT_ASSESSED",
        "HUMAN_ONLY",
    }:
        raise ValueError(f"coverage must be one of {sorted(COVERAGE_STATES)}")
    if coverage == "OBSERVED" and value is None:
        raise ValueError("observed proposals require a value")
    if not isinstance(evidence_ids, (list, tuple)) or not evidence_ids:
        raise ValueError("proposals require at least one evidence ID")
    normalized_ids = [_nonempty(item, "evidence ID") for item in evidence_ids]
    if len(set(normalized_ids)) != len(normalized_ids):
        raise ValueError("evidence IDs must be unique within a proposal")
    if not isinstance(provenance, Mapping):
        raise ValueError("provenance must be an object")
    provenance_copy = _json_copy(dict(provenance))
    _nonempty(provenance_copy.get("method"), "provenance.method")
    _nonempty(provenance_copy.get("producer"), "provenance.producer")
    if provenance_copy.get("untrusted", True) is not True:
        raise ValueError("all source and proposal data must remain untrusted")
    provenance_copy["recorded_at"] = _utc_timestamp(
        provenance_copy.get("recorded_at")
    )
    provenance_copy["untrusted"] = True
    if coverage == "NOT_APPLICABLE" and not (isinstance(reason, str) and reason.strip()):
        raise ValueError("not-applicable proposals require an explicit reason")
    if reason is not None and not isinstance(reason, str):
        raise ValueError("proposal reason must be text or null")

    proposal = {
        "value": _json_copy(value),
        "reason": reason,
        "coverage": coverage,
        "evidence_ids": normalized_ids,
        "provenance": provenance_copy,
        "untrusted": True,
    }
    field = draft["field_observations"][field_id]
    has_conflict = field["coverage"] == "CONFLICTING" or _proposals_conflict(
        field["proposals"] + [proposal]
    )
    if has_conflict:
        proposal["coverage"] = "CONFLICTING"
    field["proposals"].append(proposal)
    field["coverage"] = "CONFLICTING" if has_conflict else coverage
    return deepcopy(proposal)


def add_label_date_observation(
    draft: dict[str, Any],
    *,
    date_role: str,
    value: Any,
    evidence_ids: list[str] | tuple[str, ...],
    provenance: Mapping[str, Any],
    coverage: str = "OBSERVED",
) -> dict[str, Any]:
    """Record a date reading, retaining conflicts within each separate date role."""
    _validate_draft(draft, _load_form_schema())
    if not isinstance(date_role, str) or date_role not in DATE_ROLES:
        raise ValueError(f"date_role must be one of {sorted(DATE_ROLES)}")
    if not isinstance(coverage, str) or coverage not in DATE_COVERAGE_STATES:
        raise ValueError(f"coverage must be one of {sorted(DATE_COVERAGE_STATES)}")
    if coverage == "OBSERVED" and value is None:
        raise ValueError("observed date readings require a value")
    if not isinstance(evidence_ids, (list, tuple)) or not evidence_ids:
        raise ValueError("date readings require at least one evidence ID")
    normalized_ids = [_nonempty(item, "evidence ID") for item in evidence_ids]
    if len(set(normalized_ids)) != len(normalized_ids):
        raise ValueError("evidence IDs must be unique within a date reading")
    if not isinstance(provenance, Mapping):
        raise ValueError("provenance must be an object")
    source = _json_copy(dict(provenance))
    _nonempty(source.get("method"), "provenance.method")
    _nonempty(source.get("producer"), "provenance.producer")
    if source.get("untrusted", True) is not True:
        raise ValueError("all source and proposal data must remain untrusted")
    source["recorded_at"] = _utc_timestamp(source.get("recorded_at"))
    source["untrusted"] = True
    value_copy = _json_copy(value)
    same_role = [
        item for item in draft["label_date_observations"] if item["date_role"] == date_role
    ]
    if _proposals_conflict(
        same_role + [{"value": value_copy, "coverage": coverage}]
    ):
        coverage = "CONFLICTING"
    result = {
        "date_role": date_role,
        "value": value_copy,
        "coverage": coverage,
        "evidence_ids": normalized_ids,
        "provenance": source,
        "untrusted": True,
    }
    draft["label_date_observations"].append(result)
    return deepcopy(result)


def append_inspection_record(
    draft: dict[str, Any],
    *,
    inspection_id: str,
    column_number: int,
    inspection_date: str | None = None,
) -> dict[str, Any]:
    """Append a snapshot of the form's inspection fields; never replace history."""
    schema = _load_form_schema()
    _validate_draft(draft, schema)
    mapping = schema["x-form-mapping"]
    headings = mapping["inspection_column_headings"]
    catalog = _catalog(schema)
    inspection_pages = _inspection_pages(mapping)
    _nonempty(inspection_id, "inspection_id")
    if not isinstance(column_number, int) or isinstance(column_number, bool):
        raise ValueError(f"column_number must be an integer from 1 to {len(headings)}")
    if not 1 <= column_number <= len(headings):
        raise ValueError(f"column_number must be an integer from 1 to {len(headings)}")
    if inspection_date is not None and not isinstance(inspection_date, str):
        raise ValueError("inspection_date must be a string or None")
    history = draft.get("inspection_history")
    if not isinstance(history, list):
        raise ValueError("draft inspection_history must be a list")
    if any(item.get("inspection_id") == inspection_id for item in history):
        raise ValueError(f"inspection record already exists: {inspection_id}")
    fields = draft.get("field_observations", {})
    snapshot = {
        field_id: deepcopy(field)
        for field_id, field in fields.items()
        if catalog.get(field_id, {}).get("page", 0) in inspection_pages
    }
    record = {
        "inspection_id": inspection_id,
        "column_number": column_number,
        "source_heading": headings[column_number - 1],
        "inspection_date": inspection_date,
        "field_observations": snapshot,
    }
    history.append(record)
    return deepcopy(record)


def validate_draft(draft: Mapping[str, Any]) -> None:
    """Raise ValueError for malformed or unsafe-to-export contract data."""
    _validate_draft(draft, _load_form_schema())


def _validate_draft(draft: Mapping[str, Any], schema: Mapping[str, Any]) -> None:
    if not isinstance(draft, Mapping):
        raise ValueError("draft must be an object")
    expected_root_keys = {
        "schema_version",
        "form_id",
        "session_id",
        "ladder_id",
        "untrusted_inputs",
        "created_at",
        "field_observations",
        "inspection_history",
        "label_date_observations",
        "human_review",
    }
    if set(draft) != expected_root_keys:
        raise ValueError("draft contains missing or unsupported top-level fields")
    if (
        draft.get("schema_version") != SCHEMA_VERSION
        or draft.get("form_id") != schema["properties"]["form_id"]["const"]
    ):
        raise ValueError("unsupported checklist schema or form")
    if draft.get("untrusted_inputs") is not True:
        raise ValueError("all checklist inputs must remain explicitly untrusted")
    for key in ("session_id", "ladder_id"):
        _nonempty(draft.get(key), key)
    _utc_timestamp(draft.get("created_at"))
    catalog = _catalog(schema)
    fields = draft.get("field_observations")
    if not isinstance(fields, Mapping) or set(fields) != set(catalog):
        raise ValueError("field_observations must contain every mapped field exactly once")

    def validate_field(field_id: str, field: Mapping[str, Any]) -> None:
        definition = catalog.get(field_id)
        if not definition or not isinstance(field, Mapping):
            raise ValueError(f"invalid field observation: {field_id}")
        if set(field) != {"field_id", "source_page", "coverage", "proposals"}:
            raise ValueError(f"field observation contains unsupported or missing fields: {field_id}")
        if (
            field.get("field_id") != field_id
            or type(field.get("source_page")) is not int
            or field.get("source_page") != definition["page"]
        ):
            raise ValueError(f"field identity or source page mismatch: {field_id}")
        coverage = field.get("coverage")
        if not isinstance(coverage, str) or coverage not in COVERAGE_STATES:
            raise ValueError(f"invalid coverage state for {field_id}")
        if definition["human_only"] and coverage != "HUMAN_ONLY":
            raise ValueError(f"human-only task must remain explicit: {field_id}")
        proposals = field.get("proposals")
        if not isinstance(proposals, list):
            raise ValueError(f"proposals must be a list for {field_id}")
        if definition["human_only"] and proposals:
            raise ValueError(f"human-only field cannot contain proposals: {field_id}")
        if coverage == "OBSERVED" and not proposals:
            raise ValueError(f"observed field requires a proposal: {field_id}")
        for proposal in proposals:
            if not isinstance(proposal, Mapping):
                raise ValueError(f"proposal must be an object for {field_id}")
            if set(proposal) != {
                "value",
                "reason",
                "coverage",
                "evidence_ids",
                "provenance",
                "untrusted",
            }:
                raise ValueError(f"proposal has unsupported or missing fields: {field_id}")
            if proposal.get("untrusted") is not True:
                raise ValueError(f"proposal must be explicitly untrusted: {field_id}")
            proposal_coverage = proposal.get("coverage")
            if not isinstance(proposal_coverage, str) or proposal_coverage not in COVERAGE_STATES - {
                "NOT_ASSESSED",
                "HUMAN_ONLY",
            }:
                raise ValueError(f"invalid proposal coverage for {field_id}")
            if proposal_coverage == "OBSERVED" and proposal.get("value") is None:
                raise ValueError(f"observed proposal requires a value: {field_id}")
            ids = proposal.get("evidence_ids")
            if not isinstance(ids, list) or not ids or any(
                not isinstance(item, str) or not item.strip() for item in ids
            ):
                raise ValueError(f"proposal needs evidence references: {field_id}")
            if len(set(ids)) != len(ids):
                raise ValueError(f"proposal evidence references must be unique: {field_id}")
            if proposal.get("coverage") == "NOT_APPLICABLE" and not (
                isinstance(proposal.get("reason"), str) and proposal["reason"].strip()
            ):
                raise ValueError(f"not-applicable proposal needs a reason: {field_id}")
            if proposal.get("reason") is not None and not isinstance(proposal.get("reason"), str):
                raise ValueError(f"proposal reason must be text or null: {field_id}")
            provenance = proposal.get("provenance")
            if not isinstance(provenance, Mapping):
                raise ValueError(f"proposal provenance is required: {field_id}")
            if "recorded_at" not in provenance:
                raise ValueError(f"proposal timestamp is required: {field_id}")
            _nonempty(provenance.get("method"), "provenance.method")
            _nonempty(provenance.get("producer"), "provenance.producer")
            if provenance.get("untrusted") is not True:
                raise ValueError(f"proposal provenance must be untrusted: {field_id}")
            _utc_timestamp(provenance.get("recorded_at"))
            _json_copy(proposal.get("value"))
        if coverage != "CONFLICTING" and _proposals_conflict(proposals):
            raise ValueError(f"conflicting proposals require conflicting field coverage: {field_id}")

    for field_id, field in fields.items():
        validate_field(field_id, field)

    history = draft.get("inspection_history")
    if not isinstance(history, list):
        raise ValueError("inspection_history must be a list")
    seen_inspections: set[str] = set()
    mapping = schema["x-form-mapping"]
    headings = mapping["inspection_column_headings"]
    inspection_pages = _inspection_pages(mapping)
    for record in history:
        if not isinstance(record, Mapping):
            raise ValueError("inspection records must be objects")
        if set(record) != {
            "inspection_id",
            "column_number",
            "source_heading",
            "inspection_date",
            "field_observations",
        }:
            raise ValueError("inspection record contains unsupported or missing fields")
        inspection_id = _nonempty(record.get("inspection_id"), "inspection_id")
        if inspection_id in seen_inspections:
            raise ValueError(f"duplicate inspection record: {inspection_id}")
        seen_inspections.add(inspection_id)
        column_number = record.get("column_number")
        if not isinstance(column_number, int) or isinstance(column_number, bool) or not 1 <= column_number <= len(headings):
            raise ValueError(f"inspection column must be an integer from 1 to {len(headings)}")
        if record.get("source_heading") != headings[column_number - 1]:
            raise ValueError("inspection heading must preserve the selected catalog column")
        if record.get("inspection_date") is not None and not isinstance(
            record.get("inspection_date"), str
        ):
            raise ValueError("inspection_date must be a string or None")
        record_fields = record.get("field_observations")
        if not isinstance(record_fields, Mapping):
            raise ValueError("inspection record fields must be an object")
        for field_id, field in record_fields.items():
            if catalog.get(field_id, {}).get("page", 0) not in inspection_pages:
                raise ValueError(f"inspection history cannot contain inventory field: {field_id}")
            validate_field(field_id, field)

    date_observations = draft.get("label_date_observations")
    if not isinstance(date_observations, list):
        raise ValueError("label_date_observations must be a list")
    for observation in date_observations:
        if not isinstance(observation, Mapping) or not isinstance(
            observation.get("date_role"), str
        ) or observation.get("date_role") not in DATE_ROLES:
            raise ValueError("invalid label date observation role")
        if not isinstance(observation.get("coverage"), str) or observation.get(
            "coverage"
        ) not in DATE_COVERAGE_STATES:
            raise ValueError("invalid label date coverage")
        if observation.get("coverage") == "OBSERVED" and observation.get("value") is None:
            raise ValueError("observed date reading requires a value")
        if set(observation) != {
            "date_role",
            "value",
            "coverage",
            "evidence_ids",
            "provenance",
            "untrusted",
        }:
            raise ValueError("label date observation contains unsupported or missing fields")
        evidence_ids = observation.get("evidence_ids")
        if not isinstance(evidence_ids, list) or not evidence_ids or any(
            not isinstance(item, str) or not item.strip() for item in evidence_ids
        ):
            raise ValueError("label date observation requires evidence")
        if observation.get("untrusted") is not True:
            raise ValueError("label date observations must be explicitly untrusted")
        provenance = observation.get("provenance")
        if not isinstance(provenance, Mapping) or provenance.get("untrusted") is not True:
            raise ValueError("label date provenance must be explicitly untrusted")
        if "recorded_at" not in provenance:
            raise ValueError("label date provenance timestamp is required")
        _nonempty(provenance.get("method"), "provenance.method")
        _nonempty(provenance.get("producer"), "provenance.producer")
        _utc_timestamp(provenance.get("recorded_at"))
        _json_copy(observation.get("value"))

    for date_role in DATE_ROLES:
        same_role = [
            item for item in date_observations if item["date_role"] == date_role
        ]
        if same_role and _proposals_conflict(same_role) and same_role[-1]["coverage"] != "CONFLICTING":
            raise ValueError(f"conflicting date readings require conflicting latest coverage: {date_role}")

    review = draft.get("human_review")
    if review != {
        "required": True,
        "status": "PENDING",
        "reviewer": None,
        "reviewed_at": None,
        "signature": None,
    }:
        raise ValueError("this contract only exports drafts pending qualified human review")
    _json_copy(dict(draft))


def serialize_draft(draft: Mapping[str, Any]) -> str:
    """Validate and serialize without permitting NaN/Infinity or sign-off fields."""
    validate_draft(draft)
    return json.dumps(draft, ensure_ascii=False, indent=2, allow_nan=False)
