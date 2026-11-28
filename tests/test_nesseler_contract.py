from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import unittest
from unittest import mock
import uuid

from ultron.capture_session import CaptureSession
from ultron.checklist import (
    FORM_ID,
    add_label_date_observation,
    add_proposal,
    append_inspection_record,
    load_form_schema,
    new_draft,
    serialize_draft,
    validate_draft,
)


class DraftContractTests(unittest.TestCase):
    def setUp(self):
        environment = mock.patch.dict(os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        os.environ.pop("ULTRON_FORM_SCHEMA", None)
        self.workspace = Path("tests") / (".form-schema-" + uuid.uuid4().hex)
        self.workspace.mkdir()
        self.addCleanup(shutil.rmtree, self.workspace, ignore_errors=True)

    def _write_schema(self, schema, name="local.json"):
        path = self.workspace / name
        path.write_text(json.dumps(schema, ensure_ascii=False), encoding="utf-8")
        return path

    def test_label_date_conflicts_survive_unreadable_and_matching_readings(self):
        draft = new_draft(session_id="date-conflict", ladder_id="ladder")
        readings = ("2024-06", "2024-08", None, "2024-06", "2024-08")
        for index, value in enumerate(readings):
            observation = add_label_date_observation(
                draft,
                date_role="MANUFACTURER_FINAL_CONTROL",
                value=value,
                coverage="UNREADABLE" if value is None else "OBSERVED",
                evidence_ids=[f"date-{index}"],
                provenance={"method": "fixture", "producer": "test"},
            )
            if index:
                self.assertEqual(observation["coverage"], "CONFLICTING")
        other_role = add_label_date_observation(
            draft,
            date_role="RECURRING_INSPECTION",
            value="2024-08",
            evidence_ids=["inspection-date"],
            provenance={"method": "fixture", "producer": "test"},
        )
        self.assertEqual(other_role["coverage"], "OBSERVED")
        self.assertEqual(
            [item["value"] for item in draft["label_date_observations"][:-1]],
            list(readings),
        )
        for hide_all_flags in (False, True):
            with self.subTest(hide_all_flags=hide_all_flags):
                restored = json.loads(serialize_draft(draft))
                same_role = restored["label_date_observations"][:-1]
                same_role[-1]["coverage"] = "OBSERVED"
                if hide_all_flags:
                    for item in same_role:
                        item["coverage"] = "UNREADABLE" if item["value"] is None else "OBSERVED"
                with self.assertRaisesRegex(ValueError, "conflicting latest coverage"):
                    serialize_draft(restored)

    def test_conflict_survives_null_and_repeated_readings(self):
        for readings in (("ABC", "DEF", None, "ABC"), ("FAIL", "PASS", None, "PASS")):
            with self.subTest(readings=readings):
                draft = new_draft(session_id="conflict", ladder_id="ladder")
                for index, value in enumerate(readings):
                    add_proposal(
                        draft,
                        "demo_overview_note",
                        value=value,
                        coverage="OCCLUDED" if value is None else "OBSERVED",
                        evidence_ids=[f"evidence-{index}"],
                        provenance={"method": "fixture", "producer": "test"},
                    )
                    if index:
                        self.assertEqual(
                            draft["field_observations"]["demo_overview_note"]["coverage"],
                            "CONFLICTING",
                        )
                field = draft["field_observations"]["demo_overview_note"]
                self.assertEqual([item["value"] for item in field["proposals"]], list(readings))
                serialize_draft(draft)

    def test_explicit_conflict_survives_same_value_without_contradictory_readings(self):
        draft = new_draft(session_id="explicit-conflict", ladder_id="ladder")
        for coverage in ("CONFLICTING", "OBSERVED"):
            add_proposal(
                draft,
                "demo_label_text",
                value="ABC",
                coverage=coverage,
                evidence_ids=["label"],
                provenance={"method": "fixture", "producer": "test"},
            )
        self.assertEqual(
            draft["field_observations"]["demo_label_text"]["coverage"], "CONFLICTING"
        )

    def test_validation_rejects_hidden_conflicts_in_draft_and_history(self):
        draft = new_draft(session_id="hidden-conflict", ladder_id="ladder")
        for value in ("FAIL", "PASS"):
            add_proposal(
                draft,
                "demo_overview_note",
                value=value,
                coverage="OBSERVED",
                evidence_ids=["detail"],
                provenance={"method": "fixture", "producer": "test"},
            )
        append_inspection_record(draft, inspection_id="history", column_number=1)
        for location in ("field_observations", "inspection_history"):
            for hide_proposal_flags in (False, True):
                with self.subTest(location=location, hide_proposal_flags=hide_proposal_flags):
                    restored = json.loads(serialize_draft(draft))
                    fields = (
                        restored["field_observations"]
                        if location == "field_observations"
                        else restored["inspection_history"][0]["field_observations"]
                    )
                    field = fields["demo_overview_note"]
                    field["coverage"] = "OBSERVED"
                    if hide_proposal_flags:
                        for proposal in field["proposals"]:
                            proposal["coverage"] = "OBSERVED"
                    with self.assertRaisesRegex(ValueError, "conflicting field coverage"):
                        validate_draft(restored)
                    with self.assertRaisesRegex(ValueError, "conflicting field coverage"):
                        serialize_draft(restored)

    def test_default_catalog_is_an_independent_synthetic_example(self):
        schema = load_form_schema()
        mapping = schema["x-form-mapping"]
        catalog = mapping["field_catalog"]
        self.assertNotEqual(schema["properties"]["form_id"]["const"], FORM_ID)
        self.assertEqual(
            schema["properties"]["form_id"]["const"], "ultron_synthetic_ladder_demo_v1"
        )
        self.assertIn("Synthetic", mapping["catalog_notice"])
        self.assertIn("not an official form", mapping["catalog_notice"])
        self.assertIn("not a source-page mapping", mapping["catalog_notice"])
        self.assertEqual({field["page"] for field in catalog}, {1})
        self.assertEqual(
            mapping["inspection_column_headings"],
            ["Example A", "Example B"],
        )
        self.assertTrue(
            next(field for field in catalog if field["id"] == "demo_physical_review")["human_only"]
        )
        self.assertEqual(
            {field["date_role"]: field["id"] for field in catalog if "date_role" in field},
            {
                "RECURRING_INSPECTION": "demo_last_inspection_date",
                "NEXT_INSPECTION": "demo_next_inspection_date",
            },
        )
        for field in catalog:
            self.assertNotIn("options_de", field)
            self.assertNotIn("examples_de", field)

    def test_legacy_fourth_column_draft_round_trips_with_conflict_and_human_review(self):
        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = FORM_ID
        schema["properties"]["field_observations"] = {
            "type": "object",
            "additionalProperties": {"$ref": "#/$defs/field_observation"},
        }
        schema["x-form-mapping"]["source_pages"] = 3
        schema["x-form-mapping"].pop("inspection_pages")
        for field in schema["x-form-mapping"]["field_catalog"]:
            field["page"] = 2
        headings = ["1. Prüfung", "2. Prüfung", "3. Prüfung", "3. Prüfung", "5. Prüfung"]
        schema["x-form-mapping"]["inspection_column_headings"] = headings
        record_properties = schema["$defs"]["inspection_record"]["properties"]
        record_properties["column_number"]["maximum"] = len(headings)
        record_properties["source_heading"]["enum"] = headings
        record_properties["field_observations"] = {
            "type": "object",
            "additionalProperties": {"$ref": "#/$defs/field_observation"},
        }
        path = self._write_schema(schema, "legacy-shaped-fixture.json")
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            draft = new_draft(session_id="legacy-session", ladder_id="ladder")
            for value in ("A", "B"):
                add_proposal(
                    draft,
                    "demo_overview_note",
                    value=value,
                    coverage="OBSERVED",
                    evidence_ids=["detail"],
                    provenance={"method": "fixture", "producer": "test"},
                )
            append_inspection_record(draft, inspection_id="legacy-fourth", column_number=4)
            self.assertEqual(draft["inspection_history"][0]["source_heading"], "3. Prüfung")
            restored = json.loads(json.dumps(draft))
            validate_draft(restored)
            self.assertEqual(json.loads(serialize_draft(restored)), restored)
            self.assertEqual(
                restored["inspection_history"][0]["field_observations"]["demo_overview_note"]["coverage"],
                "CONFLICTING",
            )
            self.assertEqual(
                restored["field_observations"]["demo_physical_review"]["coverage"], "HUMAN_ONLY"
            )
            self.assertEqual(restored["human_review"]["status"], "PENDING")
        with self.assertRaisesRegex(ValueError, "schema or form"):
            serialize_draft(restored)

    def test_explicit_local_catalog_is_required_for_its_drafts(self):
        generic = new_draft(session_id="generic", ladder_id="demo")
        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "locally_selected_fixture"
        path = self._write_schema(schema)
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            local = new_draft(session_id="local", ladder_id="demo")
            self.assertEqual(local["form_id"], "locally_selected_fixture")
            serialize_draft(local)
            before = deepcopy(generic)
            operations = (
                lambda: validate_draft(generic),
                lambda: serialize_draft(generic),
                lambda: add_proposal(
                    generic, "demo_label_text", value="X", coverage="OBSERVED",
                    evidence_ids=["image"], provenance={"method": "fixture", "producer": "test"},
                ),
                lambda: add_label_date_observation(
                    generic, date_role="NEXT_INSPECTION", value="2027",
                    evidence_ids=["image"], provenance={"method": "fixture", "producer": "test"},
                ),
                lambda: append_inspection_record(generic, inspection_id="other", column_number=1),
            )
            for operation in operations:
                with self.subTest(operation=operation):
                    with self.assertRaisesRegex(ValueError, "schema or form"):
                        operation()
                    self.assertEqual(generic, before)
        with self.assertRaisesRegex(ValueError, "schema or form"):
            serialize_draft(local)
        self.assertEqual(json.loads(serialize_draft(generic)), generic)

    def test_missing_and_invalid_overrides_fail_closed_without_cache_fallback(self):
        valid = load_form_schema()
        path = self._write_schema(valid)
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            self.assertEqual(load_form_schema(), valid)
            path.unlink()
            with self.assertRaises(ValueError):
                load_form_schema()
            with self.assertRaises(ValueError):
                new_draft(session_id="invalid", ladder_id="demo")

        valid_text = json.dumps(valid, ensure_ascii=False)
        invalid_schemas = [
            "{",
            "{}",
            valid_text[:-1] + ', "type": "object"}',
            valid_text[:-1] + ', "unused": NaN}',
        ]
        bad_version = deepcopy(valid)
        bad_version["properties"]["schema_version"]["const"] = "2.0.0"
        invalid_schemas.append(json.dumps(bad_version))
        bad_human_flag = deepcopy(valid)
        bad_human_flag["x-form-mapping"]["field_catalog"][0]["human_only"] = "false"
        invalid_schemas.append(json.dumps(bad_human_flag))
        bad_evidence_mode = deepcopy(valid)
        bad_evidence_mode["x-form-mapping"]["field_catalog"][0]["evidence_mode"] = "HUMAN_CONTEXT"
        invalid_schemas.append(json.dumps(bad_evidence_mode))
        duplicate_field = deepcopy(valid)
        duplicate_field["x-form-mapping"]["field_catalog"].append(
            duplicate_field["x-form-mapping"]["field_catalog"][0]
        )
        invalid_schemas.append(json.dumps(duplicate_field))
        bad_role = deepcopy(valid)
        bad_role["x-form-mapping"]["field_catalog"][0]["date_role"] = "UNRECOGNIZED"
        invalid_schemas.append(json.dumps(bad_role))
        missing_definition = deepcopy(valid)
        del missing_definition["$defs"]["inspection_record"]
        invalid_schemas.append(json.dumps(missing_definition))
        invalid_fields = deepcopy(valid)
        invalid_fields["properties"]["field_observations"]["type"] = "array"
        invalid_schemas.append(json.dumps(invalid_fields))
        missing_field = deepcopy(valid)
        del missing_field["properties"]["field_observations"]["properties"]["demo_label_text"]
        invalid_schemas.append(json.dumps(missing_field))
        unsafe_review = deepcopy(valid)
        unsafe_review["properties"]["human_review"]["properties"]["status"]["const"] = "COMPLETE"
        invalid_schemas.append(json.dumps(unsafe_review))
        invalid_items = deepcopy(valid)
        invalid_items["properties"]["inspection_history"]["items"] = []
        invalid_schemas.append(json.dumps(invalid_items))
        reused_generic_id = deepcopy(valid)
        reused_generic_id["x-form-mapping"]["field_catalog"][0]["label_de"] = "Different label"
        invalid_schemas.append(json.dumps(reused_generic_id))
        for text in invalid_schemas:
            path.write_text(text, encoding="utf-8")
            with self.subTest(text=text[:40]):
                with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
                    with self.assertRaises(ValueError):
                        load_form_schema()
                    with self.assertRaises(ValueError):
                        new_draft(session_id="invalid", ladder_id="demo")
        for configured in ("", " ", str(self.workspace), str(self.workspace / "missing.json")):
            with self.subTest(configured=configured):
                with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": configured}):
                    with self.assertRaises(ValueError):
                        load_form_schema()
        path.write_bytes(b"\xff")
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            with self.assertRaises(ValueError):
                load_form_schema()
        self.assertEqual(load_form_schema(), valid)

    def test_observed_readings_require_values_and_evidence(self):
        draft = new_draft(session_id="s", ladder_id="demo")
        before = deepcopy(draft)
        with self.assertRaisesRegex(ValueError, "require a value"):
            add_proposal(
                draft, "demo_label_text", value=None, coverage="OBSERVED",
                evidence_ids=["label"], provenance={"method": "fixture", "producer": "test"},
            )
        with self.assertRaisesRegex(ValueError, "require a value"):
            add_label_date_observation(
                draft, date_role="NEXT_INSPECTION", value=None, evidence_ids=["label"],
                provenance={"method": "fixture", "producer": "test"},
            )
        self.assertEqual(draft, before)

        draft["field_observations"]["demo_label_text"]["coverage"] = "OBSERVED"
        with self.assertRaisesRegex(ValueError, "requires a proposal"):
            validate_draft(draft)

        draft = new_draft(session_id="s", ladder_id="demo")
        add_proposal(
            draft, "demo_label_text", value="X", coverage="OBSERVED",
            evidence_ids=["label"], provenance={"method": "fixture", "producer": "test"},
        )
        draft["field_observations"]["demo_label_text"]["proposals"][0]["value"] = None
        with self.assertRaisesRegex(ValueError, "requires a value"):
            validate_draft(draft)

        draft = new_draft(session_id="s", ladder_id="demo")
        add_label_date_observation(
            draft, date_role="NEXT_INSPECTION", value="2027", evidence_ids=["label"],
            provenance={"method": "fixture", "producer": "test"},
        )
        draft["label_date_observations"][0]["value"] = None
        with self.assertRaisesRegex(ValueError, "requires a value"):
            validate_draft(draft)

    def test_a_human_only_draft_field_cannot_be_downgraded_by_a_replaced_catalog(self):
        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "local_fixture"
        path = self._write_schema(schema)
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            draft = new_draft(session_id="s", ladder_id="demo")
            field = schema["x-form-mapping"]["field_catalog"][-1]
            field["human_only"] = False
            field["evidence_mode"] = "VISUAL_LIMITED"
            self._write_schema(schema)
            with self.assertRaisesRegex(ValueError, "human-only"):
                add_proposal(
                    draft, "demo_physical_review", value="done", coverage="OBSERVED",
                    evidence_ids=["image"], provenance={"method": "fixture", "producer": "test"},
                )
            self.assertEqual(draft["field_observations"]["demo_physical_review"]["coverage"], "HUMAN_ONLY")

    def test_draft_keeps_proposals_dates_and_inspection_history_separate(self):
        draft = new_draft(session_id="session-1", ladder_id="ladder-1")
        for value in ("ABC-123", "ABC-12B"):
            add_proposal(
                draft,
                "demo_label_text",
                value=value,
                coverage="OBSERVED",
                evidence_ids=["label-front"],
                provenance={"method": "OCR", "producer": "fixture-reader"},
            )
        add_label_date_observation(
            draft,
            date_role="MANUFACTURER_FINAL_CONTROL",
            value="2024-06",
            evidence_ids=["manufacturer-label"],
            provenance={"method": "MANUAL_TRANSCRIPTION", "producer": "operator"},
        )
        add_label_date_observation(
            draft,
            date_role="RECURRING_INSPECTION",
            value="2025-02",
            evidence_ids=["inspection-sticker"],
            provenance={"method": "MANUAL_TRANSCRIPTION", "producer": "operator"},
        )
        conflicting_date = add_label_date_observation(
            draft,
            date_role="MANUFACTURER_FINAL_CONTROL",
            value="2024-08",
            evidence_ids=["manufacturer-label-detail"],
            provenance={"method": "OCR", "producer": "fixture-reader"},
        )
        first = append_inspection_record(
            draft, inspection_id="inspection-1", column_number=2, inspection_date="2025-02"
        )
        self.assertEqual(first["source_heading"], "Example B")
        with self.assertRaises(ValueError):
            append_inspection_record(
                draft, inspection_id="inspection-1", column_number=1
            )
        self.assertEqual(len(draft["inspection_history"]), 1)
        self.assertEqual(len(draft["field_observations"]["demo_label_text"]["proposals"]), 2)
        self.assertEqual(
            draft["field_observations"]["demo_label_text"]["coverage"], "CONFLICTING"
        )
        self.assertEqual(conflicting_date["coverage"], "CONFLICTING")
        self.assertEqual(
            [item["date_role"] for item in draft["label_date_observations"]],
            [
                "MANUFACTURER_FINAL_CONTROL",
                "RECURRING_INSPECTION",
                "MANUFACTURER_FINAL_CONTROL",
            ],
        )
        self.assertEqual(draft["human_review"]["status"], "PENDING")
        serialized = serialize_draft(draft)
        self.assertNotIn('"verdict"', serialized)
        self.assertNotIn('"safe_to_use"', serialized)
        self.assertEqual(json.loads(serialized)["schema_version"], "1.0.0")

    def test_human_only_and_not_applicable_fields_stay_unresolved_or_reasoned(self):
        draft = new_draft(session_id="session-2", ladder_id="ladder-2")
        self.assertEqual(
            draft["field_observations"]["demo_physical_review"]["coverage"], "HUMAN_ONLY"
        )
        with self.assertRaises(ValueError):
            add_proposal(
                draft,
                "demo_physical_review",
                value="functional",
                coverage="OBSERVED",
                evidence_ids=["photo"],
                provenance={"method": "VLM", "producer": "untrusted"},
            )
        with self.assertRaises(ValueError):
            add_proposal(
                draft,
                "demo_overview_note",
                value="not applicable",
                coverage="NOT_APPLICABLE",
                evidence_ids=["photo"],
                provenance={"method": "OCR", "producer": "fixture"},
            )
        add_proposal(
            draft,
            "demo_overview_note",
            value="not applicable",
            coverage="NOT_APPLICABLE",
            reason="This demo observation does not apply to the selected view.",
            evidence_ids=["overview"],
            provenance={"method": "MANUAL_TRANSCRIPTION", "producer": "operator"},
        )
        self.assertEqual(
            draft["field_observations"]["demo_overview_note"]["proposals"][0][
                "untrusted"
            ],
            True,
        )

    def test_export_rejects_signoff_and_nonfinite_proposals(self):
        draft = new_draft(session_id="session-3", ladder_id="ladder-3")
        draft["human_review"]["status"] = "APPROVED"
        with self.assertRaises(ValueError):
            serialize_draft(draft)
        draft = new_draft(session_id="session-4", ladder_id="ladder-4")
        with self.assertRaises(ValueError):
            add_proposal(
                draft,
                "demo_label_text",
                value=float("nan"),
                coverage="OBSERVED",
                evidence_ids=["label"],
                provenance={"method": "OCR", "producer": "fixture"},
            )


class CaptureSessionContractTests(unittest.TestCase):
    def setUp(self):
        self.workspace = Path("tests") / (".nesseler-capture-" + uuid.uuid4().hex)
        self.workspace.mkdir()
        self.source = self.workspace / "local-fixture.png"
        self.source.write_bytes(b"fixture image bytes")
        self.addCleanup(shutil.rmtree, self.workspace, ignore_errors=True)

    def test_session_copy_hashes_associates_and_never_replaces(self):
        root = self.workspace / "sessions"
        session = CaptureSession.create(
            root, ladder_id="ladder-A", session_id="session-A", operator_id="operator-1"
        )
        image = session.add_image(
            self.source,
            kind="LABEL",
            evidence_id="evidence-A",
            pose={"x": 1.0, "y": 2.0},
            camera={"name": "inspection"},
        )
        stored = session.path / image["relative_path"]
        self.assertEqual(stored.read_bytes(), self.source.read_bytes())
        self.assertEqual(image["sha256"], hashlib.sha256(self.source.read_bytes()).hexdigest())
        self.assertEqual(image["ladder_id"], "ladder-A")
        self.assertTrue(image["untrusted"])
        self.assertNotIn(str(self.source.resolve()), json.dumps(image))
        with self.assertRaises(FileExistsError):
            CaptureSession.create(
                root, ladder_id="ladder-B", session_id="session-A"
            )
        self.assertEqual(stored.read_bytes(), self.source.read_bytes())
        with self.assertRaises(ValueError):
            session.add_image(self.source, ladder_id="neighbor-ladder")

    def test_derived_crop_must_link_to_existing_evidence_and_duplicates_fail(self):
        session = CaptureSession.create(self.workspace / "sessions", ladder_id="ladder-B")
        with self.assertRaises(ValueError):
            session.add_image(self.source, kind="DERIVED_CROP")
        parent = session.add_image(self.source, evidence_id="overview")
        child = session.add_image(
            self.source,
            kind="DERIVED_CROP",
            evidence_id="label-crop",
            parent_evidence_id=parent["evidence_id"],
        )
        self.assertEqual(child["parent_evidence_id"], "overview")
        with self.assertRaises(FileExistsError):
            session.add_image(self.source, evidence_id="overview")
        reopened = CaptureSession.open(session.path)
        self.assertEqual([item["evidence_id"] for item in reopened.list_evidence()], [
            "label-crop",
            "overview",
        ])


if __name__ == "__main__":
    unittest.main()
