import contextlib
import io
import json
import os
from pathlib import Path
import shutil
import sys
import unittest
from unittest import mock
import uuid

try:
    from PIL import Image
except ImportError:
    Image = None

from ultron.adaptive_draft import (
    _date_role_coverage,
    _pending_targets,
    _unresolved_dates,
    build_adaptive_draft,
)
from ultron.capture_session import CaptureSession
from ultron.checklist import load_form_schema, new_draft, serialize_draft


class SyntheticCatalogTestCase(unittest.TestCase):
    def setUp(self):
        environment = mock.patch.dict(os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        os.environ.pop("ULTRON_FORM_SCHEMA", None)


class AdaptiveDateHistoryTests(SyntheticCatalogTestCase):
    def _draft_with_date_history(self, rows):
        draft = new_draft(session_id="date-session", ladder_id="date-ladder")
        draft["label_date_observations"] = [
            {
                "date_role": "NEXT_INSPECTION",
                "value": value,
                "coverage": coverage,
                "evidence_ids": [evidence_id],
            }
            for value, coverage, evidence_id in rows
        ]
        return draft

    def test_conflicts_remain_gaps_after_later_matching_or_null_readings(self):
        histories = (
            [
                ("2027-01", "OBSERVED", "sticker-a"),
                ("2027-02", "CONFLICTING", "sticker-b"),
                ("2027-01", "OBSERVED", "sticker-c"),
            ],
            [
                ("2027-01", "OBSERVED", "sticker-a"),
                ("2027-02", "OBSERVED", "sticker-b"),
                (None, "UNREADABLE", "sticker-c"),
            ],
        )
        catalog = {
            field["id"]: field
            for field in load_form_schema()["x-form-mapping"]["field_catalog"]
        }
        for rows in histories:
            with self.subTest(rows=rows):
                draft = self._draft_with_date_history(rows)
                self.assertEqual(
                    _date_role_coverage(draft, "NEXT_INSPECTION"), "CONFLICTING"
                )
                unresolved = next(
                    item
                    for item in _unresolved_dates(draft)
                    if item["date_role"] == "NEXT_INSPECTION"
                )
                self.assertEqual(unresolved["coverage"], "CONFLICTING")
                self.assertEqual(
                    unresolved["conflicting_values"], ["2027-01", "2027-02"]
                )
                self.assertEqual(
                    unresolved["evidence_ids"],
                    ["sticker-a", "sticker-b", "sticker-c"],
                )
                self.assertIn(
                    "demo_next_inspection_date",
                    _pending_targets(draft, catalog),
                )

    def test_repeated_matching_value_does_not_create_a_conflict(self):
        draft = self._draft_with_date_history(
            [
                ("2027-01", "OBSERVED", "sticker-a"),
                ("2027-01", "OBSERVED", "sticker-b"),
            ]
        )
        self.assertEqual(_date_role_coverage(draft, "NEXT_INSPECTION"), "OBSERVED")


@unittest.skipUnless(Image is not None, "Pillow is required for local image review")
class AdaptiveDraftTests(SyntheticCatalogTestCase):
    def setUp(self):
        super().setUp()
        self.workspace = Path("tests") / (".adaptive-draft-" + uuid.uuid4().hex)
        self.workspace.mkdir()
        for name, color in (
            ("overview.png", "gray"),
            ("label.png", "white"),
            ("side-a.png", "red"),
            ("sticker.png", "yellow"),
            ("side-b.png", "green"),
            ("side-c.png", "purple"),
            ("unused.png", "blue"),
        ):
            Image.new("RGB", (40, 30), color).save(self.workspace / name)
        self.addCleanup(shutil.rmtree, self.workspace, ignore_errors=True)

    @staticmethod
    def _provenance(method="MANUAL_TRANSCRIPTION", producer="operator"):
        return {"method": method, "producer": producer}

    def _manifest(self, ladder_id="ladder-A"):
        manifest = {
            "schema_version": "1.0.0",
            "views": [
                {
                    "evidence_id": "overview",
                    "kind": "OVERVIEW",
                    "path": "overview.png",
                    "visual_fields": ["demo_overview_note"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [],
                    "date_observations": [],
                    "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
                    "camera": {"name": "inspection"},
                },
                {
                    "evidence_id": "label",
                    "kind": "LABEL",
                    "path": "label.png",
                    "visual_fields": [
                        "demo_label_text",
                        "demo_next_inspection_date",
                    ],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [
                        {
                            "evidence_id": "label-crop",
                            "box": [2, 3, 25, 20],
                            "provenance": self._provenance("MANUAL_ROI", "operator"),
                        }
                    ],
                    "observations": [
                        {
                            "field_id": "demo_label_text",
                            "value": "INV-17",
                            "coverage": "OBSERVED",
                            "reason": "Manual transcription of the displayed label.",
                            "evidence_ids": ["label"],
                            "provenance": self._provenance(),
                        },
                        {
                            "field_id": "demo_label_text",
                            "value": "INV-17",
                            "coverage": "OBSERVED",
                            "reason": "Fixture transcription from the linked crop.",
                            "evidence_ids": ["label-crop"],
                            "provenance": self._provenance("LOCAL_FIXTURE", "fixture-reader"),
                        },
                    ],
                    "date_observations": [
                        {
                            "date_role": "MANUFACTURER_FINAL_CONTROL",
                            "value": "2024-06",
                            "coverage": "OBSERVED",
                            "evidence_ids": ["label-crop"],
                            "provenance": self._provenance(
                                "LOCAL_FIXTURE", "fixture-reader"
                            ),
                        },
                        {
                            "date_role": "RECURRING_INSPECTION",
                            "value": None,
                            "coverage": "UNREADABLE",
                            "evidence_ids": ["label-crop"],
                            "provenance": self._provenance(),
                        },
                    ],
                    "pose": None,
                    "camera": None,
                },
                {
                    "evidence_id": "side-a",
                    "kind": "DETAIL",
                    "path": "side-a.png",
                    "visual_fields": ["demo_overview_note"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [
                        {
                            "field_id": "demo_overview_note",
                            "value": "The example color is unclear.",
                            "coverage": "UNREADABLE",
                            "reason": "The surface detail remains too small to read.",
                            "evidence_ids": ["side-a"],
                            "provenance": self._provenance("MANUAL_REVIEW", "operator"),
                        }
                    ],
                    "date_observations": [],
                    "pose": {"x": 1.2, "y": 2.2, "yaw": 0.3},
                    "camera": {"name": "inspection"},
                },
                {
                    "evidence_id": "sticker",
                    "kind": "INSPECTION_STICKER",
                    "path": "sticker.png",
                    "visual_fields": ["demo_next_inspection_date"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [
                        {
                            "field_id": "demo_next_inspection_date",
                            "value": "example marker present; date not transcribed",
                            "coverage": "OBSERVED",
                            "reason": "Presence only; date semantics are separately unresolved.",
                            "evidence_ids": ["sticker"],
                            "provenance": self._provenance(),
                        }
                    ],
                    "date_observations": [
                        {
                            "date_role": "NEXT_INSPECTION",
                            "value": None,
                            "coverage": "UNREADABLE",
                            "evidence_ids": ["sticker"],
                            "provenance": self._provenance("MANUAL_REVIEW", "operator"),
                        }
                    ],
                    "pose": None,
                    "camera": None,
                },
                {
                    "evidence_id": "side-b",
                    "kind": "DETAIL",
                    "path": "side-b.png",
                    "visual_fields": ["demo_overview_note"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [
                        {
                            "field_id": "demo_overview_note",
                            "value": "The example color appears blue.",
                            "coverage": "OBSERVED",
                            "reason": "Limited visual observation; not a pass/fail decision.",
                            "evidence_ids": ["side-b"],
                            "provenance": self._provenance(),
                        }
                    ],
                    "date_observations": [],
                    "pose": None,
                    "camera": None,
                },
                {
                    "evidence_id": "side-c",
                    "kind": "DETAIL",
                    "path": "side-c.png",
                    "visual_fields": ["demo_overview_note"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [
                        {
                            "field_id": "demo_overview_note",
                            "value": "The example color appears purple.",
                            "coverage": "OBSERVED",
                            "reason": "Second conflicting manual description.",
                            "evidence_ids": ["side-c"],
                            "provenance": self._provenance(),
                        }
                    ],
                    "date_observations": [],
                    "pose": None,
                    "camera": None,
                },
                {
                    "evidence_id": "unused",
                    "kind": "DETAIL",
                    "path": "unused.png",
                    "visual_fields": ["demo_label_text"],
                    "provenance": self._provenance("SIMULATED_CAPTURE", "operator"),
                    "crops": [],
                    "observations": [],
                    "date_observations": [],
                    "pose": None,
                    "camera": None,
                },
            ],
        }
        for view in manifest["views"]:
            view["ladder_id"] = ladder_id
        return manifest

    def test_selects_views_by_gaps_and_keeps_crops_and_unused_sources(self):
        result = build_adaptive_draft(
            self._manifest(),
            sessions_root=self.workspace / "sessions",
            ladder_id="ladder-A",
            operator_id="operator-A",
            session_id="session-A",
            base_dir=self.workspace,
        )
        session = Path(result["session_path"])
        draft = json.loads((session / "draft.json").read_text())
        selection = json.loads((session / "selection.json").read_text())

        selected_ids = [item["evidence_id"] for item in selection["selected_views"]]
        self.assertEqual(selected_ids[:2], ["overview", "label"])
        self.assertIn("side-a", selected_ids)
        self.assertIn("side-b", selected_ids)
        self.assertIn("side-c", selected_ids)
        self.assertIn("sticker", selected_ids)
        self.assertNotIn("unused", selected_ids)
        self.assertIn("unused", selection["unused_original_ids"])
        self.assertEqual(selection["selected_original_view_count"], len(selected_ids))
        self.assertEqual(selection["crop_ids"], ["label-crop"])
        stored_evidence = CaptureSession.open(session).list_evidence()
        self.assertEqual(
            len([item for item in stored_evidence if item["kind"] == "DERIVED_CROP"]), 1
        )
        crop = next(item for item in stored_evidence if item["evidence_id"] == "label-crop")
        self.assertEqual(crop["parent_evidence_id"], "label")
        self.assertEqual(
            crop["provenance"]["source_metadata"]["box_xyxy_original_pixels"],
            [2, 3, 25, 20],
        )
        self.assertTrue((session / "evidence" / "unused.png").is_file())
        self.assertTrue((session / "evidence" / "label.png").is_file())

        color = draft["field_observations"]["demo_overview_note"]
        self.assertEqual(color["coverage"], "CONFLICTING")
        self.assertEqual(
            [p["coverage"] for p in color["proposals"]],
            ["UNREADABLE", "CONFLICTING", "CONFLICTING"],
        )
        color_label = next(
            field["label_de"]
            for field in load_form_schema()["x-form-mapping"]["field_catalog"]
            if field["id"] == "demo_overview_note"
        )
        self.assertIn(
            {"field_id": "demo_overview_note", "label_de": color_label,
             "source_page": 1, "coverage": "CONFLICTING", "human_only": False,
             "reason": "Second conflicting manual description."},
            selection["unresolved_fields"],
        )
        token = draft["field_observations"]["demo_label_text"]["proposals"][1]
        self.assertEqual(token["evidence_ids"], ["label-crop"])
        self.assertEqual(token["provenance"]["method"], "LOCAL_FIXTURE")
        self.assertEqual(
            [item["date_role"] for item in draft["label_date_observations"]],
            [
                "MANUFACTURER_FINAL_CONTROL",
                "RECURRING_INSPECTION",
                "NEXT_INSPECTION",
            ],
        )
        self.assertEqual(draft["label_date_observations"][1]["coverage"], "UNREADABLE")
        self.assertIn(
            {"date_role": "NEXT_INSPECTION", "coverage": "UNREADABLE",
             "latest_value": None, "conflicting_values": [],
             "evidence_ids": ["sticker"]},
            selection["unresolved_dates"],
        )
        self.assertTrue(any(
            item["field_id"] == "demo_physical_review" and item["human_only"]
            for item in selection["unresolved_fields"]
        ))
        self.assertTrue(any(
            item["field_id"] == "demo_last_inspection_date" and item["coverage"] == "NOT_ASSESSED"
            for item in selection["unresolved_fields"]
        ))
        self.assertEqual(draft["human_review"]["status"], "PENDING")
        self.assertEqual(draft["human_review"]["signature"], None)
        self.assertNotIn("verdict", draft)
        self.assertNotIn("safe_to_use", draft)
        self.assertIn("no", (session / "review.html").read_text().lower())
        html = (session / "review.html").read_text()
        self.assertIn("ultron_synthetic_ladder_demo_v1", html)
        self.assertNotIn("Nesseler", html)
        self.assertIn("not an official form", html)
        self.assertIn("not a source-page mapping", html)
        self.assertIn("Retained source, not selected", html)
        self.assertIn("Crops are not independent views", html)
        self.assertIn("Unresolved date roles", html)
        self.assertIn('href="evidence/label-crop.png">label-crop</a>', html)
        self.assertIn('href="evidence/side-b.png">side-b</a>', html)
        self.assertNotIn("<input", html)
        self.assertEqual(serialize_draft(draft), (session / "draft.json").read_text().rstrip())

    def test_manifest_rejects_nonvisual_fields_and_out_of_bounds_crop_before_session(self):
        manifest = self._manifest()
        manifest["views"][1]["visual_fields"].append("demo_physical_review")
        with self.assertRaisesRegex(ValueError, "non-visual or unknown"):
            build_adaptive_draft(
                manifest,
                sessions_root=self.workspace / "bad-sessions",
                ladder_id="ladder-A",
                base_dir=self.workspace,
            )
        self.assertFalse((self.workspace / "bad-sessions").exists())

        manifest = self._manifest()
        manifest["views"][1]["crops"][0]["box"] = [2, 3, 45, 50]
        with self.assertRaisesRegex(ValueError, "exceeds source image dimensions"):
            build_adaptive_draft(
                manifest,
                sessions_root=self.workspace / "bad-crop-sessions",
                ladder_id="ladder-A",
                base_dir=self.workspace,
            )
        self.assertFalse((self.workspace / "bad-crop-sessions").exists())

    def test_explicit_local_catalog_is_used_and_invalid_override_creates_no_session(self):
        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "local_adaptive_fixture"
        path = self.workspace / "local-catalog.json"
        path.write_text(json.dumps(schema), encoding="utf-8")
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(path)}):
            result = build_adaptive_draft(
                self._manifest(), sessions_root=self.workspace / "local-sessions",
                ladder_id="ladder-A", session_id="local-session", base_dir=self.workspace,
            )
            draft = json.loads(Path(result["draft_json"]).read_text())
            self.assertEqual(draft["form_id"], "local_adaptive_fixture")
            serialize_draft(draft)
        with self.assertRaisesRegex(ValueError, "schema or form"):
            serialize_draft(draft)
        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(self.workspace / "missing.json")}):
            with self.assertRaises(ValueError):
                build_adaptive_draft(
                    self._manifest(), sessions_root=self.workspace / "invalid-sessions",
                    ladder_id="ladder-A", base_dir=self.workspace,
                )
        self.assertFalse((self.workspace / "invalid-sessions").exists())

    def test_catalog_switch_during_manifest_validation_is_rejected(self):
        from ultron import adaptive_draft

        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "mutable_local_fixture"
        initial = self.workspace / "initial-catalog.json"
        initial.write_text(json.dumps(schema), encoding="utf-8")
        schema["x-form-mapping"]["field_catalog"][0]["label_de"] = "Changed synthetic label"
        path = self.workspace / "switched-catalog.json"
        path.write_text(json.dumps(schema), encoding="utf-8")
        original = adaptive_draft._validate_manifest

        def switch_catalog(*args, **kwargs):
            result = original(*args, **kwargs)
            os.environ["ULTRON_FORM_SCHEMA"] = str(path)
            return result

        with mock.patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(initial)}):
            with mock.patch.object(adaptive_draft, "_validate_manifest", side_effect=switch_catalog):
                with self.assertRaisesRegex(ValueError, "catalog changed"):
                    build_adaptive_draft(
                        self._manifest(), sessions_root=self.workspace / "switched-sessions",
                        ladder_id="ladder-A", base_dir=self.workspace,
                    )
        self.assertFalse((self.workspace / "switched-sessions").exists())

    def test_manifest_rejects_neighbor_ladder_sources(self):
        manifest = self._manifest()
        manifest["views"][2]["ladder_id"] = "neighbor-ladder"
        with self.assertRaisesRegex(ValueError, "not the requested ladder"):
            build_adaptive_draft(
                manifest,
                sessions_root=self.workspace / "neighbor-sessions",
                ladder_id="ladder-A",
                base_dir=self.workspace,
            )
        self.assertFalse((self.workspace / "neighbor-sessions").exists())

    def test_cli_builds_local_review_without_network_or_hardware(self):
        manifest_path = self.workspace / "manifest.json"
        manifest_path.write_text(
            json.dumps(self._manifest(ladder_id="ladder-CLI")), encoding="utf-8"
        )
        from ultron import __main__ as cli

        argv = [
            "ultron",
            "adaptive-draft",
            str(manifest_path),
            "--sessions-root",
            str(self.workspace / "cli-sessions"),
            "--ladder-id",
            "ladder-CLI",
            "--session-id",
            "session-CLI",
        ]
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout), mock.patch.object(sys, "argv", argv):
            cli.main()
        output = json.loads(stdout.getvalue())
        self.assertEqual(output["ladder_id"], "ladder-CLI")
        self.assertTrue(Path(output["draft_json"]).is_file())
        self.assertTrue(Path(output["review_html"]).is_file())

    def test_teleop_cli_forwards_options_and_uses_teleop_help_without_starting_sim(self):
        from ultron import __main__ as cli
        from ultron import teleop

        help_output = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["ultron", "--help"]),
            contextlib.redirect_stdout(help_output),
            self.assertRaises(SystemExit) as top_help,
        ):
            cli.main()
        self.assertEqual(top_help.exception.code, 0)
        self.assertIn("teleop", help_output.getvalue())

        help_output = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["ultron", "teleop", "--help"]),
            mock.patch.object(
                teleop,
                "KitSimulation",
                side_effect=AssertionError("help must not initialize a simulator"),
            ),
            contextlib.redirect_stdout(help_output),
            self.assertRaises(SystemExit) as teleop_help,
        ):
            cli.main()
        self.assertEqual(teleop_help.exception.code, 0)
        self.assertIn("--kit-root", help_output.getvalue())
        self.assertIn("--output", help_output.getvalue())
        self.assertIn("--host", help_output.getvalue())

        argv = [
            "ultron",
            "teleop",
            "--kit-root",
            "/opt/unitree_mujoco",
            "--output",
            "runs/teleop-session",
            "--host",
            "127.0.0.1",
            "--port",
            "8765",
        ]
        with (
            mock.patch.object(sys, "argv", argv),
            mock.patch.object(teleop, "main", return_value=0) as teleop_main,
        ):
            self.assertEqual(cli.main(), 0)
        teleop_main.assert_called_once_with(argv[2:])


if __name__ == "__main__":
    unittest.main()
