import base64
from io import BytesIO
import hashlib
import json
import os
from pathlib import Path
import shutil
import unittest
import uuid
from urllib.error import HTTPError
from unittest.mock import patch

from ultron.capture_session import CaptureSession
from ultron.checklist import load_form_schema, new_draft
from ultron.perception.openai_adapter import (
    AdapterLimits,
    OpenAIAdapter,
    TransportResponse,
    _NoRedirectHandler,
    _http_transport,
)


class OpenAIAdapterTests(unittest.TestCase):
    def setUp(self):
        environment = patch.dict(os.environ)
        environment.start()
        self.addCleanup(environment.stop)
        os.environ.pop("ULTRON_FORM_SCHEMA", None)
        self.workspace = Path("tests") / (".openai-adapter-" + uuid.uuid4().hex)
        self.workspace.mkdir()
        self.source_label = self.workspace / "label.png"
        self.source_detail = self.workspace / "detail.png"
        self.source_label.write_bytes(b"fixture-label-image")
        self.source_detail.write_bytes(b"fixture-detail-image")
        self.session = CaptureSession.create(
            self.workspace / "sessions",
            ladder_id="ladder-1",
            session_id="session-1",
        )
        self.label = self.session.add_image(
            self.source_label, kind="LABEL", evidence_id="label-1"
        )
        self.detail = self.session.add_image(
            self.source_detail, kind="DETAIL", evidence_id="detail-1"
        )
        self.draft = new_draft(session_id="session-1", ladder_id="ladder-1")
        self.addCleanup(shutil.rmtree, self.workspace, ignore_errors=True)

    def _fixture(self, *, evidence_id="label-1", field_id="demo_label_text"):
        return {
            "proposals": [
                {
                    "field_id": field_id,
                    "value": "INV-17",
                    "coverage": "OBSERVED",
                    "reason": "Identifier characters are visible.",
                    "evidence_ids": [evidence_id],
                }
            ]
        }

    def _manifest(self, *records):
        return {
            "schema_version": "1.0.0",
            "session_id": self.session.metadata["session_id"],
            "rights_privacy_approved": True,
            "approval_reference": "privacy-review-42",
            "images": [
                {
                    "evidence_id": record["evidence_id"],
                    "sha256": record["sha256"],
                    "approved": True,
                }
                for record in records
            ],
        }

    def test_default_is_local_fixture_and_never_uses_transport(self):
        def must_not_send(*_args):
            raise AssertionError("default mode attempted networking")

        result = OpenAIAdapter(transport=must_not_send).propose(
            self.session,
            self.draft,
            selected_evidence_ids=["label-1"],
        )
        self.assertEqual(result.status, "INCOMPLETE")
        self.assertEqual(result.errors, ())
        self.assertEqual(result.provenance["mode"], "LOCAL_FIXTURE")
        self.assertEqual(result.draft, self.draft)

    def test_local_fixture_applies_only_evidence_linked_untrusted_proposals(self):
        result = OpenAIAdapter().propose(
            self.session,
            self.draft,
            selected_evidence_ids=["label-1"],
            fixture_response=self._fixture(),
        )
        self.assertEqual(result.status, "PROPOSALS_READY")
        proposal = result.draft["field_observations"]["demo_label_text"]["proposals"][0]
        self.assertEqual(proposal["value"], "INV-17")
        self.assertEqual(proposal["evidence_ids"], ["label-1"])
        self.assertTrue(proposal["untrusted"])
        self.assertEqual(proposal["provenance"]["method"], "LOCAL_FIXTURE_PROPOSAL")
        self.assertEqual(proposal["provenance"]["prompt_version"], "catalog-proposal-v2")
        self.assertEqual(
            self.draft["field_observations"]["demo_label_text"]["proposals"], []
        )

    def test_invalid_json_and_refusal_preserve_the_incomplete_draft(self):
        adapter = OpenAIAdapter()
        for response, code in (("{", "INVALID_JSON"), ({"refusal": "not permitted"}, "REFUSAL")):
            with self.subTest(code=code):
                result = adapter.propose(
                    self.session,
                    self.draft,
                    selected_evidence_ids=["label-1"],
                    fixture_response=response,
                )
                self.assertEqual(result.status, "INCOMPLETE")
                self.assertEqual(result.draft, self.draft)
                self.assertEqual(result.errors[0].code, code)
                self.assertNotIn("not permitted", repr(result))

    def test_unknown_fields_forged_decisions_and_unselected_evidence_are_rejected(self):
        adapter = OpenAIAdapter()
        invalid_responses = (
            {"proposals": [], "verdict": "PASS"},
            self._fixture(field_id="not_a_form_field"),
            self._fixture(field_id="demo_physical_review"),
            self._fixture(evidence_id="detail-1"),
            {"proposals": [dict(self._fixture()["proposals"][0], value=None)]},
            {
                "proposals": [
                    dict(
                        self._fixture()["proposals"][0],
                        value={"safe_to_use": True},
                    )
                ]
            },
            {
                "proposals": [
                    dict(self._fixture()["proposals"][0], signoff="approved")
                ]
            },
        )
        for response in invalid_responses:
            with self.subTest(response=response):
                result = adapter.propose(
                    self.session,
                    self.draft,
                    selected_evidence_ids=["label-1"],
                    fixture_response=response,
                )
                self.assertEqual(result.status, "INCOMPLETE")
                self.assertEqual(result.draft, self.draft)
                self.assertEqual(result.errors[0].code, "INVALID_RESPONSE")

    def test_live_request_is_scoped_to_approved_selected_images_and_bounded(self):
        captured = {}
        api_response = {
            "id": "resp-fixture",
            "output": [
                {
                    "type": "message",
                    "content": [
                        {
                            "type": "output_text",
                            "text": json.dumps(self._fixture()),
                        }
                    ],
                }
            ],
        }

        def fake_transport(url, headers, body, timeout):
            captured.update(
                url=url,
                headers=dict(headers),
                body=json.loads(body),
                timeout=timeout,
            )
            return TransportResponse(
                200, json.dumps(api_response).encode("utf-8")
            )

        limits = AdapterLimits(
            max_images=2,
            max_image_bytes=1024,
            max_total_image_bytes=1024,
            max_retries=0,
            timeout_seconds=4.0,
        )
        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            result = OpenAIAdapter(
                live_opt_in=True,
                limits=limits,
                transport=fake_transport,
                sleeper=lambda _delay: None,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label, self.detail),
            )

        self.assertEqual(result.status, "PROPOSALS_READY")
        self.assertEqual(captured["url"], "https://api.openai.com/v1/responses")
        self.assertEqual(captured["headers"]["Authorization"], "Bearer test-secret")
        self.assertEqual(captured["timeout"], 4.0)
        content = captured["body"]["input"][0]["content"]
        self.assertIn("demo_label_text", content[0]["text"])
        self.assertNotIn("demo_physical_review", content[0]["text"])
        self.assertNotIn("Nesseler", content[0]["text"])
        self.assertIn("not an official form", content[0]["text"])
        self.assertIn("RECURRING_INSPECTION", content[0]["text"])
        self.assertIn("NEXT_INSPECTION", content[0]["text"])
        self.assertEqual(
            sum(part["type"] == "input_image" for part in content), 1
        )
        transmitted = next(part for part in content if part["type"] == "input_image")
        expected = base64.b64encode(b"fixture-label-image").decode("ascii")
        self.assertTrue(transmitted["image_url"].endswith(expected))
        self.assertNotIn("detail-1", json.dumps(content))
        self.assertEqual(
            result.provenance["selected_evidence_ids"], ["label-1"]
        )
        self.assertEqual(
            result.provenance["response_sha256"],
            hashlib.sha256(json.dumps(api_response).encode("utf-8")).hexdigest(),
        )
        self.assertNotIn("test-secret", repr(result))

    def test_live_request_requires_approval_and_never_sends_unapproved_images(self):
        sends = []

        def fake_transport(*args):
            sends.append(args)
            return TransportResponse(200, b'{"output":[]}')

        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            result = OpenAIAdapter(
                live_opt_in=True,
                transport=fake_transport,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=None,
            )
        self.assertEqual(result.errors[0].code, "APPROVAL_OR_IMAGE_INVALID")
        self.assertEqual(result.draft, self.draft)
        self.assertEqual(sends, [])

    def test_selected_local_catalog_is_fixture_only_and_cannot_mix_drafts(self):
        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "local_adapter_fixture"
        path = self.workspace / "local-catalog.json"
        path.write_text(json.dumps(schema), encoding="utf-8")
        sends = []
        transport = lambda *args: sends.append(args)
        with patch.dict(
            os.environ,
            {"ULTRON_FORM_SCHEMA": str(path), "OPENAI_API_KEY": "test-secret"},
            clear=False,
        ):
            local = new_draft(session_id="session-1", ladder_id="ladder-1")
            fixture = OpenAIAdapter().propose(
                self.session, local, selected_evidence_ids=["label-1"],
                fixture_response=self._fixture(),
            )
            self.assertEqual(fixture.status, "PROPOSALS_READY")
            live = OpenAIAdapter(live_opt_in=True, transport=transport).propose(
                self.session, local, selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
            self.assertEqual(live.errors[0].code, "CATALOG_LOCAL_ONLY")
            cross_form = OpenAIAdapter(live_opt_in=True, transport=transport).propose(
                self.session, self.draft, selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
            self.assertEqual(cross_form.errors[0].code, "INVALID_INPUT")
        cross_form = OpenAIAdapter().propose(
            self.session, local, selected_evidence_ids=["label-1"],
            fixture_response=self._fixture(),
        )
        self.assertEqual(cross_form.errors[0].code, "INVALID_INPUT")
        self.assertEqual(sends, [])

    def test_catalog_change_during_fixture_response_does_not_apply_proposals(self):
        from ultron.perception import openai_adapter

        schema = load_form_schema()
        schema["properties"]["form_id"]["const"] = "mutable_local_fixture"
        initial = self.workspace / "initial-catalog.json"
        initial.write_text(json.dumps(schema), encoding="utf-8")
        schema["x-form-mapping"]["field_catalog"][0]["label_de"] = "Changed synthetic label"
        path = self.workspace / "changed-catalog.json"
        path.write_text(json.dumps(schema), encoding="utf-8")
        original = openai_adapter._json_response_bytes

        def switch_catalog(response):
            payload = original(response)
            os.environ["ULTRON_FORM_SCHEMA"] = str(path)
            return payload

        with patch.dict(os.environ, {"ULTRON_FORM_SCHEMA": str(initial)}):
            draft = new_draft(session_id="session-1", ladder_id="ladder-1")
            with patch.object(openai_adapter, "_json_response_bytes", side_effect=switch_catalog):
                result = OpenAIAdapter().propose(
                    self.session, draft, selected_evidence_ids=["label-1"],
                    fixture_response=self._fixture(),
                )
        self.assertEqual(result.status, "INCOMPLETE")
        self.assertEqual(result.errors[0].code, "INVALID_RESPONSE")
        self.assertEqual(result.draft, draft)

    def test_timeout_retries_are_bounded_and_secrets_are_not_returned(self):
        calls = []

        def timeout_transport(*_args):
            calls.append(True)
            raise TimeoutError("test-secret should not be exposed")

        limits = AdapterLimits(max_retries=1, timeout_seconds=2.0)
        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            result = OpenAIAdapter(
                live_opt_in=True,
                limits=limits,
                transport=timeout_transport,
                sleeper=lambda _delay: None,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
        self.assertEqual(len(calls), 2)
        self.assertEqual(result.errors[0].code, "TIMEOUT")
        self.assertEqual(result.errors[0].attempts, 2)
        self.assertEqual(result.draft, self.draft)
        self.assertNotIn("test-secret", repr(result))
        self.assertNotIn("should not be exposed", repr(result))

    def test_live_request_enforces_image_count_before_transport(self):
        sends = []
        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            result = OpenAIAdapter(
                live_opt_in=True,
                limits=AdapterLimits(max_images=1),
                transport=lambda *args: sends.append(args),
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1", "detail-1"],
                approved_image_manifest=self._manifest(self.label, self.detail),
            )
        self.assertEqual(result.errors[0].code, "APPROVAL_OR_IMAGE_INVALID")
        self.assertEqual(result.draft, self.draft)
        self.assertEqual(sends, [])

    def test_live_request_enforces_byte_limit_and_environment_credential(self):
        sends = []
        transport = lambda *args: sends.append(args)
        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            oversized = OpenAIAdapter(
                live_opt_in=True,
                limits=AdapterLimits(max_image_bytes=4),
                transport=transport,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
        self.assertEqual(oversized.errors[0].code, "APPROVAL_OR_IMAGE_INVALID")
        self.assertEqual(sends, [])

        with patch.dict("os.environ", {}, clear=True):
            missing_credential = OpenAIAdapter(
                live_opt_in=True,
                transport=transport,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
        self.assertEqual(missing_credential.errors[0].code, "MISSING_CREDENTIAL")
        self.assertEqual(sends, [])

    def test_http_transport_disables_redirects_without_networking(self):
        state = {}

        class FakeOpener:
            def __init__(self, handlers):
                state["handlers"] = handlers

            def open(self, request, timeout):
                state["request"] = request
                state["timeout"] = timeout
                redirect_handler = next(
                    handler
                    for handler in state["handlers"]
                    if isinstance(handler, _NoRedirectHandler)
                )
                state["redirect_request"] = redirect_handler.redirect_request(
                    request,
                    BytesIO(),
                    302,
                    "Found",
                    {"Location": "https://unapproved.invalid/redirect"},
                    "https://unapproved.invalid/redirect",
                )
                raise HTTPError(
                    request.full_url,
                    302,
                    "Found",
                    {},
                    BytesIO(b"redirect not followed"),
                )

        with patch(
            "ultron.perception.openai_adapter.build_opener",
            side_effect=lambda *handlers: FakeOpener(handlers),
        ) as build:
            response = _http_transport(
                "https://api.openai.com/v1/responses",
                {"Authorization": "Bearer test-secret"},
                b"selected image payload",
                3.0,
            )

        build.assert_called_once()
        self.assertEqual(len(state["handlers"]), 1)
        self.assertIsInstance(state["handlers"][0], _NoRedirectHandler)
        self.assertIsNone(state["redirect_request"])
        self.assertEqual(state["timeout"], 3.0)
        self.assertEqual(response.status_code, 302)
        self.assertEqual(response.body, b"redirect not followed")

    def test_redirect_from_injected_transport_is_rejected_without_retry(self):
        calls = []

        def redirect_transport(*_args):
            calls.append(True)
            return TransportResponse(307, b"")

        with patch.dict("os.environ", {"OPENAI_API_KEY": "test-secret"}, clear=False):
            result = OpenAIAdapter(
                live_opt_in=True,
                limits=AdapterLimits(max_retries=2),
                transport=redirect_transport,
                sleeper=lambda _delay: None,
            ).propose(
                self.session,
                self.draft,
                selected_evidence_ids=["label-1"],
                approved_image_manifest=self._manifest(self.label),
            )
        self.assertEqual(calls, [True])
        self.assertEqual(result.errors[0].code, "REDIRECT_REJECTED")
        self.assertEqual(result.draft, self.draft)


if __name__ == "__main__":
    unittest.main()
