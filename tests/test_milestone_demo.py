from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ultron.milestone_demo import build_server_command, choose_model, create_milestone_app, select_manifest


class MilestoneBootstrapTests(unittest.TestCase):
    def test_missing_explicit_evidence_cannot_be_replaced_by_synthetic_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'missing-real-evidence.json'
            with self.assertRaises(ValueError):select_manifest(p)
            p.write_text('{}')
            self.assertEqual(select_manifest(p),p)
        self.assertEqual(select_manifest(None).name,'keyframes.json')

    def test_server_is_loopback_only_and_does_not_include_credentials(self):
        command=build_server_command('python3',8766)
        self.assertIn('127.0.0.1',command)
        self.assertNotIn('0.0.0.0',command)
        self.assertEqual(command[-1],'8766')
        self.assertFalse(any('key' in p.lower() for p in command))

    def test_available_model_selection_preserves_explicit_choice(self):
        self.assertEqual(choose_model(['gpt-4.1','gpt-6.1-terra']), 'gpt-6.1-terra')
        self.assertEqual(choose_model(['gpt-4.1']), 'gpt-4.1')
        self.assertEqual(choose_model([],explicit='my-tested-model'),'my-tested-model')
        with self.assertRaises(ValueError):choose_model(['unverified-unknown-model'])

    def test_dashboard_factory_disables_external_channels(self):
        import sys,types
        captured={}
        def app(**kwargs):captured.update(kwargs);return 'app'
        fake_app=types.ModuleType('ultron.dashboard.app');fake_app.create_app=app
        fake_notify=types.ModuleType('ultron.dashboard.notify');fake_notify.load_config=lambda:{'rules':{},'recipients':[]}
        with tempfile.TemporaryDirectory() as tmp,patch.dict(sys.modules,{'ultron.dashboard.app':fake_app,'ultron.dashboard.notify':fake_notify}),patch.dict('os.environ',{'ULTRON_MILESTONE_SERVICE_DIR':tmp,'ULTRON_MILESTONE_PORT':'8766'}):
            self.assertEqual(create_milestone_app(),'app')
        self.assertEqual(captured['channels'],[])
        self.assertIn('127.0.0.1',captured['notify_config']['public_url'])
