import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('prepare_photoreal_dataset',Path(__file__).parents[1]/'scripts/prepare_photoreal_dataset.py')
module=importlib.util.module_from_spec(spec)


class PhotorealDatasetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):spec.loader.exec_module(module)

    def test_completed_pngs_are_excluded_from_paid_jobs_and_invalid_files_stop_resume(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'images').mkdir()
            image=root/'images'/'one.png';Image.new('RGB',(20,20),'red').save(image)
            rows=[{'image_id':'one','asset_group':'A','split':'train','prompt':'one','filename':'one.png'},
                  {'image_id':'two','asset_group':'B','split':'val','prompt':'two','filename':'two.png'}]
            report=module.refresh(root,rows)
            self.assertEqual(report['images_present'],1)
            pending=[json.loads(line) for line in (root/'jobs.pending.jsonl').read_text().splitlines()]
            self.assertEqual([r['out'] for r in pending],['two.png'])
            image.write_bytes(b'broken PNG')
            with self.assertRaises(ValueError):module.refresh(root,rows)

    def test_split_leakage_and_duplicate_image_ids_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows=[{'image_id':'one','asset_group':'A','split':'train','prompt':'one','filename':'one.png'},
                  {'image_id':'two','asset_group':'A','split':'val','prompt':'two','filename':'two.png'}]
            with self.assertRaises(ValueError):module.refresh(Path(tmp),rows)
            rows[1].update(image_id='one',asset_group='B')
            with self.assertRaises(ValueError):module.refresh(Path(tmp),rows)
