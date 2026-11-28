import unittest

from ultron.site_safety import KIND_CRITERIA, SAFETY_CLASSES, YOLO_CLASS_FOR_KIND
from ultron.perception.models import parse_judge_text


class JudgeParsingTests(unittest.TestCase):
    def test_kind_maps_cover_all_site_safety_criteria(self):
        from ultron.assessment import _criteria
        keys = sorted(k for kinds in KIND_CRITERIA.values() for k in kinds)
        self.assertEqual(keys, sorted(_criteria('site_safety_v1')))
        self.assertEqual(set(YOLO_CLASS_FOR_KIND), set(KIND_CRITERIA))
        self.assertTrue(set(YOLO_CLASS_FOR_KIND.values()) <= set(SAFETY_CLASSES))

    def test_parse_judge_text_maps_states_and_survives_garbage(self):
        text = '```json\n{"station_visible": true, "psa.helm": "NO", "psa.warnweste": "YES", "reason": "bare head"}\n```'
        result = parse_judge_text(text, 'person')
        self.assertTrue(result['station_visible'])
        self.assertEqual(result['states'], {'psa.helm': 'DEFECT', 'psa.warnweste': 'OK'})
        bad = parse_judge_text('not json at all', 'person')
        self.assertFalse(bad['station_visible'])
        self.assertEqual(set(bad['states'].values()), {'UNKNOWN'})


class FakeDetector:
    def __init__(self, boxes): self.boxes = boxes
    def detect(self, image): return self.boxes


class FakeJudge:
    def __init__(self, text): self.text = text
    def judge(self, paths, kind): return self.text


class RuntimeSafetyTests(unittest.TestCase):
    def perception(self):
        from ultron.runtime_perception import RuntimePerception
        return RuntimePerception()  # oracle mode, no models

    def test_oracle_mode_has_no_model_safety(self):
        p = self.perception()
        self.assertFalse(p.model_safety_mode)
        with self.assertRaises(RuntimeError):
            p.safety_judge_station(['x.jpg'], 'person')

    def test_fake_judge_station_states_flow_through(self):
        p = self.perception()
        p.safety_judge = FakeJudge('{"station_visible": true, "psa.helm": "NO", "psa.warnweste": "NO", "reason": "r"}')
        result = p.safety_judge_station(['x.jpg'], 'person')
        self.assertEqual(result['states']['psa.helm'], 'DEFECT')
        self.assertTrue(p.model_safety_mode)


if __name__ == '__main__':
    unittest.main()
