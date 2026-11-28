"""Ladder detector backends.

* ``oracle_sim``: simulation ground truth, labelled ORACLE. Refused outside ``run_mode=sim``.
* ``yolo``: Ultralytics YOLO weights supplied by the perception owner (AGPL-3.0, licence review).
* ``owlv2``: open-vocabulary OWLv2 from a local model directory (no network download).

Weights are never in the repository. A backend whose weights or libraries are missing raises
``BackendUnavailable``; the node then reports UNAVAILABLE and publishes nothing. There is no
fallback to the oracle in any mode.
"""
import os

DEFAULT_OWL_PROMPTS = ('ladder', 'step ladder', 'A-frame ladder')


class BackendUnavailable(RuntimeError):
    pass


class OracleBackend:
    name = 'oracle_sim'
    is_oracle = True


class YoloBackend:
    name = 'yolo'
    is_oracle = False

    def __init__(self, weights, threshold, class_keywords=('ladder',)):
        if not weights or not os.path.isfile(weights):
            raise BackendUnavailable(f'yolo weights not found: {weights!r} (owner: perception, Om)')
        try:
            from ultralytics import YOLO  # noqa: PLC0415 (optional, heavy)
        except Exception as exc:  # pragma: no cover - environment dependent
            raise BackendUnavailable(f'ultralytics not importable: {exc}') from exc
        self.model = YOLO(weights)
        self.threshold = float(threshold)
        self.keywords = tuple(k.lower() for k in class_keywords)

    def detect(self, rgb):  # pragma: no cover - needs weights
        out = []
        for result in self.model.predict(rgb, verbose=False, conf=self.threshold):
            names = result.names
            for box, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                                      result.boxes.conf.tolist()):
                label = str(names.get(int(cls), cls))
                if any(k in label.lower() for k in self.keywords):
                    out.append({'label': label, 'confidence': float(conf), 'bbox': tuple(box)})
        return out


class Owlv2Backend:
    name = 'owlv2'
    is_oracle = False

    def __init__(self, model_dir, threshold, prompts=DEFAULT_OWL_PROMPTS):
        if not model_dir or not os.path.isdir(model_dir):
            raise BackendUnavailable(f'owlv2 model directory not found: {model_dir!r} (owner: perception, Om)')
        try:
            import torch  # noqa: PLC0415
            from transformers import Owlv2ForObjectDetection, Owlv2Processor  # noqa: PLC0415
        except Exception as exc:  # pragma: no cover
            raise BackendUnavailable(f'transformers/torch not importable: {exc}') from exc
        self.torch = torch
        self.processor = Owlv2Processor.from_pretrained(model_dir, local_files_only=True)
        self.model = Owlv2ForObjectDetection.from_pretrained(model_dir, local_files_only=True).eval()
        self.threshold = float(threshold)
        self.prompts = list(prompts)

    def detect(self, rgb):  # pragma: no cover - needs weights
        inputs = self.processor(text=[self.prompts], images=rgb, return_tensors='pt')
        with self.torch.no_grad():
            outputs = self.model(**inputs)
        size = max(rgb.shape[:2])  # OWLv2 pads to a square
        results = self.processor.post_process_object_detection(
            outputs, threshold=self.threshold, target_sizes=self.torch.tensor([[size, size]]))[0]
        return [{'label': self.prompts[int(lbl)], 'confidence': float(score), 'bbox': tuple(box.tolist())}
                for box, score, lbl in zip(results['boxes'], results['scores'], results['labels'])]


def make_backend(name, run_mode, *, weights='', model_dir='', threshold=0.25, prompts=DEFAULT_OWL_PROMPTS):
    if name == 'oracle_sim':
        if run_mode != 'sim':
            raise BackendUnavailable(f'oracle_sim is simulation ground truth; refused in run_mode={run_mode!r}')
        return OracleBackend()
    if name == 'yolo':
        return YoloBackend(weights, threshold)
    if name == 'owlv2':
        return Owlv2Backend(model_dir, threshold, prompts)
    raise BackendUnavailable(f'unknown backend {name!r}')
