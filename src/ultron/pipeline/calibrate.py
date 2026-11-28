"""Choose the detector's prompt and threshold for each kind on this site, without a person.

An open-vocabulary detector is sensitive to wording: on the hall recording "red fire
extinguisher" scored 0.9 where "fire extinguisher" found nothing. Instead of tuning that by
hand, every candidate prompt of a kind is tried on sample frames and the vision model judges
a sample of the boxes it produces. Each candidate gets the score cut that keeps the most
confirmed boxes over refuted ones, and the candidate expected to deliver the most real boxes wins.
The result is saved as `prompts.json` in the workspace, where the survey picks it up.
"""
import json
from pathlib import Path

from .catalog import DEFAULT_PROMPTS, KINDS

TRIAL_SCORE = 0.15   # candidates are run at this low threshold so weak but real boxes are seen
MIN_THRESHOLD = 0.25
MARGIN = 0.03        # the threshold sits this far below the weakest box it should keep


def _spread(boxes, count):
    """`count` boxes spread evenly over the score range, so the threshold can be placed."""
    ranked = sorted(boxes, key=lambda b: b[1].score)
    if len(ranked) <= count:
        return ranked
    step = (len(ranked) - 1) / (count - 1)
    return [ranked[round(i * step)] for i in range(count)]


def _threshold(judged):
    """The score cut that keeps the most confirmed boxes over refuted ones, or None if none was confirmed.

    Deliberately forgiving: the judge is sometimes wrong about a blurred crop, and every object
    is checked again by the vision model before it counts, so recall matters more here.
    """
    best, best_gain = None, 0
    for cut in sorted({score for score, _ in judged}):
        gain = sum(1 if correct else -1 for score, correct in judged if score >= cut)
        if gain > best_gain:
            best, best_gain = cut, gain
    return None if best is None else round(max(MIN_THRESHOLD, best - MARGIN), 2)


def try_prompt(kind, prompt, frames, detector, verifier, chosen, per_prompt, verdicts):
    """Run one candidate and have a sample of its boxes judged. Returns a result record."""
    trial = {**chosen, kind: {"prompt": prompt, "min_score": TRIAL_SCORE}}
    detector.use(trial)
    boxes = [(frame, d) for frame in frames for d in detector.detect(frame) if d.kind == kind]
    judged = []
    for frame, detection in _spread(boxes, per_prompt):
        key = (frame.name, kind, tuple(round(v / 50) for v in detection.box))  # the same region is judged once
        if key not in verdicts:
            opinion = verifier.classify(frame.image, detection.box, f"{frame.name}-{kind}")
            verdicts[key] = None if opinion is None else opinion[0] == kind
        if verdicts[key] is not None:
            judged.append((detection.score, verdicts[key]))
    threshold = _threshold(judged)
    kept = [d for _, d in boxes if threshold is not None and d.score >= threshold]
    above = [correct for score, correct in judged if threshold is not None and score >= threshold]
    precision = sum(above) / len(above) if above else 0.0
    return {"prompt": prompt, "boxes": len(boxes), "judged": len(judged),
            "confirmed": sum(correct for _, correct in judged), "min_score": threshold,
            "precision": round(precision, 2),
            # Real boxes this prompt is expected to deliver at its threshold: the number that decides.
            "kept": round(len(kept) * precision, 1)}


def calibrate(frames, detector, verifier, kinds=None, per_prompt=6, log=print):
    """Best prompt and threshold per kind for these frames. Kinds that never show up keep the default.

    Returns {kind: {"prompt", "min_score", "tried": [...]}}.
    """
    frames = list(frames)
    chosen = {kind: dict(value) for kind, value in DEFAULT_PROMPTS.items()}
    verdicts, report = {}, {}
    for kind in kinds or list(KINDS):
        tried = [try_prompt(kind, prompt, frames, detector, verifier, chosen, per_prompt, verdicts)
                 for prompt in KINDS[kind].prompts]
        usable = [t for t in tried if t["min_score"] is not None]
        if usable:
            best = max(usable, key=lambda t: (t["kept"], t["confirmed"]))
            chosen[kind] = {"prompt": best["prompt"], "min_score": best["min_score"]}
        report[kind] = {**chosen[kind], "calibrated": bool(usable), "tried": tried}
        log(f"{kind}: {chosen[kind]['prompt']!r} at {chosen[kind]['min_score']}"
            + ("" if usable else " (never confirmed in these frames: default kept)"))
    detector.use(chosen)
    return report


def save(report, workspace):
    path = Path(workspace) / "prompts.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=1))
    return path
