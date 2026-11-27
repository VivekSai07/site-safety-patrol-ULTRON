"""Conservative demonstration policy; replace checks with the official protocol."""
from datetime import date
import math

CHECKS = ("rails", "rungs", "feet", "spreaders")
MIN_CONFIDENCE = 0.85


def assess(observation, inspection_date):
    """A fail always wins; unknown, incomplete or weak evidence requires review.

    A candidate is explicitly NOT a declaration that the ladder is safe.
    """
    reasons = []
    checks = observation.get("checks", {})
    failed = [name for name, value in checks.items() if value == "fail"]
    if failed:
        reasons.append("Observed defect: " + ", ".join(sorted(failed)))
    for name in CHECKS:
        if checks.get(name) != "pass":
            reasons.append(f"{name}: {checks.get(name, 'missing')}")
    if any(value not in ("pass", "fail", "unknown") for value in checks.values()):
        reasons.append("Unrecognized check result")
    confidence = observation.get("confidence")
    if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not math.isfinite(confidence) or not MIN_CONFIDENCE <= confidence <= 1):
        reasons.append("Missing, invalid or low confidence")
    if not observation.get("evidence"):
        reasons.append("Missing evidence")
    label = observation.get("label", {})
    if not label.get("readable") or not label.get("asset_id"):
        reasons.append("Label unreadable or asset identity missing")
    try:
        due = date.fromisoformat(label.get("valid_until", ""))
        if due < inspection_date:
            reasons.append("Inspection label expired")
    except (ValueError, TypeError):
        reasons.append("Inspection date unavailable or invalid")
    status = "unsafe" if failed else "needs_review" if reasons else "candidate_for_review"
    return {"status": status, "reasons": reasons,
            "human_signoff_required": True, "safe": False}
