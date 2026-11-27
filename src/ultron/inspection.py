"""Scoped visual prescreening from the team's build bible, not certification."""
REQUIRED = {
    "LEANING_SINGLE": ["stiles", "rungs", "bottom_feet"],
    "A_FRAME_STEPLADDER": ["stiles", "rungs", "front_feet", "rear_feet", "spreader", "hinge_visual"],
    "EXTENSION_LADDER": ["stiles", "rungs", "bottom_feet", "guide_brackets_visual"],
    "PLATFORM_PODEST": ["stiles", "rungs", "front_feet", "rear_feet", "platform_deck", "wheel_brakes_visual"],
    "FOLDED_STORED": ["stiles", "rungs", "usage_state"],
    "UNKNOWN": ["stiles", "rungs", "feet", "type_scope"],
}


def fuse(previous, new):
    result = dict(previous)
    for component, state in new.items():
        old = result.get(component, "UNKNOWN")
        if "FAIL" in (old, state):
            result[component] = "FAIL"
        elif "UNKNOWN_CONFLICT" in (old, state):
            result[component] = "UNKNOWN_CONFLICT"
        elif state == "PASS" or old == "PASS":
            result[component] = "PASS"
        else:
            result[component] = "UNKNOWN"
    return result


def verdict(checks, required, label_status):
    failures = [k for k,v in checks.items() if v == "FAIL"]
    missing = [k for k in required if checks.get(k) != "PASS"]
    if failures or label_status == "OVERDUE":
        status = "FLAGGED"
        reason = "Visible defect: " + ", ".join(failures) if failures else "Inspection label overdue"
    elif missing or label_status != "VALID":
        status = "MANUAL_REVIEW"
        reason = "Unresolved: " + ", ".join(missing + ([] if label_status == "VALID" else ["label"]))
    else:
        status = "AUTOMATED_PASS"
        reason = "No defect detected within scoped visual fields; tactile/functional checks pending"
    return dict(status=status, reason=reason, missing=missing, safe_to_use=False,
                tactile_checks_pending=True, human_signoff_required=True)
