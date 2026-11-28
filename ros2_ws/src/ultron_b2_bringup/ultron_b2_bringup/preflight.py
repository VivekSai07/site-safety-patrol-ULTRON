"""Preflight: refuse to start the stack unless domain, network, namespace and hardware flags match the mode.

Pure checks (no ROS import) so it runs *before* any DDS participant is created. Exit codes:
0 = pass, 2 = refused, 64 = usage error. Agents never create the hardware token.

Hardware token (live + hardware_enable:=true only): a YAML file at ``$ULTRON_HW_AUTHORIZED_FILE``
(default ``/etc/ultron/ULTRON_HW_AUTHORIZED``) written by the authorized operator, containing
``operator``, ``plan`` (reference to the bounded hardware test plan) and ``expires`` (ISO-8601
datetime with timezone). Missing, unreadable, incomplete or expired tokens refuse.
"""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import sys

from .modes import DEFAULT_HW_TOKEN, HW_TOKEN_ENV, MODES

TRUE = ('1', 'true', 'yes', 'on')


def _bool(value):
    return str(value).strip().lower() in TRUE


def check_hw_token(path, now=None):
    """Return (ok, reason) for a hardware authorization token file."""
    import yaml
    p = Path(path)
    if not p.is_file():
        return False, f'hardware token {p} does not exist (only the authorized operator creates it)'
    try:
        data = yaml.safe_load(p.read_text()) or {}
    except Exception as exc:                       # noqa: BLE001 - any parse failure refuses
        return False, f'hardware token {p} unreadable: {exc}'
    if not isinstance(data, dict):
        return False, f'hardware token {p} is not a mapping'
    missing = [k for k in ('operator', 'plan', 'expires') if not str(data.get(k) or '').strip()]
    if missing:
        return False, f'hardware token {p} missing {missing}'
    try:
        expires = data['expires'] if isinstance(data['expires'], datetime) else datetime.fromisoformat(str(data['expires']))
    except ValueError:
        return False, f'hardware token {p} has invalid expires {data["expires"]!r}'
    if expires.tzinfo is None:
        return False, 'hardware token expires must include a timezone'
    if (now or datetime.now(timezone.utc)) >= expires:
        return False, f'hardware token expired at {expires.isoformat()}'
    return True, f'hardware token by {data["operator"]} for plan {data["plan"]}'


def check(mode, env, namespace, *, hardware_enable=False, odom_source=None, launch_control=None,
          hw_token=None, now=None):
    """Return a list of refusal reasons (empty list = pass)."""
    errors = []
    if mode not in MODES:
        return [f'unknown mode {mode!r}; expected one of {sorted(MODES)}']
    spec = MODES[mode]

    domain = env.get('ROS_DOMAIN_ID')
    if domain is None or domain.strip() == '':
        errors.append(f'ROS_DOMAIN_ID unset; mode {mode} requires {spec["domain_id"]}')
    elif domain.strip() != str(spec['domain_id']):
        errors.append(f'ROS_DOMAIN_ID={domain} but mode {mode} requires {spec["domain_id"]}')

    localhost = _bool(env.get('ROS_LOCALHOST_ONLY', '0'))
    if spec['localhost_only'] and not localhost:
        errors.append(f'mode {mode} requires ROS_LOCALHOST_ONLY=1')
    if not spec['localhost_only'] and localhost:
        errors.append(f'mode {mode} needs the robot network; ROS_LOCALHOST_ONLY=1 would hide the robot')

    dds = env.get('CYCLONEDDS_URI', '')
    if spec['localhost_only'] and 'robot' in dds:
        errors.append(f'CYCLONEDDS_URI={dds} points at a robot-network profile in mode {mode}')

    if namespace != spec['namespace']:
        errors.append(f'namespace {namespace!r} does not match mode {mode} ({spec["namespace"]})')

    if odom_source is not None and odom_source not in spec['odom_sources']:
        errors.append(f'odom_source {odom_source!r} not available in mode {mode} (allowed {spec["odom_sources"]})')

    if launch_control and not spec['control']:
        errors.append(f'mode {mode} must not launch control/command nodes')

    if hardware_enable:
        if not spec['hardware_allowed']:
            errors.append(f'hardware_enable:=true is not allowed in mode {mode}')
        else:
            token = hw_token or env.get(HW_TOKEN_ENV) or DEFAULT_HW_TOKEN
            ok, reason = check_hw_token(token, now=now)
            if not ok:
                errors.append(f'live hardware_enable refused: {reason}')
    return errors


def _strip_ros_args(argv):
    if '--ros-args' in argv:
        i = argv.index('--ros-args')
        end = argv.index('--', i) if '--' in argv[i:] else len(argv)
        return argv[:i] + argv[end + 1:]
    return argv


def main(argv=None):
    parser = argparse.ArgumentParser(prog='preflight', description=__doc__.splitlines()[0])
    parser.add_argument('--mode', required=True, choices=sorted(MODES))
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--hardware-enable', default='false')
    parser.add_argument('--odom-source', default=None)
    parser.add_argument('--launch-control', default=None)
    parser.add_argument('--hw-token', default=None)
    try:
        args = parser.parse_args(_strip_ros_args(list(sys.argv[1:] if argv is None else argv)))
    except SystemExit as exc:
        return 64 if exc.code else 0
    errors = check(args.mode, os.environ, args.namespace, hardware_enable=_bool(args.hardware_enable),
                   odom_source=args.odom_source,
                   launch_control=None if args.launch_control is None else _bool(args.launch_control),
                   hw_token=args.hw_token)
    if errors:
        for e in errors:
            print(f'[preflight] REFUSED: {e}', file=sys.stderr, flush=True)
        return 2
    print(f'[preflight] PASS mode={args.mode} namespace={args.namespace} '
          f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")} hardware_enable={_bool(args.hardware_enable)}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
