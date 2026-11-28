"""rosbag2 recording profiles (owner: Agent F): topic regexes and output folders under runs/."""
from datetime import datetime, timezone
import os
from pathlib import Path
import re

import yaml


def load_profiles(path):
    data = yaml.safe_load(Path(path).read_text())
    if data.get('schema') != 'ultron.record_profiles/v1':
        raise ValueError(f'{path}: unexpected schema {data.get("schema")!r}')
    return data['profiles']


def profile_patterns(profiles, name):
    """(global, relative) regex lists with `extends` resolved."""
    if name not in profiles:
        raise ValueError(f'unknown record profile {name!r}; known: {sorted(profiles)}')
    p = profiles[name]
    glob, rel = [], []
    if p.get('extends'):
        glob, rel = profile_patterns(profiles, p['extends'])
    return glob + [g for g in p.get('global', []) if g not in glob], rel + [r for r in p.get('relative', []) if r not in rel]


def profile_regex(profiles, name, namespace, use_sim_time=True):
    """One anchored regex for `ros2 bag record -e`."""
    glob, rel = profile_patterns(profiles, name)
    if not use_sim_time:
        glob = [g for g in glob if g != '/clock']
    parts = [re.escape(g) for g in glob]
    if rel:
        parts.append(re.escape(namespace.rstrip('/')) + '/(?:' + '|'.join(rel) + ')')
    return '^(?:' + '|'.join(parts) + ')$'


def _repo_root(start):
    for parent in [start] + list(start.parents):
        if (parent / 'src' / 'ultron' / 'autonomy' / 'sim_bridge.py').is_file():
            return parent
    return None


def runs_dir():
    """$ULTRON_RUNS_DIR, else <repo>/runs (git-ignored) found from cwd or this file, else ./runs."""
    env = os.environ.get('ULTRON_RUNS_DIR')
    if env:
        return Path(env)
    for start in (Path.cwd().resolve(), Path(__file__).resolve()):
        root = _repo_root(start)
        if root:
            return root / 'runs'
    return Path.cwd() / 'runs'


def bag_output(run_mode, profile, now=None):
    stamp = (now or datetime.now(timezone.utc)).strftime('%Y%m%dT%H%M%SZ')
    return runs_dir() / 'bags' / f'{run_mode}_{profile}_{stamp}'
