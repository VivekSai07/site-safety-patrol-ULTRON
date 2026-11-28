"""foxglove_probe: connect to a foxglove_bridge websocket and report what it serves.

    ros2 run ultron_b2_viz foxglove_probe --url ws://127.0.0.1:8765 --duration 10 --subscribe

Lists advertised channels (topics) and, with ``--subscribe``, measures received bytes per topic.
Also reports advertised services and server capabilities. Read-only: never publishes or calls.
"""
import argparse
import asyncio
import json
from pathlib import Path
import struct
import time

SUBPROTOCOLS = ('foxglove.sdk.v1', 'foxglove.websocket.v1')


async def probe(url, duration, subscribe):
    import websockets
    out = {'url': url, 'server_info': None, 'channels': {}, 'services': [], 'bytes': {}, 'msgs': {}}
    async with websockets.connect(url, subprotocols=list(SUBPROTOCOLS), max_size=None,
                                  open_timeout=10) as ws:
        out['subprotocol'] = ws.subprotocol
        sub_id = 0
        subs = {}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            try:
                frame = await asyncio.wait_for(ws.recv(), timeout=max(0.05, end - time.monotonic()))
            except asyncio.TimeoutError:
                break
            if isinstance(frame, bytes):
                if frame[0] == 1 and len(frame) >= 13:
                    sid = struct.unpack_from('<I', frame, 1)[0]
                    topic = subs.get(sid)
                    if topic:
                        out['bytes'][topic] = out['bytes'].get(topic, 0) + len(frame) - 13
                        out['msgs'][topic] = out['msgs'].get(topic, 0) + 1
                continue
            msg = json.loads(frame)
            op = msg.get('op')
            if op == 'serverInfo':
                out['server_info'] = {k: msg.get(k) for k in ('name', 'capabilities', 'supportedEncodings')}
            elif op == 'advertise':
                new = []
                for ch in msg.get('channels', []):
                    out['channels'][ch['topic']] = {'id': ch['id'], 'schemaName': ch.get('schemaName'),
                                                    'encoding': ch.get('encoding')}
                    if subscribe:
                        sub_id += 1
                        subs[sub_id] = ch['topic']
                        new.append({'id': sub_id, 'channelId': ch['id']})
                if new:
                    await ws.send(json.dumps({'op': 'subscribe', 'subscriptions': new}))
            elif op == 'unadvertise':
                for cid in msg.get('channelIds', []):
                    for t, ch in list(out['channels'].items()):
                        if ch['id'] == cid:
                            out['channels'].pop(t)
            elif op == 'advertiseServices':
                out['services'] += [s.get('name') for s in msg.get('services', [])]
    out['duration_s'] = duration
    out['topics'] = sorted(out['channels'])
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--url', default='ws://127.0.0.1:8765')
    p.add_argument('--duration', type=float, default=8.0)
    p.add_argument('--subscribe', action='store_true')
    p.add_argument('--out', type=Path)
    args = p.parse_args(argv)
    result = asyncio.run(probe(args.url, args.duration, args.subscribe))
    if args.subscribe:
        result['bytes_per_s'] = {t: round(b / args.duration) for t, b in result['bytes'].items()}
        result['total_mbit_s'] = round(sum(result['bytes'].values()) * 8 / args.duration / 1e6, 3)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + '\n')


if __name__ == '__main__':
    main()
