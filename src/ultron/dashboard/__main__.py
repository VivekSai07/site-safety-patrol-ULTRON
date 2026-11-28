"""Start the dashboard: python -m ultron.dashboard [--lan] [--notify config/notify.json]

Also: python -m ultron.dashboard fetch-form   (download nesseler's ladder form for the PDF export)
"""
import argparse
import sys
import urllib.request


def fetch_form():
    from .protocol_pdf import FORM_URL, form_path
    target = form_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(FORM_URL, timeout=30) as response:
        data = response.read()
    if not data.startswith(b'%PDF'):
        raise SystemExit('download did not return a PDF')
    target.write_bytes(data)
    print(f'saved {len(data)} bytes to {target}')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['fetch-form']:
        return fetch_form()
    parser = argparse.ArgumentParser(description='ULTRON ladder patrol dashboard')
    parser.add_argument('--db', default='runs/dashboard.sqlite', help='SQLite file (created if missing)')
    parser.add_argument('--media', default='runs/dashboard-media', help='folder for evidence photos')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--lan', action='store_true',
                        help='listen on all interfaces without authentication; unsafe on shared networks')
    parser.add_argument('--goals', action='store_true',
                        help='enable unauthenticated goals; a connected ROS bridge may reach hardware')
    parser.add_argument('--notify', help='notification config JSON (see config/notify.example.json)')
    args = parser.parse_args(argv)

    import uvicorn
    from .app import create_app
    from .notify import load_config

    config = load_config(args.notify)
    if args.notify is None:
        config['public_url'] = f'http://127.0.0.1:{args.port}'
    uvicorn.run(create_app(args.db, args.media, config, allow_goals=args.goals), host='0.0.0.0' if args.lan else '127.0.0.1', port=args.port)


if __name__ == '__main__':
    main()
