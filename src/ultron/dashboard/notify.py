"""Live notifications to site safety officers.

Flow: a verdict or robot event arrives -> rules pick the roles to tell -> every channel
delivers to each recipient -> the alert stays open until someone acknowledges it ->
an unacknowledged critical alert escalates once to the escalation roles.

Channels:
- dashboard: always on. The alert appears on every open dashboard with an Acknowledge button.
- ntfy: phone push via an ntfy server (https://ntfy.sh or self-hosted). The push carries an
  Acknowledge button that calls back to this server, so the phone must reach `public_url`.
"""
import json
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_CONFIG = {
    'public_url': 'http://127.0.0.1:8000',
    'escalate_after_s': 300,
    'ntfy': {'server': None},
    'recipients': [
        {'role': 'site_safety_officer', 'title': 'Site safety officer', 'name': '[NAME]', 'where': 'on site', 'ntfy_topic': None},
        {'role': 'safety_engineer', 'title': 'Safety engineer', 'name': '[NAME]', 'where': 'off site', 'ntfy_topic': None},
        {'role': 'site_manager', 'title': 'Site manager', 'name': '[NAME]', 'where': 'on site', 'ntfy_topic': None},
        {'role': 'robot_operator', 'title': 'Robot operator', 'name': '[NAME]', 'where': 'on site', 'ntfy_topic': None},
    ],
    'rules': {
        'FLAGGED': ['site_safety_officer', 'safety_engineer'],
        'MANUAL_REVIEW': ['site_safety_officer'],
        'ROBOT_STOPPED': ['robot_operator'],
        'MISSION_SUMMARY': ['safety_engineer'],
        'ESCALATION': ['site_manager'],
    },
}

ROBOT_STOP_EVENTS = {'OPERATOR_INTERVENTION', 'ESTOP', 'EMERGENCY_STOP', 'LINK_LOST'}


def load_config(path=None):
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    if path:
        config.update(json.loads(Path(path).read_text(encoding='utf-8')))
    return config


class NtfyChannel:
    name = 'ntfy'

    def __init__(self, server, public_url, timeout_s=5):
        self.server, self.public_url, self.timeout_s = server.rstrip('/'), public_url.rstrip('/'), timeout_s

    def send(self, alert, recipient):
        topic = recipient.get('ntfy_topic')
        if not topic:
            return 'skipped', 'no ntfy topic configured'
        ack = f"{self.public_url}/api/alerts/{alert['id']}/ack?by={recipient['role']}"
        headers = {
            'Title': ascii_only(alert['title']),
            'Priority': 'urgent' if alert['severity'] == 'critical' else 'high' if alert['severity'] == 'warning' else 'default',
            'Tags': 'warning' if alert['severity'] != 'info' else 'clipboard',
            'Click': f"{self.public_url}/",
        }
        if alert['severity'] != 'info':
            headers['Actions'] = f'http, Acknowledge, {ack}, method=POST, clear=true'
        body = f"{alert['message']}\n{alert.get('action', '')}".strip().encode('utf-8')
        request = urllib.request.Request(f'{self.server}/{topic}', data=body, headers=headers, method='POST')
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return ('sent', '') if 200 <= response.status < 300 else ('failed', f'HTTP {response.status}')
        except Exception as error:  # delivery failure is recorded and shown, never hidden
            return 'failed', str(error)[:200]


def ascii_only(text):
    table = str.maketrans({'ä': 'ae', 'ö': 'oe', 'ü': 'ue', 'Ä': 'Ae', 'Ö': 'Oe', 'Ü': 'Ue', 'ß': 'ss', '–': '-', '·': '-'})
    return text.translate(table).encode('ascii', 'replace').decode('ascii')


class Notifier:
    def __init__(self, store, bus, config=None, channels=None, clock=None):
        self.store, self.bus = store, bus
        self.config = config or load_config()
        if channels is None:
            server = self.config.get('ntfy', {}).get('server')
            channels = [NtfyChannel(server, self.config['public_url'])] if server else []
        self.channels = channels
        self.clock = clock or (lambda: datetime.now(timezone.utc))
        self.recipients = {r['role']: r for r in self.config['recipients']}

    def roles_for(self, rule):
        return [self.recipients[r] for r in self.config['rules'].get(rule, []) if r in self.recipients]

    # Triggers
    def on_inspection(self, record):
        status = record['status']
        if status not in ('FLAGGED', 'MANUAL_REVIEW'):
            return None
        asset = record['asset_id']
        if status == 'FLAGGED':
            title = f'FLAGGED: {asset} take out of use'
            action = f'Take {asset} out of use now and check it in person. The robot cannot release a ladder.'
            severity = 'critical'
        else:
            title = f'MANUAL REVIEW: {asset} check in person'
            action = f'Check {asset} in person: the robot could not see everything it needs.'
            severity = 'warning'
        return self.raise_alert(rule=status, kind='LADDER', severity=severity, mission_id=record['mission_id'],
                                asset_id=asset, title=title, message=record['reason'], action=action)

    def on_event(self, event):
        if event['type'] not in ROBOT_STOP_EVENTS:
            return None
        return self.raise_alert(rule='ROBOT_STOPPED', kind='ROBOT', severity='critical', mission_id=event['mission_id'],
                                asset_id=None, title=f"Robot: {event['type'].replace('_', ' ').lower()}",
                                message=f"Mission time {event['time']:.0f} s", action='Robot operator: check the robot now.')

    def on_mission_end(self, mission, inspections):
        counts = {s: sum(1 for i in inspections if i['status'] == s) for s in ('FLAGGED', 'MANUAL_REVIEW', 'AUTOMATED_PASS')}
        message = (f"{len(inspections)} ladders assessed: {counts['FLAGGED']} flagged, {counts['MANUAL_REVIEW']} manual review, "
                   f"{counts['AUTOMATED_PASS']} automated pass. Mission status {mission['status']}.")
        return self.raise_alert(rule='MISSION_SUMMARY', kind='SUMMARY', severity='info', mission_id=mission['id'],
                                asset_id=None, title=f"Mission summary {mission['id']}", message=message,
                                action='Protocol PDF is on the dashboard.')

    # Delivery
    def raise_alert(self, rule, **alert):
        alert['rule'] = rule
        alert['recipients'] = [self._recipient_view(r) for r in self.roles_for(rule)]
        stored = self.store.add_alert(alert)
        self._deliver(stored, self.roles_for(rule), key='recipients')
        return self.store.alert(stored['id'])

    def ack(self, alert_id, by):
        alert = self.store.ack_alert(alert_id, by)
        if alert:
            self.bus.publish('alert', alert)
        return alert

    def check_escalations(self):
        after = self.config.get('escalate_after_s', 300)
        escalated = []
        for alert in self.store.unacked_critical():
            created = datetime.fromisoformat(alert['created'])
            if (self.clock() - created).total_seconds() < after:
                continue
            roles = self.roles_for('ESCALATION')
            alert['escalation'] = [self._recipient_view(r) for r in roles]
            self.store.mark_escalated(alert['id'])
            self.store.update_alert_payload(alert['id'], self._payload(alert))
            self._deliver(self.store.alert(alert['id']), roles, key='escalation', prefix='ESCALATED: ')
            escalated.append(alert['id'])
        return escalated

    def _recipient_view(self, recipient):
        return {'role': recipient['role'], 'title': recipient.get('title', recipient['role']),
                'name': recipient.get('name', ''), 'where': recipient.get('where', ''),
                'deliveries': [{'channel': 'dashboard', 'state': 'shown'}]}

    def _deliver(self, alert, recipients, key, prefix=''):
        payload = self._payload(alert)
        message = {**alert, 'title': prefix + alert['title']}
        for view, recipient in zip(payload[key], recipients):
            for channel in self.channels:
                state, error = channel.send(message, recipient)
                view['deliveries'].append({'channel': channel.name, 'state': state, 'error': error})
        self.store.update_alert_payload(alert['id'], payload)
        self.bus.publish('alert', self.store.alert(alert['id']))

    @staticmethod
    def _payload(alert):
        keep = ('mission_id', 'asset_id', 'kind', 'severity', 'rule', 'title', 'message', 'action', 'recipients', 'escalation')
        return {k: alert[k] for k in keep if k in alert}
