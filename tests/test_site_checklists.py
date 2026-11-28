"""nesseler's other site checklists: catalogue shape, and agreement with the pipeline's answered rows and kinds."""
import unittest

from ultron.dashboard import site_checklists as sc
from ultron.pipeline import catalog, checklists


class Catalogue(unittest.TestCase):
    def test_all_five_forms_listed(self):
        self.assertEqual(set(sc.CHECKLISTS), {'erste_hilfe_brandschutz', 'geruest', 'staubarme_baustelle',
                                              'wirksamkeitskontrolle', 'einweisung_nachunternehmen'})
        self.assertEqual([len(c.rows) for c in sc.CHECKLISTS.values()], [34, 20, 10, 13, 15])

    def test_row_ids_unique_and_modes_known(self):
        ids = [r.id for c in sc.CHECKLISTS.values() for r in c.rows]
        self.assertEqual(len(ids), len(set(ids)))
        for row in sc.ROWS.values():
            self.assertIn(row.mode, sc.MODES, row.id)

    def test_robot_rows_name_what_answers_them(self):
        for row in sc.ROWS.values():
            if row.mode in (sc.PRESENCE, sc.VISUAL):
                self.assertTrue(row.kinds, row.id)
            elif row.mode == sc.DERIVED:
                self.assertTrue(row.source in sc.CHECKLISTS or row.source == 'ladders', row.id)
            else:
                self.assertEqual((row.kinds, row.source), ((), ''), row.id)

    def test_briefing_form_has_nothing_for_the_robot(self):
        self.assertEqual(sc.CHECKLISTS['einweisung_nachunternehmen'].robot_rows, [])


class SameAsPipeline(unittest.TestCase):
    def test_rows_the_pipeline_answers_are_quoted_identically(self):
        for kind, rows in checklists.CHECKLISTS.items():
            for row in rows:
                self.assertIn(row.id, sc.ROWS, row.id)
                self.assertEqual(sc.ROWS[row.id].text, row.text, row.id)
                self.assertIn(kind, sc.ROWS[row.id].kinds, row.id)

    def test_every_kind_is_detected_or_declared_missing(self):
        detected = set(catalog.KINDS)
        for kind in sc.kinds():
            self.assertTrue((kind in detected) != (kind in sc.NO_DETECTOR), kind)

    def test_describe_marks_rows_without_a_detector(self):
        rows = {r['id']: r for c in sc.describe() for r in c['rows']}
        self.assertTrue(rows['BS-1']['detector'])
        self.assertFalse(rows['EH-9']['detector'])


def row(summary, row_id):
    return next(r for c in summary['checklists'] for r in c['rows'] if r['id'] == row_id)


def checklist(summary, key):
    return next(c for c in summary['checklists'] if c['key'] == key)


class PatrolSummary(unittest.TestCase):
    EXT = {'object_id': 'FEX-001', 'kind': 'fire_extinguisher', 'rows': {
        'BS-1': {'answer': 'ja', 'remark': 'roter Löscher an der Wand', 'photo': 'fex-1'},
        'BS-2': {'answer': 'offen', 'remark': 'Plakette nicht lesbar', 'photo': 'fex-2'}}}

    def test_nothing_seen_is_never_a_no(self):
        s = sc.patrol_summary([])
        self.assertEqual(row(s, 'BS-1')['answer'], sc.NOT_FOUND)     # detector exists, object not seen
        self.assertEqual(row(s, 'EH-9')['answer'], sc.NO_DETECTOR_YET)  # escape sign: nothing detects it
        self.assertEqual(row(s, 'EH-4')['answer'], sc.BY_PERSON)
        self.assertNotIn(sc.NO, [r['answer'] for c in s['checklists'] for r in c['rows']])

    def test_found_object_answers_its_rows(self):
        s = sc.patrol_summary([self.EXT])
        self.assertEqual((row(s, 'BS-1')['answer'], row(s, 'BS-1')['photo'], row(s, 'BS-1')['objects']),
                         (sc.YES, 'fex-1', ['FEX-001']))
        self.assertEqual((row(s, 'BS-2')['answer'], row(s, 'BS-2')['remark']), (sc.OPEN, 'Plakette nicht lesbar'))
        self.assertEqual(checklist(s, 'erste_hilfe_brandschutz')['headline'], sc.OPEN)
        presence = {p['kind']: p for p in s['presence']}
        self.assertEqual((presence['fire_extinguisher']['found'], presence['fire_extinguisher']['objects']), (1, ['FEX-001']))
        self.assertEqual((presence['scaffold']['found'], presence['scaffold']['detector']), (0, False))

    def test_one_bad_object_makes_the_row_a_no(self):
        expired = {'object_id': 'FEX-002', 'kind': 'fire_extinguisher',
                   'rows': {'BS-2': {'answer': 'nein', 'remark': 'Prüffrist 03/2025 abgelaufen', 'photo': 'fex-3'}}}
        s = sc.patrol_summary([self.EXT, expired])
        self.assertEqual((row(s, 'BS-2')['answer'], row(s, 'BS-2')['photo_object']), (sc.NO, 'FEX-002'))
        self.assertEqual(row(s, 'BS-1')['answer'], sc.YES)
        self.assertEqual(checklist(s, 'erste_hilfe_brandschutz')['headline'], sc.NO)

    def test_ladder_row_of_the_effectiveness_check(self):
        self.assertEqual(row(sc.patrol_summary([]), 'WK-5')['answer'], sc.NOT_FOUND)
        passed = row(sc.patrol_summary([], [{'status': 'AUTOMATED_PASS'}]), 'WK-5')
        self.assertEqual(passed['answer'], sc.OPEN)  # screening is not "measures sufficient"
        flagged = row(sc.patrol_summary([], [{'status': 'AUTOMATED_PASS'}, {'status': 'FLAGGED'}]), 'WK-5')
        self.assertEqual((flagged['answer'], flagged['remark']), (sc.NO, '2 Leiter(n) geprüft, 1 mit Mangel'))

    def test_briefing_form_is_all_for_a_person(self):
        self.assertEqual(checklist(sc.patrol_summary([self.EXT]), 'einweisung_nachunternehmen')['headline'], sc.BY_PERSON)


if __name__ == '__main__':
    unittest.main()
