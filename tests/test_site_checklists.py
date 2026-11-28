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


if __name__ == '__main__':
    unittest.main()
