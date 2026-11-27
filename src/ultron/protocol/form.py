"""The nesseler "Kontrollblatt / Checkliste zur Überprüfung von Leitern und Tritten" as data.

Source: kontrollblatt_checkliste_leitern_tritte.pdf (BGI 694 template). Labels are kept
verbatim in German so the digital protocol can be laid over the paper form one to one.
"""
from dataclasses import dataclass

LADDER_TYPES = ("Anlegeleiter", "Mehrzweckleiter", "Schiebeleiter", "Podestleitern", "Seilzugleiter",
                "Steckleiter", "Stehleiter", "Tritt", "Sonstige")
MATERIALS = ("Aluminium", "Stahl", "Kunststoff", "Edelstahl", "Holz")

# Page 1 of the form: (key, German label). `naechste_pruefung` is printed as "Siehe Prüfplakette".
MASTER_DATA = (
    ("inventar_nummer", "Inventar-Nummer der Leiter"),
    ("standort", "Standort/Abteilung"),
    ("aufstiegsart", "Aufstiegsart"),
    ("werkstoff", "Werkstoff der Leiter"),
    ("anzahl_sprossen", "Anzahl der Sprossen/Stufen"),
    ("leiterlaenge", "Leiterlänge / Leiter gekürzt auf"),
    ("hersteller", "Hersteller / Händler"),
    ("artikel_typ_nummer", "Artikel- / Typ-Nummer"),
    ("datum_anschaffung", "Datum der Anschaffung"),
    ("datum_aussonderung", "Datum der Aussonderung"),
    ("sachkundiger", "Name des Sachkundigen/Beauftragten"),
    ("naechste_pruefung", "Nächste Prüfung"),
)

# A camera can judge appearance. It cannot operate a hinge or feel a burr: those
# criteria stay open for the human inspector unless a defect is plainly visible.
VISUAL, HANDS_ON = "visual", "hands_on"

SPREADER_TYPES = ("Stehleiter", "Mehrzweckleiter", "Podestleitern", "Tritt")


@dataclass(frozen=True)
class Criterion:
    id: str
    section: str
    label: str
    english: str
    mode: str = VISUAL
    only_types: tuple = ()       # empty: applies to every ladder type
    only_materials: tuple = ()   # empty: applies to every material


SECTIONS = (
    ("1", "Holme", "Rails"),
    ("2", "Sprossen/Stufen/Plattform", "Rungs / steps / platform"),
    ("3", "Spreizsicherung", "Spreader"),
    ("4", "Beschlagteile", "Fittings"),
    ("5", "Leiter-/Trittfüße/Rollen", "Feet / rollers"),
    ("6", "Zubehör (z.B. Holmverlängerung, Fußverbreiterung, Wandabstützung)", "Accessories"),
    ("7", "Kennzeichnung", "Marking"),
)

SECTION_TITLES = {number: title for number, title, _ in SECTIONS}

CRITERIA = (
    Criterion("1.1", "1", "Verformung", "Rails bent or deformed"),
    Criterion("1.2", "1", "Beschädigung (z.B. Risse)", "Rails damaged, e.g. cracks"),
    Criterion("1.3", "1", "Scharfe Kanten, Splitter, Grat", "Sharp edges, splinters, burrs on rails"),
    Criterion("1.4", "1", "Abnutzung", "Rails worn"),
    Criterion("1.5", "1", "Schutzbehandlung (bei Holz)", "Protective coating (wood only)",
              only_materials=("Holz",)),
    Criterion("2.1", "2", "Verformung", "Rungs/steps/platform bent or deformed"),
    Criterion("2.2", "2", "Beschädigung", "Rungs/steps/platform damaged or missing"),
    Criterion("2.3", "2", "Scharfe Kanten, Splitter, Grat", "Sharp edges, splinters, burrs on rungs"),
    Criterion("2.4", "2", "Verbindung zum Holm (z.B. Bördelung, Schraub-/Niet-Verbindung, Schweißnaht)",
              "Joint between rung and rail"),
    Criterion("2.5", "2", "Abnutzung (z.B. Trittfläche, Plattformauflage)", "Tread surface worn"),
    Criterion("3.1", "3", "Vollständigkeit/Befestigung", "Spreader complete and fixed",
              only_types=SPREADER_TYPES),
    Criterion("3.2", "3", "Funktionsfähigkeit", "Spreader works", HANDS_ON, SPREADER_TYPES),
    Criterion("3.3", "3", "Beschädigung", "Spreader damaged", only_types=SPREADER_TYPES),
    Criterion("4.1", "4", "Beschädigung/Korrosion", "Fittings damaged or rusty"),
    Criterion("4.2", "4", "Vollständigkeit/Befestigung", "Fittings complete and fixed"),
    Criterion("4.3", "4", "Funktionsfähigkeit", "Fittings work", HANDS_ON),
    Criterion("4.4", "4", "Abnutzung", "Fittings worn"),
    Criterion("4.5", "4", "Schmierung (mechanische Teile)", "Mechanical parts greased", HANDS_ON),
    Criterion("5.1", "5", "Vollständigkeit/Befestigung", "Feet complete and fixed"),
    Criterion("5.2", "5", "Abnutzung/Beschädigung", "Feet worn or damaged"),
    Criterion("5.3", "5", "Funktionsfähigkeit", "Feet/rollers work", HANDS_ON),
    Criterion("6.1", "6", "Vollständigkeit/Befestigung", "Accessories complete and fixed"),
    Criterion("7.1", "7", "Betriebsanleitung (z.B. Piktogramm)", "Instructions / pictograms present"),
)
BY_ID = {c.id: c for c in CRITERIA}

# Section 8 of the form, "Kontrollergebnis": exactly one of these is ticked.
RESULT_OK, RESULT_REPAIR, RESULT_REMOVE = "io", "reparatur", "aussondern"
RESULTS = {
    RESULT_OK: "Leiter i. O. und verwendungsfähig",
    RESULT_REPAIR: "Reparatur notwendig",
    RESULT_REMOVE: "Leiter sofort aussondern",
}

# Status of one criterion in one inspection column.
OK, DEFECT, NOT_VISIBLE, HUMAN_CHECK, NOT_APPLICABLE = "ok", "defect", "not_visible", "human_check", "n/a"
INSPECTIONS_PER_SHEET = 5


def applies(criterion, ladder_type, material):
    """Whether a row of the form is relevant for this ladder (unknown type/material: keep it)."""
    if criterion.only_types and ladder_type and ladder_type not in criterion.only_types:
        return False
    if criterion.only_materials and material and material not in criterion.only_materials:
        return False
    return True
