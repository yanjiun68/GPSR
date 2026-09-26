from datetime import datetime
from pathlib import Path
from fixed_information_dialogue.core import FOUND, NO_MATCH, FixedInformationStore

CONFIG = Path(__file__).resolve().parents[1] / "config" / "fixed_information.yaml"

def test_static():
    r = FixedInformationStore(CONFIG).query("what is your team name")
    assert r.status == FOUND and r.key == "team_name"

def test_time():
    r = FixedInformationStore(CONFIG).query(
        "what time is it", now=datetime(2026, 8, 29, 13, 7)
    )
    assert r.status == FOUND and "1:07 PM" in r.answer_text

def test_tomorrow():
    r = FixedInformationStore(CONFIG).query(
        "what day is tomorrow", now=datetime(2026, 8, 29, 13, 7)
    )
    assert r.status == FOUND and "August 30, 2026" in r.answer_text

# Section: The scheduler example wording must remain an exact provider trigger.
def test_tell_todays_date():
    r = FixedInformationStore(CONFIG).query(
        "tell today's date", now=datetime(2026, 9, 24, 9, 30)
    )
    assert r.status == FOUND
    assert r.answer_text == "Today is Thursday, September 24, 2026."

# Section: The provider accepts an action payload without interrogative wording.
def test_direct_todays_date_trigger():
    r = FixedInformationStore(CONFIG).query(
        "today's date", now=datetime(2026, 9, 24, 9, 30)
    )
    assert r.status == FOUND
    assert r.key == "today"
    assert r.answer_text == "Today is Thursday, September 24, 2026."

def test_unknown_not_forced():
    r = FixedInformationStore(CONFIG).query("where did you see the coke")
    assert r.status == NO_MATCH
