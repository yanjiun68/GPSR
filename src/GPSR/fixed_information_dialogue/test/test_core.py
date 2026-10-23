from datetime import datetime, timezone
from pathlib import Path

from fixed_information_dialogue.core import FixedInformationStore, FOUND


CONFIG = Path(__file__).parents[1] / "config" / "fixed_information.yaml"


def test_static_team_fact_comes_from_yaml() -> None:
    store = FixedInformationStore(CONFIG)

    result = store.query("what is your team name")

    assert result.status == FOUND
    assert result.key == "team_name"
    assert result.answer_text == "My team is Putrabot UPM."


def test_runtime_date_uses_configured_kuala_lumpur_timezone() -> None:
    store = FixedInformationStore(CONFIG)
    # 2026-10-22 16:30 UTC is already 2026-10-23 in Kuala Lumpur.
    utc_time = datetime(2026, 10, 22, 16, 30, tzinfo=timezone.utc)

    result = store.query("today's date", now=utc_time)

    assert result.status == FOUND
    assert result.key == "today"
    assert result.answer_text == "Today is Friday, October 23, 2026."
    assert result.metadata == {
        "timezone": "Asia/Kuala_Lumpur",
        "timezone_label": "Malaysia Time (MYT)",
    }
