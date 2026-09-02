from gpsr_sys_ui.event_model import MonitorEvent


def test_event_normalizes_severity_and_subsystem():
    event = MonitorEvent('warn', 'nav2', 'X', 'message')
    assert event.severity == 'WARN'
    assert event.subsystem == 'NAV2'
    assert event.repeat_count == 1
