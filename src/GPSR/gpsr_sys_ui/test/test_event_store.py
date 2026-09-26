from gpsr_sys_ui.event_model import MonitorEvent
from gpsr_sys_ui.event_store import EventStore


def test_event_deduplication_increments_repeat_count():
    store = EventStore(max_events=10, dedupe_window_sec=10.0)
    a = MonitorEvent('WARN', 'NAV2', 'COLLISION_STOP', 'stop', node='/collision_monitor', timestamp=10.0)
    b = MonitorEvent('WARN', 'NAV2', 'COLLISION_STOP', 'stop', node='/collision_monitor', timestamp=11.0)
    store.emit(a)
    store.emit(b)
    events = store.snapshot()
    assert len(events) == 1
    assert events[0]['repeat_count'] == 2
    assert events[0]['last_seen'] == 11.0


def test_different_event_code_is_not_deduped():
    store = EventStore(max_events=10, dedupe_window_sec=10.0)
    store.emit(MonitorEvent('WARN', 'NAV2', 'A', 'same', timestamp=10.0))
    store.emit(MonitorEvent('WARN', 'NAV2', 'B', 'same', timestamp=11.0))
    assert len(store.snapshot()) == 2
