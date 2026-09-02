from __future__ import annotations

import fnmatch
from collections import Counter

from ..event_model import MonitorEvent


def _full_name(name: str, namespace: str) -> str:
    ns = namespace.rstrip('/')
    return f'{ns}/{name}' if ns else f'/{name}'


class GraphSource:
    def __init__(self, node, cache, events, rate_hz: float, expected_nodes, optional_patterns) -> None:
        self.node = node
        self.cache = cache
        self.events = events
        self.expected_nodes = list(expected_nodes)
        self.optional_patterns = list(optional_patterns)
        self._missing = set()
        self.timer = node.create_timer(1.0 / max(0.1, rate_hz), self.poll)

    def poll(self) -> None:
        try:
            names = [_full_name(name, ns) for name, ns in self.node.get_node_names_and_namespaces()]
        except Exception as exc:  # graph discovery must never take monitor down
            self.events.emit(MonitorEvent('WARN', 'DIAGNOSTICS', 'ROS_GRAPH_QUERY_FAILED', str(exc), node=self.node.get_name()))
            return
        counts = Counter(names)
        rows = []
        for name in sorted(counts):
            duplicate = counts[name] > 1
            health = 'WARN' if duplicate else 'OK'
            message = f'duplicate count={counts[name]}' if duplicate else ''
            rows.append({'name': name, 'health': health, 'state': 'present', 'message': message, 'count': counts[name]})
            if duplicate:
                self.events.emit(MonitorEvent('WARN', 'DIAGNOSTICS', 'DUPLICATE_NODE', message, node=name))

        known = set(names)
        for required in self.expected_nodes:
            if required not in known:
                rows.append({'name': required, 'health': 'ERROR', 'state': 'missing', 'message': 'required node not discovered'})
                if required not in self._missing:
                    self.events.emit(MonitorEvent('ERROR', 'DIAGNOSTICS', 'NODE_MISSING', 'required node not discovered', node=required))
                    self._missing.add(required)
            else:
                self._missing.discard(required)

        self.cache.set('nodes', rows)
        self.cache.update_section('summary', {
            'node_count': len(names),
            'unique_node_count': len(known),
            'duplicate_node_count': sum(1 for c in counts.values() if c > 1),
            'missing_required_node_count': len(self._missing),
        })
