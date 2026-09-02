from __future__ import annotations

from ..event_model import MonitorEvent
from ..system_resources import LinuxResourceReader


class SystemSource:
    def __init__(self, node, cache, events, rate_hz: float, *, nvidia_smi: bool,
                 cpu_warn: float, ram_warn: float, disk_warn: float) -> None:
        self.cache = cache
        self.events = events
        self.reader = LinuxResourceReader(nvidia_smi_enabled=nvidia_smi)
        self.cpu_warn = float(cpu_warn)
        self.ram_warn = float(ram_warn)
        self.disk_warn = float(disk_warn)
        self.timer = node.create_timer(1.0 / max(0.1, rate_hz), self.poll)

    def poll(self) -> None:
        sample = self.reader.sample()
        self.cache.set('system', sample)
        for key, threshold, code in (
            ('system_cpu_percent', self.cpu_warn, 'SYSTEM_CPU_HIGH'),
            ('system_ram_percent', self.ram_warn, 'SYSTEM_RAM_HIGH'),
            ('system_disk_percent', self.disk_warn, 'SYSTEM_DISK_HIGH'),
        ):
            value = sample.get(key)
            if isinstance(value, (int, float)) and value >= threshold:
                self.events.emit(MonitorEvent('WARN', 'SYSTEM', code, f'{key}={value:.1f}% >= {threshold:.1f}%'))
