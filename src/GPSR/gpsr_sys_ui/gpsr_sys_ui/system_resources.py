from __future__ import annotations

import glob
import os
import subprocess
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple


def parse_proc_stat(text: str) -> Dict[str, Tuple[int, int]]:
    out: Dict[str, Tuple[int, int]] = {}
    for line in text.splitlines():
        fields = line.split()
        if not fields or not fields[0].startswith('cpu'):
            continue
        try:
            values = [int(v) for v in fields[1:]]
        except ValueError:
            continue
        if len(values) < 4:
            continue
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        out[fields[0]] = (total, idle)
    return out


def cpu_percent(previous: Dict[str, Tuple[int, int]], current: Dict[str, Tuple[int, int]]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for key, (total, idle) in current.items():
        if key not in previous:
            continue
        p_total, p_idle = previous[key]
        dt = total - p_total
        di = idle - p_idle
        if dt <= 0:
            continue
        result[key] = max(0.0, min(100.0, 100.0 * (1.0 - (di / dt))))
    return result


def parse_meminfo(text: str) -> Dict[str, float]:
    values: Dict[str, float] = {}
    for line in text.splitlines():
        if ':' not in line:
            continue
        key, rest = line.split(':', 1)
        parts = rest.strip().split()
        if not parts:
            continue
        try:
            value = float(parts[0])
        except ValueError:
            continue
        if len(parts) > 1 and parts[1].lower() == 'kb':
            value *= 1024.0
        values[key] = value
    return values


@dataclass
class LinuxResourceReader:
    proc_root: str = '/proc'
    sys_root: str = '/sys'
    disk_path: str = '/'
    nvidia_smi_enabled: bool = True

    def __post_init__(self) -> None:
        self._previous_cpu: Dict[str, Tuple[int, int]] = {}

    def _read_text(self, path: str) -> str:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            return handle.read()

    def read_cpu(self) -> Dict[str, float]:
        try:
            current = parse_proc_stat(self._read_text(os.path.join(self.proc_root, 'stat')))
        except OSError:
            return {}
        percentages = cpu_percent(self._previous_cpu, current) if self._previous_cpu else {}
        self._previous_cpu = current
        result: Dict[str, float] = {}
        if 'cpu' in percentages:
            result['system_cpu_percent'] = round(percentages['cpu'], 1)
        cores = [v for k, v in percentages.items() if k != 'cpu']
        if cores:
            result['system_cpu_core_max_percent'] = round(max(cores), 1)
            result['system_cpu_cores_over_90'] = sum(1 for v in cores if v >= 90.0)
        return result

    def read_memory(self) -> Dict[str, float]:
        try:
            info = parse_meminfo(self._read_text(os.path.join(self.proc_root, 'meminfo')))
        except OSError:
            return {}
        total = info.get('MemTotal', 0.0)
        available = info.get('MemAvailable', info.get('MemFree', 0.0))
        swap_total = info.get('SwapTotal', 0.0)
        swap_free = info.get('SwapFree', 0.0)
        out: Dict[str, float] = {}
        if total > 0:
            used = max(0.0, total - available)
            out['system_ram_percent'] = round(100.0 * used / total, 1)
            out['system_ram_available_gb'] = round(available / (1024 ** 3), 2)
        if swap_total > 0:
            swap_used = max(0.0, swap_total - swap_free)
            out['system_swap_percent'] = round(100.0 * swap_used / swap_total, 1)
            out['system_swap_used_gb'] = round(swap_used / (1024 ** 3), 2)
        else:
            out['system_swap_percent'] = 0.0
            out['system_swap_used_gb'] = 0.0
        return out

    def read_load(self) -> Dict[str, float]:
        try:
            fields = self._read_text(os.path.join(self.proc_root, 'loadavg')).split()
            return {
                'system_load_1m': float(fields[0]),
                'system_load_5m': float(fields[1]),
                'system_load_15m': float(fields[2]),
            }
        except (OSError, ValueError, IndexError):
            return {}

    def read_disk(self) -> Dict[str, float]:
        try:
            stat = os.statvfs(self.disk_path)
        except OSError:
            return {}
        total = stat.f_blocks * stat.f_frsize
        available = stat.f_bavail * stat.f_frsize
        used = max(0, total - available)
        if total <= 0:
            return {}
        return {
            'system_disk_percent': round(100.0 * used / total, 1),
            'system_disk_used_gb': round(used / (1024 ** 3), 2),
            'system_disk_available_gb': round(available / (1024 ** 3), 2),
        }

    def read_temperature(self) -> Dict[str, object]:
        candidates = []
        candidates += glob.glob(os.path.join(self.sys_root, 'class/thermal/thermal_zone*/temp'))
        candidates += glob.glob(os.path.join(self.sys_root, 'class/hwmon/hwmon*/temp*_input'))
        best: Optional[Tuple[float, str]] = None
        for path in candidates:
            try:
                raw = float(self._read_text(path).strip())
            except (OSError, ValueError):
                continue
            temp_c = raw / 1000.0 if raw > 1000.0 else raw
            if not (-20.0 <= temp_c <= 150.0):
                continue
            label = os.path.basename(path)
            label_path = path.replace('_input', '_label') if path.endswith('_input') else os.path.join(os.path.dirname(path), 'type')
            try:
                label = self._read_text(label_path).strip() or label
            except OSError:
                pass
            if best is None or temp_c > best[0]:
                best = (temp_c, label)
        if best is None:
            return {}
        return {'system_cpu_temp_c': round(best[0], 1), 'system_cpu_temp_sensor': best[1]}

    def read_gpu(self) -> Dict[str, float]:
        if not self.nvidia_smi_enabled:
            return {}
        cmd = [
            'nvidia-smi',
            '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw',
            '--format=csv,noheader,nounits',
        ]
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True, timeout=1.5).strip().splitlines()
        except (OSError, subprocess.SubprocessError):
            return {}
        if not output:
            return {}
        try:
            util, used, total, temp, power = [float(x.strip()) for x in output[0].split(',')[:5]]
        except (ValueError, IndexError):
            return {}
        out = {
            'gpu_percent': util,
            'gpu_memory_used_mb': used,
            'gpu_memory_total_mb': total,
            'gpu_temperature_c': temp,
            'gpu_power_w': power,
        }
        if total > 0:
            out['gpu_memory_percent'] = round(100.0 * used / total, 1)
        return out

    def sample(self) -> Dict[str, object]:
        out: Dict[str, object] = {}
        for part in (self.read_cpu(), self.read_memory(), self.read_load(), self.read_disk(), self.read_temperature(), self.read_gpu()):
            out.update(part)
        return out
