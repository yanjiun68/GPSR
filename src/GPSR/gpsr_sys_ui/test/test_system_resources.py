from pathlib import Path

from gpsr_sys_ui.system_resources import (
    LinuxResourceReader,
    cpu_percent,
    parse_meminfo,
    parse_proc_stat,
)


def test_parse_proc_stat_and_cpu_delta():
    before = parse_proc_stat('cpu  100 0 50 850 0 0 0 0\ncpu0 50 0 25 425 0 0 0 0\n')
    after = parse_proc_stat('cpu  150 0 70 930 0 0 0 0\ncpu0 80 0 35 455 0 0 0 0\n')
    values = cpu_percent(before, after)
    assert round(values['cpu'], 1) == 46.7
    assert round(values['cpu0'], 1) == 57.1


def test_parse_meminfo_bytes():
    info = parse_meminfo('MemTotal:       1000 kB\nMemAvailable:    250 kB\nSwapTotal:       100 kB\nSwapFree:         75 kB\n')
    assert info['MemTotal'] == 1000 * 1024
    assert info['MemAvailable'] == 250 * 1024


def test_reader_uses_fake_proc_and_sys(tmp_path: Path):
    proc = tmp_path / 'proc'
    sys = tmp_path / 'sys'
    (proc).mkdir()
    (sys / 'class' / 'thermal' / 'thermal_zone0').mkdir(parents=True)
    (proc / 'stat').write_text('cpu  100 0 50 850 0 0 0 0\ncpu0 50 0 25 425 0 0 0 0\n')
    (proc / 'meminfo').write_text('MemTotal: 1048576 kB\nMemAvailable: 524288 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n')
    (proc / 'loadavg').write_text('1.00 2.00 3.00 1/100 1\n')
    (sys / 'class' / 'thermal' / 'thermal_zone0' / 'temp').write_text('55000\n')
    (sys / 'class' / 'thermal' / 'thermal_zone0' / 'type').write_text('x86_pkg_temp\n')

    reader = LinuxResourceReader(proc_root=str(proc), sys_root=str(sys), disk_path=str(tmp_path), nvidia_smi_enabled=False)
    first = reader.sample()
    assert first['system_ram_percent'] == 50.0
    assert first['system_load_1m'] == 1.0
    assert first['system_cpu_temp_c'] == 55.0

    (proc / 'stat').write_text('cpu  150 0 70 930 0 0 0 0\ncpu0 80 0 35 455 0 0 0 0\n')
    second = reader.sample()
    assert round(second['system_cpu_percent'], 1) == 46.7
    assert second['system_cpu_core_max_percent'] > 50.0
