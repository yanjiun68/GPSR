from pathlib import Path


def test_package_identity_and_email():
    root = Path(__file__).resolve().parents[1]
    package_xml = (root / 'package.xml').read_text()
    setup_py = (root / 'setup.py').read_text()
    assert '<name>gpsr_sys_ui</name>' in package_xml
    assert 'putrabotupm@gmail.com' in package_xml
    assert "package_name = 'gpsr_sys_ui'" in setup_py
    assert 'putrabotupm@gmail.com' in setup_py


def test_energy_panel_removed_from_tui_tabs():
    root = Path(__file__).resolve().parents[1]
    tui = (root / 'gpsr_sys_ui' / 'monitor_tui.py').read_text()
    tabs_block = tui.split('TABS = (', 1)[1].split(')', 1)[0]
    assert 'Energy' not in tabs_block
    assert 'System Telemetry' in tabs_block
