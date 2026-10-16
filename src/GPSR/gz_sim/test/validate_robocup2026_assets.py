#!/usr/bin/env python3
"""Opt-in live washer and object validation in ROS domain 185 (keep it unused)."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String


def main():
    share = Path(get_package_share_directory('gz_sim'))
    config = json.loads((share/'config/robocup2026_arena.json').read_text())
    root = ET.parse(share/'world/robocup2026_incheon.sdf').getroot()
    items = config['task_objects']
    assert len(items) == 36, 'Expected 30 inventory objects plus six extra shirts'
    for item in items:
        model = root.find(f"world/model[@name='{item['name']}']")
        assert model is not None and model.findtext('static') == 'false', item['name']
        assert model.find('link/inertial/mass') is not None, item['name']
        assert model.find('joint') is None, f"{item['name']} must not be fixed to furniture"
    washer = root.find("world/model[@name='washing_machine']")
    assert washer.find("joint[@name='door_hinge']") is not None
    assert len(washer.findall("link[@name='drum']/collision")) > 20

    run_dir = Path(tempfile.mkdtemp(prefix='putrabot-washer-test-'))
    os.environ.update(ROS_DOMAIN_ID='185', IGN_PARTITION=f'washer_test_{os.getpid()}',
                      GZ_PARTITION=f'washer_test_{os.getpid()}', ROS_LOG_DIR=str(run_dir/'ros'),
                      QT_QPA_PLATFORM='offscreen', PYTHONDONTWRITEBYTECODE='1')
    print('Log:', run_dir/'simulation.log', flush=True)
    log = (run_dir/'simulation.log').open('w')
    process = subprocess.Popen(['ros2','launch','my_robot_nav','robocup_arena.launch.py','headless:=true'],
                               stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node('washer_validation')
        status = {}
        node.create_subscription(String,'/washing_machine/status',lambda msg: status.update(json.loads(msg.data)),10)
        publisher = node.create_publisher(String,'/washing_machine/command',10)

        # Allow headless rendering to run below real time. These are wall-clock
        # watchdogs; the controller's cycle itself uses simulation seconds.
        def wait_for(predicate, label, timeout=180):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited; see log')
                if predicate():
                    print('PASS:',label,flush=True)
                    return
            raise RuntimeError(f'{label} timed out; status={status}')

        def command(value):
            publisher.publish(String(data=value))

        def snapshot():
            result = subprocess.run(['ign','topic','-e','-t',
                                     '/world/robocup2026_incheon/dynamic_pose/info',
                                     '-n','1','--json-output'],capture_output=True,text=True,timeout=15,check=True)
            message = json.loads(result.stdout)
            return {p['name']:p for p in message['pose']}

        wait_for(lambda: bool(status) and publisher.get_subscription_count()>0,'washer online',60)
        command('power_on')
        wait_for(lambda: status.get('phase')=='idle','power on')
        command('open')
        wait_for(lambda: status.get('door_angle',0)>1.55,'door opens for drum access')
        command('start')
        wait_for(lambda: 'latched' in status.get('message','') and status.get('phase')=='idle',
                 'open-door start rejected')

        poses = snapshot()
        for spec in items:
            assert spec['name'] in poses, f"Missing dynamic object {spec['name']}"
            z = poses[spec['name']].get('position',{}).get('z',0)
            assert z > 0.035, f"{spec['name']} fell through its support: z={z}"
        for name in ('white_shirt','grey_shirt','blue_shirt'):
            p = poses[name]['position']
            assert 0.17 < p['z'] < 0.7, f'Laundry escaped the drum: {name}: {p}'
        print('PASS: all 36 loose objects remain supported; three shirts inside drum',flush=True)

        command('close')
        wait_for(lambda: abs(status.get('door_angle',99))<0.03,'door closes')
        command('start')
        wait_for(lambda: status.get('phase')=='running' and abs(status.get('drum_velocity',0))>0.3,
                 'drum spins during cycle')
        command('open')
        wait_for(lambda: 'locked' in status.get('message','') and status.get('door_target')==0,
                 'running door interlock')
        command('pause')
        wait_for(lambda: status.get('phase')=='paused' and abs(status.get('drum_velocity',99))<0.05,
                 'pause stops drum')
        command('start')
        wait_for(lambda: status.get('phase')=='complete','timed cycle completes',600)
        wait_for(lambda: abs(status.get('drum_velocity',99))<0.05,
                 'drum stops after completed cycle')
        command('open')
        wait_for(lambda: status.get('door_angle',0)>1.55,'door opens after cycle')
        command('power_off')
        wait_for(lambda: status.get('phase')=='off','power off')
        print('PASS: washer physics, interlocks, cycle and full object inventory.',flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
