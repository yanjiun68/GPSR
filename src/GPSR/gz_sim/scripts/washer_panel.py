#!/usr/bin/env python3
"""Operator controls for the simulated washer; communicates only via ROS topics."""
import json
import tkinter as tk
from tkinter import ttk

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('washer_panel')
    pub = node.create_publisher(String, '/washing_machine/command', 10)
    root = tk.Tk()
    root.title('RoboCup 2026 — Washing machine')
    root.geometry('380x390')
    frame = ttk.Frame(root, padding=18)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='WASHING MACHINE', font=('',16,'bold')).pack(pady=8)
    status = tk.StringVar(value='Connecting to simulation…')
    ttk.Label(frame, textvariable=status, wraplength=340, justify='left').pack(pady=12)
    buttons = ttk.Frame(frame)
    buttons.pack(fill='x')
    for i, (label, command) in enumerate([
        ('Power ON','power_on'), ('Power OFF','power_off'),
        ('Open door','open'), ('Close door','close'),
        ('Start / Resume','start'), ('Pause','pause'), ('Reset cycle','reset'),
    ]):
        ttk.Button(buttons, text=label,
                   command=lambda c=command: pub.publish(String(data=c))).grid(
                       row=i//2, column=i%2, sticky='ew', padx=4, pady=5)
    buttons.columnconfigure(0,weight=1); buttons.columnconfigure(1,weight=1)
    ttk.Label(frame, text='Door unlocks only when the drum has stopped.\nLaundry is movable rigid-body geometry.',
              wraplength=340).pack(pady=12)

    def on_status(msg):
        data = json.loads(msg.data)
        status.set(f"{data['phase'].upper()}  |  Door: {data['door_angle']*57.2958:.0f}°\n"
                   f"Drum: {data['drum_velocity']:.2f} rad/s  |  {data['remaining_seconds']:.1f} s\n"
                   f"{data['message']}")

    node.create_subscription(String, '/washing_machine/status', on_status, 10)

    def poll():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            root.after(50,poll)
        else:
            root.destroy()

    root.after(50,poll)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
