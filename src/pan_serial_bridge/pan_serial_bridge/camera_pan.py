import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import time

class PanSerialBridge(Node):
    def __init__(self):
        super().__init__('pan_serial_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('default_pwm', 150)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.default_pwm = int(self.get_parameter('default_pwm').value)

        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)  # Arduino reboot time on serial open
        self.get_logger().info(f'Connected to Arduino on {port} @ {baud}')

        # Subscribe: desired angle (deg)
        self.sub = self.create_subscription(Float32, 'pan_cmd_deg', self.on_angle, 10)

        # Subscribe: raw command strings (optional)
        self.sub_cmd = self.create_subscription(String, 'pan_cmd_raw', self.on_raw, 10)

        # Read Arduino prints
        self.timer = self.create_timer(0.05, self.read_serial)

    def send_line(self, line: str):
        if not line.endswith('\n'):
            line += '\n'
        self.ser.write(line.encode('utf-8'))

    def on_angle(self, msg: Float32):
        deg = float(msg.data)
        cmd = f"a {deg:.1f} {self.default_pwm}"
        self.send_line(cmd)

    def on_raw(self, msg: String):
        self.send_line(msg.data)

    def read_serial(self):
        try:
            data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                self.get_logger().info(f"[arduino] {data}")
        except Exception:
            pass

def main():
    rclpy.init()
    node = PanSerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()