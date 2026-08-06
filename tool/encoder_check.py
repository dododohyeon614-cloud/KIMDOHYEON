#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')
        self.encoder_pub = self.create_publisher(Int32, 'encoder', 10)
        self.port_name = '/dev/ttyACM0'
        self.baudrate = 57600
        
        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self.get_logger().info(f"✅ 시리얼 포트 연결 성공: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ 시리얼 포트 연결 실패: {e}")
            return
            
        self.timer = self.create_timer(0.02, self.read_serial_data)

    def read_serial_data(self):
        # [수정 핵심] if를 while로 변경하여 버퍼에 쌓인 밀린 데이터를 한 번에 싹 다 비웁니다! (딜레이 완벽 제거)
        while hasattr(self, 'serial_port') and self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                parts = line.split(',')
                
                if len(parts) == 2 and parts[0] == 'E':
                    d_val = int(parts[1])
                    msg = Int32()
                    msg.data = d_val
                    self.encoder_pub.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_port'):
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()