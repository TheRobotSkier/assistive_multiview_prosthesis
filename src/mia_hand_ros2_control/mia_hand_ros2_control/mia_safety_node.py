#!/usr/bin/env python3
"""MIA Hand Safety Node — emergency stop and play services via serial."""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import time


class MiaSafetyNode(Node):
    def __init__(self):
        super().__init__('mia_safety')
        
        self.declare_parameter('serial_port', '/dev/mia_hand')
        self.declare_parameter('baudrate', 115200)
        
        self._port = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        
        self._emergency_stop_srv = self.create_service(
            Trigger, '/mia_hand/emergency_stop', self._on_emergency_stop)
        self._play_srv = self.create_service(
            Trigger, '/mia_hand/play', self._on_play)
        
        self.get_logger().info(
            f'MiaSafetyNode ready on {self._port}')
    
    def _send_command(self, cmd: str) -> bool:
        try:
            ser = serial.Serial(self._port, self._baudrate, timeout=0.5)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.05)
            ser.flushInput()
            ser.flushOutput()
            ser.write(cmd.encode())
            ser.flush()
            ack = ser.read(17)
            ser.close()
            return len(ack) == 17 and ack[0] == ord('<')
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return False
    
    def _on_emergency_stop(self, request, response):
        success = self._send_command('@AS.............*\r')
        response.success = success
        response.message = 'Emergency stop triggered' if success else 'Failed to stop'
        self.get_logger().info(response.message)
        return response
    
    def _on_play(self, request, response):
        success = self._send_command('@AR.............*\r')
        response.success = success
        response.message = 'Normal operation restored' if success else 'Failed to resume'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MiaSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
