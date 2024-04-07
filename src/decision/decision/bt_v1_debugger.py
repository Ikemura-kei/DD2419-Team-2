import rclpy
from rclpy.node import Node

class BTV1Debugger(Node):
    def __init__(self):
        super().__init__('bt_v1_debugger')
        self.loop = self.create_timer(0.2, self.run)

    def run(self):
        print("bt_v1_debugger is running")

def main(args=None):
    rclpy.init(args=args)

    bt_v1_debugger = BTV1Debugger()
    rclpy.spin(bt_v1_debugger)

    rclpy.shutdown()

if __name__ == '__main__':
    main()