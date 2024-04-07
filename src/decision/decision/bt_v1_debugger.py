import rclpy
from rclpy.node import Node
from dd2419_interfaces.msg import ObjectList

class BTV1Debugger(Node):
    OBJECTS = ['cube:green_0', 'cube:green_1', 'cube:blue_0', 'doll:jacob_0', 'sphere:red_0']

    def __init__(self):
        super().__init__('bt_v1_debugger')
        self.loop = self.create_timer(0.2, self.run)

        self.object_list_pub = self.create_publisher(ObjectList, '/object_list', 10)
        self.object_list = ObjectList()
        for obj in self.OBJECTS:
            self.object_list.object_list.append(obj)

        self.start_t = self.get_clock().now()
        self.new_obj_added = False

    def run(self):
        print("bt_v1_debugger is running")
        self.object_list_pub.publish(self.object_list)

        dt = self.get_clock().now() - self.start_t
        if dt > rclpy.time.Duration(seconds=5) and not self.new_obj_added:
            self.object_list.object_list.append('sphere:yellow_0')
            self.new_obj_added = True

def main(args=None):
    rclpy.init(args=args)

    bt_v1_debugger = BTV1Debugger()
    rclpy.spin(bt_v1_debugger)

    rclpy.shutdown()

if __name__ == '__main__':
    main()