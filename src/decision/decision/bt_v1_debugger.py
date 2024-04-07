import rclpy
from rclpy.node import Node
from dd2419_interfaces.msg import ObjectList, ObjectPoses
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class BTV1Debugger(Node):
    OBJECTS = ['cube:green_0', 'cube:green_1', 'cube:blue_0', 'doll:jacob_0', 'sphere:red_0']
    POSES = [[1.0, 1.0, 0.0], [5.0, 0.0, 0.0], [0.0, -5.0, 0.0], [1.0, 4.0, 0.0], [-5.0, 0.2, 0.0]] # x, y, theta

    def __init__(self):
        super().__init__('bt_v1_debugger')
        self.loop = self.create_timer(0.2, self.run)

        self.object_list_pub = self.create_publisher(ObjectPoses, '/object_list', 10)
        self.object_list = ObjectPoses()
        stamp = self.get_clock().now().to_msg()
        for i, obj in enumerate(self.OBJECTS):
            self.object_list.object_list.object_list.append(obj)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = self.POSES[i][0]
            pose.pose.position.y = self.POSES[i][1]
            pose.pose.orientation.w = 1.0
            self.object_list.poses.append(pose)

        self.start_t = self.get_clock().now()
        self.new_obj_added = False
        
        # -- use joy stick to control if the pick action is completed --
        self.pick_done_pub = self.create_publisher(Bool, '/is_pick_done', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
    def joy_callback(self, msg:Joy):
        self.pick_done_pub.publish(Bool(data=msg.buttons[2] == 1))

    def run(self):
        print("bt_v1_debugger is running")
        stamp = self.get_clock().now().to_msg()
        for i in range(len(self.object_list.poses)):
            self.object_list.poses[i].header.stamp = stamp
        self.object_list_pub.publish(self.object_list)

        dt = self.get_clock().now() - self.start_t
        if dt > rclpy.time.Duration(seconds=5) and not self.new_obj_added:
            self.object_list.object_list.object_list.append('sphere:yellow_0')
            self.object_list.poses.append(self.object_list.poses[0])
            self.new_obj_added = True

def main(args=None):
    rclpy.init(args=args)

    bt_v1_debugger = BTV1Debugger()
    rclpy.spin(bt_v1_debugger)

    rclpy.shutdown()

if __name__ == '__main__':
    main()