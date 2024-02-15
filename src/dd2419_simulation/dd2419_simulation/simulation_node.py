import rclpy
from rclpy.node import Node 
from nav_msgs.msg import MapMetaData, Path, OccupancyGrid
import numpy as np

from gazebo_msgs.srv import SpawnEntity

HEIGHT = 2
MODEL = "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><cylinder><radius>RRRRR</radius><length>{}</length></cylinder></geometry></visual></link></model></sdf>".format(HEIGHT)

class SimulationNode(Node):
    # -- user defined parameters --
    CYLINDER_POSITIONS = [[0.5+3, 1.72+3], [-1+3, 2.75+3], [0.3+3, -2.5+3], [2.5+3, -2.7+3], [-2+3, 2+3], [-3, -4]]
    CYLINDER_RADII = [0.25, 0.45, 0.275, 0.35, 0.185, 0.254]
    MAP_TOPIC = '/map'
    MAP_FREQ = 5
    INFLATION_RADIUS = 0.05
    
    def __init__(self):
        super().__init__('simulation_node')
    
        self.map_pub = self.create_publisher(OccupancyGrid, self.MAP_TOPIC, 10)
    
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.generate_objects_and_map()
    
        self.map_pub_timer = self.create_timer(float(1.0/self.MAP_FREQ), self.publish_map)
        
    def generate_objects_and_map(self):
        
        self.request = SpawnEntity.Request()
        for i in range(len(self.CYLINDER_POSITIONS)):
            
            self.request.name = 'cylinder_' + str(i)
            self.request.xml = MODEL.replace('RRRRR', str(self.CYLINDER_RADII[i]))
            self.request.initial_pose.position.x = float(self.CYLINDER_POSITIONS[i][0])
            self.request.initial_pose.position.y = float(self.CYLINDER_POSITIONS[i][1])
            self.request.initial_pose.position.z = HEIGHT / 2.0
            self.request.initial_pose.orientation.w = 1.0
            self.request.initial_pose.orientation.x = self.request.initial_pose.orientation.y = self.request.initial_pose.orientation.z = 0.0
            future = self.spawn_entity_client.call_async(self.request)
        
        self.map:OccupancyGrid = OccupancyGrid()
        
        self.map.header.frame_id = 'odom'
        
        self.map.info.resolution = 0.05
        W = H = 16
        self.map.info.width = int(W / self.map.info.resolution)
        self.map.info.height = int(H / self.map.info.resolution)
        self.map.info.origin.position.x = -W/2.0
        self.map.info.origin.position.y = -H/2.0
        self.map.info.map_load_time = self.get_clock().now().to_msg()
        
        self.map_raw = np.zeros((self.map.info.height, self.map.info.width), dtype=np.int8)

        xx, yy = np.meshgrid(np.arange(0, self.map.info.width, 1), np.arange(0, self.map.info.height, 1))
        yy = yy * self.map.info.resolution + self.map.info.resolution/2.0 + self.map.info.origin.position.y
        xx = xx * self.map.info.resolution + self.map.info.resolution/2.0 + self.map.info.origin.position.x
        
        for i, r in enumerate(self.CYLINDER_RADII):
            dists = np.sqrt((yy - self.CYLINDER_POSITIONS[i][1])**2 + (xx - self.CYLINDER_POSITIONS[i][0])**2)
            mask = np.where(dists <= r+self.INFLATION_RADIUS, 1, 0)
            self.map_raw[mask==1] = 100

        self.map.data = self.map_raw.reshape((-1)).tolist()
    
    def publish_map(self):
        
        self.map.header.stamp = self.get_clock().now().to_msg()
        
        self.map_pub.publish(self.map) 
    
def main():
    rclpy.init()
    
    node = SimulationNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()