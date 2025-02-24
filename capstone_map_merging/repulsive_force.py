import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String

class RepulsiveForce(Node):
    def __init__(self):
        super().__init__('repulsive_force')
        self.obstacles = []
        map_topic = '/map0'
        self.subscription = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        self.arrival = self.create_subscription(String, '/arrival', self.arrival_callback, 10)
        goal_topic = '/goal_state'
        self.publisher = self.create_publisher(Point,goal_topic , 10)


    def arrival_callback(self, msg):
        self.get_logger().info(f'massage: {msg.data}')
##########여기서 플래그를 사용해서 msg를 받아야지 map_callback이 일어나도록#################

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data).reshape((height, width))

        self.obstacles = []
        for i in range(height):
            for j in range(width):
                volume = data[i, j]
                if volume > 50:
                    obs_x = origin_x + j * resolution
                    obs_y = origin_y + i * resolution
                    self.obstacles.append((obs_x, obs_y, volume))

        min_force = np.inf
        min_x, min_y = None, None
        # min_k, min_l = None, None

        for k in range(height):
            for l in range(width):
                if data[k, l] == 0:
                    x = origin_x + l * resolution
                    y = origin_y + k * resolution

                    force_sum = self.force_calculator(x, y, self.obstacles)
                    if min_force > force_sum:
                        min_force = force_sum
                        min_x = x
                        min_y = y
        if min_x is not None and min_y is not None:
            self.get_logger().info(f'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            self.get_logger().info(f'min_x = {min_x}, min_y = {min_y}')

            point_msg = Point()
            point_msg.x = float(min_x)
            point_msg.y = float(min_y)
            point_msg.z = 0.0

            self.publisher.publish(point_msg)
            self.get_logger().info(f'goal_x = {point_msg.x}, goal_y = {point_msg.y}')
        

    def force_calculator(self, x, y, obstacles):
        k_rep = 100.0
        d_safe = 200.0
        
        Fx_sum = 0.0
        Fy_sum = 0.0

        for obs_x, obs_y, volume in obstacles:
            x_d = x - obs_x
            y_d = y - obs_y
            distance = np.sqrt(x_d**2 + y_d**2)

            if distance == 0 or distance > d_safe:
                continue

            k_rep_volume = k_rep * (volume / 100.0)
            force_size = k_rep_volume * ((1 / distance) - (1 / d_safe)) * (1 / distance ** 2)

            Fx = force_size * (x_d / distance)
            Fy = force_size * (y_d / distance)

            Fx_sum += Fx
            Fy_sum += Fy

        force = np.sqrt(Fx_sum ** 2 + Fy_sum ** 2)

        return force


def main(args=None):
    rclpy.init(args=args)
    node = RepulsiveForce()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()