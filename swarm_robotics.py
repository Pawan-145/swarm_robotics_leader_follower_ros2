import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_shape_controller')
        
        # 1. Dictionary defining our shapes (X and Y offsets relative to the leader)
        self.shapes = {
            'LINE': {
                'turtle2': (-1.0, 0.0),
                'turtle3': (-2.0, 0.0),
                'turtle4': (-3.0, 0.0)
            },
            'TRIANGLE': {
                'turtle2': (-1.5, 1.0),
                'turtle3': (-1.5, -1.0),
                'turtle4': (-3.0, 0.0)
            },
            'V_SHAPE': {
                'turtle2': (-1.0, 1.0),
                'turtle3': (-2.0, 2.0),
                'turtle4': (-1.0, -1.0)
            }
        }
        
        self.shape_list = list(self.shapes.keys())
        self.current_shape = 'LINE'
        self.get_logger().info(f"Initial Swarm Shape: {self.current_shape}")

        # 2. Storage for tracking positions
        self.leader_pose = None
        self.follower_poses = {'turtle2': None, 'turtle3': None, 'turtle4': None}
        
        # 3. Create publishers for followers
        self.publishers_map = {
            'turtle2': self.create_publisher(Twist, '/turtle2/cmd_vel', 10),
            'turtle3': self.create_publisher(Twist, '/turtle3/cmd_vel', 10),
            'turtle4': self.create_publisher(Twist, '/turtle4/cmd_vel', 10)
        }

        # 4. Create subscribers
        self.create_subscription(Pose, '/turtle1/pose', self.leader_pose_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', lambda msg: self.follower_pose_callback('turtle2', msg), 10)
        self.create_subscription(Pose, '/turtle3/pose', lambda msg: self.follower_pose_callback('turtle3', msg), 10)
        self.create_subscription(Pose, '/turtle4/pose', lambda msg: self.follower_pose_callback('turtle4', msg), 10)

        # 5. Spawn the followers immediately
        self.spawn_followers()

        # 6. Timers: One for updating movement (fast), one for changing shapes (slow)
        self.create_timer(0.05, self.control_loop)       # 20 Hz control loop
        self.create_timer(10.0, self.change_shape_loop)   # Change shape every 10 seconds

    def spawn_followers(self):
        """Calls the /spawn service to generate 3 extra turtles."""
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim spawn service...')
            
        for name in ['turtle2', 'turtle3', 'turtle4']:
            request = Spawn.Request()
            request.x = random.uniform(2.0, 8.0)
            request.y = random.uniform(2.0, 8.0)
            request.theta = 0.0
            request.name = name
            client.call_async(request)
            self.get_logger().info(f'Spawn request sent for {name}')

    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    def follower_pose_callback(self, name, msg):
        self.follower_poses[name] = msg

    def change_shape_loop(self):
        """Randomly cycles to a different shape command system."""
        available_shapes = [s for s in self.shape_list if s != self.current_shape]
        self.current_shape = random.choice(available_shapes)
        self.get_logger().info(f"--- COMMAND CHANGED! New Shape: {self.current_shape} ---")

    def control_loop(self):
        """Core swarm logic: Calculates paths and drives followers."""
        # Active safety check: Only run if we have received positioning data
        if self.leader_pose is None:
            return
            
        th = self.leader_pose.theta

        for name, publisher in self.publishers_map.items():
            f_pose = self.follower_poses[name]
            if f_pose is None:
                continue  # Skip if this follower hasn't reported its pose yet

            # Get offsets for the active shape command
            offset_x, offset_y = self.shapes[self.current_shape][name]

            # Matrix Rotation: Calculate global target target (X, Y) based on leader's heading
            target_x = self.leader_pose.x + (offset_x * math.cos(th) - offset_y * math.sin(th))
            target_y = self.leader_pose.y + (offset_x * math.sin(th) + offset_y * math.cos(th))

            # Proportional Navigation (P-Controller)
            dx = target_x - f_pose.x
            dy = target_y - f_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - f_pose.theta
            # Normalize angle error between -pi and pi
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            twist = Twist()
            # If it's close enough, stop moving to prevent jittering
            if distance > 0.2:
                twist.linear.x = 2.0 * distance     # Linear Speed proportional to distance
                twist.angular.z = 6.0 * angle_error # Angular Speed proportional to alignment error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()