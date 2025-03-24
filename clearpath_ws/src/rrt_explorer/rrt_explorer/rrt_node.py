import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random

class Vertex:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent

class RRTConnectPlanner:
    def __init__(self, lidar_ranges, max_distance=2.0, safe_distance=0.7, sample_area=4.0):
        self.lidar_ranges = lidar_ranges
        self.max_distance = max_distance
        self.safe_distance = safe_distance
        self.sample_area = sample_area

    def steer(self, start, goal):
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        distance = math.sqrt(dx**2 + dy**2)
        if distance < self.max_distance:
            return goal
        scale = self.max_distance / distance
        return (start[0] + dx * scale, start[1] + dy * scale)

    def is_collision(self, point):
        x, y = point
        if x == 0 and y == 0:
            return True
        lidar_index = int((math.atan2(y, x) + math.pi) / (2 * math.pi) * len(self.lidar_ranges))
        lidar_index = max(0, min(len(self.lidar_ranges) - 1, lidar_index))
        distance_to_point = math.sqrt(x**2 + y**2)
        return self.lidar_ranges[lidar_index] < (distance_to_point + self.safe_distance)

    def plan(self, start, goal):
        tree_a = [Vertex(start)]
        tree_b = [Vertex(goal)]

        for _ in range(1000):
            rand_point = self.generate_safe_random_point_towards(goal)
            nearest_a = min(tree_a, key=lambda v: math.dist(v.pos, rand_point))
            new_point_a = self.steer(nearest_a.pos, rand_point)

            if not self.is_collision(new_point_a):
                new_vertex_a = Vertex(new_point_a, nearest_a)
                tree_a.append(new_vertex_a)

                nearest_b = min(tree_b, key=lambda v: math.dist(v.pos, new_point_a))
                new_point_b = self.steer(nearest_b.pos, new_point_a)

                if not self.is_collision(new_point_b):
                    new_vertex_b = Vertex(new_point_b, nearest_b)
                    tree_b.append(new_vertex_b)

                    if math.dist(new_vertex_a.pos, new_vertex_b.pos) < self.max_distance:
                        path = []
                        v = new_vertex_a
                        while v:
                            path.append(v.pos)
                            v = v.parent
                        path.reverse()
                        v = new_vertex_b
                        while v:
                            path.append(v.pos)
                            v = v.parent
                        return path
        return None

    def generate_safe_random_point(self):
        for _ in range(50):
            x = random.uniform(-self.sample_area, self.sample_area)
            y = random.uniform(-self.sample_area, self.sample_area)
            if not self.is_collision((x, y)):
                return (x, y)
        return (random.uniform(-self.sample_area, self.sample_area), random.uniform(-self.sample_area, self.sample_area))

    def generate_safe_random_point_towards(self, goal):
        gx, gy = goal
        for _ in range(50):
            angle_to_goal = math.atan2(gy, gx)
            deviation = random.uniform(-math.pi / 6, math.pi / 6)
            angle = angle_to_goal + deviation
            distance = random.uniform(1.0, self.sample_area)
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if not self.is_collision((x, y)):
                return (x, y)
        return self.generate_safe_random_point()

class ObstacleAvoidAndExploreRRT(Node):
    def __init__(self):
        super().__init__('obstacle_avoid_and_explore_rrt_with_unstuck')

        self.cmd_vel_pub = self.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/a200_0000/sensors/lidar2d_0/scan', self.lidar_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/a200_0000/person_goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/a200_0000/platform/odom', self.odom_callback, 10)

        self.lidar_ranges = []
        self.person_goal = None
        self.visited_cells = set()
        self.grid_size = 3.0

        self.unstuck_mode = False
        self.unstuck_cycles = 0

        self.timer = self.create_timer(1.0, self.explore_or_unstuck)
        self.get_logger().info("Nodo con lógica de desatasco y exploración inteligente iniciado.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cell_x = int(x // self.grid_size)
        cell_y = int(y // self.grid_size)
        self.visited_cells.add((cell_x, cell_y))

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges

        # Si encuentra un obstáculo muy cerca activa modo desbloqueo
        if min(self.lidar_ranges) < 0.15 and not self.unstuck_mode:
            self.get_logger().warn("Obstáculo detectado demasiado cerca. Activando modo de desatasco.")
            self.unstuck_mode = True
            self.unstuck_cycles = 10

    def goal_callback(self, msg):
        self.person_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Persona detectada en {self.person_goal}, planificando ruta.")
        self.navigate_to_goal_with_rrt()

    def explore_or_unstuck(self):
        if self.unstuck_mode:
            twist = Twist()
            twist.linear.x = -0.2  # retroceder
            twist.angular.z = random.uniform(-1.0, 1.0)  # giro aleatorio
            self.cmd_vel_pub.publish(twist)
            self.unstuck_cycles -= 1

            if self.unstuck_cycles <= 0:
                self.unstuck_mode = False
                self.get_logger().info("Desatasco completado. Reanudando exploración.")
            return

        self.explore_randomly_with_rrt()

    def explore_randomly_with_rrt(self):
        if self.person_goal or not self.lidar_ranges:
            return

        planner = RRTConnectPlanner(self.lidar_ranges, max_distance=1.0, safe_distance=0.9, sample_area=5.0)

        for _ in range(50):
            random_goal = planner.generate_safe_random_point()
            cell_x = int(random_goal[0] // self.grid_size)
            cell_y = int(random_goal[1] // self.grid_size)
            if (cell_x, cell_y) not in self.visited_cells:
                path = planner.plan((0, 0), random_goal)

                if path:
                    for waypoint in path:
                        twist = Twist()
                        angle = math.atan2(waypoint[1], waypoint[0])
                        distance = math.sqrt(waypoint[0]**2 + waypoint[1]**2)

                        twist.linear.x = min(0.5, distance * 0.5)
                        twist.angular.z = angle * 0.8
                        self.cmd_vel_pub.publish(twist)
                    break

    def navigate_to_goal_with_rrt(self):
        planner = RRTConnectPlanner(self.lidar_ranges, max_distance=2.0, safe_distance=0.5, sample_area=5.0)
        path = planner.plan((0, 0), self.person_goal)

        if path:
            for waypoint in path:
                twist = Twist()
                angle = math.atan2(waypoint[1], waypoint[0])
                distance = math.sqrt(waypoint[0]**2 + waypoint[1]**2)

                twist.linear.x = min(0.5, distance * 0.5)
                twist.angular.z = angle * 0.9
                self.cmd_vel_pub.publish(twist)

            self.get_logger().info("Meta alcanzada con RRT-Connect.")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidAndExploreRRT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

