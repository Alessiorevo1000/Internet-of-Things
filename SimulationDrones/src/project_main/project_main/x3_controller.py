import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

import math_utils
from project_interfaces.action import PatrolDrone  # Update to use PatrolDrone

MIN_ALTITUDE_TO_PERFORM_PATROL = 15



class X3Controller(Node):
    def __init__(self):
        super().__init__("x3_controller")

        # Cache per dati temporanei (lista di stringhe "tipo_valore")
        self.cache = []
        self.entry = []

        # Posizione e orientamento corrente
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.yaw = 0

        # Messaggio per fermare il drone
        self.stop_msg = Twist()

        # Publisher per i comandi di velocità
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Publisher per i dati della cache
        self.data_publisher = self.create_publisher(
            String,
            'drone_to_balloon',
            10
        )

        # Publisher per i dati della cache
        self.data_base_publisher = self.create_publisher(
            String,
            'drone_to_base',
            10
        )

        # Subscriber per l'odometria
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )

        # Subscriber per i dati ricevuti
        self.rx_data = self.create_subscription(
            String,
            'rx_data',
            self.rx_callback,
            10
        )

        # Server di azione per la pattuglia
        self.patrol_action_server = ActionServer(
            self,
            PatrolDrone,
            'patrol',
            self.execute_patrol_action
        )

        # Timer per pubblicare i dati della cache ogni secondo
        self.timer = self.create_timer(1.0, self.publish_cache_data)

    def rx_callback(self, msg: String):
        """
        Callback per la ricezione dei dati.
        """
        self.entry.append(msg.data)
        for e in self.entry:
            parts = e.split('_')
            sensor_type = parts[0]
            sensor_value = parts[1]
            new_entry = f"{sensor_type}_{sensor_value}"

            # Trova se esiste un elemento che inizia con 'sensor_type_'
            matching_entry = next((item for item in self.cache if item.startswith(sensor_type + "_")), None)

            if matching_entry:
                i = self.cache.index(matching_entry)
                self.cache[i] = new_entry
            else:
                self.cache.append(new_entry)




        
        # Verifica se il tipo di sensore è già presente nella cache

        #self.get_logger().info(f"CACHE AGGIORNATA DEL DRONE: {self.cache}")

    def store_position(self, odometry_msg: Odometry):
        """
        Memorizza la posizione e l'orientamento attuali del drone.
        """
        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )

    def execute_patrol_action(self, goal_handle: ServerGoalHandle):
        """
        Esegue l'azione di pattuglia.
        """
        command_goal: PatrolDrone.Goal = goal_handle.request

        self.fly_to_altitude(MIN_ALTITUDE_TO_PERFORM_PATROL)

        targets_patrolled = 0
        #self.get_logger().info(f"command_goal.targets: {command_goal.targets}")
        for target in command_goal.targets:
            self.rotate_to_target(target)
            self.move_to_target(target)

            targets_patrolled += 1
            self.get_logger().info(f"target controllati: {targets_patrolled}\n")

        goal_handle.succeed()

        result = PatrolDrone.Result()
        result.result = "Movement completed"

        return result

    def fly_to_altitude(self, altitude):
        """
        Raggiunge l'altitudine desiderata.
        """
        move_up = Twist()
        move_up.linear = Vector3(x=0.0, y=0.0, z=1.0)
        move_up.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(move_up)

        while self.position.z < altitude:
            time.sleep(0.1)

        stop_mov = Twist()
        stop_mov.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_mov.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_mov)

    def rotate_to_target(self, target, eps=0.5):
        """
        Ruota il drone verso il target specificato.
        """
        target_angle = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = target_angle - self.yaw

        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi
        rotation_dir = 1 if angle_to_rotate < 0 else -1
        
        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=0.5 * rotation_dir)
        self.cmd_vel_publisher.publish(move_msg)

        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.yaw

        stop_msg = Twist()
        stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_msg)

    def move_to_target(self, target, eps=13.0, angle_eps=0.02):
        """
        Muove il drone verso il target specificato.
        """
        distance = math_utils.point_distance(self.position, target)

        while distance > eps:
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0]
            twist_msg.linear.z = mv[1]

            target_angle = math_utils.angle_between_points(self.position, target)
            if not (target_angle - angle_eps < self.yaw < target_angle + angle_eps):
                angle_diff = (self.yaw - target_angle)
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))

            self.cmd_vel_publisher.publish(twist_msg)

            distance = math_utils.point_distance(self.position, target)

        self.cmd_vel_publisher.publish(self.stop_msg)

    def publish_cache_data(self):
        """
        Pubblica i dati della cache sul topic 'drone_to_balloon'.
        """
        if self.cache:
            cache_data = ','.join(self.cache)
            msg = String()
            msg.data = cache_data
            self.data_publisher.publish(msg)
            self.data_base_publisher.publish(msg)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    x3_controller = X3Controller()
    executor.add_node(x3_controller)

    executor.spin()

    x3_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
