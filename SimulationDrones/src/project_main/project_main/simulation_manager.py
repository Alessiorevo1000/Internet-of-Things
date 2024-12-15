import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import math_utils

WORLD_NAME = "iot_project_world"

NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
NUMBER_OF_DRONES = int(sys.argv[3])
HOVERING_HEIGHT = 15
SENSORS_RANGE = 20
DRONE_TO_BALLOON_RANGE = 30

class SimulationManager(Node):

    def __init__(self):
        super().__init__('simulation_manager')

        # Initialize positions dictionaries
        self.sensor_positions = {}
        self.balloon_positions = {}
        self.drone_positions = {}

        # Variable to be set to True after 10 seconds or Drone 0 reaches altitude 15
        self.ten_seconds_passed = False
        self.drone_0_at_altitude = False
        self.start_time = self.get_clock().now()

        # Create a timer to check if 10 seconds have passed
        self.create_timer(1.0, self.check_time)

        # Initialize subscriptions for sensors
        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda odometry_msg, sensor_id=i: self.store_sensor_position(sensor_id, odometry_msg),
                10
            )
            self.create_subscription(
                String,
                f'Sensor_{i}/tx_data',
                lambda string_msg, sensor_id=i: self.forward_data_baloon(sensor_id, string_msg),
                10
            )
            self.create_subscription(
                String,
                f'Sensor_{i}/tx_data',
                lambda string_msg, sensor_id=i: self.forward_data_drone(sensor_id, string_msg),
                10
            )

        # Initialize subscriptions and publishers for balloons
        self.balloons_rx = {}
        for i in range(NUMBER_OF_BALLOONS):
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id=i: self.store_balloon_position(balloon_id, odometry_msg),
                10
            )
            self.balloons_rx[i] = self.create_publisher(
                String,
                f'Balloon_{i}/rx_data',
                10
            )

        # Initialize subscriptions and publishers for drones
        self.drones_rx = {}
        for i in range(NUMBER_OF_DRONES):
            self.create_subscription(
                Odometry,
                f'X3_{i}/odometry',
                lambda odometry_msg, drone_id=i: self.store_drone_position(drone_id, odometry_msg),
                10
            )
            self.drones_rx[i] = self.create_publisher(
                String,
                f'X3_{i}/rx_data',
                10
            )
            self.create_subscription(
                String,
                f'X3_{i}/drone_to_balloon',
                lambda msg, drone_id=i: self.handle_drone_to_balloon(drone_id, msg),
                10
            )

    def check_time(self):
        """
        Check if 10 seconds have passed since the start of the simulation or if Drone 0 is at altitude 15.
        """
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        if elapsed_time >= 10 and not self.ten_seconds_passed:

        # Check if Drone 0 is at altitude 15
            if self.drone_0_at_altitude and not self.ten_seconds_passed:
                self.ten_seconds_passed = True
                self.get_logger().info("Drone 0 is at the correct altitude of 15 units.")

    def store_sensor_position(self, sensor_id, position: Odometry):
        self.sensor_positions[sensor_id] = position.pose.pose.position

    def store_balloon_position(self, balloon_id, position: Odometry):
        self.balloon_positions[balloon_id] = position.pose.pose.position

    def store_drone_position(self, drone_id, position: Odometry):
        self.drone_positions[drone_id] = position.pose.pose.position
        
        # Check if Drone 0 is at the correct altitude
        if drone_id == 0:
            z_coordinate = position.pose.pose.position.z
            if abs(z_coordinate - HOVERING_HEIGHT) < 0.1:  # Tolerance of 0.1 for floating-point comparison
                self.drone_0_at_altitude = True

    def forward_data_baloon(self, sensor_id, msg: String):
        for i in range(NUMBER_OF_BALLOONS):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                if math_utils.point_distance(self.sensor_positions[sensor_id], self.balloon_positions[i]) < SENSORS_RANGE:
                    self.balloons_rx[i].publish(msg)

    def forward_data_drone(self, sensor_id, msg: String):
        if self.ten_seconds_passed:
            for i in range(NUMBER_OF_DRONES):
                if sensor_id in self.sensor_positions and i in self.drone_positions:
                    if math_utils.point_distance(self.sensor_positions[sensor_id], self.drone_positions[i]) < SENSORS_RANGE:
                        self.drones_rx[i].publish(msg)

    def handle_drone_to_balloon(self, drone_id, msg: String):
        """
        Handle incoming messages from drones and forward them to nearby balloons.
        """
        if drone_id in self.drone_positions:
            drone_pos = self.drone_positions[drone_id]
            for balloon_id in self.balloon_positions:
                if math_utils.point_distance(drone_pos, self.balloon_positions[balloon_id]) < DRONE_TO_BALLOON_RANGE:
                    self.balloons_rx[balloon_id].publish(msg)

def main():
    rclpy.init()
    simulation_manager = SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulation_manager)

    executor.spin()

    executor.shutdown()
    simulation_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
