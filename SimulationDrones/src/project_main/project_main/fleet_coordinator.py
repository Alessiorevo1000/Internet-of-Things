import sys
from time import sleep
from threading import Thread
from enum import Enum
import random
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import math_utils

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol
from project_interfaces.action import PatrolDrone  # Import the PatrolDrone action for drones

from std_msgs.msg import String
import json

# Initialize the number of drones from command line arguments
NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
NUMBER_OF_DRONES = int(sys.argv[3])
HOVERING_HEIGHT = 15.0

class FleetCoordinator(Node):
    """
    Fleet Coordinator class, used to manage the whole fleet of Balloons and Drones.
    This is where most of the tasks should be submitted by default.
    """

    def __init__(self):
        super().__init__('fleet_coordinator')

        self.balloon_positions = {}  # Initialize balloon positions
        self.sensor_positions = {}   # Initialize sensor positions
        self.drone_positions = {}    # Initialize drone positions
        self.balloon_action_clients = {}  # Initialize action clients for balloons
        self.drone_action_clients = {}    # Initialize action clients for drones
        self.balloon_states = {}  # Initialize balloon states
        self.drone_states = {}    # Initialize drone states

        self.sensor_list = []
        self.c=0

        self.latest_sensor_data = {}
        self.assignments = {}  # To store sensor assignments

        # Initialize action clients and states for balloons
        for i in range(NUMBER_OF_BALLOONS):
            self.balloon_action_clients[i] = ActionClient(
                self,
                Patrol,
                f'/Balloon_{i}/patrol'
            )
            self.balloon_states[i] = BalloonState.LANDED

            # Create a subscription for each balloon's odometry
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',  # Assuming this is the topic name for balloons
                lambda msg, id=i: self.store_balloon_position(id, msg),
                10
            )

        # Initialize action clients and states for drones
        for i in range(NUMBER_OF_DRONES):
            self.drone_action_clients[i] = ActionClient(
                self,
                PatrolDrone,
                f'/X3_{i}/patrol'
            )
            self.drone_states[i] = DroneState.LANDED

            # Create a subscription for each drone's odometry
            self.create_subscription(
                Odometry,
                f'X3_{i}/odometry',  # Assuming this is the topic name for drones
                lambda msg, id=i: self.store_drone_position(id, msg),
                10
            )

        # Create subscriptions for sensors
        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda msg, id=i: self.store_sensor_position(id, msg),
                10
            )




        # Inizializza altre variabili/membri della classe
        self.latest_sensor_data = {}



    def patrol_targets(self):
        """
        Method used to keep the fleet of Balloons and Drones constantly patrolling the set of targets.
        When a patrolling task has been completed, a new one with the same targets is given again.
        """
        def patrol_targets_inner():
            for i in range(NUMBER_OF_BALLOONS):
                # Do not resubmit tasks to already moving balloons
                if self.balloon_states[i] != BalloonState.MOVING:
                    self.submit_task_balloon(i)


            if len(self.sensor_positions)==NUMBER_OF_SENSORS and len(self.drone_positions)==NUMBER_OF_DRONES:

                self.calculate_and_print_distance_matrix()
                
                while True:
                    for i in range(NUMBER_OF_BALLOONS):
                        # Do not resubmit tasks to already moving balloons
                        if self.balloon_states[i] != BalloonState.MOVING:
                            self.submit_task_balloon(i)

                    for i in range(NUMBER_OF_DRONES):
                        # Do not resubmit tasks to already moving drones
                        if self.drone_states[i] != DroneState.MOVING:
                            self.submit_task_drone(i)
        sleep(5.0) 
        # Start this function in another thread
        Thread(target=patrol_targets_inner).start()
    

    def calculate_and_print_distance_matrix(self):
        """
        Calculate and print the distance matrix between all sensors and drones.
        """
        # Create the distance matrix
        sensor_ids = list(self.sensor_positions.keys())
        distance_matrix = [[0.0] * NUMBER_OF_SENSORS for _ in range(NUMBER_OF_DRONES)]

        # Calculate distances between drones and sensors
        for i in range(NUMBER_OF_DRONES):
            for j in range(NUMBER_OF_SENSORS):
                distance = math_utils.point_distance(self.drone_positions[i], self.sensor_positions[j])
                distance_matrix[i][j] = distance

        # Print the distance matrix
        header = '     ' + ' '.join([f'{i:5}' for i in range(NUMBER_OF_SENSORS)])  # Header with sensor indices
        self.get_logger().info(header)

        for i, row in enumerate(distance_matrix):
            row_str = f'{i:3} ' + ' '.join([f'{elem:5.1f}' for elem in row])  # Add row index and format each element
            self.get_logger().info(row_str)

        # Assign sensors to drones
        self.assignments = self.assign_sensors_to_drones(distance_matrix)




    def assign_sensors_to_drones(self, distance_matrix):
        """
        Assign sensors to drones based on the distance matrix.
        Each drone is assigned a balanced number of sensors, prioritizing closer sensors.
        """
        # Number of sensors per drone
        sensors_per_drone = math.ceil(NUMBER_OF_SENSORS / NUMBER_OF_DRONES)
        
        # Initialize the assignments
        assignments = {i: [] for i in range(NUMBER_OF_DRONES)}
        sensor_assigned = set()

        # Iterate until all sensors are assigned
        while len(sensor_assigned) < NUMBER_OF_SENSORS:
            # For each drone, find the closest unassigned sensor
            for drone_id in range(NUMBER_OF_DRONES):
                if len(assignments[drone_id]) < sensors_per_drone:
                    # Find the closest sensor for this drone
                    min_distance = float('inf')
                    best_sensor = None
                    for sensor_id in range(NUMBER_OF_SENSORS):
                        if sensor_id not in sensor_assigned:
                            if distance_matrix[drone_id][sensor_id] < min_distance:
                                min_distance = distance_matrix[drone_id][sensor_id]
                                best_sensor = sensor_id

                    if best_sensor is not None:
                        assignments[drone_id].append(best_sensor)
                        sensor_assigned.add(best_sensor)

        # Log assignments
        #for drone_id, sensors in assignments.items():
        #    self.get_logger().info(f"Drone {drone_id} assigned to Sensors {sensors}")

        # Return assignments for further processing if needed
        return assignments

    def get_assigned_sensors_for_drone(self, drone_id):
        """
        Get the list of sensors assigned to a specific drone.
        """
        # Retrieve the assignments from the stored assignments
        return self.assignments.get(drone_id, [])


    def submit_task_balloon(self, uav_id: int):
        # Wait for the action server to go online and sensors to announce their position
        while not self.balloon_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
            pass

        # Set the Balloon to moving state
        self.balloon_states[uav_id] = BalloonState.MOVING

        # Check if the balloon position is available
        if uav_id not in self.balloon_positions:
            self.get_logger().error(f"Position for Balloon {uav_id} not found.")
            self.balloon_states[uav_id] = BalloonState.HOVERING
            return  # Exit if position is not available

        # Get the current position of the balloon
        current_position = self.balloon_positions[uav_id]

        # Generate a random direction and distance
        angle = random.uniform(0, 2 * math.pi)  # Random angle in radians
        distance = 1.0  # Small distance to move

        # Calculate the new position
        new_x = current_position.x + distance * math.cos(angle)
        new_y = current_position.y + distance * math.sin(angle)

        # Define the goal to move the balloon slightly
        move_goal = Patrol.Goal()
        move_goal.targets = [Point(x=new_x, y=new_y, z=HOVERING_HEIGHT)]

        # Define the goal to return to the original position
        return_goal = Patrol.Goal()
        return_goal.targets = [current_position]

        # Submit the movement task and add a callback
        move_future = self.balloon_action_clients[uav_id].send_goal_async(move_goal)
        move_future.add_done_callback(
            lambda future, uav_id=uav_id, return_goal=return_goal: self.movement_done_callback(uav_id, future, return_goal, is_balloon=True)
        )

    #DA MODIFICARE: PRENDERE LA LISTA DEI SENSORI VICINI AI BALLOON, DIVIDERE LE DUE LISTE PER PRIORITA' E INFINE UNIRE LE DUE LISTE

    #DA MODIFICARE: PRENDERE LA LISTA DEI SENSORI VICINI AI BALLOON, DIVIDERE LE DUE LISTE PER PRIORITA' E INFINE UNIRE LE DUE LISTE
    def submit_task_drone(self, uav_id: int):
        #sleep(3)            
        #self.get_logger().info(f'Dati dei sensori aggiornati: {self.latest_sensor_data}')
        #AGGIUNGERE SLEEP DI TRE SECONDI

        # Wait for the action server to go online and sensors to announce their position
        while not self.drone_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
            sleep(3)

        # Set the Drone to moving state
        self.drone_states[uav_id] = DroneState.MOVING

        
        # Get the current position of the drone
        current_position = self.drone_positions[uav_id]

        # Retrieve assigned sensors for this drone
        assigned_sensors = self.get_assigned_sensors_for_drone(uav_id)


        # Generate goals for each assigned sensor
        goals = [Point(x=self.sensor_positions[sensor_id].x, y=self.sensor_positions[sensor_id].y, z=(HOVERING_HEIGHT - (uav_id * 2.5)))
                for sensor_id in assigned_sensors]

        goals1 = [sensor_id
                for sensor_id in assigned_sensors ]
        # Define the goal to move the drone to each sensor
        move_goal = PatrolDrone.Goal()
        move_goal.targets = goals

            
                
        self.get_logger().info(f'{uav_id},Targets : {goals1}\n')



            

        #self.get_logger().info(f"lista targets:: {move_goal.targets}")
        # Define the goal to return to the original position
        move_goal.targets.append(Point(x=float(-5), y=float(-5), z=float(20)))
        move_goal.targets[-1].z = HOVERING_HEIGHT 
        return_goal = PatrolDrone.Goal()  # Use PatrolDrone.Goal for drones
        return_goal.targets = [self.sensor_positions[assigned_sensors[0]]]

        # Submit the movement task and add a callback
        move_future = self.drone_action_clients[uav_id].send_goal_async(move_goal)
        move_future.add_done_callback(
            lambda future, uav_id=uav_id, return_goal=return_goal: self.movement_done_callback(uav_id, future, return_goal, is_balloon=False)
        )

    def movement_done_callback(self, uav_id: int, future, return_goal, is_balloon: bool):
        # Check if the movement goal was accepted and completed
        result = future.result()
        if result.accepted:
            # Log that the movement was accepted
            # Optionally log success
            # Submit the return task
            if is_balloon:
                return_future = self.balloon_action_clients[uav_id].send_goal_async(return_goal)
                return_future.add_done_callback(lambda future: self.patrol_submitted_callback(uav_id, future, is_balloon=True))
            else:

                result_future = result.get_result_async()
                result_future.add_done_callback(lambda future: self.patrol_submitted_callback(uav_id, future, is_balloon=False))
        else:
            # Handle the case where the movement goal was not accepted
            # Optionally log failure
            if is_balloon:
                self.balloon_states[uav_id] = BalloonState.HOVERING
            else:
                self.drone_states[uav_id] = DroneState.HOVERING

    def patrol_submitted_callback(self, uav_id, future, is_balloon: bool):
        # Check if the patrol action was accepted
        goal_handle = future.result()

        if goal_handle is None or not hasattr(goal_handle, 'accepted') or not goal_handle.accepted:
            # Se il goal non è stato accettato
            #self.get_logger().info("Task has been refused by the action server")
            if is_balloon:
                self.balloon_states[uav_id] = BalloonState.HOVERING
            else:
                self.drone_states[uav_id] = DroneState.HOVERING
            return

        # Il task è stato accettato, ora possiamo ottenere il risultato
        result_future = goal_handle.get_result_async()

        # Aggiungi un callback per quando l'azione è completata
        result_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_completed_callback(uav_id, future, is_balloon))


    def patrol_completed_callback(self, uav_id, future, is_balloon: bool):
        # Action completed, UAV can go back to hovering
        if is_balloon:
            self.balloon_states[uav_id] = BalloonState.HOVERING
        else:
            self.drone_states[uav_id] = DroneState.HOVERING

    def store_sensor_position(self, id, msg: Odometry):
        self.sensor_positions[id] = msg.pose.pose.position

    def store_balloon_position(self, id, msg: Odometry):
        self.balloon_positions[id] = msg.pose.pose.position

    def store_drone_position(self, id, msg: Odometry):
        self.drone_positions[id] = msg.pose.pose.position

    #def store_sensor_list(self, msg: String):
    #    self.get_logger().info(f'Ho ricevuto la lista dei nodi: "{msg.data}"')
    
    def sensor_data_callback(self, msg: String):
        """
        Callback per gestire i dati dei sensori ricevuti dal topic 'sensor_data'.
        """
        if self.c<10:
            try:
                # Decodifica i dati JSON ricevuti
                self.c=self.c+1
                sensor_data = json.loads(msg.data)
                self.latest_sensor_data.update(sensor_data)
                for key in self.latest_sensor_data.keys():
                    #self.get_logger().info(f'TEST: {key}')
                    sensor_id = key.split(' ')[2]
                    if int(sensor_id) not in self.sensor_list:
                        self.sensor_list.append(int(sensor_id))
                
                #self.get_logger().info(f'Dati dei sensori aggiornati: {self.sensor_list}')
            except json.JSONDecodeError:
                self.get_logger().error('Errore nella decodifica del messaggio JSON')
    

class BalloonState(Enum):
    LANDED = 1
    HOVERING = 2
    MOVING = 3

class DroneState(Enum):
    LANDED = 1
    HOVERING = 2
    MOVING = 3


def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    fleet_coordinator = FleetCoordinator()

    executor.add_node(fleet_coordinator)
    fleet_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    fleet_coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()