import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_main.math_utils import point_distance
import json

NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_DRONES = int(sys.argv[2])
BASE_POSITION = Point(x=float(-5), y=float(-5), z=float(0))
DISTANCE_THRESHOLD = 30  # Soglia di distanza entro la quale il pallone deve trovarsi

class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')
        self.i=0
        self.j=0
        # Dizionario per memorizzare le posizioni dei palloni
        self.balloon_positions = {}

        self.drone_positions = {}

        # Dizionario per memorizzare l'ultimo dato ricevuto da ogni pallone
        self.latest_sensor_data = {}

        # Publisher per i dati della cache


        # Creazione delle subscription per ogni pallone
        for i in range(NUMBER_OF_BALLOONS):
            # Subscription per ricevere i dati dai palloni
            self.create_subscription(
                String,
                f'Balloon_{i}/tx_data_balloon',
                lambda msg, balloon_id=i: self.process_data(balloon_id, msg),
                10
            )

            # Subscription per ricevere la posizione dei palloni
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda msg, balloon_id=i: self.store_balloon_position(balloon_id, msg),
                10
            )
        
        for i in range(NUMBER_OF_DRONES):
            # Subscription per ricevere i dati dai palloni
            self.create_subscription(
                String,
                f'X3_{i}/drone_to_base',
                lambda msg, drone_id=i: self.process_data_drone(drone_id, msg),
                10
            )

            # Subscription per ricevere la posizione dei palloni
            self.create_subscription(
                Odometry,
                f'X3_{i}/odometry',
                lambda msg, drone_id=i: self.store_drone_position(drone_id, msg),
                10
            )

    def store_balloon_position(self, balloon_id, msg: Odometry):
        # Memorizza la posizione del pallone
        self.balloon_positions[balloon_id] = msg.pose.pose.position
    
    def store_drone_position(self, drone_id, msg: Odometry):
        # Memorizza la posizione del pallone
        self.drone_positions[drone_id] = msg.pose.pose.position

    def process_data(self, balloon_id, msg: String):
        # Controlla se abbiamo la posizione del pallone'''

        if balloon_id in self.balloon_positions:
            distance = point_distance(self.balloon_positions[balloon_id], BASE_POSITION)

            if distance < DISTANCE_THRESHOLD:
                # Solo aggiorna se i dati sono cambiati
                new_data = msg.data.split(',')  # Gestisce più valori separati da virgole
                new_data = [data.strip() for data in new_data]  # Pulisce eventuali spazi extra
                # Aggiorna i dati del sensore, mantenendo solo l'ultimo valore per ogni tipo di sensore
                if balloon_id not in self.latest_sensor_data:
                    self.latest_sensor_data[balloon_id] = {}
                
                for data in new_data:
                    key, value = data.split('_', 1)  # Divide il dato in chiave e valore
                    self.latest_sensor_data[balloon_id][key] = value

                # Stampa la lista aggiornata dei dati dei sensori entro il range
                self.print_latest_sensor_data()
            #else:
                #self.get_logger().info(f"Balloon {balloon_id} is too far: {distance:.2f} units away.")
    
    def process_data_drone(self, drone_id, msg: String):
        # Controlla se abbiamo la posizione del pallone
        if drone_id in self.drone_positions:
            distance = point_distance(self.drone_positions[drone_id], BASE_POSITION)

            if distance < DISTANCE_THRESHOLD:
                # Solo aggiorna se i dati sono cambiati
                new_data = msg.data.split(',')  # Gestisce più valori separati da virgole
                new_data = [data.strip() for data in new_data]  # Pulisce eventuali spazi extra

                # Aggiorna i dati del sensore, mantenendo solo l'ultimo valore per ogni tipo di sensore
                if drone_id not in self.latest_sensor_data:
                    self.latest_sensor_data[drone_id] = {}
                
                for data in new_data:
                    key, value = data.split('_', 1)  # Divide il dato in chiave e valore
                    if key not in self.latest_sensor_data[drone_id]:
                        self.latest_sensor_data[drone_id][key] = value
                    else:
                        # Se esiste, controlla il valore e lo aggiorna solo se il nuovo valore è maggiore o uguale
                        if value >= self.latest_sensor_data[drone_id][key]:
                            self.latest_sensor_data[drone_id][key] = value

                # Stampa la lista aggiornata dei dati dei sensori entro il range
                self.print_latest_sensor_data()
                #self.get_logger().info(f"QUESTO VALORE {msg.data} VIENE PASSATO DAL DRONE {drone_id} A BASE")

    def print_latest_sensor_data(self):
        # Ottieni i dati unici basati sui valori correnti
        all_data = {}
        for balloon_data in self.latest_sensor_data.values():
            all_data.update(balloon_data)
        
        # Stampa i dati
        formatted_data = ', '.join(f'{key}_{value}' for key, value in all_data.items())
        if self.j%15==0:
            lunghezza=len(formatted_data)
            self.get_logger().info(f"Current number of sensor data detected:{len(all_data.items())}")
            self.get_logger().info(f"Current Latest Sensor Data:{formatted_data}\n")
        self.j=self.j+1

        #self.i=self.i+1
def main():
    rclpy.init()
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
