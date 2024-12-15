import sys
from math import cos, sin, pi
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from project_main.sim_utils import spawn_sdf
import random
from random import randint

WORLD_NAME = "iot_project_world"
NUMBER_OF_BALLOONS = 3
NUMBER_OF_SENSORS = 10
NUMBER_OF_DRONES = 3
SENSOR_RADIUS = 25  # circle range used to set the position of sensors
HOVERING_HEIGHT = 15

#-----------------------------------------------------------------------------------------------
# function used to generate positions on the circle
#-----------------------------------------------------------------------------------------------
def generate_circle_positions(num_positions, radius, altezza, base_position=(0, 0)):
    positions = []
    if num_positions <= 0:
        return positions

    angle_increment = 2 * pi / num_positions
    for i in range(num_positions):
        angle = i * angle_increment
        x = base_position[0] + radius * cos(angle)
        y = base_position[1] + radius * sin(angle)
        positions.append((x, y, altezza))
    return positions



def generate_launch_description():
    targets_to_spawn = []

    try:
        #------------------- Launch Gazebo here, with the iot_project_world world --------------------
        targets_to_spawn.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros_gz_sim'),
                        'launch/',
                        'gz_sim.launch.py',
                    ])
                ]),
                launch_arguments={
                    'gz_args': f"resources/{WORLD_NAME}.sdf"
                }.items()
            )
        )

        #-------------------------------- Bridge for the world control -------------------------------
        targets_to_spawn.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/world/{WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld"
                ]
            )
        )

        #------------------------------ Bridge for the simulation clock ------------------------------
        targets_to_spawn.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/world/{WORLD_NAME}/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
                ]
            )
        )

        # base station position
        base_station_pos = (-5, -5, 0)
        # converts the tuple into string "x,y,z"
        base_station_pos_str = f"{base_station_pos[0]},{base_station_pos[1]},{base_station_pos[2]}"


        #  generate positions ballons
   
        balloon_positions = generate_circle_positions(NUMBER_OF_BALLOONS, SENSOR_RADIUS, HOVERING_HEIGHT+2, base_position=base_station_pos)

        #-------------------------- Spawn balloons and bridge their topics ---------------------------
        for i, pos in enumerate(balloon_positions):
            targets_to_spawn.append(spawn_sdf("resources/balloon/balloon.sdf", id=i, pos=pos))

            # Spawn bridge for cmd_vel and odometry for each of the spawned objects
            targets_to_spawn.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    arguments=[
                        f"/Balloon_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
                    ]
                )
            )
            targets_to_spawn.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    arguments=[
                        f"/Balloon_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                    ]
                )
            )

            targets_to_spawn.append(
                Node(
                    package="project_main",
                    executable="balloon_controller",
                    namespace=f"Balloon_{i}",
                    name=f"BalloonController{i}"
                )
            )

        #-------------------------- Spawn sensors and bridge their topics ---------------------------
 

        for i in range(NUMBER_OF_SENSORS):
            targets_to_spawn.append(spawn_sdf("resources/sensor/sensor.sdf", id=i, pos = (randint(-40, 40), randint(-40, 40), 0)))


            targets_to_spawn.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    arguments=[
                        f"/Sensor_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                    ]
                )
            )

            targets_to_spawn.append(
                Node(
                    package="project_main",
                    executable="sensor_controller",
                    namespace=f"Sensor_{i}",
                    parameters=[
                        {'id': i}
                    ]
                )
            )

        #------------------------------------ Spawn base station -------------------------------------
        targets_to_spawn.append(spawn_sdf("resources/base_station/base_station.sdf", pos=base_station_pos))
        targets_to_spawn.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    f"/base_station/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                ]
            )
        )

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="simulation_manager",
                arguments=[
                    f"{NUMBER_OF_BALLOONS}",
                    f"{NUMBER_OF_SENSORS}",
                    f"{NUMBER_OF_DRONES}"
                ]
            )
        )

        targets_to_spawn.append(
            Node(
                package="project_main",
                executable="fleet_coordinator",
                arguments=[
                    f"{NUMBER_OF_BALLOONS}",
                    f"{NUMBER_OF_SENSORS}",
                    f"{NUMBER_OF_DRONES}"
                ]
            )
        )

        targets_to_spawn.append(
           Node(
            package='project_main',
            executable='base_controller',
            name='base_controller',
            output='screen',
             arguments=[
                    f"{NUMBER_OF_BALLOONS}",
                    f"{NUMBER_OF_DRONES}",
                    base_station_pos_str
                ]
             ),
        )



        # Generate drone positions 
        drone_positions = generate_circle_positions(NUMBER_OF_DRONES, 20, 0, base_position=base_station_pos)

        #-------------------------- Spawn balloons and bridge their topics ---------------------------
        for i, pos in enumerate(drone_positions):
            targets_to_spawn.append(spawn_sdf("resources/x3/x3.sdf", id=i, pos=pos))


            # Spawn bridge for cmd_vel and odometry for each of the spawned object
            targets_to_spawn.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                f"/X3_{i}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
                ]
            )
            )
            targets_to_spawn.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                f"/X3_{i}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
                ]
            )
            )

            targets_to_spawn.append(
                Node(
                    package="project_main",
                    executable="x3_controller",
                    namespace=f"X3_{i}",
                    name=f"X3_controller{i}",
                )
            )







    except Exception as e:
        print(f"Error in generating launch description: {e}", file=sys.stderr)

    return LaunchDescription(targets_to_spawn)
