# Internet-of-Things

## Overview  

This project involves optimizing **drones** and **balloons** for sensor detection in an IoT simulation. The goal was to minimize time and data exchange while ensuring continuous and up-to-date communication between components.  

Key challenges addressed:  
1. **Range Problem**: Balloons initially failed to detect sensor values effectively.  
2. **Drone Path Optimization**: Prevented overlapping paths to reduce collisions and data transmission delays.  

## Implementation  

1. **Drone and Balloon Positioning**:  
   - Deployed drones on the ground and balloons in a circular formation around the base station to minimize movement.  
   - Introduced varying flight heights to avoid collisions.  

2. **Distance Matrix**:  
   - Created a matrix to calculate distances between drones and sensors, optimizing drone paths.  
   - Sensors were assigned to drones based on proximity and even distribution.  

3. **Detection Mechanism**:  
   - Drones detect sensors when within a defined range, avoiding the need for direct positioning.  
   - Drones continuously patrol and update sensor data.  

## Features  

1. **Key Variables**:  
   - `SENSOR_RADIUS`: Defines the placement radius for sensors (25 meters).  
   - `DISTANCE_THRESHOLD`: Sets the range for data processing (30 units).  
   - `assignments`: Tracks sensor-to-drone assignments.  
   - `latest_sensor_data`: Maintains the most recent sensor readings.  

2. **Important Functions**:  
   - **`calculate_and_print_distance_matrix()`**: Logs distances between drones and sensors for optimized assignment.  
   - **`assign_sensors_to_drones()`**: Ensures balanced sensor distribution among drones.  
   - **`process_data()`**: Updates sensor data based on proximity.  
   - **`print_latest_sensor_data()`**: Publishes aggregated sensor data.  

3. **ROS2 Communication**:  
   - Publishes data between drones, balloons, and the base station via dedicated topics.  

## Test Results  

1. **Test with 10 Sensors and 3 Drones**:  
   - Time range: 59 seconds to 2 minutes 20 seconds.  

2. **Test with 20 Sensors and 5 Drones**:  
   - Time range: 3 minutes 15 seconds to 3 minutes 24 seconds.  

## Conclusion  

By optimizing drone and balloon positioning, implementing height variations, and using a distance matrix for dynamic sensor assignment, the system achieved efficient sensor detection while reducing collisions and flight time. This approach ensures real-time data updates with improved overall system management.
