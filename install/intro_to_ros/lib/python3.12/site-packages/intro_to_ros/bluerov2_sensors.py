#!/usr/bin/env python3
#!/usr/bin/env python3
# Shebang line to indicate the script should be run with Python 3

import rclpy  # Import ROS 2 Python client library for ROS 2 interaction
from rclpy.node import Node  # Import Node class for creating and managing ROS 2 nodes

# Import specific message types for geometry and sensor data handling

from rclpy.qos import (  # Import QoS (Quality of Service) classes for managing data flow
    QoSProfile,  # Import QoSProfile for defining QoS policies
    QoSHistoryPolicy,  # Import QoSHistoryPolicy for message history management
    QoSReliabilityPolicy,  # Import QoSReliabilityPolicy for message delivery guarantees
    QoSDurabilityPolicy,  # Import QoSDurabilityPolicy for message persistence settings
)

from sensor_msgs.msg import (  # Import sensor message types for sensor data handling
    BatteryState,  # Import BatteryState message type for battery information
    Imu,  # Import Imu message type for Inertial Measurement Unit data
    FluidPressure,  # Import FluidPressure message type for pressure data
)  # Grouped import for sensor message types

class BlueROV2SensorsSubscriber(Node):

    def __init__(self):
        """
        Initializes the node. Creates new data attributes for subscription messages received. Subscribes to topics related to the 
        battery, IMU, and pressure. Also creates a timer to use a method to check battery voltage for unsafe levels every 5.0 seconds.
        """
        super().__init__("bluerov2_sensor_subscriber") # Call the superclass constructor with node name
        qos_profile = QoSProfile( # QosProfile used for each subscription, listed here for the sake of repetition and "cleanness" (lines 21-26)
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )


        # Initialize sensor data variables to store incoming data
        self.battery_message = BatteryState() # Battery messages
        self.imu_message = Imu() # IMU messages
        self.diff_pressure_message = FluidPressure() # Differential pressure messages
        self.static_pressure_message = FluidPressure() # Static pressure messages

        self.battery = self.create_subscription( # Battery topic subscription (lines 35-40)
            BatteryState, # type of message
            "/mavros/battery", # topic address
            self.callbackBattery, # callback function
            qos_profile # We love Quality of Service

        )

        self.imu = self.create_subscription( # IMU data topic subscription (lines 43-48)
            Imu,
            "/mavros/imu/data",
            self.callbackImu,
            qos_profile
        )

        self.static_pressure = self.create_subscription( # Static pressure topic subscription (lines 57-62)
            FluidPressure,
            "/mavros/imu/static_pressure",
            self.static_pressure_callback,
            qos_profile,
        )

        self.battery_check_timer = self.create_timer( # Timer checking for battery unsafe levels
            5.0, # Time interval between the executions of the below method
            self.checkBattery # Actual method that checks the unsafe battery level
        )

        self.battery # actually creates the subscriptions + timer (lines 69-73)
        self.imu
        self.static_pressure
        self.battery_check_timer 
        self.get_logger().info("starting subscriber node") # Reports to the terminal that the node is starting to receive messages.

    def callbackBattery(self, msg): #callback function for battery
        """
        Updates the battery data to the latest message. Reports back to the terminal the battery voltage. 'msg' is the latest message.
        """
        self.battery_message = msg # updates
        self.get_logger().info(f"\n\tVoltage: {self.battery_message.voltage}") # reports

    def callbackImu(self, msg): # callback function for IMU
        """
        Updates the IMU data to the latest message. Reports back to the terminal the full orientation of the ROV. 'msg' is the latest message.
        """
        self.imu_message = msg # updates and reports in the next line
        self.get_logger().info(f"IMU\n\tIMU x: {self.imu_message.orientation.x}\tIMU y: {self.imu_message.orientation.y}\tIMU z: {self.imu_message.orientation.z}\tIMU w: {self.imu_message.orientation.w}")

    def static_pressure_callback(self, msg): # callback function for static pressure
        """
        Updates the static pressure data to the latest nessage. Reports back to the terminal the fluid pressure experienced and depth by the ROV.
        'msg' is the latest message.
        """
        self.static_pressure_message = msg # updates
        self.get_logger().info(f"\nStatic Pressure: {self.static_pressure_message.fluid_pressure}\nDepth: {self.calculate_depth()}") # reports

    def calculate_depth(self):
        """
        Calculates the depth of the ROV based on its static pressure that is experienced. SI units are expected. Fluid is water on Earth.
        Pressure = fluid_density (1000) * gravitational_acceleration (9.81) * depth
        """
        return ((self.static_pressure_message.fluid_pressure - 101325) / (9.81 * 1000)) # 101325 pascals subtracted to not account for air.

    def checkBattery(self):
        """
        This is the function that is run every 5 seconds to check for unsafe battery voltage levels.
        """
        if self.battery_message.voltage < 12.0: # 3.0 x 4 (unsafe level battery voltage * 4)
            print ("\nBattery voltage is below safe level, please take advisory measures.") # I'm in danger.


def main(args=None): # main function that is run as a result of the console script 
        rclpy.init(args=args) # Starts the ROS2 Python 3 client
        node = BlueROV2SensorsSubscriber() # Creates an instance of the BlueROV2SensorsSubscriber class

        try:
            rclpy.spin(node) # keeps node running until there is an exception
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received, shutting down...") # 99.9% of the time, you clicked ^C
        finally:
            node.destroy_node() # destroys the node at the end of the program's lifespan
            if rclpy.ok():
                rclpy.shutdown() # closes the ROS2 Python3 client if it is still active 

if __name__=="__main__": 
    main()
