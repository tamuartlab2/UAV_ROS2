import rclpy
from rclpy.node import Node
import time
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Int16, String
from geometry_msgs.msg import Pose
from dronekit import connect, VehicleMode, LocationGlobalRelative
import haversine as hs
# from haversine import Unit
import tf_transformations

takeoffHeight = 5.0    #m
set_ground_speed = 5.0    #m/s
# connection_string = '127.0.0.1:14550'     #when connecting SITL simulation
connection_string = '/dev/ttyACM0'           #Real vehicle

def arm_and_takeoff(targetHeight):
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(0.1)
    print("Look out! Virtual props are spinning!!")
    vehicle.simple_takeoff(targetHeight)
    while True:
        print("Current Altitude: %s"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
    print("Target altitude reached! Takeoff finished!")

#connect vehicle
vehicle = connect(connection_string, wait_ready=False)      #set wait_ready to be False when connecting SITL simulation

vehicle.mode = VehicleMode("GUIDED")
while vehicle.is_armable!=True:
    print("Waiting for vehicle to become armable.")
    time.sleep(1)
print("Vehicle is now armable")
# lat_now = vehicle.location.global_relative_frame.lat
# lon_now = vehicle.location.global_relative_frame.lon
# lat_to_m = hs.haversine((lat_now, lon_now), (lat_now + 0.01, lon_now), unit=Unit.METERS) / 0.01
# lon_to_m = hs.haversine((lat_now, lon_now), (lat_now, lon_now + 0.01), unit=Unit.METERS) / 0.01
vehicle.mode = VehicleMode("LAND")

class UAVPublisher(Node):

    set_pose_lon_p = 0.0
    set_pose_lat_p = 0.0
    set_pose_alt_p = 0.0

    def __init__(self):
        super().__init__('uav_node')

        #declare parameters
        # self.declare_parameter('Max_linear_speed', max_linear_speed_default)
        # self.declare_parameter('Max_angular_speed', max_angular_speed_default)
        # self.declare_parameter('ConnectionString', connection_string_default)

        #get values from parameters
        # self.max_linear_speed = self.get_parameter('Max_linear_speed').get_parameter_value().double_value
        # self.max_angular_speed = self.get_parameter('Max_angular_speed').get_parameter_value().double_value
        # self.connection_string = self.get_parameter('ConnectionString').get_parameter_value().string_value

        #declare topics
        self.pose_pub = self.create_publisher(Pose, '/uav/pose', 10)
        self.mode_pub = self.create_publisher(String, '/uav/mode', 10)
        self.compass_pub = self.create_publisher(Int16, '/uav/compass', 10)
        timer_period = 0.5  # seconds, 2 Hz
        self.set_position_subscription = self.create_subscription(Pose, '/uav/set_pose', self.set_pose_callback, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_pose_callback(self, pose_msg):
        set_pose_lat = pose_msg.position.x     #latitude
        set_pose_lon = pose_msg.position.y     #longitude           
        set_pose_alt = pose_msg.position.z     #altitude
        if vehicle.mode == 'GUIDED':
            if set_pose_lon == 0.0 and set_pose_lat == 0.0 and set_pose_alt == 0.0:     # RTL cmd
                vehicle.mode = VehicleMode("RTL")
                print("Return to LZ.")
            elif  set_pose_lon != self.set_pose_lon_p or set_pose_lat != self.set_pose_lat_p or set_pose_alt != self.set_pose_alt_p:

                #avoid sending target info frequently
                self.set_pose_lon_p = set_pose_lon
                self.set_pose_lat_p = set_pose_lat
                self.set_pose_alt_p = set_pose_alt
                
                #range within 1.0 km, height within 90 m
                goal_distance = hs.haversine((vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon), (set_pose_lat, set_pose_lon))
                print("The target distance: %s km." %goal_distance)
                if goal_distance < 1.0 and set_pose_alt > -90.0 and set_pose_alt < 0.0:     # 1.0km range, 90m ceiling
                    a_location = LocationGlobalRelative(set_pose_lat, set_pose_lon, -set_pose_alt)
                    vehicle.simple_goto(a_location, groundspeed=set_ground_speed)   
                    print("Moving to target location.")   
                else:
                    print('WARNING! The goal point is out of range!')     
        elif vehicle.mode == 'RTL':
            if set_pose_lon == 0.0 and set_pose_lat == 0.0 and set_pose_alt == 1.0:     # Land cmd
                vehicle.mode = VehicleMode("LAND")
                print("Landing on LZ")
        elif vehicle.mode == 'LAND':
            if set_pose_lon == 0.0 and set_pose_lat == 0.0 and set_pose_alt == -5.0:    # takeoff cmd

                #avoid sending target info
                self.set_pose_lon_p = set_pose_lon
                self.set_pose_lat_p = set_pose_lat
                self.set_pose_alt_p = set_pose_alt

                #arm the UAV and take off
                arm_and_takeoff(takeoffHeight)
        else:
            print('WARNING! The vehicle is not in operation mode!')

    def timer_callback(self):
        pose_msg = Pose()
        compass_msg = Int16()
        mode_msg = String()

        pose_msg.position.x = vehicle.location.global_relative_frame.lat
        pose_msg.position.y = vehicle.location.global_relative_frame.lon
        pose_msg.position.z = -vehicle.location.global_relative_frame.alt
        
        q = self.ypr_to_quaternion(vehicle.attitude.yaw, vehicle.attitude.pitch, vehicle.attitude.roll)
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        compass_msg.data = vehicle.heading
        mode_msg.data = str(vehicle.mode)

        self.pose_pub.publish(pose_msg)
        self.compass_pub.publish(compass_msg)
        self.mode_pub.publish(mode_msg)

    def ypr_to_quaternion(self, yaw, pitch, roll):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)

        w = cy * cr * cp + sy * sr * sp
        x = cy * sr * cp - sy * cr * sp
        y = cy * cr * sp + sy * sr * cp
        z = sy * cr * cp - cy * sr * sp

        return [x, y, z, w]


def main(args=None):
    rclpy.init()
    my_pub = UAVPublisher()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
