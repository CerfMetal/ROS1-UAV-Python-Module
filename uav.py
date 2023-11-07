"""
* File: uav.py
* Stack and tested in Gazebo Classic 9 SITL
<node pkg="demo_eng_conf" type="main.py" name="demo_engineering_conference" required="true" output="screen" />
"""
#here you must define the python interpreter to be used by ROS
#! /usr/bin/python3.8

# ROS imports
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import State, Waypoint, WaypointReached, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, WaypointPush
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

# Other imports
from Logger import Logger
from threading import Thread
from time import sleep
from math import cos, pi, atan2

L = Logger(0)

class uav():
    # Constants
    EARTH_RADIUS = 6371000.0  # Earth radius in meters
    waypoints = []
    target_position = GeoPoint()
    RC_Control = OverrideRCIn()

    # ------------ Helper Functions ------------ #
    # Decorator, adds a delay after each function call
    def wait(func):
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)
            self.rate.sleep()
            return result
        return wrapper
    


    def __init__(self, node_name: str, rate_value: int):
        self.node_name = node_name
        self.state = State()
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(rate_value)

    def heart_beat(self):
        while(not rospy.is_shutdown()):
            self.local_pos_pub.publish(self.target_pose)
            sleep(0.01)

    # ------------ Callbacks ------------ #
    # Do not put @wait
    def state_cb(self, msg):
        self.state = msg

    def current_pose_cb(self, data):
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.y
        self.current_z = data.pose.position.z

    # Do not put @wait
    def waypoint_reached_cb(self, data):
        self.waypoint_reached = data.wp_seq

    # Do not put @wait
    def gps_cb(self, data):
        self.current_pose = data
        self.current_latitude = data.latitude
        self.current_longitude = data.longitude
        self.current_altitude = data.altitude

    # ------------ Setup Functions ------------ #
    # Connects to ros master
    @wait
    def connnect(self):
        self.init_ros()

        L.INFO("Connecting to the UAV...")
        while(not rospy.is_shutdown() and not self.state.connected):
            self.rate.sleep()
        L.INFO("Connected")

        self.target_pose = PoseStamped() # Define positioning
        self.heartbeat_thread = Thread(target=self.heart_beat).start() # Start heartbeat thread (used to stay in offboard mode and send target position)

        L.INFO("Starting, please wait...")
        for _ in range(50):
            self.rate.sleep()
        self.set_home()

    # Initializes ROS components
    def init_ros(self):
        """ ---- Subscribers and Publishers ----
            * Subscribers enable the node to receive messages from other nodes
                - They can be setup with a callback function that is called when a message is received
            * Publishers enable the node to send messages to other nodes
        """
        L.INFO("Initializing ROS components...")
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.current_pose_cb)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.waypoint_service = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.waypoint_reached_cb)

        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)

        self.rc_override = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)


    # ------------ Main Functions ------------ #
    # Set the arm state of the UAV
    @wait
    def arm(self, arm: bool):
        L.INFO(f"Setting UAV's arm state to {str(arm)}")
        if (self.state.armed == arm):
            L.INFO(f"UAV's arm state is already {str(arm)}")
            return True
        
        for i in range(30):
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = arm
            self.arming_client.call(arm_cmd).success
            self.rate.sleep()
            if self.state.armed == arm:
                return True
            
        L.ERROR(f"Failed to set UAV's arm state to {str(arm)}")
            
    @wait 
    def emergency_disarm(self):
        L.INFO("Emergency Disarm")
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = False
        return self.arming_client.call(arm_cmd).success
    
    # Set the flight mode of the UAV
    @wait
    def set_mode(self, mode: str):
        L.INFO("Setting mode to " + mode)
        mode_req = SetModeRequest()
        mode_req.custom_mode = mode
        self.set_mode_client.call(mode_req)

    # Set the home location
    @wait
    def set_home(self):
        if not self.state.armed:
            self.home_location = (self.current_latitude, self.current_longitude, self.current_altitude)
            L.INFO(f"Home location set to {self.home_location}")
        else :
            # Prevents accidental home location change
            L.ERROR("Cannot set home location, UAV is armed! Make sure the UAV is landed and disarmed.")

    # Send the UAV back to home
    @wait
    def return_to_home(self):
        if self.state.armed:
            L.INFO("Returning to home location")
            self.set_mode("AUTO.RTL")
        else :
            L.WARNING("Cannot RTL, UAV is not armed!")
    
    # Take off to a given altitude
    @wait
    def take_off(self, altitude):
        if(self.state.armed):
            L.INFO("Taking off")
            self.move(0, 0, altitude)
            return True
        else:
            L.WARNING("UAV is not armed")
            return False
    
    # Set the UAV mode to AUTO.LAND and wait for landing
    # SHOULD NOT be used if obstacle avoidance is required!!!
    @wait
    def auto_land(self):
        if(self.state.armed and not self.state.mode == "AUTO.LAND"):
            L.INFO("Landing...")
            self.set_mode("AUTO.LAND")
            while self.state.mode == "AUTO.LAND":
                self.rate.sleep()
            self.arm(False)
            L.INFO("Landed and disarmed")
            return True
        else:
            L.WARNING("UAV is not armed or is already landed")
            return False

    # Simple displacement of the UAV (relative to the home position)
    # Ex: move(0, 3, 10) will move the UAV; 0 meters in the x, 3 meters in the y, and 10 meters in the z
    @wait
    def move(self, target_x, target_y, target_z):
        # Calculate the yaw angle
        yaw_angle = atan2(target_y - self.current_y, target_x - self.current_x)
        quaternion = quaternion_from_euler(0, 0, yaw_angle)
        self.target_pose.pose.orientation = Quaternion(*quaternion)

        L.INFO(f"Moving to {str(target_x)}, {str(target_y)}, {str(target_z)}")
        self.target_pose.pose.position.x = target_x
        self.target_pose.pose.position.y = target_y
        self.target_pose.pose.position.z = target_z
        # The heartbeart thread publishes the target position

    # Translate gps position into local coordinates and moves the UAV
    def move_to_gps(self, lat, lon, alt):
        target_x, target_y, target_z = self.calculate_distance(self.home_location[0], self.home_location[1], alt, lat, lon, 2*alt) # TODO: fix the altitude implementation
        self.move(target_x, target_y, target_z)

    def calculate_distance(self, ini_lat, ini_lon, ini_alt, tar_lat, tar_lon, tar_alt) :     
        x = (tar_lon- ini_lon) * (pi / 180.0) * cos(self.home_location[0] * (pi / 180.0)) * self.EARTH_RADIUS
        y = (tar_lat - ini_lat) * (pi / 180.0) * self.EARTH_RADIUS
        z = tar_alt - ini_alt # TODO: fix the altitude implementation
        return x, y, z
    
    # Create a waypoint
    def create_waypoint(self, frame, command, is_current, autocontinue, param1, x_lat, y_long, z_alt):
        wp = Waypoint()
        wp.frame = frame  # Global frame
        wp.command = command  # MAV_CMD_NAV_WAYPOINT
        wp.is_current = is_current
        wp.autocontinue = autocontinue
        wp.param1 = param1  # Delay
        wp.x_lat = x_lat
        wp.y_long = y_long
        wp.z_alt = z_alt
        self.waypoints.append(wp)

    # Start a waypoint mission
    # CAREFULL! This will set the flight mode to AUTO.MISSION
    @wait
    def start_mission(self):
        # Upload waypoints
        try:
            response = self.waypoint_service(waypoints=self.waypoints)
            if response.success:
                L.INFO("Mission successfully pushed!")
                self.total_waypoints = len(self.waypoints)
                self.waypoint_reached = 0
                self.waypoints = [] # Clear waypoints list
            else:
                L.ERROR("Failed to push mission")
        except rospy.ServiceException as e:
            L.ERROR("Service call failed: %s", e)

        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        set_mode_client(base_mode=0, custom_mode="AUTO.MISSION")

    # CHECK for mission completion
    @wait
    def check_for_mission_completion(self):
        if self.waypoint_reached >= self.total_waypoints - 1:
            L.INFO("Mission complete")
            return True
        return False
    
    # CHECK for target location to be reached
    @wait
    def check_for_target_reached(self):
        # Compares the position of the UAV relative to home and the target relative to home
        current_x, current_y, current_z = self.calculate_distance(self.home_location[0], self.home_location[1], self.home_location[2], self.current_latitude, self.current_longitude, self.current_altitude)
        distance_to_target = ((current_x - self.target_pose.pose.position.x)**2 + (current_y - self.target_pose.pose.position.y)**2 + (current_z - self.target_pose.pose.position.z)**2)**0.5

        tolerance = 1  # Set your own tolerance value in meters
        if distance_to_target < tolerance:
            L.INFO("Reached target position")
            return True
        return False
    
    #stops anyfurther execution while waiting to reach the target
    @wait
    def wait_for_target_pos(self):
        while not self.check_for_target_reached(): # TODO: include object avoidance check
            self.rate.sleep()
            pass
    
    @wait
    def send_pwm(self, pin, value):
        self.RC_Control.channels[pin] = value
        self.rc_override.publish(self.RC_Control)
    
    #reads a drop mission from a file, follows waypoints and drops on arrival where specified
    #NOTE: coordinates should be stored in a texfile in format: lat, long, alt
    #to specify a coordinate as a drop point, place a 1 following the coordinate
    #to not drop, place a zero following the coordinate    
    @wait
    def drop_mission(self,text_file):
        file = open(text_file)
        content = file.readlines()
        for line in content:
            line = line.rstrip()
            vals = line.split(', ')
            if len(line) > 0:
                vals = line.split(', ')
                Lat = float(vals[0])
                Long = float(vals[1])
                Alt = int(vals[2])
                Drop = int(vals[3])
                print(f'Lat: {Lat} Long: {Long} Alt: {Alt} Drop?: {Drop}')
                self.move_to_gps(Lat, Long, Alt)
                self.wait_for_target_pos()
                if(Drop == 1):
                    self.drop_package()
                    
            else:
                L.ERROR("Invalid Input String")
    
    #function for dropping packages        
    @wait
    def drop_package(self):
        L.INFO("Arrived at dropping location")
        sleep(2)
        self.send_pwm(8, 2000)
        sleep(2)
        L.INFO("Package dropped")
        self.send_pwm(8, 1100)

        