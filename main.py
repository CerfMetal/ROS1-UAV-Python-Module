#! /usr/bin/python3.8

from uav import uav
from Logger import Logger
import time

L = Logger(0)

def wait():
    while not drone.check_for_target_reached(): # TODO: include object avoidance check
        drone.rate.sleep()
        pass

time.sleep(3)
drone = uav("offb_node_py", 20.0)
drone.connnect()

drone.set_mode("OFFBOARD")
drone.arm(True)

# Needed, for the OFFBOARD to work, idk why
drone.take_off(2)
wait()

drone.move_to_gps(43.64475, -79.38499, 25)
wait()
drone.move_to_gps(43.64375, -79.38464, 25)
wait()
drone.move_to_gps(43.64312, -79.38800, 25)
wait()
drone.move_to_gps(43.64344, -79.38868, 25)
wait()

L.INFO("Arrived at dropping location")
time.sleep(2)
drone.send_pwm(8, 2000)
time.sleep(2)
L.INFO("Package dropped")
drone.send_pwm(8, 1100)

drone.return_to_home()




# Example by using waypoints
def waypoint_mission():
    drone.create_waypoint(3, 16, True, True, 0, 43.64475, -79.38499, 25)
    drone.create_waypoint(3, 16, False, True, 0, 43.64375, -79.38464, 25)
    drone.create_waypoint(3, 16, True, True, 0, 43.64312, -79.38800, 25)
    drone.create_waypoint(3, 16, False, True, 0, 43.64344, -79.38868, 25)

    drone.arm(True)
    drone.start_mission()

    while drone.check_for_mission_completion(): # TODO: include object avoidance check
        pass

    drone.auto_land() # TODO: land with object avoidance





