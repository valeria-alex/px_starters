#!/usr/bin/env python2

import rospy
import mavros
from geometry_msgs.msg import Point #,PoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandLong
from topic_defines import *
from drone_status_info import DroneInformation
from std_msgs.msg import Bool, Int16
import utm
from defines import *
from topic_defines import *

class DroneCommandHandler():
    def __init__(self):
        rospy.loginfo("DroneCommandHandler Class initialization")
        rospy.init_node('drone_command_handler_node', disable_signals=True)
        self.current_state = State() 
        self.drone_info = DroneInformation()
        self.rate = rospy.Rate(0.5)
        self.marker_lat = 0
        self.marker_lon = 0
        self.track_marker_target = False
        self.tracker_lower_alt = False
        self.drone_state = 0
        self.ais_target = [0, 0] # lat, lon
        rospy.Subscriber(drone_arm_topic, Bool, self.arm_callback)
        rospy.Subscriber(drone_takeoff_topic, Bool, self.takeoff_callback)
        rospy.Subscriber(ship_ais_gps_signal, Point, self.ais_target_callback)
        rospy.Subscriber(drone_set_mission_topic, Bool, self.set_mission_mode_callback)
        rospy.Subscriber(drone_set_loiter_topic, Bool, self.set_loiter_mode_callback)
        rospy.Subscriber(drone_set_altitude, Int16, self.set_altitude_callback)
        rospy.Subscriber(drone_precision_land_topic, Bool, self.perform_precision_landing_callback)
        rospy.Subscriber(markerlocator_position, Point, self.markerlocator_position_callback)
        rospy.Subscriber(drone_track_marker, Bool, self.track_marker_target_callback)
        rospy.Subscriber(drone_track_marker_alt, Bool, self.track_marker_target_lower_alt_callback)
        rospy.Subscriber(drone_place_corner, Bool, self.track_corner_callback)
        rospy.Subscriber(drone_set_state, Int16, self.set_drone_state)
        

        mavros.set_namespace()
        rospy.Subscriber(mavros.get_topic('state'), State, self.mavros_state_callback)
        self.arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
        self.takeoff_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'takeoff'), CommandTOL)
        self.set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 
        self.command_long = rospy.ServiceProxy(mavros.get_topic('cmd', 'command'), CommandLong) 

        # wait for FCU connection
        while not self.current_state.connected:
            self.rate.sleep()
        rospy.sleep(1)
        
        rospy.loginfo("DroneCommandHandler Class initialization finished")

    def mavros_state_callback(self, msg):
        self.current_state = msg

    def set_drone_state(self, msg):     
        if msg.data > len(DRONE_STATE)-1 or msg.data == 0: 
            rospy.logwarn("DroneCommandHandler: Invalid drone state")
        else:
            self.drone_state = msg.data
            rospy.loginfo("DroneCommandHandler: Drone state changed to [" + DRONE_STATE[self.drone_state] + "]")

    def track_corner_callback(self, msg):
        self.command_long(False, 192, 1.0, 12, 1,0, 0.0, 55.057948, 10.5701486, self.drone_info.amsl_altitude)

    def track_marker_target_lower_alt_callback(self, msg):
        if msg.data:
            self.tracker_lower_alt = True
            rospy.loginfo("DroneCommandHandler: Track marker lower altitude activated")
        elif not msg.data:
            self.tracker_lower_alt = False
            rospy.loginfo("DroneCommandHandler: Track marker lower altitude deactivated")

    def track_marker_target_callback(self, msg):
        if msg.data:
            self.track_marker_target = True
            rospy.loginfo("DroneCommandHandler: Track marker activated")
        elif not msg.data:
            self.track_marker_target = False
            rospy.loginfo("DroneCommandHandler: Track marker deactivated")

    def markerlocator_position_callback(self, msg):
        easting_error = msg.x
        norting_error = msg.y

        # take local offset and convert to global utm
        #drone position utm
        (EASTING_DRONE, 
         NORTHING_DRONE, 
         ZONE_NUM_DRONE, 
         ZONE_LET_DRONE) = utm.from_latlon(self.drone_info.global_position.lat, 
                                           self.drone_info.global_position.lon)
        
        marker_easting = EASTING_DRONE + easting_error
        marker_northing = NORTHING_DRONE + norting_error

        # convert utm to geogetic
        (LATITUDE, LONGITUDE) = utm.to_latlon(marker_easting, marker_northing, ZONE_NUM_DRONE, ZONE_LET_DRONE)

        self.marker_lat = LATITUDE
        self.marker_lon = LONGITUDE

    def perform_precision_landing_callback(self, msg):
        pass

    def set_altitude_callback(self, msg):
        msg.data 
        self.command_long(False, 192, 0, 12, 1,0, 0.0,
                          self.drone_info.global_position.lat, 
                          self.drone_info.global_position.lon,
                          self.drone_info.amsl_altitude + msg.data)

    def set_mission_mode_callback(self, msg):
        rospy.loginfo("Attempting to set MISSION mode")
        self.set_mode_client(base_mode=4, custom_mode="AUTO.MISSION")

        # Information on how to select correct base and custom mode
        # LOITER = (base_mode=3, custom_mode="AUTO.LOITER")
        # MISSION = (base_mode=4, custom_mode="AUTO.MISSION")
        #MODE: Known modes are: AUTO.PRECLAND AUTO.FOLLOW_TARGET AUTO.RTGS AUTO.LAND 
        # AUTO.RTL AUTO.MISSION RATTITUDE AUTO.LOITER STABILIZED AUTO.TAKEOFF OFFBOARD 
        # POSCTL ALTCTL AUTO.READY ACRO MANUAL

    def set_loiter_mode_callback(self, msg):
        rospy.loginfo("Attempting to set LOITER mode")
        self.set_mode_client(base_mode=3, custom_mode="AUTO.LOITER")

    def arm_callback(self, msg):
        if msg.data:
            self.arming_client(True)
            rospy.loginfo("Arming drone")
            # todo - need to check if drone was armed

            #     # older versions of PX4 always return success==True, so better to check Status instead
            #     if prev_state.armed != current_state.armed:
            #         rospy.loginfo("Vehicle armed: %r" % current_state.armed)
            #     if prev_state.mode != current_state.mode: 
            #         rospy.loginfo("Current mode: %s" % current_state.mode)
            #     prev_state = current_state

    def takeoff_callback(self, msg):
        # takeoff
        rospy.loginfo("Takeoff issued")
        min_pitch = 0
        yaw = 0 
        lat = self.drone_info.global_position.lat
        lon = self.drone_info.global_position.lon
        alt = self.drone_info.amsl_altitude+5
        self.takeoff_client(min_pitch, yaw, lat, lon, alt)
        # todo - check if system is running properly before takeoff

    def ais_target_callback(self, msg):
        self.ais_target[0] = msg.x # lat from msg
        self.ais_target[1] = msg.y # lon from msg

    # old landing hackup
    # def follow_ais_target_callback(self, target):
    #     # target_msg = GlobalPositionTarget()
    #     # print(self.drone_info.initial_gps_position.alt)
    #     (EASTING_INIT, 
    #      NORTHING_INIT, 
    #      ZONE_NUM_INIT, 
    #      ZONE_LET_INIT) = utm.from_latlon(self.drone_info.initial_gps_position.lat, 
    #                                       self.drone_info.initial_gps_position.lon)
        
    #     (EASTING_NOW, 
    #      NORTHING_NOW, 
    #      ZONE_NUM_NOW, 
    #      ZONE_LET_NOW) = utm.from_latlon(self.drone_info.global_position.lat, 
    #                                       self.drone_info.global_position.lon)

    #     easting_target = EASTING_INIT + target.x
    #     northing_target = NORTHING_INIT + target.y
    #     # print("n.e targets", easting_target, northing_target)
    #     (LATITUDE, LONGITUDE) = utm.to_latlon(easting_target, northing_target, ZONE_NUM_INIT, ZONE_LET_INIT)

    def run(self):
        print ("befur while")
        while not rospy.is_shutdown():
            # print("rel", self.drone_info.relative_altitude)
            # print("pos", self.drone_info.global_position.lat, 
            #              self.drone_info.global_position.lon, 
            #              self.drone_info.global_position.alt)
            # print("voltage", self.drone_info.battery_voltage)
            # print("soc", self.drone_info.battery_soc)
            # print("compasss", self.drone_info.compass_heading)
            # print(self.track_marker_target)
            print ('anything?')
            if self.drone_state == STATE_FOLLOW_AIS:
                print("drone state:", DRONE_STATE[self.drone_state])
                print("follow AIS target:", self.ais_target[0], self.ais_target[1], self.drone_info.amsl_altitude+10)
                lat = self.ais_target[0]
                lon = self.ais_target[1]
                print('here?')
                # self.command_long(False, 192, 0, 12, 1,0, 0.0, 550578830, 105796775, 530) 
                res = self.command_long(False, 192, 0, 12, 1,0, 0.0, lat, lon, self.drone_info.amsl_altitude)
                print("res",res)
            
            if self.drone_state == STATE_LOITER:
                lat = self.drone_info.global_position.lat
                lon = self.drone_info.global_position.lon
                print('wtbhere?')
                self.command_long(False, 192, 0, 12, 1,0, 0.0, lat, lon, self.drone_info.amsl_altitude)

            self.command_long(False, 192, 0, 12, 1,0, 0.0, 55.0578830, 10.5796775, 530) 

            
            self.rate.sleep()
       

if __name__ == '__main__':
    cmd = DroneCommandHandler()
    try:
        print ("in try")
        cmd.run()
        print ("supposed")
    except rospy.ROSInterruptException:
        pass