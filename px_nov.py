#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import Point #,PoseStamped
from mavros_msgs.msg import State, GlobalPositionTarget, Altitude
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandLong
from std_msgs.msg import Bool, Int16, Float32
import utm


class aioli():
	def __init__(self):
		self.current_state = State() 
		self.c_altitude = 0.0
		self.takenoff = False

		rospy.init_node('drone_command_handler_node', disable_signals=True)
		self.rate = rospy.Rate(0.5)

		rospy.Subscriber("mavros/state", State, self.state_cb)
		rospy.Subscriber("mavros/altitude", Altitude, self.altitude_cb)


		self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)


		#some ROS subscribe 

	def state_cb(self, msg):
		self.current_state = msg 

	def altitude_cb(self, msg): #BUT why do you need a separate topic for this and can't just get the altitude from state()????? bcs thats the mode, not the value. 
		self.c_altitude = msg.amsl
		#amsl = above mean sea level 

	def arming(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			armService(True)
			rospy.loginfo("Arming drone")
		except rospy.ServiceException, e:
			print "Service arm call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service arm call failed: %s"%e

	def setTakeoffMode(self):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		self.set_mode('AUTO.TAKEOFF', 100)
		if self.takenoff == False:
			self.initial_altitude = self.c_altitude
			self.takenoff = True
		try:
			self.arming()

			#takeoffService(altitude = 495, latitude = 55.057883, longitude = 10.569678, min_pitch = 0, yaw = 0)
			self.takeoffService(altitude = self.initial_altitude + 5, latitude = 55.057883, longitude = 10.569678, min_pitch = 0, yaw = 0)


		# export PX4_HOME_LAT=55.057883   
		# export PX4_HOME_LON=10.569678
		except rospy.ServiceException, e:
			print "Service takeoff call failed: %s"%e

	def setLandMode(self):
		rospy.wait_for_service('/mavros/cmd/land')
		self.set_mode('AUTO.LAND', 100)
		try:
			landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
			#http://wiki.ros.org/mavros/CustomModes for custom modes
			isLanding = landService(altitude = 0, latitude = 55.057883, longitude = 10.569678, min_pitch = 0, yaw = 0)
		except rospy.ServiceException, e:
			print "service land call failed: %s. The vehicle cannot land "%e



	def set_mode(self, mode, timeout):
		"""mode: PX4 mode string, timeout(int): seconds"""
		rospy.loginfo("setting FCU mode: {0}".format(mode))
		old_mode = self.current_state.mode
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		mode_set = False
		for i in xrange(timeout * loop_freq):
			if self.current_state.mode == mode:
				mode_set = True
				rospy.loginfo("set mode success | seconds: {0} of {1}".format(
					i / loop_freq, timeout))
				break
			else:
				try:
					res = self.set_mode_srv(0, mode)  # 0 is custom mode
					if not res.mode_sent:
						rospy.logerr("failed to send mode command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				self.fail(e)



	def main(self):
		print (self.current_state)
		self.arming()
		while not rospy.is_shutdown():
			
			self.setTakeoffMode()

			while self.current_state.mode == "AUTO.TAKEOFF":
				if self.current_state.mode == "AUTO.LOITER":
					break
				
				self.rate.sleep()
			##need a wait or smth here##
			print self.c_altitude
			#if self.c_altitude > 490:
			print (self.current_state.mode)
			#if self.current_state.mode == "AUTO.LOITER":
			
			self.setLandMode()

			while self.current_state.mode == "AUTO.LAND":
				self.rate.sleep()
				if self.current_state.armed == False:
					break
			print (self.current_state)
			self.rate.sleep()



if __name__ == '__main__':
	cmd = aioli()
	try:
		print ("in try")
		cmd.main()
		print ("supposed")
	except rospy.ROSInterruptException:
		pass

# "mavros/state"
# "mavros/setpoint_position/local"
# "mavros/cmd/arming"
# "mavros/set_mode"