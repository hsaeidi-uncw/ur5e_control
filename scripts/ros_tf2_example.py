#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('ros_tf_example', anonymous = True)
	# add a ros transform listener
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	q_rot = Quaternion()	
	while not rospy.is_shutdown():

		# try getting the most update transformation between the tool frame and the base frame
		try:
			trans = tfBuffer.lookup_transform("base", "fk_tooltip", rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Frames not available!!!')
			loop_rate.sleep()
			continue
		# extract the xyz coordinates
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# extract the quaternion and converto RPY
		q_rot = trans.transform.rotation
		roll, pitch, yaw, = euler_from_quaternion([q_rot.x, q_rot.y, q_rot.z, q_rot.w])
		# a quick check of the readings
		print('x: ', format(x, '.3f'), ', y: ', format(y, '.3f'), ', z: ', format(z, '.3f'))
		print('roll: ', format(roll, '.2f'), ', pitch: ', format(pitch, '.2f'), ', yaw: ', format(yaw, '.2f')) 
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

