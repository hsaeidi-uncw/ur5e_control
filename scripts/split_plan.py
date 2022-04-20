#!/usr/bin/env python3
import rospy

# import the plan message
from ur5e_control.msg import Plan
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

plan = Plan()


plan_received = False

def plan_callback(data):
	global plan
	global plan_received 
	# update the plan message
	plan = data
	plan_received = True
	#print(plan)

traj_expired = False

def traj_expired_callback(data):
	global traj_expired
	traj_expired = data.data
	if traj_expired:
		print('traj expired')	


rob_gripper_status = 3 # 3: UR5e control mode running, 4: gripper opened, 5: gripper closed
 
def status_callback(data):
	global rob_gripper_status
	rob_gripper_status = data.data
	
		
def split_plan(plan):
	approach_plan = Plan()
	drop_plan = Plan()
	retract_plan = Plan()
	approach_generated = False
	drop_generated = False
	retract_generated = False
	for point, mode in zip(plan.points, plan.modes):
		print(mode, approach_generated, drop_generated, retract_generated)
		if mode.data == 0 and not approach_generated:
			approach_plan.points.append(point)
			approach_plan.modes.append(mode)
			print('added to the approach')
		elif mode.data != 0 and not approach_generated:
			approach_generated = True
		elif mode.data == 0 and not drop_generated and approach_generated:
			drop_plan.points.append(point)
			drop_plan.modes.append(mode)
			print('added to the drop')
		elif mode.data != 0 and not drop_generated and approach_generated:
			drop_generated = True
		elif mode.data == 0 and not retract_generated and drop_generated and approach_generated:
			retract_plan.points.append(point)
			retract_plan.modes.append(mode)
			print('added to the retract')
		else:
			retract_generated = True
	# make it go to the initial position
	retract_plan.points.append(plan.points[0])
	retract_plan.modes.append(plan.modes[0])	
	print('-------------')
	print('Approach')
	print(approach_plan)
	print('-------------')
	print('Drop')
	print(drop_plan)
	print('-------------')
	print('Retract')
	print(retract_plan)
	return approach_plan, drop_plan, retract_plan
	
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('split_plan', anonymous = True)
	# add a subscriber to it to read the position information
	pos_sub = rospy.Subscriber('/plan', Plan, plan_callback)
	traj_expired_sub = rospy.Subscriber('/traj_expired', Bool, traj_expired_callback)
	rob_gripper_status_sub = rospy.Subscriber('/gripper_robot_status', UInt8, status_callback)
	
	# publisher for splitted plan
	splitted_plan_pub = rospy.Publisher('/splitted_plan', Plan, queue_size = 10)
	# robot mode control
	gripper_cmd_pub = rospy.Publisher('/gripper_cmd', UInt8, queue_size = 10)
	
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
		
	plan_splitted = False
	
	approach_plan = Plan()
	drop_plan = Plan()
	retract_plan = Plan()
	states = ['approach', 'close', 'drop', 'open', 'retract', 'finished']
	state = states[0]
	
	plan_submitted = False
	
	gripper_cmd = UInt8()
	
	while not rospy.is_shutdown():
		if not plan_splitted and plan_received:
			approach_plan, drop_plan, retract_plan = split_plan(plan)
			plan_splitted = True
		if plan_splitted:
			if state == states[0] and rob_gripper_status == 3:
				if not plan_submitted:
					splitted_plan_pub.publish(approach_plan)
					print(state, 'plan submitted')
					plan_submitted = True
					traj_expired = False
				if traj_expired:
					state = states[1]
					print('Switching to state:', state)
					plan_submitted = False
			if state == states[1]:
				if not plan_submitted:
					gripper_cmd.data = 2 # close the gripper
					gripper_cmd_pub.publish(gripper_cmd)
					print(state, 'plan submitted')
					plan_submitted = True
				if rob_gripper_status == 5: # if gripper closed
					state = states[2]
					print('Switching to state:', state)
					gripper_cmd.data = 0 # switch to robot motion mode
					gripper_cmd_pub.publish(gripper_cmd)
					plan_submitted = False
			if state == states[2] and rob_gripper_status == 3:
				if not plan_submitted:
					splitted_plan_pub.publish(drop_plan)
					print(state, 'plan submitted')
					plan_submitted = True
					traj_expired = False
				if traj_expired:
					state = states[3]
					print('Switching to state:', state)
					plan_submitted = False
			if state == states[3]:
				if not plan_submitted:
					gripper_cmd.data = 1 # open the gripper
					gripper_cmd_pub.publish(gripper_cmd)
					print(state, 'plan submitted')
					plan_submitted = True
				if rob_gripper_status == 4: # if gripper opened
					state = states[4]
					print('Switching to state:', state)
					gripper_cmd.data = 0 # switch to robot motion mode
					gripper_cmd_pub.publish(gripper_cmd)
					plan_submitted = False
			if state == states[4] and rob_gripper_status == 3:
				if not plan_submitted:
					splitted_plan_pub.publish(retract_plan)
					print(state, 'plan submitted')
					plan_submitted = True
					traj_expired = False
				if traj_expired:
					state = states[5]
					print('Switching to state:', state)					

		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
