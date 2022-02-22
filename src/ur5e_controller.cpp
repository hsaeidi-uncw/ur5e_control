#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h> 
#include <trajectory_msgs/JointTrajectoryPoint.h> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>

// define the kinematic chain
KDL::Chain UR5e(){

  KDL::Chain chain;
  // input parameters for the DH Frames are (double a,double alpha,double d,double theta)
  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH(0, 0, 0, 0)));

  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0, M_PI_2, 0.1625, 0)));

  //joint 2 
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(-0.425, 0, 0, 0)));

  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(-0.3922, 0, 0, 0)));

  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0, M_PI_2, 0.1333,0)));

  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0, -M_PI_2, 0.0997, 0)));

  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH(0, 0, 0.0996, 0)));

 
  return chain;

}


//reading the joint positions
sensor_msgs::JointState joint_positions;
// for the frist reading
bool joints_initialized = false;
//callback for reading joint values
void get_joint_pose(const sensor_msgs::JointState & data){
	// if this is not the first time the callback function is read, obtain the joint positions
	if(joints_initialized){
		joint_positions.position[0] = data.position[2];	//sensor readings are in a differnt order!
		joint_positions.position[1] = data.position[1];	
		joint_positions.position[2] = data.position[0];	//sensor readings are in a differnt order!
		joint_positions.position[3] = data.position[3];	
		joint_positions.position[4] = data.position[4];	
		joint_positions.position[5] = data.position[5];	
	// otherwise initilize them with 0.0 values
	}else{
		for (int i = 0; i < data.position.size();++i){
			joint_positions.position.push_back(0.0);
		}
		std::cout << "joints initialized" << std::endl;
	}
		

	joints_initialized = true;	
	
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd){	
		_cmd.joint_names.push_back("elbow_joint");
		_cmd.joint_names.push_back("shoulder_lift_joint");
		_cmd.joint_names.push_back("shoulder_pan_joint");
		_cmd.joint_names.push_back("wrist_1_joint");
		_cmd.joint_names.push_back("wrist_2_joint");
		_cmd.joint_names.push_back("wrist_3_joint");
}


// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}


// loads the joint space points to be sent as a command to the robot
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj){
	// check if the solver returns large values and trim them
	for (int i = 0; i < _nj; ++i){
		 while(_jointpositions(i) > M_PI)
				_jointpositions(i) -= 2*M_PI;
		 while(_jointpositions(i) < -M_PI)
				_jointpositions(i) += 2*M_PI;

	}	
	_point.positions[0] = _jointpositions(2); // actuator orders in the driver are different than the chain!
	_point.positions[1] = _jointpositions(1);
	_point.positions[2] = _jointpositions(0); // actuator orders in the driver are different than the chain!
	_point.positions[3] = _jointpositions(3);
	_point.positions[4] = _jointpositions(4);
	_point.positions[5] = _jointpositions(5);
}

KDL::Frame update_ref(geometry_msgs::Twist _ref){
	// define a rpy rotation using KDL
	KDL::Rotation rpy = KDL::Rotation::RPY(0.0, 0.0, 0.0);
	// define a KDL frame
	KDL::Frame _cartpos;
	// update the reference cartesian positions
	_cartpos.p[0]= _ref.linear.x;
	_cartpos.p[1]= _ref.linear.y;
	_cartpos.p[2]= _ref.linear.z;			
	rpy = KDL::Rotation::RPY(_ref.angular.x, _ref.angular.y, _ref.angular.z);
	_cartpos.M = rpy;
	
	return _cartpos;
}


tf::Transform update_tool_frame(KDL::Frame _cartpos){

	// define roll, pitch, yaw variables
	double roll, pitch, yaw;
	//extract the roll, pitch, yaw from the KDL frame after the fk calculations
	_cartpos.M.GetRPY(roll,pitch, yaw);
	// define the quaternions and vectors for that frame
	tf::Vector3 tf_pose;
	tf::Quaternion tf_q;
	tf_pose = tf::Vector3(_cartpos.p[0], _cartpos.p[1], _cartpos.p[2]);
	tf_q.setRPY(roll, pitch, yaw);
	// update and return the frame
	tf::Transform tool_frame;
	tool_frame.setOrigin(tf_pose);
	tool_frame.setRotation(tf_q);
	return tool_frame;
}


geometry_msgs::Twist update_xyzrpy(KDL::Frame _cartpos){
	// define roll, pitch, yaw variables
	double roll, pitch, yaw;
	//extract the roll, pitch, yaw from the KDL frame after the fk calculations
	_cartpos.M.GetRPY(roll,pitch, yaw);
	geometry_msgs::Twist _xyzrpy;
	// update and return the values
	_xyzrpy.linear.x = _cartpos.p[0];
	_xyzrpy.linear.y = _cartpos.p[1];
	_xyzrpy.linear.z = _cartpos.p[2];
	_xyzrpy.angular.x = roll;
	_xyzrpy.angular.y = pitch;
	_xyzrpy.angular.z = yaw;
	return _xyzrpy;
}


// read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
bool ref_received= false;
geometry_msgs::Twist ref;
void get_ref(const geometry_msgs::Twist & data){
	std::cout << "inside get ref" << std::endl;
	ref = data;
	ref_received = true;
}

int main(int argc, char * argv[]){
	// define the kinematic chain
	KDL::Chain chain = UR5e();
	// define the forward kinematic solver via the defined chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	// define the inverse kinematics solver
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6

	// get the number of joints from the chain
	unsigned int no_of_joints = chain.getNrOfJoints();
	// define a joint array in KDL format for the joint positions
    KDL::JntArray q_current = KDL::JntArray(no_of_joints);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray q_next = KDL::JntArray(no_of_joints);
	
	

	// define the ros node
	ros::init(argc,argv, "ur5e_controller");
	ros::NodeHandle nh_;
	

	// publisher for sending control commands to the UR5e
	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command",1);
	// a publisher for quick debugging in the terminal
	ros::Publisher xyzrpy_pub = nh_.advertise<geometry_msgs::Twist>("/ur5e/toolpose",10);
	
	
	// subscriber for reading the joint angles
	ros::Subscriber joints_sub = nh_.subscribe("/joint_states",10, get_joint_pose);
	
	// subscriber for reading the reference trajectories from the reflexxes-based node	
	ros::Subscriber ref_sub = nh_.subscribe("/reftraj",10, get_ref);
	
	// setting up the loop frequency 
	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);
	// define a transforma broadcaster to check the FK results
	tf::TransformBroadcaster br;

	// a debugging variable
	geometry_msgs::Twist xyzrpy;
	
	// define a KDL frame for use in the kinematic solver
	KDL::Frame cartpos; 
	

	// define the joint control command and a point in it
	trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint joint_cmd_point;
	// set up the message with proper joint names
	name_joints(joint_cmd);
	// initialize the point message with zero for all joints
	initialize_points(joint_cmd_point,no_of_joints, 0.0);

	// quick debugging
	KDL::JntArray manual_joint_cmd = KDL::JntArray(no_of_joints);
	//manual_joint_cmd(1) = -M_PI/2;
	eval_points(joint_cmd_point, manual_joint_cmd, no_of_joints);	
	joint_cmd_point.time_from_start = ros::Duration(dt);
	joint_cmd.points.push_back(joint_cmd_point);
	

	while(ros::ok()){

			if (joints_initialized){
				// get the most current value of the joint positions
				for (int k = 0; k < no_of_joints; ++k){
					q_current(k) = joint_positions.position[k];
				}
					
				// if references for autonomous control are received
				if (ref_received){
					std::cout << "inside ref received" << std::endl;
					// convert the reference to a KDL frame
					cartpos = update_ref(ref);
					// use the frame with the current joint positions to get the next joint positions via IK solver
					int ret = iksolver.CartToJnt(q_current, cartpos, q_next);
					// update the control command point
					eval_points(joint_cmd_point, q_next, no_of_joints);
					joint_cmd_point.time_from_start = ros::Duration(dt);
					// update the command via the updated point
					joint_cmd.points[0] = joint_cmd_point;
					//std::cout << "inside while" << std::endl;
					joint_cmd.header.stamp = ros::Time::now();
					cmd_pub.publish(joint_cmd);
				}
				// flag for the fk results
				bool kinematics_status;
				kinematics_status = fksolver.JntToCart(q_current, cartpos);
				// show the frames if the fk works well
				if(kinematics_status >= 0){
					//std::cout << "inside fk" << std::endl;
					// define a transformation in ROS to show the frame in rviz
   			 		tf::Transform tool_in_world;
   			 		tool_in_world = update_tool_frame(cartpos);	
					// boradcast the frame
					br.sendTransform(tf::StampedTransform(tool_in_world, ros::Time::now(), "base", "fk_tooltip"));	
	
					xyzrpy = update_xyzrpy(cartpos);
					
					xyzrpy_pub.publish(xyzrpy);
					
				}// end of kintamitc_status
			}// end of joints_initialized
			
			
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
