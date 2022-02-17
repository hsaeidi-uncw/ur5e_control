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
    KDL::JntArray jointpositions = KDL::JntArray(no_of_joints);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(no_of_joints);
	
	

	// define the ros node
	ros::init(argc,argv, "ur5e_controller");
	ros::NodeHandle nh_;
	
	// a publisher for quick debugging in the terminal
	ros::Publisher xyzrpy_pub = nh_.advertise<geometry_msgs::Twist>("/ur5e/toolpose",10);
	
	// setting up the loop frequency 
	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);
	// define a transforma broadcaster to check the FK results
	tf::TransformBroadcaster br;
	
	
	
	geometry_msgs::Twist xyzrpy;
	// define a KDL frame for use in the kinematic solver
	KDL::Frame cartpos; 

	while(ros::ok()){

			// define roll, pitch, yaw variables
			double roll, pitch, yaw;
			// flag for the fk results
			bool kinematics_status;
			kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
			// show the frames if the fk works well
			if(kinematics_status >= 0){
				//extract the roll, pitch, yaw from the KDL frame after the fk calculations
				cartpos.M.GetRPY(roll,pitch, yaw);
				// define a transformation in ROS to show the frame in rviz
   			 	tf::Transform tool_in_world;
				tf::Vector3 tf_pose;
				tf::Quaternion tf_q;
				tf_pose = tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]);
				tf_q.setRPY(roll, pitch, yaw);
				tool_in_world.setOrigin(tf_pose);
				tool_in_world.setRotation(tf_q);
				// boradcast the frame
				br.sendTransform(tf::StampedTransform(tool_in_world, ros::Time::now(), "base", "fk_tooltip"));	
				xyzrpy.linear.x = cartpos.p[0];
				xyzrpy.linear.y = cartpos.p[1];
				xyzrpy.linear.z = cartpos.p[2];
				xyzrpy.angular.x = roll;
				xyzrpy.angular.y = pitch;
				xyzrpy.angular.z = yaw;
				xyzrpy_pub.publish(xyzrpy);
				
			}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
