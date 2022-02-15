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
	unsigned int nj = chain.getNrOfJoints();
	// define a joint array in KDL format for the joint positions
    KDL::JntArray jointpositions = KDL::JntArray(nj);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	// define a manual joint command array for debugging	
	KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);
	

	// define the ros node
	ros::init(argc,argv, "ur5e_controller");
	ros::NodeHandle nh_;

	// setting up the loop frequency 
	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	tf::TransformBroadcaster br;
	while(ros::ok()){
			KDL::Frame cartpos; 
			double roll, pitch, yaw;
			bool kinematics_status;
			kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
			std::cout <<  " in the while loop" << std::endl;	
			if(kinematics_status>=0){
				cartpos.M.GetRPY(roll,pitch, yaw);
				
   			 	tf::Transform tool_in_world;
				tf::Vector3 tf_pose;
				tf::Quaternion tf_q;
				tf_pose = tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]);
				tf_q.setRPY(roll, pitch, yaw);
				tool_in_world.setOrigin(tf_pose);
				tool_in_world.setRotation(tf_q);
				br.sendTransform(tf::StampedTransform(tool_in_world, ros::Time::now(), "world", "tool_tip"));
				std::cout <<  " in the kinematic chain" << std::endl;	

			}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
