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


KDL::Chain LWR(){

  double tool_length = 0.506;//0.01735;


  KDL::Chain chain;

  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(0,0,0.33989,0)));

  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 2 
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0.40011,0)));

  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));

  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0.40003,0)));

  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, M_PI_2,0,0)));

  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
  KDL::Frame::DH_Craig1989(0,0, tool_length,0)));

  return chain;

}



int main(int argc, char * argv[]){
	// define the kinematic chain
	KDL::Chain chain = LWR();
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

	
	return 0;
}
