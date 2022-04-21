//  ---------------------- Doxygen info ----------------------
//! \file 01_RMLPositionSampleApplication.cpp
//!
//! \brief
//! Test application number 1 for the Reflexxes Motion Libraries
//! (basic position-based interface)
//!
//! \date March 2014
//!
//! \version 1.2.6
//!
//! \author Torsten Kroeger, <info@reflexxes.com> \n
//!
//! \copyright Copyright (C) 2014 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <ur5e_control/Plan.h>
#include <trajectory_msgs/JointTrajectory.h> 
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <cmath>


#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.1
#define NUMBER_OF_DOFS                          6


//*************************************************************************
// Main function to run the process that contains the test application
//
// This function contains source code to get started with the Type II
// Reflexxes Motion Library. Only a minimum amount of functionality is
// contained in this program: a simple trajectory for a
// three-degree-of-freedom system is executed. This code snippet
// directly corresponds to the example trajectories shown in the
// documentation.
//*************************************************************************


ur5e_control::Plan plan;
ur5e_control::Plan prev_plan;
bool plan_available = false;
bool rob_pos_received = false;
int number_of_points = 0;


geometry_msgs::Twist rob_pos;
int control_mode = 1;

void get_pos(const geometry_msgs::Twist & _data){
	rob_pos = _data;
	rob_pos_received = true;
}

void get_plan(const ur5e_control::Plan & _data){
	plan = _data;
	plan_available = true;
}

// get the updated mode
unsigned int mode = 3;


void get_mode(const std_msgs::UInt8 & data){
	mode = data.data;
	std::cout << "mode changed to: " << mode << std::endl;
}

void initialize_plan(RMLPositionInputParameters  *_IP){
		_IP->CurrentPositionVector->VecData      [0] =    plan.points[0].linear.x; //x
		_IP->CurrentPositionVector->VecData      [1] =    plan.points[0].linear.y; // y
		_IP->CurrentPositionVector->VecData      [2] =    plan.points[0].linear.z; // z
		_IP->CurrentPositionVector->VecData      [3] =    plan.points[0].angular.x; // roll
		_IP->CurrentPositionVector->VecData      [4] =    plan.points[0].angular.y; // pitch
		_IP->CurrentPositionVector->VecData      [5] =    plan.points[0].angular.z; // yaw

		_IP->CurrentVelocityVector->VecData      [0] =    0.0;
		_IP->CurrentVelocityVector->VecData      [1] =    0.0;
		_IP->CurrentVelocityVector->VecData      [2] =    0.0;
		_IP->CurrentVelocityVector->VecData      [3] =    0.0;
		_IP->CurrentVelocityVector->VecData      [4] =    0.0;
		_IP->CurrentVelocityVector->VecData      [5] =    0.0;

		_IP->CurrentAccelerationVector->VecData  [0] =    0.0;
		_IP->CurrentAccelerationVector->VecData  [1] =    0.0;
		_IP->CurrentAccelerationVector->VecData  [2] =    0.0;
		_IP->CurrentAccelerationVector->VecData  [3] =    0.0;
		_IP->CurrentAccelerationVector->VecData  [4] =    0.0;
		_IP->CurrentAccelerationVector->VecData  [5] =    0.0;


		_IP->MaxVelocityVector->VecData          [0] =    0.1      ;
		_IP->MaxVelocityVector->VecData          [1] =    0.1      ;
		_IP->MaxVelocityVector->VecData          [2] =    0.1      ;
		_IP->MaxVelocityVector->VecData          [3] =    0.1      ;
		_IP->MaxVelocityVector->VecData          [4] =    0.1      ;
		_IP->MaxVelocityVector->VecData          [5] =    0.1      ;


		_IP->MaxAccelerationVector->VecData      [0] =    0.1      ;
		_IP->MaxAccelerationVector->VecData      [1] =    0.1      ;
		_IP->MaxAccelerationVector->VecData      [2] =    0.1      ;
		_IP->MaxAccelerationVector->VecData      [3] =    0.1      ;
		_IP->MaxAccelerationVector->VecData      [4] =    0.1      ;
		_IP->MaxAccelerationVector->VecData      [5] =    0.1      ;



		_IP->MaxJerkVector->VecData              [0] =    0.1      ;
		_IP->MaxJerkVector->VecData              [1] =    0.1      ;
		_IP->MaxJerkVector->VecData              [2] =    0.1      ;
		_IP->MaxJerkVector->VecData              [3] =    0.1      ;
		_IP->MaxJerkVector->VecData              [4] =    0.1      ;
		_IP->MaxJerkVector->VecData              [5] =    0.1      ;

		//setting the target velocities and positions
		_IP->TargetPositionVector->VecData       [0] =   plan.points[1].linear.x;
		_IP->TargetPositionVector->VecData       [1] =   plan.points[1].linear.y;
		_IP->TargetPositionVector->VecData       [2] =   plan.points[1].linear.z;
		_IP->TargetPositionVector->VecData       [3] =   plan.points[1].angular.x;
		_IP->TargetPositionVector->VecData       [4] =   plan.points[1].angular.y;
		_IP->TargetPositionVector->VecData       [5] =   plan.points[1].angular.z;

		_IP->TargetVelocityVector->VecData       [0] =   0.0;
		_IP->TargetVelocityVector->VecData       [1] =   0.0;
		_IP->TargetVelocityVector->VecData       [2] =   0.0;
		_IP->TargetVelocityVector->VecData       [3] =   0.0;
		_IP->TargetVelocityVector->VecData       [4] =   0.0;
		_IP->TargetVelocityVector->VecData       [5] =   0.0;

		//determine which Degrees of freedom should be calculated
		_IP->SelectionVector->VecData            [0] =   true        ;
		_IP->SelectionVector->VecData            [1] =   true        ;
		_IP->SelectionVector->VecData            [2] =   true        ;
		_IP->SelectionVector->VecData            [3] =   true        ;
		_IP->SelectionVector->VecData            [4] =   true        ;
		_IP->SelectionVector->VecData            [5] =   true        ;

}


bool is_new_plan(ur5e_control::Plan _plan, ur5e_control::Plan _prev_plan){

	bool new_plan = false;
	if (_plan.points.size() != _prev_plan.points.size()){
		new_plan = true;
	}else{
		for (int i=0; i < _plan.points.size(); i++){
			new_plan = (new_plan || (fabs(_plan.points[i].linear.x - _prev_plan.points[i].linear.x) > 0.01) );
			new_plan = (new_plan || (fabs(_plan.points[i].linear.y - _prev_plan.points[i].linear.y) > 0.01) );
			new_plan = (new_plan || (fabs(_plan.points[i].linear.z - _prev_plan.points[i].linear.z) > 0.01) );
			new_plan = (new_plan || (fabs(_plan.points[i].angular.x - _prev_plan.points[i].angular.x) > 0.01) );
			new_plan = (new_plan || (fabs(_plan.points[i].angular.y - _prev_plan.points[i].angular.y) > 0.01) );
			new_plan = (new_plan || (fabs(_plan.points[i].angular.z - _prev_plan.points[i].angular.z) > 0.01) );						
		}
	}
	return new_plan;
	
}

int main(int argc, char * argv[])
{

    ros::init(argc,argv,"task_space_traj");
    ros::NodeHandle nh_;


    int loop_freq = 10;
    float dt = (float) 1/loop_freq;
    ros::Rate loop_rate(loop_freq);
    ros::Publisher reflexxes_pub = nh_.advertise<geometry_msgs::Twist>("/reftraj",1);

    ros::Subscriber plan_sub = nh_.subscribe("/splitted_plan",10,get_plan);
    ros::Subscriber pos_sub = nh_.subscribe("/ur5e/toolpose" ,1, get_pos);
    // subscriber for checking the status mode of the robot and gripper
    ros::Subscriber mode_sub = nh_.subscribe("/gripper_robot_status",10, get_mode);
    

    geometry_msgs::Twist ref; // for publishing the reference traj
    // some default codes form the solver

    // ********************************************************************
    // Variable declarations and definitions

    int                         ResultValue                 =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;
    
    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML =   new ReflexxesAPI(                   NUMBER_OF_DOFS
                                            ,   CYCLE_TIME_IN_SECONDS   );

    IP  =   new RMLPositionInputParameters(     NUMBER_OF_DOFS          );

    OP  =   new RMLPositionOutputParameters(    NUMBER_OF_DOFS          );

    // ********************************************************************
    // Set-up a timer 
    
    // ********************************************************************
    
    
    // ********************************************************************
    // Set-up the input parameters

    // In this test program, arbitrary values are chosen. If executed on a
    // real robot or mechanical system, the position is read and stored in
    // an RMLPositionInputParameters::CurrentPositionVector vector object.
    // For the very first motion after starting the controller, velocities
    // and acceleration are commonly set to zero. The desired target state
    // of motion and the motion constraints depend on the robot and the
    // current task/application.
    // The internal data structures make use of native C data types
    // (e.g., IP->CurrentPositionVector->VecData is a pointer to
    // an array of NUMBER_OF_DOFS double values), such that the Reflexxes
    // Library can be used in a universal way.

	// wait for a plan
	while (!plan_available){
		loop_rate.sleep();
		
		ros::spinOnce();
	}
	initialize_plan(IP);
	prev_plan = plan;
	number_of_points = plan.points.size();
	std::cout << "received a plan with " << number_of_points << " points"<< std::endl;
	
    // ********************************************************************
    // Starting the control loop
    int ctr = 2;
    while(ros::ok()){
	    if (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && mode == 3)
	    {

			// ****************************************************************
			// Wait for the next timer tick
			// (not implemented in this example in order to keep it simple)
			// ****************************************************************

			// Calling the Reflexxes OTG algorithm
			ResultValue =   RML->RMLPosition(       *IP
				                                ,   OP
				                                ,   Flags       );

			if (ResultValue < 0)
			{
				printf("An error occurred (%d).\n", ResultValue );
				break;
			}
			if (ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED && (ctr < number_of_points) ){
				//setting the target velcoity and positions
				int next_wp;
				
				next_wp = ctr % number_of_points;
				std::cout << "counter is: " << ctr<< ", moving to point:" << next_wp+1 << std::endl;
				
				IP->TargetPositionVector->VecData       [0] =   plan.points[next_wp].linear.x;
				IP->TargetPositionVector->VecData       [1] =   plan.points[next_wp].linear.y;
				IP->TargetPositionVector->VecData       [2] =   plan.points[next_wp].linear.z;
				IP->TargetPositionVector->VecData       [3] =   plan.points[next_wp].angular.x;
				IP->TargetPositionVector->VecData       [4] =   plan.points[next_wp].angular.y;
				IP->TargetPositionVector->VecData       [5] =   plan.points[next_wp].angular.z;
				// update the start point via the current robot positionsn
				if (rob_pos_received){
					IP->CurrentPositionVector->VecData[0] = rob_pos.linear.x;
					IP->CurrentPositionVector->VecData[1] = rob_pos.linear.y;
					IP->CurrentPositionVector->VecData[2] = rob_pos.linear.z;
					IP->CurrentPositionVector->VecData[3] = rob_pos.angular.x;
					IP->CurrentPositionVector->VecData[4] = rob_pos.angular.y;
					IP->CurrentPositionVector->VecData[5] = rob_pos.angular.z;
				}
			
				ctr ++;
				std::cout << "counter updated to: " << ctr<< " for next loop." << std::endl;
				ResultValue =   RML->RMLPosition(       *IP
					                                ,   OP
					                                ,   Flags       );
			}
			// ****************************************************************
			// Here, the new state of motion, that is
			//
			// - OP->NewPositionVector
			// - OP->NewVelocityVector
			// - OP->NewAccelerationVector
			//
			// can be used as input values for lower level controllers. In the
			// most simple case, a position controller in actuator space is
			// used, but the computed state can be applied to many other
			// controllers (e.g., Cartesian impedance controllers,
			// operational space controllers).
			// ****************************************************************

			// ****************************************************************
			// Feed the output values of the current control cycle back to
			// input values of the next control cycle

			*IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
			*IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
			*IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

			// update a piece of trajectoy based on the recent calculations
			ref.linear.x = IP->CurrentPositionVector->VecData[0];
			ref.linear.y = IP->CurrentPositionVector->VecData[1];
			ref.linear.z = IP->CurrentPositionVector->VecData[2];
			ref.angular.x = IP->CurrentPositionVector->VecData[3];
			ref.angular.y = IP->CurrentPositionVector->VecData[4];
			ref.angular.z = IP->CurrentPositionVector->VecData[5];

			//dbg.angular.x = ctr;
			reflexxes_pub.publish(ref);
		
	    }else{
	    	
	    	if (is_new_plan(plan, prev_plan)){
		    	std::cout << "\n\n\n\n\n\n\n Initialized the plan again \n\n\n\n\n\n\n\n" << std::endl;
		    	ResultValue = 0 ;
		    	ctr = 2;
			initialize_plan(IP);
			number_of_points = plan.points.size();
			std::cout << "New plan with " << number_of_points << " points"<< std::endl;
			prev_plan = plan;
	    	}
	    }
		
		loop_rate.sleep();
		
		ros::spinOnce();
		

	    // ********************************************************************
	    // Deleting the objects of the Reflexxes Motion Library end terminating
	    // the process
    }
    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
}


