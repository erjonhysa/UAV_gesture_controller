/**@file controller.cpp
*Ros node to control a drone with pre-designed gesture
*/

//===============================================================================//
// Name			: controller.cpp
// Author(s)		: Erjon Hysa
// Description		: Ros node to control a drone with pre-designed gesture
//===============================================================================//

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <asctec_hl_comm/mav_ctrl.h>
#include "geometry_msgs/PoseStamped.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>
//#include <asctec_hl_comm/mav_ctrl_motors.h>

using namespace std;

//publisher
ros::Publisher pub; 
//global variables of position
double position_x = 0; 
double position_y = 0;
double acc_x;
double acc_y;
double vel_yaw;
//area for the integration of yaw
double area_integration = 0;

//for debug
ofstream myfile("smartwatch_data.txt", ios::out | ios::binary);
ofstream myfile2("control_data.txt", ios::out | ios::binary);



//! function that modify smartwatch data
//! @param[in] my_value 	value that comes from the smartwatch
//! @param[in] max		upper limit		
//! @param[in] min_error	minimum treshold 
//! @param[in] min		lower limit
float get_var_inRange (float my_value, float min, float max, float min_error)
{
	if(my_value > 0){
		if(my_value < min_error) return 0;
		else if(my_value > max) return 0.5;
		else{
			float diff1,diff2,div,mul;
			diff1 = my_value - min_error;
			diff2 = max - min_error;
			div = diff1/diff2;
			mul = div*0.5;
			return mul;
			}
	}else{
		if(my_value > -min_error) return 0;		
		else if(my_value < min) return -0.5;
		else{
			float diff1,diff2,div,mul;
			diff1 = my_value + min_error;
			diff2 = min + min_error;
			div = diff1 / diff2;
			mul = div*(-0.5);
			return mul;
			} 
	}
}

//callback for position messages of drone
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	position_x = msg->pose.position.x;
	position_y = msg->pose.position.y;
}

//callback for smartwatch messages
void messageCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	//for debug
	myfile << msg -> position.x << "," <<msg -> position.y <<","<<msg -> position.z <<"\n";
	
	double vel_z;
	double area_temp;
	//modify messages
	
	//pitch
	acc_x = get_var_inRange(msg -> position.y, -1.95, 2, 0.9);	
	//roll
	acc_y = get_var_inRange(msg -> position.x, -1.6, 3, 0.9);	
	//integration of acc for yaw
	vel_z = msg -> position.z;	
	area_temp = vel_z * 0.025;
	area_integration = area_integration + area_temp;
	vel_yaw = get_var_inRange(area_integration, -1.7, 1.7, 1.0);

	//for debug
	//myfile2 << acc_y << "," <<	acc_x << "," << vel_yaw <<"\n";
	//pub.publish(cmd_vel);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	//publisher
	pub = n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",10);
	//first subscriber	
	ros::Subscriber sub = n.subscribe("wearami_acc",10,messageCallback); //ricorda
	//second subscriber  
	ros::Subscriber sub2 = n.subscribe("fcu/current_pose",10,callback);  
	ros::Rate loop_rate(40); 
	while (ros::ok())
	{	
		//message to be published on drone's node
		asctec_hl_comm::mav_ctrl cmd_vel;
		//if the drone is within the desired position we can publish
		//the message 
		if(fabs(position_x) < 0.7 && fabs(position_y) < 0.7)
		{
		
			cmd_vel.type = 1;
			//pitch
			cmd_vel.x = acc_x;
			//roll
			cmd_vel.y = acc_y;
			cmd_vel.z = 0;		
			//yaw
			cmd_vel.yaw = vel_yaw; 	
		
			//publish of the message 
			//for debug
			//myfile2 << cmd_vel.y << "," <<	cmd_vel.x << "," << cmd_vel.yaw <<"\n";
			pub.publish(cmd_vel); 		
		}else{	
			//stop message
			//we set velocities to 0
			asctec_hl_comm::mav_ctrl cmd_vel;
			cmd_vel.type = 2;
			cmd_vel.x = 0;
       			cmd_vel.y = 0;
        		cmd_vel.z = 0;
        		cmd_vel.yaw = 0;
			pub.publish(cmd_vel);
		}
		ros::spinOnce();  
		loop_rate.sleep();
	}
	return 0;
}

