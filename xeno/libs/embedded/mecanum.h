/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#ifndef EMBD_MECANUM_H
#define EMBD_MECANUM_H
/****************************************************************************/
#include <embdCOMMON.h>  //./libs/embedded
//#include <embdMATH.h>  //./libs/embedded
#include <mobile_robot.h>
/****************************************************************************/

typedef struct mecanum_robot{
	robot_phy physical_limits;
	robot_wheel FL,FR,RL,RR;  // ** try to use array next time **
	robot_pos pose;
	robot_pos prev_pose;

	float Ts; // sampling time[s] 
	float vel; // [m/s]
	float vx;
	float prev_vx;
	float vy;
	float prev_vy;
	float feed_vel;
	float theta_dot; // [theta/s]
	float feed_theta_dot;
}mecanum_robot;

/****************************************************************************/
void init_mecanum(mecanum_robot *foo, int sampling_ns);
void mecanum_print_info(mecanum_robot *foo);
void mecanum_joint_control(mecanum_robot *foo,float front_left,float front_right,float rear_left, float rear_right);
void mecanum_forward_kinematics(mecanum_robot *foo);
/****************************************************************************/
#endif // EMBD_MECANUM_H