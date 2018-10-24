/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#ifndef EMBD_MOBILE_ROBOT_H
#define EMBD_MOBILE_ROBOT_H
/****************************************************************************/
#include <stdio.h>
#include <math.h>
#include <embdCOMMON.h>  //./libs/embedded
//#include <embdMATH.h>  //./libs/embedded
/****************************************************************************/
typedef enum{
	none = 0,
	differential,
	omnidirectional,
	mecanum,
}mobile_drive;

typedef struct wheel{
	int index;
	float diameter;
	float radius;
	float width;

	float max_a_vel; // rad/s
	float max_a_acc; // rad/s^2
	float max_a_dec;
	float max_a_jrk; // rad/s^3
	float max_rpm; // rpm

	float a_vel;
	float a_acc;
	float a_dec;
	float a_jrk;

	float prev_a_vel;
	float prev_a_acc;
	float prev_a_dec;
	float prev_a_jrk;

	float feed_a_vel;
	float prev_feed_a_vel;

	float enc_val;
	float prev_enc_val;

}robot_wheel;

typedef struct robot_physical{
	mobile_drive drive_type;
	int wheel_number;
	int gear_ratio;

	/* [kg] */ 
	float weight;
	float payload; 

	/* [m] */
	float length; 
	float width;  
	float height; 
	float wheel_radius;
	float wheel_diameter;
	float wheel_width;

	float max_vel; // [m/s]
	float max_acc; // [m/s^2]
	float max_jrk; // [m/s^3]
}robot_phy;

typedef struct pos{
	float x; //[m]
	float y; //[m]
	float theta; //[rad]
}robot_pos;
/****************************************************************************/
void print_robot_physical_limits_info(robot_phy phy);
void print_robot_wheel_info(robot_wheel wheel);
void wheel_controller(robot_wheel* wheels, float Ts, float target_vel);

/****************************************************************************/
#endif // EMBD_MOBILE_ROBOT_H