/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#include <mecanum.h>
/****************************************************************************/
void init_mecanum_wheels(robot_wheel *wheel,int index);
/****************************************************************************/
void init_mecanum(mecanum_robot *foo, int sampling_ns)
{

	/* Physical limits */
	foo->physical_limits.drive_type = mecanum;
	foo->physical_limits.wheel_number = 4;
	foo->physical_limits.gear_ratio = 100;

	foo->physical_limits.weight = 59.7;
	foo->physical_limits.payload = 190.3;
	
	foo->physical_limits.length = 0.775; //including wheels | without: 0.696
	foo->physical_limits.width = 0.675; // including wheels | without: 0.452
	foo->physical_limits.height = 0.125;
	foo->physical_limits.wheel_diameter = 0.203;
	foo->physical_limits.wheel_radius = 0.203/2;
	foo->physical_limits.wheel_width = 0.078;

	foo->physical_limits.max_vel = 0.317;
	foo->physical_limits.max_acc = 0. ;
	foo->physical_limits.max_jrk = 0. ;

	/*wheel information*/
	init_mecanum_wheels(&foo->FL,0); // front left
	init_mecanum_wheels(&foo->FR,1); // front right
	init_mecanum_wheels(&foo->RL,2); // rear left
	init_mecanum_wheels(&foo->RR,3); // rear right

	/* Position */
	foo->pose.x = 0;
	foo->pose.y = 0;
	foo->pose.theta = 0;

	foo->prev_pose.x = 0;
	foo->prev_pose.y = 0;
	foo->prev_pose.theta = 0;

	/* Etc */
	foo->Ts = (float)(1. / FREQ_PER_SEC(sampling_ns));
	foo->vel = 0.;
	foo->feed_vel = 0.;
	foo->theta_dot = 0.;
	foo->feed_theta_dot =0.;
	foo->vx = 0.;
	foo->prev_vx = 0.;
	foo->vy = 0.;
	foo->prev_vy = 0.; 
}
/****************************************************************************/
void init_mecanum_wheels(robot_wheel *wheel,int index)
{
	wheel->index = index;
	wheel->diameter = 0.203;
	wheel->radius = wheel->diameter/2;
	wheel->width = 0.078;

	wheel->max_a_vel = 3.142;
	wheel->max_a_acc = 3.142; // 1sec @ 1ms 
	wheel->max_a_dec = 3.142;//15.708; // 200ms @ 1ms 
	wheel->max_a_jrk = 0.;

	wheel->a_vel = 0.;
	wheel->a_acc = 0.;
	wheel->a_dec = 0.;
	wheel->a_jrk = 0.;

	wheel->prev_a_vel = 0.;
	wheel->prev_a_acc = 0.;
	wheel->prev_a_dec = 0.;
	wheel->prev_a_jrk = 0.;

	wheel->feed_a_vel = 0;
	wheel->prev_feed_a_vel = 0;

	wheel->enc_val = 0;
	wheel->prev_enc_val = 0;

}
/****************************************************************************/
void mecanum_print_info(mecanum_robot *foo)
{
	printf("########################################################################\n");
	printf("Information about the mobile robot:\n");
	print_robot_physical_limits_info(foo->physical_limits);
	print_robot_wheel_info(foo->FL);
	print_robot_wheel_info(foo->FR);
	print_robot_wheel_info(foo->RL);
	print_robot_wheel_info(foo->RR);
}
/****************************************************************************/
void mecanum_joint_control(mecanum_robot *foo,float front_left,float front_right,float rear_left, float rear_right)
{
	wheel_controller(&foo->FL,foo->Ts, front_left);
	wheel_controller(&foo->FR,foo->Ts, front_right);
	wheel_controller(&foo->RL,foo->Ts, rear_left);
	wheel_controller(&foo->RR,foo->Ts, rear_right);
}
/****************************************************************************/
void mecanum_forward_kinematics(mecanum_robot *foo)
{
	float r = foo->physical_limits.wheel_radius;
	float l = (foo->physical_limits.width/2)  - (foo->physical_limits.wheel_width/2);
	float L = (foo->physical_limits.length/2) - r;

	foo->vx = (r/4) * (foo->FL.feed_a_vel - foo->FR.feed_a_vel - foo->RL.feed_a_vel + foo->RR.feed_a_vel);
	foo->vy = (r/4) * (foo->FL.feed_a_vel + foo->FR.feed_a_vel + foo->RL.feed_a_vel + foo->RR.feed_a_vel);
	foo->feed_theta_dot = r/(4*(l+L)) * (-foo->FL.feed_a_vel + foo->FR.feed_a_vel - foo->RL.feed_a_vel + foo->RR.feed_a_vel);

	foo->pose.x = foo->pose.x + ((foo->prev_vx + foo->vx) * foo->Ts);
	foo->pose.y = foo->pose.y + ((foo->prev_vy + foo->vy) * foo->Ts); 
	foo->pose.theta = foo->pose.theta + foo->feed_theta_dot * foo->Ts;
		if(foo->pose.theta >= TWO_PI)
			foo->pose.theta = foo->pose.theta - TWO_PI;
		else if (foo->pose.theta < -TWO_PI)
			foo->pose.theta = foo->pose.theta + TWO_PI;

	foo->feed_vel = sqrt((foo->vx*foo->vx) + (foo->vy*foo->vy));

	foo->prev_vx = foo->vx;
	foo->prev_vy = foo->vy;

}
/****************************************************************************/