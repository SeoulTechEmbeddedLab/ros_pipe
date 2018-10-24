/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#include <mobile_robot.h>
/****************************************************************************/

char* _check_drive_type(mobile_drive drive)
{
	switch(drive & 0xff){
		case none:
			return "Not defined";
			break;
		case differential:
			return "Differential";
			break;
		case omnidirectional:
			return "Omnidirectional";
			break;
		case mecanum:
			return "Mecanum";
			break;
		default:
			return "Not a moble robot";
			break;
	}
}

void print_robot_physical_limits_info(robot_phy phy)
{
	printf("########################################################################\n");
	printf("#####Physical limits####\n");
	printf("Drive type:%s \n",_check_drive_type(phy.drive_type));
	printf("No. of wheels:%d Gear Ratio:%d\n",phy.wheel_number,phy.gear_ratio);
	printf("Weight [kg]:%.03f Payload [kg]:%.03f\n",phy.weight,phy.payload);
	printf("Lenght[m]:%.03f Width[m]:%.03f Height[m]:%.03f\n",phy.length,phy.width,phy.height);
	printf("Maximum Vel[m/s]:%.03f Maximum Acc[m/s2]:%.03f Max Jerk[m/s3]:%.03f\n", phy.max_vel, phy.max_acc,phy.max_jrk);
}
/****************************************************************************/
void print_robot_wheel_info(robot_wheel wheel)
{
	printf("########################################################################\n");
	printf("#####Wheel %d information####\n",wheel.index);
	printf("Diameter[m]:%.03f Radius[m]:%.03f \n",wheel.diameter, wheel.radius);
	printf("Max A_Vel:%.03f Max A_Acc:%.03f Max A_Dec:%.03f Max A_Jerk:%.03f\n",wheel.max_a_vel,wheel.max_a_acc,wheel.max_a_dec,wheel.max_a_jrk);
}
/****************************************************************************/
void wheel_controller(robot_wheel* wheels, float Ts, float target_vel)
{
	if (wheels->prev_a_vel <= target_vel)
		wheels->a_vel = wheels->prev_a_vel + (wheels->max_a_acc*Ts);
	else if(wheels->prev_a_vel > target_vel)
		wheels->a_vel = wheels->prev_a_vel - (wheels->max_a_dec*Ts); 
	
	wheels->prev_a_vel = wheels->a_vel;

	if (wheels->a_vel > wheels->max_a_vel)
		wheels->a_vel =  wheels->max_a_vel;
	else if (wheels->a_vel < -wheels->max_a_vel)
		wheels->a_vel =  -wheels->max_a_vel;
}
/****************************************************************************/