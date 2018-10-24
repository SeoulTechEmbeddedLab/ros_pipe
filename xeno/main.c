/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to test EtherCAT master protocol using the IgH EtherCAT master userspace library.	
 *  
 *  
 *  IgH EtherCAT master library for Linux is found at the following URL: 
 *  <http://www.etherlab.org/en/ethercat>
 *
 *
 *
 *
 *  2018 Raimarius Delgado
*/
/****************************************************************************/
#include <embdECATM.h> //./libs/ecatservo 
#include <embdCONIO.h>  //./libs/embedded
#include <embdCOMMON.h>  //./libs/embedded
#include <embdMATH.h>  //./libs/embedded
#include <mecanum.h>
/*****************************************************************************/
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <pthread.h>
/*****************************************************************************/
/* Xenomai */
/*****************************************************************************/
#include <xeno_task.h>
#include <native/pipe.h>
#include <native/event.h>
#include <native/mutex.h>

#define ECATCTRL 1
#define ECATCTRL_TASK_PRIORITY	(99) // xeno: 99 , preempt: 80
#define ECATCTRL_TASK_PERIOD	(1000000L)

#define INPTCTRL 1
#define INPTCTRL_TASK_PRIORITY	(50)
#define INPTCTRL_TASK_PERIOD	(1000000000L)

#define ROSBTN 1
#define ROSBTN_TASK_PRIORITY	(90)
#define ROSBTN_TASK_PERIOD	(10000000L)

#define ROSJOYCTRL 1
#define ROSJOYCTRL_TASK_PRIORITY	(80)

RT_TASK TskEcatCtrl;
RT_TASK TskInptCtrl;
RT_TASK TskRosBtn;
RT_TASK TskRosJoy;

RTIME RtmEcatMasterAppTime;
RTIME RtmEcatMasterStartTime;

#define PIPE_BTN 0
RT_PIPE pipe_btn;

#define PIPE_JOY 1
RT_PIPE pipe_joy;

#define PIPE_ODOM 2
RT_PIPE pipe_odom;

#define EVENT_INIT 0x0
#define EVENT_MODE EV_PRIO

#define EVENT_SIGNAL_JOY 0x1
#define EVENT_SIGNAL_NAV 0x2
#define EVENT_SIGNAL_PER1 0x3
#define EVENT_SIGNAL_PER2 0x4
//#define DEBUG

RT_EVENT ev_mode;
RT_MUTEX mtx_vel;
/*****************************************************************************/
/* global variables */
/*****************************************************************************/

/* EtherCAT variables */
unsigned short mtrStatusWord[LSMECAPION_SLAVENUM] = {0,};
unsigned short mtrPrevStatusWord[LSMECAPION_SLAVENUM] = {0,}; 
int mtrVelocityRpm[LSMECAPION_SLAVENUM] = {0,};
float mtrVelocityRads[LSMECAPION_SLAVENUM] = {0,};

typedef enum{
	READY = 0,
	RUN,
	STOP,
} SERVO_OP;

SERVO_OP mtrState = READY;

/* ECAT_STATE: EtherCAT State Machine (libs/ecatservo/embdECATM.h)
 * unsigned int  master_state;
 * unsigned int	 slave_state;
 * unsigned int	 slave_number */
ECAT_STATE	EcatState;
FLAG		quitFlag	= OFF;
char		InptChar	= 0; // command character

/* Mecanum Control Variables */
#define SHOW_MECA_INFO
mecanum_robot MECAT; // ./mecanum.h
float turbo_rate = 0.8;
float joint_fl = 0.,joint_fr = 0.,joint_rl = 0.,joint_rr = 0.; 

/* temporary */
float Vx=0.,Vy=0.,Vc=0.;
float r = 0.1015;
/*****************************************************************************/
void XenoInit();
void XenoStart();
void XenoQuit();
void DoInput();
void SignalHandler(int signum);
int check_all_zeros(int arraySample[], int n);
void mecanum_drive(float targetFL, float targetFR, float targetRL, float targetRR,float turbos);
void update_mecanum_position_encoder(float feedbacks[]);
/****************************************************************************/
void EcatCtrlTask(void *arg){

	int iTaskTick = 0;
	int iSlaveCnt = 0;
	rt_pipe_create(&pipe_odom,"Odometry Pipe", PIPE_ODOM, 0);
	char odom_buff[100];
	
	while (1) {
		rt_task_wait_period(NULL);

    	/* Receive Process Data */
		EcatReceiveProcessDomain();
    	/* check process data state (optional) */
		EcatState = EcatStatusCheck(); 

		/* Character Input Command */
		DoInput();  
		/*
		* Do reading of the current process data from here
		* before processing and sending to Tx
		*/
		if (EcatState.master_state == OP && EcatState.slave_state == OP){
			for(iSlaveCnt=0;iSlaveCnt<LSMECAPION_SLAVENUM;++iSlaveCnt){
				mtrVelocityRads[iSlaveCnt] = lsmecaGetActualAngularVelocityN(iSlaveCnt); 
				mtrVelocityRpm[iSlaveCnt] = lsmecaGetActualVelocityN(iSlaveCnt);
				mtrStatusWord[iSlaveCnt] = 	lsmecaGetStatusWordN(iSlaveCnt);
			
				if(mtrPrevStatusWord[iSlaveCnt] == mtrStatusWord[iSlaveCnt])
					continue;
				switch(mtrStatusWord[iSlaveCnt]){
					case LSMECA_SWITCH_ON_DISABLED:
					case LSMECA_READY_TO_SWITCH_ON:
					case LSMECA_SWITCH_ON_ENABLED:
						mtrState = READY;
						break;
					case LSMECA_OPERATION_ENABLED:
						mtrState = RUN;
						break;
					case LSMECA_FAULT:
						lsmecaFaultReset(iSlaveCnt);
						break;
					default:
						printf("unknown state\n");
						break;
				}
				mtrPrevStatusWord[iSlaveCnt] = mtrStatusWord[iSlaveCnt];
			}
		}
		update_mecanum_position_encoder(mtrVelocityRads);
		sprintf(odom_buff,"%f %f %f",MECAT.pose.x,MECAT.pose.y,MECAT.pose.theta);
		rt_pipe_write(&pipe_odom,odom_buff,sizeof(odom_buff),P_NORMAL);
		switch(mtrState){
			case READY:
				break;
			case RUN:
			rt_mutex_acquire(&mtx_vel,TM_INFINITE);
				mecanum_drive(joint_fl,joint_fr, joint_rl,joint_rr,turbo_rate);
			
			rt_mutex_release(&mtx_vel);
				break;
			case STOP:
				mecanum_drive(0,0,0,0,turbo_rate);
				break;
			default:
				break;
		}
		/* write application time to master */
		RtmEcatMasterAppTime = rt_timer_read();
		EcatWriteAppTimeToMaster((uint64_t)RtmEcatMasterAppTime);

		/* Send process data */
		EcatSendProcessDomain();
		/*
		* Do other processes starting here
		*/
#ifdef DEBUG //  0 to omit : 1 for processing
			if (!(iTaskTick % FREQ_PER_SEC(ECATCTRL_TASK_PERIOD))){
			/*Do every 1 second */
			//rt_printf("Task Duration: %d s\n", iTaskTick/FREQ_PER_SEC(ECATCTRL_TASK_PERIOD));
			rt_printf("EtherCAT Control Task:\n");
			rt_printf("FL:%f FR:%f RL:%f RR:%f \n", MECAT.FL.feed_a_vel,MECAT.FR.feed_a_vel,MECAT.RL.feed_a_vel,MECAT.RR.feed_a_vel);
			rt_printf("Vx:%f Vy:%f Vc:%f Theta_dot:%f \n", MECAT.vx,MECAT.vy,MECAT.feed_vel, MECAT.feed_theta_dot);
			rt_printf("x:%f y:%f theta:%f \n", MECAT.pose.x,MECAT.pose.y,MECAT.pose.theta);
			}
#endif
		iTaskTick++;
	}
}
/****************************************************************************/
void InptCtrlTask(void *arg){
	while (1) {
	
		rt_task_wait_period(NULL); 
		InptChar = getche(); /* libs/embedded/embdCONIO.h */
   	}
}
/****************************************************************************/
void RosBtnTask(void *arg){
	rt_pipe_create(&pipe_btn,"Button Pipe", PIPE_BTN, 0);
	
    char btn_buff[100];
    int fTurbo=0, fEmergency=0, fServoOff=0, fMode=0, fServoReady=0, fServoOn=0;
    int iModeCnt = 0, iPrevModeCnt = 0, iModeDelay = 0;
    int iCnt = 0;
    FLAG sservo = OFF;
	while (1) {
		rt_task_wait_period(NULL);
		rt_pipe_read(&pipe_btn,btn_buff,sizeof(btn_buff),TM_NONBLOCK);
	      /*Button mapping
		   * joy->buttons[0]  = Button 1 = Turbo
		   * joy->buttons[1]  = Button 2 = Emergency Off
		   * joy->buttons[8]  = Button 9 = Servo Off
		   * joy->buttons[9]  = Button 10 = Mode Select
		   * joy->buttons[10] = Button 11 = Servo Ready
		   * joy->buttons[11] = Button 12 = Servo On
		  */
        sscanf(btn_buff,"%d %d %d %d %d %d",&fTurbo,&fEmergency,&fServoOff,&fMode,&fServoReady,&fServoOn);

#ifdef DEBUG 
	    if (!(iCnt % FREQ_PER_SEC(ROSBTN_TASK_PERIOD))){
	    	rt_printf("ROS Button Task:\n");
		    rt_printf("Turbo:%d Emergency:%d Servooff:%d\n",fTurbo,fEmergency,fServoOff);
	        rt_printf("Mode:%d ServoReady:%d ServoOn:%d\n",fMode,fServoReady,fServoOn);
        }
#endif
        	/* Robot runs 80% maximum speed all the time 
			 * 100% when turbo is pressed
        	*/
        	if(fTurbo)
	        	turbo_rate = 1.0;
	        else
	        	turbo_rate = 0.8;

			if(fServoReady)
        		InptChar = 'r';

	        if(fServoOn)
	        	InptChar = 'o';

	        if(fServoOff)
	        	InptChar = 'x';
	        if(fEmergency)
	        	InptChar = 'q';

	        if(fMode){
	        	if (!(iModeDelay++ % 200)) //hold for 2 seconds before change
	        		iModeCnt++; 
	        }
	       	
	       	/* iModeCnt = 0 -> nothing
	       	 * iModeCnt = 1 -> Case 1
	       	 * iModeCnt = 2 -> Case 2
			 */ 

	        if (iPrevModeCnt == iModeCnt)
	        	continue;
	        else
	        	/* print status change */
	        	rt_printf("Current Mode: %d\n", iModeCnt);  	     
	   	    
	   	    switch(iModeCnt){
	        	case 0: 
	        		break;
	        	case 1:
	        		rt_event_signal(&ev_mode,EVENT_SIGNAL_JOY);
	        		break;
	        	case 2:
	        		//rt_event_signal(&ev_mode,EVENT_SIGNAL_NAV); 
	        		break;
	        	default:
	        		iModeCnt = 0; 
	        		break;		
	        }
	        iPrevModeCnt = iModeCnt;
        iCnt++;
   	}
}
/****************************************************************************/
void RosJoyTask(void *arg){
	rt_event_create(&ev_mode,"Mode Selection Event Flag",EVENT_INIT,EVENT_MODE);
	rt_pipe_create(&pipe_joy,"Joystick Pipe", PIPE_JOY, 0);
	unsigned long mask_ret;
	char btn_joy[100];
	float joy_x=0.,joy_y=0.,joy_z=0.;
	float max_wheel_vel = MECAT.RL.max_a_vel;
	int iCnt = 0;
	float noise_filter = 0.4;
	
	while (1) {
		rt_event_wait(&ev_mode,EVENT_SIGNAL_JOY|EVENT_SIGNAL_PER1,&mask_ret,EV_ANY,TM_INFINITE);

		rt_pipe_read(&pipe_joy,btn_joy,sizeof(btn_joy),TM_INFINITE);	
		sscanf(btn_joy,"%f %f %f",&joy_x,&joy_y,&joy_z);

		/* Joystick noise cancellation */
		if(joy_x >= -noise_filter && joy_x <= noise_filter )
			joy_x = 0.;
		if(joy_y >= -noise_filter && joy_y <= noise_filter )
			joy_y = 0.;
		if(joy_z >= -noise_filter && joy_z <= noise_filter )
			joy_z = 0.;	

		rt_printf("joy_x:%f joy_y:%f joy_z:%f\n",joy_x,joy_y,joy_z);
		if(mtrState == RUN)
		{
		rt_mutex_acquire(&mtx_vel,TM_INFINITE);
			joint_fl = (joy_x  + joy_y - joy_z) * max_wheel_vel;
			joint_fr = (-joy_x + joy_y + joy_z) * max_wheel_vel;
			joint_rl = (-joy_x + joy_y - joy_z) * max_wheel_vel;
			joint_rr = (joy_x  + joy_y + joy_z) * max_wheel_vel;
		rt_mutex_release(&mtx_vel);
		}
#ifdef DEBUG 
	    if (!(iCnt % FREQ_PER_SEC(ROSBTN_TASK_PERIOD))){
		    rt_printf("ROS Joystick Task:\n");
		    rt_printf("joy_x:%f joy_y:%f joy_z:%f\n",joy_x,joy_y,joy_z);
	        rt_printf("FL:%f FR:%f RL:%f RR:%f\n",joint_fl,joint_fr,joint_rl,joint_rr);
        }
#endif
	++iCnt;
   	}
}
/****************************************************************************/
int main(int argc, char **argv){
   
	int ret = 0;
	rt_print_auto_init(1); //RTDK
	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
	signal(SIGINT, SignalHandler);	 

	RtmEcatMasterStartTime = rt_timer_read();
	/* EtherCAT Init */
   	if ((ret = EcatInit(ECATCTRL_TASK_PERIOD,LSMECA_CYCLIC_VELOCITY,(uint64_t)RtmEcatMasterStartTime))!=0){
		fprintf(stderr, 
			"Failed to Initiate IgH EtherCAT Master!\n");
		return ret;
	}

	/* Mecanum Robot */
	init_mecanum(&MECAT, ECATCTRL_TASK_PERIOD);
#ifdef SHOW_MECA_INFO	
	mecanum_print_info(&MECAT);
#endif
	mlockall(MCL_CURRENT|MCL_FUTURE); //Lock Memory to Avoid Memory Swapping

	/* RT-task */
	XenoInit();
	XenoStart();

	while (1) {
		usleep(1);
		if (quitFlag) break;
	}

	XenoQuit();
	EcatQuit();

	return ret;
}
/****************************************************************************/
void DoInput(){

	switch(InptChar)
	{
	case 'Q'	:
	case 'q'	:
		lsmecaShutDown(_ALLNODE);
		quitFlag = ON;
		break;
	case 'r'	:
	case 'R'	:
		lsmecaReady(_ALLNODE);
		break;
	case 'O'	:
	case 'o'	:
		lsmecaServoOn(_ALLNODE);
		break;
	case 'x'	:
	case 'X'	:
		mtrState = STOP;
		break;
	default		:
		break;
	}
	InptChar = 0;
}
/****************************************************************************/
void SignalHandler(int signum){
		quitFlag=ON;
}
/****************************************************************************/
void XenoInit(){

	//rt_print_auto_init(1); //RTDK
	rt_mutex_create(&mtx_vel,"Target Velocity Mutex");

	printf("Creating Real-time task(s)...");
		create_rt_task(&TskEcatCtrl,"EtherCAT Control Task", ECATCTRL_TASK_PRIORITY);
		create_rt_task(&TskInptCtrl,"Keyboard Input Task", INPTCTRL_TASK_PRIORITY);
		create_rt_task(&TskRosBtn,"ROS Button Task", ROSBTN_TASK_PRIORITY);
		create_rt_task(&TskRosJoy,"ROS Joystick Control Task", ROSJOYCTRL_TASK_PRIORITY);
	printf("OK!\n");

	printf("Making Real-time task(s) Periodic...");
		set_rt_task_period(&TskEcatCtrl, ECATCTRL_TASK_PERIOD);
		set_rt_task_period(&TskInptCtrl, INPTCTRL_TASK_PERIOD);
		set_rt_task_period(&TskRosBtn, ROSBTN_TASK_PERIOD);
	printf("OK!\n");
}
/****************************************************************************/
void XenoStart(){
	printf("Starting Xenomai Real-time Task(s)...");
		start_rt_task(ECATCTRL,&TskEcatCtrl,&EcatCtrlTask);
		start_rt_task(INPTCTRL,&TskInptCtrl,&InptCtrlTask);
		start_rt_task(ROSBTN,&TskRosBtn,&RosBtnTask);
		start_rt_task(ROSJOYCTRL,&TskRosJoy,&RosJoyTask);
	printf("OK!\n");
}
/****************************************************************************/
void XenoQuit(void){
	rt_task_suspend(&TskRosJoy);
	rt_task_suspend(&TskRosBtn);
	rt_task_suspend(&TskInptCtrl);
	rt_task_suspend(&TskEcatCtrl);

	rt_mutex_delete(&mtx_vel);
	rt_event_delete(&ev_mode);

	rt_pipe_delete(&pipe_odom);
	rt_pipe_delete(&pipe_btn);
	rt_pipe_delete(&pipe_joy);



	rt_task_delete(&TskRosJoy);
	rt_task_delete(&TskRosBtn);
	rt_task_delete(&TskInptCtrl);
	rt_task_delete(&TskEcatCtrl);
	printf("\033[%dm%s\033[0m",95,"Xenomai Task(s) Deleted!\n");
}
/****************************************************************************/
void mecanum_drive(float targetFL, float targetFR, float targetRL, float targetRR,float turbos)
{
	int gear_ratio = MECAT.physical_limits.gear_ratio;

	/*Trajectory generator for trapezoidal velocity*/ 
	mecanum_joint_control(&MECAT,targetFL,targetFR,targetRL,targetRR);

	lsmecaSetAngularVelocity(0,MECAT.FL.a_vel  * gear_ratio * turbos);
	lsmecaSetAngularVelocity(1,-MECAT.FR.a_vel * gear_ratio * turbos);
	lsmecaSetAngularVelocity(2,MECAT.RL.a_vel  * gear_ratio * turbos);
	lsmecaSetAngularVelocity(3,-MECAT.RR.a_vel * gear_ratio * turbos); 
}

/****************************************************************************/
void update_mecanum_position_encoder(float feedbacks[])
{
	int gear_ratio = MECAT.physical_limits.gear_ratio;

	MECAT.FL.feed_a_vel = feedbacks[0]  / gear_ratio;
	MECAT.FR.feed_a_vel = -feedbacks[1] / gear_ratio;
	MECAT.RL.feed_a_vel = feedbacks[2]  / gear_ratio;
	MECAT.RR.feed_a_vel = -feedbacks[3] / gear_ratio;

	mecanum_forward_kinematics(&MECAT);
}
/****************************************************************************/
int check_all_zeros(int arraySample[], int n)
{
	int i,sum=0;
	
	for (i = 0; i < n; ++i) {
  			sum += MathAbsValI(arraySample[i]);
  	}
	return sum; 
}
/****************************************************************************/

