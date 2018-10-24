#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
 #include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>

#define PIPE_BTN "/dev/rtp0"
#define PIPE_JOY "/dev/rtp1"

int btn_pipe_fd;
int joy_pipe_fd;

class JoyPrint{
public:
JoyPrint();

private:
void joyCallback(const sensor_msgs::Joy::ConstPtr&joy);

ros::NodeHandle nh_;

int linear_, angular_;
double l_scale_, a_scale_;

ros::Subscriber joy_sub_;
};

JoyPrint::JoyPrint():
linear_(1),
angular_(2){
nh_.param("axis_linear", linear_, linear_);
nh_.param("axis_angular",angular_,angular_);
nh_.param("scale_angular",a_scale_,a_scale_);
nh_.param("scale_linear",l_scale_,l_scale_);

joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,&JoyPrint::joyCallback, this);
}

void JoyPrint::joyCallback(const sensor_msgs::Joy::ConstPtr&joy){
	
	char buf_joy[100] = {0,} , buf_btn[100] = {0,};
	
	float x = 0., y = 0., z = 0.;
	
	sprintf(buf_btn,"%d %d %d %d",joy->buttons[0],joy->buttons[1],joy->buttons[2],joy->buttons[3]);
	write(btn_pipe_fd,buf_btn,sizeof(buf_btn));


	x = joy->axes[0];
	y = joy->axes[1];
	z = joy->axes[2];
	

	sprintf(buf_joy,"%f %f %f",x,y,z);
	write(joy_pipe_fd,buf_joy,sizeof(buf_joy));
	

	//printf("buttons : (%d , %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d)\n", joy->buttons[0],joy->buttons[1],joy->buttons[2],joy->buttons[3],joy->buttons[4],joy->buttons[5],joy->buttons[6],joy->buttons[7],joy->buttons[8],joy->buttons[9],joy->buttons[10],joy->buttons[11]);
	//printf("sensitve : %lf\n", joy->axes[3]);

}

int main(int argc, char** argv){
ros::init(argc, argv, "joy_print");
	
	JoyPrint joy_print;
	
	btn_pipe_fd = open(PIPE_BTN,O_RDWR);
	joy_pipe_fd = open(PIPE_JOY,O_RDWR);

ros::spin();
return 0;
}
