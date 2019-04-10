#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <termios.h>

/*
char getch()
{
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
//	lseek(0, -1L, SEEK_END);
	if (read(0, &buf, 1) < 0)
		perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");
	return (buf);
}
*/

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "key_input_node");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Bool>("/vins_restart", 1000);
	ros::Publisher pub_imu_switch = n.advertise<std_msgs::Bool>("/vins_imu_switch", 1000);
	ros::Publisher pub_cam_switch = n.advertise<std_msgs::Bool>("/vins_cam_switch", 1000);

	ros::Rate loop_rate(10);
	std_msgs::Bool restart;
	std_msgs::Bool imu_switch;
	std_msgs::Bool cam_switch;

	while (ros::ok())
	{
		int c = 0;
		switch((c=getch()))
		{
			case 'r':
				ROS_INFO("restart");//cout << endl << "Up" << endl;//key up
				restart.data = true;
				pub.publish(restart);
				break;
			case 'i':
				ROS_INFO("use imu");
				imu_switch.data = true;
				pub_imu_switch.publish(imu_switch);
				break;
			case 'o':
				ROS_INFO("disable imu");
				imu_switch.data = false;
				pub_imu_switch.publish(imu_switch);
				break;
			case 's':
				ROS_INFO("stereo");
				cam_switch.data = true;
				pub_cam_switch.publish(cam_switch);
				break;
			case 'm':
				ROS_INFO("mono");
				cam_switch.data = false;
				pub_cam_switch.publish(cam_switch);
				break;
			default:
				ROS_INFO("other input");
				break;
		}


		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
