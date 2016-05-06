#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <termios.h>
using namespace std;

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

int main(int argc, char **argv) {
	ros::init(argc, argv, "main_node");
	ros::NodeHandle n;
	ros::Publisher keyPublisher = n.advertise<std_msgs::String>("wheelSpeeds", 1);
	while (ros::ok())
	{
	  int key = getch();   // call your non-blocking input function
	  std_msgs::String msg;
	  std::stringstream ss;
	  if (key == 'w'){
	  	ss << "F";
   	  }
	  if (key == 's'){
	  	ss << "B";
   	  }
	  if (key == 'a'){
	  	ss << "L";
   	  }
	  if (key == 'd'){
	  	ss << "R";
   	  }
	  if (key == 'p'){
	  	ss << "S";
   	  }
	  msg.data = ss.str();
	  keyPublisher.publish(msg);

	}
	ros::spin();
	return(0);
}