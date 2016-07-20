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
	ros::init(argc, argv, "keyboard_node");
	ros::NodeHandle n;
	ros::Publisher keyPublisher = n.advertise<std_msgs::String>("keys", 1);
	while (ros::ok())
	{
	  int key = getch();   // call your non-blocking input function
	  std_msgs::String msg;
	  std::stringstream ss;
	  if (key == 'w'){
	  	ss << "w";
   	  }
	  if (key == 's'){
	  	ss << "s";
   	  }
	  if (key == 'a'){
	  	ss << "a";
   	  }
	  if (key == 'd'){
	  	ss << "d";
   	  }
	  if (key == 'q'){
	  	ss << "q";
   	  }
	  if (key == 'i'){
	  	ss << "i";
   	  }
	  if (key == 'k'){
	  	ss << "k";
   	  }
	  if (key == 'l'){
	  	ss << "l";
   	  }
	  if (key == 'j'){
	  	ss << "j";
   	  }
	  if (key == 'o'){
	  	ss << "o";
   	  }
	  msg.data = ss.str();
	  keyPublisher.publish(msg);

	}
	ros::spin();
	ros::shutdown();
	return(0);
}
