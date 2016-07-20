#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <termios.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <ctime>
// Include for V-REP
#include "../include/v_repConst.h"
// Used data structures:
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/VisionSensorData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosAuxiliaryConsolePrint.h"
#include "vrep_common/simRosAddStatusbarMessage.h"
#include "vrep_common/simRosAuxiliaryConsoleOpen.h"
#include "vrep_common/simRosAuxiliaryConsoleShow.h"
using namespace std;

bool simulationRunning = true;
float simulationTime = 0.0f;
int leftMotorHandle;
int rightMotorHandle;
string robotID;

ros::Publisher motorSpeedPub;
ofstream simulationLog;
string simulationHSINumber = "7";

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

void writeToLog(string msg) {
	simulationLog << msg;
	simulationLog << " at: ";
	simulationLog << currentDateTime();
	simulationLog << "\n";
}

void infoCallback(const vrep_common::VrepInfo::ConstPtr& info) {
	simulationTime = info->simulationTime.data;
	simulationRunning = (info->simulatorState.data & 1) != 0;
}

void RequestPublisher(ros::NodeHandle node, string topicName, int queueSize,
		int streamCmd, int objectHandle) {

	ros::ServiceClient enablePublisherClient = node.serviceClient<
			vrep_common::simRosEnablePublisher>("/vrep/simRosEnablePublisher");

	vrep_common::simRosEnablePublisher publisherRequest;

	publisherRequest.request.topicName = topicName;
	publisherRequest.request.queueSize = queueSize;
	publisherRequest.request.streamCmd = streamCmd;
	publisherRequest.request.auxInt1 = objectHandle;

	enablePublisherClient.call(publisherRequest);

	printf("A publisher just started with topic name %s\n", topicName.c_str());
}

void RequestSubscriber(ros::NodeHandle node, string topicName, int queueSize,
		int streamCmd) {

	ros::ServiceClient enableSubscriberClient = node.serviceClient<
			vrep_common::simRosEnableSubscriber>(
			"/vrep/simRosEnableSubscriber");

	vrep_common::simRosEnableSubscriber subscriberRequest;

	subscriberRequest.request.topicName = topicName;
	subscriberRequest.request.queueSize = 1;
	subscriberRequest.request.streamCmd = streamCmd;

	enableSubscriberClient.call(subscriberRequest);
	printf("A subscriber just started with topic name %s\n", topicName.c_str());
}

void publishMotorSpeed(float leftMotorSpeed, float rightMotorSpeed) {
	vrep_common::JointSetStateData motorSpeeds;
	motorSpeeds.handles.data.push_back(leftMotorHandle);
	motorSpeeds.handles.data.push_back(rightMotorHandle);
	motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
	motorSpeeds.setModes.data.push_back(2);
	motorSpeeds.values.data.push_back(leftMotorSpeed);
	motorSpeeds.values.data.push_back(rightMotorSpeed);
	motorSpeedPub.publish(motorSpeeds);
}

void moveForwards() {
	//publishMotorSpeed(3.1415, 3.1415);
	publishMotorSpeed(2.0, 2.0);
}

void moveBackwards() {
	publishMotorSpeed(-2.0, -2.0);
}

void turnRight() {
	publishMotorSpeed(2.0, 0.0);
}

void turnLeft() {
	publishMotorSpeed(0.0, 2.0);
}

void stop() {
	//printf("stopping\n");
	publishMotorSpeed(0.0, 0.0);
}

void keyboardCallback(const std_msgs::String::ConstPtr& msg) {
	//printf("hello");
	if (robotID == "0") {
		if (msg->data == "w") {
			writeToLog("Beacon Control: Move Forwards");
			moveForwards();
		}
		if (msg->data == "s") {
			writeToLog("Beacon Control: Move Backwards");
			moveBackwards();
		}
		if (msg->data == "d") {
			writeToLog("Beacon Control: Turn Right");
			turnRight();
		}
		if (msg->data == "a") {
			writeToLog("Beacon Control: Turn Left");
			turnLeft();
		}
		if (msg->data == "q") {
			writeToLog("Beacon Control: Stop");
			stop();
		}
	}

	if (robotID == "1") {
		if (msg->data == "i") {
			writeToLog("Beacon Control: Move Forwards");
			moveForwards();
		}
		if (msg->data == "k") {
			writeToLog("Beacon Control: Move Backwards");
			moveBackwards();
		}
		if (msg->data == "l") {
			writeToLog("Beacon Control: Turn Right");
			turnRight();
		}
		if (msg->data == "j") {
			writeToLog("Beacon Control: Turn Left");
			turnLeft();
		}
		if (msg->data == "o") {
			writeToLog("Beacon Control: Stop");
			stop();
		}
	}

}
int main(int argc, char **argv) {

	if (argc >= 2) {
		leftMotorHandle = atoi(argv[1]);
		rightMotorHandle = atoi(argv[2]);
		robotID = argv[3];

	} else {
		printf(
				"Indicate following arguments: 'leftMotorHandle rightMotorHandle frontVisionSensor'!\n");
		sleep(5000);
		return 0;
	}
	int _argc = 0;
	char** _argv = NULL;
	std::string nodeName("Beacon");
	nodeName += robotID;
	ros::init(_argc, _argv, nodeName.c_str());

	if (!ros::master::check())
		return (0);

	ros::NodeHandle node("~");
	printf("This rosNode just started with node name %s\n", nodeName.c_str());

	ros::Subscriber subInfo = node.subscribe("/vrep/info", 1, infoCallback);

	motorSpeedPub = node.advertise<vrep_common::JointSetStateData>("wheels", 1);
	RequestSubscriber(node, "/" + nodeName + "/wheels", 1,
			simros_strmcmd_set_joint_state);
	ros::Subscriber keySubscriber = node.subscribe("/keys", 1, keyboardCallback);

	string filename =
				"/home/dalia/My Stuff/College Work/Research/Thesis/Results/OneLane/WithHSI/Simulation";
		filename += simulationHSINumber;
		filename += "/";
		filename += nodeName;
		simulationLog.open(filename.c_str());
		writeToLog("Simulation Started");
	ros::Rate r(10);
	while (ros::ok() && simulationRunning) {

		ros::spinOnce();
		r.sleep();
	}
	writeToLog("Simulation Terminated");
	simulationLog.close();
	ros::shutdown();
	return (0);
}

