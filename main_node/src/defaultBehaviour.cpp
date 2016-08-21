#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <opencv/cvwimage.h>
#include <sstream>
#include <zbar.h>
#include "std_msgs/String.h"
#include <math.h>
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

#include "apriltags/AprilTags/TagDetector.h"
#include "apriltags/AprilTags/Tag36h11.h"
#include "apriltags/AprilTags/Tag16h5.h"
#include "AprilTag.h"
#include "DetectedColour.h"
#include "Robot.h"

using namespace cv;
using namespace std;
using namespace zbar;

const double PI = 3.14159265358979323846;
const double TWOPI = 2.0 * PI;
AprilTags::TagDetector* tag_detector;
AprilTags::TagCodes tag_codes(AprilTags::tagCodes16h5);

double camera_focal_length_x(700); // in pixels. late 2013 macbookpro retina = 700
double camera_focal_length_y(700); // in pixels
double tag_size(0.029); // tag side length of frame in meters

ros::Publisher barcodeLocationPublisher;
ros::Publisher colourLocationPublisher;
ros::Publisher motorSpeedPub;
int leftMotorHandle;
int rightMotorHandle;
int frontVisionSensor;
string robotID;
int nodeID;
// Global variables (modified by topic subscribers):
bool simulationRunning = true;
float simulationTime = 0.0f;

int state = 0;
int subState = 0;
vector<DetectedColour> detectedBlueColours, detectedRedColours,
		detectedPurpleColours, detectedYellowColours, detectedGreenColours,
		detectedCyanColours, detectedBeacons;

vector<AprilTag> tagsFound;
vector<Robot> neighbors;
int factor = 2;

int puckYCoor = 245; //////////////////////////
int puckXLCoor = 245;
int puckXRCoor = 265;

int yellowYCoor = 80;
int yellowLCoor = 220;
int yellowRCoor = 295;

int robotPositionInImgX = 256;
int robotPositionInImgY = 256;

int rightTurnCheckX1 = 80;
int rightTurnCheckX2 = 210;
int rightTurnCheckY = 180;

int leftTurnCheckX1 = 300;
int leftTurnCheckX2 = 430;
int leftTurnCheckY = 185;

int moveForwardsCheckX1 = 200;
int moveForwardsCheckX2 = 310;
int moveForwardsCheckY = 190;

int aheadX1 = 240;
int aheadX2 = 265;

int wanderCounterDefault = 3;
int wanderCounter = wanderCounterDefault;
int wanderDirection = 0;

bool canTurnRight = true;
bool canTurnLeft = true;
bool canMoveForwards = true;
bool hasPuck = false;

ofstream simulationLog;
string simulationNumber = "15";
string simulationHSINumber = "8";  //REMEMBER to update the same variable in beaconControl.cpp!!!!
int logState = -1;

//set beacon priorities according to passed id in input by the lua script and nN
int beaconPriority;
// Topic subscriber callbacks:
inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t + PI, TWOPI) - PI;
	} else {
		t = fmod(t - PI, -TWOPI) + PI;
	}
	return t;
}
/**
 * Convert rotation matrix to Euler angles
 */

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
		double& roll) {
	yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad((-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
	roll = standardRad(
			atan2(wRo(0, 2) * s - wRo(1, 2) * c,
					-wRo(0, 1) * s + wRo(1, 1) * c));
}

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

AprilTag convertToAprilTag(AprilTags::TagDetection& detection, int width,
		int height) {
	// recovering the relative pose of a tag:

	// NOTE: for this to be accurate, it is necessary to use the
	// actual camera parameters here as well as the actual tag size
	// (m_fx, m_fy, m_px, m_py, m_tagSize)

	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	detection.getRelativeTranslationRotation(tag_size, camera_focal_length_x,
			camera_focal_length_y, width / 2, height / 2, translation,
			rotation);

	Eigen::Matrix3d F;
	F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
	Eigen::Matrix3d fixed_rot = F * rotation;
	double yaw, pitch, roll;
	wRo_to_euler(fixed_rot, yaw, pitch, roll);

	AprilTag tag;

	tag.id = detection.id;
	tag.hammingDistance = detection.hammingDistance;
	tag.distance = translation.norm() * 100.0;
	tag.z = translation(0) * 100.0; // depth from camera
	tag.x = translation(1) * 100.0; // horizontal displacement (camera pov right = +ve)
	tag.y = translation(2) * 100.0; // vertical displacement
	tag.yaw = yaw;
	tag.pitch = pitch;
	tag.roll = roll;
	tag.code = detection.code;
	return tag;
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

//Scalar(100,50,50), Scalar(130,255,255) || Scalar(120,100,100), Scalar(179,255,255) -> blue
//Scalar(0, 50, 50), Scalar(0, 255, 255) -> red
//Scalar(50, 100, 100), Scalar(70, 255, 255) -> green

vector<DetectedColour> getDetectedColours(
		const cv_bridge::CvImageConstPtr &cvSegmentationImage,
		Scalar lowerBound, Scalar higherBound, string colour) {

	Mat HSV;
	cvtColor(cvSegmentationImage->image, HSV, COLOR_BGR2HSV);
	vector<DetectedColour> detectedColours;
	Mat threshold;
	inRange(HSV, lowerBound, higherBound, threshold);
	/*
	 * Attempt to detect obstacles here because the image is already on-hand
	 * Seperate this into other methods later to clean the code up
	 */

	if (colour == "blue" /*|| colour == "cyan" || colour == "green"*/) {
		canTurnRight = true;
		canTurnLeft = true;
		canMoveForwards = true;
		for (int i = rightTurnCheckX1; i < rightTurnCheckX2 + 1; i++) {
			for (int j = 180; j < threshold.rows; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canTurnRight = false;
				}
			}
		}
		for (int i = leftTurnCheckX1; i < leftTurnCheckX2 + 1; i++) {
			for (int j = leftTurnCheckY; j < threshold.rows; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canTurnLeft = false;
				}
			}
		}
		for (int i = moveForwardsCheckX1; i < moveForwardsCheckX2 + 1; i++) {
			for (int j = moveForwardsCheckY; j < threshold.rows; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canMoveForwards = false;
				}
			}
		}
	}

	if (colour == "red") {
		hasPuck = false;
		int farPuckYCoor = 235;
		for (int i = puckXLCoor; i < puckXRCoor + 1; i++) {
			for (int j = puckYCoor; j < threshold.rows; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					hasPuck = true;
				}
			}
			for (int j = farPuckYCoor; j < puckYCoor - 1; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					hasPuck = false;
				}
			}

		}

	}

	vector < vector<Point> > contours;

	vector < Vec4i > hierarchy;
	findContours(threshold, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Get the moments
	// mu[i].m00 represents how big the blob of color is
	vector < Moments > mu(contours.size());

	for (int i = 0; i < contours.size(); i++) {
		mu[i] = moments(contours[i], false);
	}
	//std::string foundBlobs = "";
	///  Get the mass centers
	for (int i = 0; i < contours.size(); i++) {
		DetectedColour dc;
		dc.colour = colour;
		dc.colourBlobSize = (int) mu[i].m00;
		dc.x = (int) mu[i].m10 / mu[i].m00;
		dc.y = (int) mu[i].m01 / mu[i].m00;
		if (dc.x >= 0 && dc.y >= 0) {
			detectedColours.push_back(dc);
		}

	}
	return detectedColours;
}

vector<AprilTag> getDetectedTags(cv_bridge::CvImagePtr &cv_ptr) {
	cv::Mat image_gray;
	cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
	vector<AprilTags::TagDetection> detections = tag_detector->extractTags(
			image_gray);
	vector < AprilTag > tagsFound;
	for (int i = 0; i < detections.size(); i++) {
		detections[i].draw(cv_ptr->image);
		tagsFound.push_back(
				convertToAprilTag(detections[i], cv_ptr->image.cols,
						cv_ptr->image.rows));
	}
	return tagsFound;
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

int getDistance(int x1, int y1, int x2, int y2) {
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
//only one beacon per color allowed so far
bool canSeeRed() {
	if (detectedRedColours.size() > 0) {
		return true;
	}
	return false;
}
bool canSeePurple() {
	if (detectedPurpleColours.size() > 0
			&& detectedPurpleColours[0].colourBlobSize > 5) {
		return true;
	}
	return false;
}
bool canSeeYellow() {
	if (detectedYellowColours.size() > 0
			&& detectedYellowColours[0].colourBlobSize > 5) {
		return true;
	}
	return false;
}
bool canSeeBlue() {
	if (detectedBlueColours.size() > 0) {
		return true;
	}
	return false;
}
bool canSeeGreen() {
	if (detectedGreenColours.size() > 0) {
		return true;
	}
	return false;
}
bool canSeeCyan() {
	if (detectedCyanColours.size() > 0) {
		return true;
	}
	return false;
}
//should be always paired with canSeeBlue
void getClosestBlue(int &nearestBlueX, int &nearestBlueY) {
	nearestBlueY = 0;
	for (uint i = 0; i < detectedBlueColours.size(); i++) {
		if (detectedBlueColours[i].y > nearestBlueY) {
			nearestBlueX = detectedBlueColours[i].x;
			nearestBlueY = detectedBlueColours[i].y;
		}
	}
}

bool atDestination(int destY) {
	if (destY >= 145) {
		return true;
	}
	return false;
}
bool destinationIsAhead(int destX) {
	if (destX <= aheadX2 && destX >= aheadX1) {
		return true;
	}
	return false;
}
bool destinationIsRight(int destX) {
	if (destX < aheadX1) {
		return true;
	}
	return false;
}
bool destinationIsLeft(int destX) {
	if (destX > aheadX2) {
		return true;
	}
	return false;
}

void moveForwards() {
	//publishMotorSpeed(3.1415, 3.1415);
	publishMotorSpeed(10.0, 10.0);
}
void moveBackwards() {
	publishMotorSpeed(-3.1415, -3.1415);
}
void moveForwardsAccelerated() {
	publishMotorSpeed(3.1415 * 2, 3.1415 * 2);
}
void moveForwardsSlowed() {
	publishMotorSpeed(3.1415 / 2, 3.1415 / 2);
}
void turnRight() {
	publishMotorSpeed(3.1415, 0.0);
}
void turnRightAvoid() {
	publishMotorSpeed(3.1415, -2.0);
}
void turnRightAccelerated() {
	publishMotorSpeed(3.1415 * 2, 0.0);
}
void turnRightSlowed() {
	publishMotorSpeed(3.1415 / 2, 0.0);

}
void turnLeft() {
	publishMotorSpeed(0.0, 3.1415);
}
void turnLeftAvoid() {
	publishMotorSpeed(-2.0, 3.1415);
}

void turnLeftAccelerated() {
	publishMotorSpeed(0.0, 3.1415 * 2);
}
void turnLeftSlowed() {
	publishMotorSpeed(0.0, 3.1415 / 2);
}
void stop() {
	//printf("stopping\n");
	publishMotorSpeed(0.0, 0.0);
}
void avoid() {
	if (canTurnRight) {
		turnRightAvoid();
		//printf("Avoiding Right...");
	} else {
		if (canTurnLeft) {
			turnLeftAvoid();
			//printf("Avoiding Forwards...");
		} else {
			if (canMoveForwards) {
				moveForwards();
				//printf("Avoiding Left...");
			}//EXPERIMENTAL!!!
			else{
				if(hasPuck){
					stop();
				}else{
					moveBackwards();
				}
			}
		}
	}

}

void moveTo(int destX, int destY) {
	//printf("moving to destX: %d destY: %d\n", destX, destY);

	if (destinationIsAhead(destX)) {
		//printf("Destination is ahead...\n");
		if (canMoveForwards) {
			moveForwards();
		} else {

			//printf("Can't move forwards! Avoiding...\n");
			avoid();
		}
		return;
	}

	if (destinationIsRight(destX)) {
		//printf("Destination is right...\n");
		if (canTurnRight) {
			//printf("Turning RIGHT\n");
			turnRight();
		} else {
			//printf("Can't move right! Avoiding...\n");
			avoid();
		}
		return;
	}
	if (destinationIsLeft(destX)) {
		//printf("Destination is left...\n");
		if (canTurnLeft) {
			//printf("Turning LEFT\n");
			turnLeft();
		} else {
			//printf("Can't move left! Avoiding...\n");
			avoid();
		}

	}

}

void wander() {
	// ToDo: collision detection when wandering
	if (wanderCounter > 0) {
		wanderCounter--;
		switch (wanderDirection) {
		case 0:
			if (canTurnLeft) {
				turnLeft();
			} else {
				//	printf("Can't wander left! Avoiding \n");
				avoid();
			}
			break;
		case 1:
			if (canTurnRight) {
				turnRight();
			} else {
				//printf("Can't wander right! Avoiding\n");
				avoid();
			}
			break;
		default:
			if (canMoveForwards) {
				moveForwards();
			} else {
				//printf("Can't wander forwards! Avoiding\n");
				avoid();
			}
			break;
		}
	} else {
		wanderDirection = rand() % 3;
		wanderCounter = wanderCounterDefault;
		wander();
	}

}
bool canSeeBothBeacons() {
	return canSeeCyan() && canSeeGreen();
}
bool canSeeCyanBeacon() {
	return canSeeCyan();
}
bool canSeeGreenBeacon() {
	return canSeeGreen();
}
bool atSource() {
	if (canSeePurple()) {
		if (detectedPurpleColours[0].y >= 130) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}

}
void goToSource() {
	if (canSeePurple()) {
		moveTo(detectedPurpleColours[0].x, detectedPurpleColours[0].y);
	}
}
void goToNest() {
	if (canSeeYellow()) {
		moveTo(detectedYellowColours[0].x, detectedYellowColours[0].y);
	}
}
void goToCyanBeacon() {
	if (canSeeCyan()) {
		moveTo(detectedCyanColours[0].x, detectedCyanColours[0].y);
	}
}
void goToGreenBeacon() {
	if (canSeeGreen()) {
		moveTo(detectedGreenColours[0].x, detectedGreenColours[0].y);
	}
}
bool canSeePuck() {
	return canSeeRed();
}

bool canSeeNest() {
	return canSeeYellow();
}
bool canSeeSource() {
	return canSeePurple();
}
bool canSeeAgent() {
	return canSeeBlue();
}
void pickUpPuck() {

	int max = 0;
	int maxX = 0;
	for (int i = 0; i < detectedRedColours.size(); i++) {
		if (detectedRedColours[i].y > max) {
			max = detectedRedColours[i].y;
			maxX = detectedRedColours[i].x;
		}
	}
	moveTo(maxX, max);

}
bool atNest() {
	if (canSeeYellow()) {
		if (detectedYellowColours[0].y >= 145) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}

}
bool atGreenBeacon() {
	if (canSeeGreen()) {
		if (detectedGreenColours[0].y >= 148) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}
bool atCyanBeacon() {
	if (canSeeCyan()) {
		if (detectedCyanColours[0].y >= 148) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}
void depositPuck() {
	//printf("moving backwards");
	moveBackwards();
	ros::Duration(1.5).sleep();
	avoid();
	ros::Duration(1.5).sleep();
	return;
}
void abandonPuck() {
	for (int i = 0; i < 3; i++) {
		moveBackwards();
	}

}
void reactToBeacon() {

}

// this method is called multiple times per spin
// setting states and operating on them in another method to take advantage of that
void begin() {
	//change source color to indicate depletion
	//maybe change nest color when experiment is done to have a stopping condition that they all need to go back to this place
	//printf("beginning\n");

	/*
	 * Log states are: 0 for wandering, 1 for going to nest, 2 for going to source, 3 for picking up puck, 4 for depositing puck, 5 for beacon action
	 */

	if (hasPuck && atNest()) {

		if (logState != 4) {
			//printf("depositing puck\n");
			logState = 4;
			writeToLog("Depositing a Puck");
		}
		depositPuck();
		return;
	}
	if (hasPuck && canSeeNest()) {

		if (logState != 1) {
			//printf("going to nest\n");
			logState = 1;
			writeToLog("Going to Nest");
		}
		goToNest();
		return;
	}
	//changed for can see source to at source
	if (atSource() && !hasPuck && canSeePuck()) {

		if (logState != 3) {
			//	printf("Picking up a puck\n");
			logState = 3;
			writeToLog("Picking up a Puck");
		}
		pickUpPuck();
		return;
	}
	// might need to remove this one
	if (canSeeSource() && !hasPuck) {

		if (logState != 2) {
			//	printf("Approaching Source\n");
			logState = 2;
			writeToLog("Going to Source");
		}
		goToSource();
		return;
	}
	//priority 0 is for cyan beacon and 1 is for green beacon
	//this case implies that the robot can see the beacon but not source when no puck and not nest when has puck
	/*if (canSeeBothBeacons()) {
	 if (beaconPriority == 0) {
	 //printf("Going to cyan beacon, can see both\n");
	 if (logState != 5) {
	 //	printf("Approaching Source\n");
	 logState = 5;
	 writeToLog("Going to Beacon");
	 }

	 goToCyanBeacon();
	 return;


	 } else {

	 //printf("Going to green beacon, can see both\n");
	 if (logState != 5) {
	 //	printf("Approaching Source\n");
	 logState = 5;
	 writeToLog("Going to Beacon");
	 }
	 goToGreenBeacon();
	 return;
	 }

	 }
	 if(canSeeCyanBeacon()){

	 //printf("Going to cyan beacon\n");
	 if (logState != 5) {
	 //	printf("Approaching Source\n");
	 logState = 5;
	 writeToLog("Going to Beacon");
	 }
	 goToCyanBeacon();
	 return;

	 }
	 if(canSeeGreenBeacon()){

	 //printf("Going to green beacon\n");
	 if (logState != 5) {
	 //	printf("Approaching Source\n");
	 logState = 5;
	 writeToLog("Going to Beacon");
	 }
	 goToGreenBeacon();
	 return;

	 }*/
	if (canSeeBothBeacons()) {
		if (beaconPriority == 0) {

			if (atCyanBeacon()) {
				if (logState != 0) {
					logState = 0;
					if (hasPuck) {
						writeToLog("Wandering with a Puck");
					} else {
						writeToLog("Wandering without a Puck");
					}

				}
				wander();
				return;
			} else {
				if (logState != 5) {
					//	printf("Approaching Source\n");
					logState = 5;
					writeToLog("Going to Beacon");
				}

				goToCyanBeacon();
				return;
			}

		} else {
			if (atGreenBeacon()) {
				if (logState != 0) {
					logState = 0;
					if (hasPuck) {
						writeToLog("Wandering with a Puck");
					} else {
						writeToLog("Wandering without a Puck");
					}

				}
				wander();
				return;
			} else {
				if (logState != 5) {
					//	printf("Approaching Source\n");
					logState = 5;
					writeToLog("Going to Beacon");
				}
				goToGreenBeacon();
				return;
			}
		}
	}
	if (canSeeCyanBeacon()) {
		if (atCyanBeacon()) {
			if (logState != 0) {
				logState = 0;
				if (hasPuck) {
					writeToLog("Wandering with a Puck");
				} else {
					writeToLog("Wandering without a Puck");
				}

			}
			wander();
			return;
		} else {
			if (logState != 5) {
				//	printf("Approaching Source\n");
				logState = 5;
				writeToLog("Going to Beacon");
			}
			goToCyanBeacon();
			return;
		}
	}
	if (canSeeGreenBeacon()) {
		if (atGreenBeacon()) {
			if (logState != 0) {
				logState = 0;
				if (hasPuck) {
					writeToLog("Wandering with a Puck");
				} else {
					writeToLog("Wandering without a Puck");
				}

			}
			wander();
			return;
		} else {
			if (logState != 5) {
				//	printf("Approaching Source\n");
				logState = 5;
				writeToLog("Going to Beacon");
			}
			goToGreenBeacon();
			return;
		}
	}
	if ((!hasPuck && !canSeeSource()) || (hasPuck && !canSeeNest())) {
		//printf("Wandering...\n");
		if (logState != 0) {
			logState = 0;
			if (hasPuck) {
				writeToLog("Wandering with a Puck");
			} else {
				writeToLog("Wandering without a Puck");
			}

		}
		wander();
		return;
	}

	if (atSource() && !hasPuck && !canSeePuck()) {

		if (logState != 0) {
			//	printf("Can't find pucks at source, wandering!\n");
			logState = 0;
			writeToLog("Wandering without a Puck");
		}
		wander();
		return;
	}

	//printf("don't know what to do\n");
	stop();

}

void frontImageCallback(const sensor_msgs::ImageConstPtr &image) {
//	printf("image call back\n");
	cv_bridge::CvImageConstPtr cv_image, cvSegmentationImage;
	cv_image = cv_bridge::toCvCopy(image, "mono16");
	cvSegmentationImage = cv_bridge::toCvCopy(image, "bgr8");

	detectedBlueColours = getDetectedColours(cvSegmentationImage,
			Scalar(100, 50, 50), Scalar(130, 255, 255), "blue");

	detectedRedColours = getDetectedColours(cvSegmentationImage,
			Scalar(0, 50, 50), Scalar(0, 255, 255), "red");

	detectedPurpleColours = getDetectedColours(cvSegmentationImage,
			Scalar(149, 50, 50), Scalar(150, 255, 255), "purple");

	detectedYellowColours = getDetectedColours(cvSegmentationImage,
			Scalar(30, 50, 50), Scalar(31, 255, 255), "yellow");

	detectedGreenColours = getDetectedColours(cvSegmentationImage,
			Scalar(60, 50, 50), Scalar(61, 255, 255), "green");

	detectedCyanColours = getDetectedColours(cvSegmentationImage,
			Scalar(90, 50, 50), Scalar(91, 255, 255), "cyan");
	/*if(canSeePurple()){
	 printf("purple x : %d purple y : %d\n", detectedPurpleColours[0].x,	detectedPurpleColours[0].y);
	 }
	 if(canSeeBlue()){
	 printf("blue x : %d blue y : %d\n", detectedBlueColours[0].x,	detectedBlueColours[0].y);
	 }

	 /*printf("can turn right: %d\n", canTurnRight);
	 printf("can turn left: %d\n", canTurnLeft);
	 printf("can move forwards: %d\n", canMoveForwards);
	 printf("at source: %d\n", atSource());
	 printf("at nest: %d\n", atNest());

	 if(canSeeRed()){
	 printf("red x : %d red y : %d\n", detectedRedColours[0].x,	detectedRedColours[0].y);
	 }ros::Duration(1.0).sleep();*/
	//printf("has puck: %d\n", hasPuck);
	//printf("can move forwards: %d\n", canMoveForwards);
	//turnRight();
	/*if(canSeeCyan()){
	 printf("Cyan x : %d cyan y : %d\n", detectedCyanColours[0].x, detectedCyanColours[0].y);
	 }else{
	 printf("No Cyan found\n");
	 }*/
	/*if(canSeePurple()){
	 printf("purple x : %d purple y : %d\n", detectedPurpleColours[0].x, detectedPurpleColours[0].y);
	 }else{
	 printf("No purple found\n");
	 }*/
	/*if(canSeeGreen()){
	 printf("green x : %d green y : %d blobsize: %d\n", detectedGreenColours[0].x, detectedGreenColours[0].y, detectedGreenColours[0].colourBlobSize);
	 }else{
	 printf("No Green found\n");
	 }*/
	/*if(canSeeYellow()){
	 printf("yellow x : %d yellow y : %d blobsize: %d\n", detectedYellowColours[0].x,	detectedYellowColours[0].y, detectedYellowColours[0].colourBlobSize);
	 }else{
	 printf("No Yellow Found\n");
	 }*/

}

int main(int argc, char **argv) {

	if (argc >= 2) {
		leftMotorHandle = atoi(argv[1]);
		rightMotorHandle = atoi(argv[2]);
		frontVisionSensor = atoi(argv[3]);
		robotID = argv[4];

	} else {
		printf(
				"Indicate following arguments: 'leftMotorHandle rightMotorHandle frontVisionSensor'!\n");
		sleep(5000);
		return 0;
	}
	int _argc = 0;
	char** _argv = NULL;
	std::string nodeName("Robot");
	nodeName += robotID;
	ros::init(_argc, _argv, nodeName.c_str());

	if (!ros::master::check())
		return (0);

	ros::NodeHandle node("~");
	printf("This rosNode just started with node name %s\n", nodeName.c_str());

	ros::Subscriber subInfo = node.subscribe("/vrep/info", 1, infoCallback);

	barcodeLocationPublisher = node.advertise<std_msgs::String>("barcodeExt",
			10);
	colourLocationPublisher = node.advertise<std_msgs::String>("colorsFound",
			10);

	RequestPublisher(node, "frontVisionSensorData" + robotID, 1,
			simros_strmcmd_get_vision_sensor_image, frontVisionSensor);

	string frontVisionSensorTopicName("/vrep/frontVisionSensorData");
	frontVisionSensorTopicName += robotID;

	ros::Subscriber frontVisionSensorSub = node.subscribe(
			frontVisionSensorTopicName.c_str(), 10, frontImageCallback);

	motorSpeedPub = node.advertise<vrep_common::JointSetStateData>("wheels", 1);
	RequestSubscriber(node, "/" + nodeName + "/wheels", 1,
			simros_strmcmd_set_joint_state);
	tag_detector = new AprilTags::TagDetector(tag_codes);

	/*	while (ros::ok() && simulationRunning) {
	 //printf("rosSpin\n");
	 ros::spinOnce();
	 }*/
	ros::Rate r(10);
	string filename =
			"/home/dalia/My Stuff/College Work/Research/Thesis/Results/OneLane/WithHSI/Simulation";
	filename += simulationHSINumber;
	filename += "/";
	filename += nodeName;
	simulationLog.open(filename.c_str());
	writeToLog("Simulation Started");
	struct timeval tv;
	unsigned int timeVal = 0;
	if (gettimeofday(&tv, NULL) == 0)
		timeVal = (tv.tv_sec * 1000 + tv.tv_usec / 1000) & 0x00ffffff;
	std::string randNumber(
			boost::lexical_cast < std::string
					> (timeVal + int(999999.0f * (rand() / (float) RAND_MAX))));
	int randomSeed = atoi(randNumber.c_str());
	srand(randomSeed);

	int robotIntID = atoi(robotID.c_str());
	if (robotIntID % 2 == 0) {
		beaconPriority = 0;
	} else {
		beaconPriority = 1;
	}
	while (ros::ok() && simulationRunning) {
		begin();
		//stop();
		//wander();
		ros::spinOnce();
		r.sleep();
	}
	writeToLog("Simulation Terminated");
	simulationLog.close();
	ros::shutdown();
	printf("Agent just ended!!!!\n");
	return (0);
}
