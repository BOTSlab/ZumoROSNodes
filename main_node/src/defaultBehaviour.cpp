#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <std_msgs/String.h>
#include <colour_detector/ColourDetection.h>
#include <colour_detector/ColourDetectionArray.h>

using namespace std;

ros::Publisher motorPublisher;


int state = 0;
int subState = 0;

int factor = 2;
bool hasPuck = false;
int puckYCoor = 0;//////////////////////////
int puckXLCoor = 0;
int puckXRCoor = 0;
string beaconPriorities [3] = {"0","1","2"};

std::vector<colour_detector::ColourDetection> detectedColours;

void publishMotorSpeed(string wheelSpeed) {

	std_msgs::String msg;	
	std::stringstream ss;
	ss << wheelSpeed;
	msg.data = ss.str();
	motorPublisher.publish(msg);
	

}

//only one beacon per color allowed so far
bool canSeeRed() {
	for(int i=0; i<detectedColours.size(); i++){
		if(detectedColours[i].colour == "red"){
			return true;
		}
	}

	return false;
}
bool canSeeBlue() {
	for(int i=0; i<detectedColours.size(); i++){
		if(detectedColours[i].colour == "blue"){
			return true;
		}
	}

	return false;
}
//should be always paired with canSeeBlue
void getClosestBlue(int &nearestBlueX, int &nearestBlueY) {
	nearestBlueY = 0;
	for (uint i = 0; i < detectedColours.size(); i++) {
		if(detectedColours[i].colour == "blue"){
			if (detectedColours[i].y > nearestBlueY) {
				nearestBlueX = detectedColours[i].x;
				nearestBlueY = detectedColours[i].y;
			}
		}

	}
}

bool canMoveForwards() {

	for (uint i = 0; i < detectedColours.size(); i++) {
		/*if (detectedColours[i].x <= 615/factor && detectedColours[i].x >= 409/factor
			&& detectedColours[i].y > 410/factor) {
			//printf("HERE2!\n");
			return false;
		}*/ //use distance and bearing instead
	}

	return true;
}
bool canTurnRight() {

	for (uint i = 0; i < detectedColours.size(); i++) {
		/*if (detectedColours[i].x < 409/factor && detectedColours[i].x >= 384/factor
				&& detectedColours[i].y > 410/factor) {
			return false;
		}*/ //use distance and bearing instead
	}

	return true;
}

bool canTurnLeft() {

	for (uint i = 0; i < detectedColours.size(); i++) {
		/*if ((detectedColours[i].x <= 640/factor || detectedColours[i].x > 615/factor)
				&& detectedColours[i].y > 410/factor) {
			return false;
		}*/ //use distance and bearing instead
	}

	return true;
}

bool atDestination(int destY) { //update values
	if (destY >= 405/factor) {
		return true;
	}
	return false;
}
bool destinationIsAhead(int destX) { //update values
	if (destX <= 550/factor && destX >= 475/factor) {
		return true;
	}
	return false;
}
bool destinationIsRight(int destX) { //update values
	if (destX < 475/factor) {
		return true;
	}
	return false;
}
bool destinationIsLeft(int destX) { //update values
	if (destX > 550/factor) {
		return true;
	}
	return false;
}
/*bool hasAPuck(){
	for(int i=0; i<detectedColours.size(); i++){
		if(detectedColours[i].colour == "red"){
			if(detectedColours[i].y > puckYCoor && detectedColours[i].x < puckXLCoor && detectedColours[i].x > puckXRCoor){
			return true;
		}		
		}

	}
	return false;
}*/
void moveForwards() {
	publishMotorSpeed("S");
}
void turnRight() {
	publishMotorSpeed("R");
}

void turnLeft() {
	publishMotorSpeed("L");

}

void stop() {
	publishMotorSpeed("S");
}

/*void moveTo(int destX, int destY) {
	printf("moving\n");
	if (atDestination(destY)) {
		stop();
		printf("x: %d, y: %d\n", destX, destY );
	} else {
		if (destinationIsAhead(destX)) {
			 printf("destination IS ahead\n");
			if (canMoveForwards()) {
				moveForwards();
				avoidCounter = 0;
			} else {
				avoidCounter = 20;
				turnRight();
			}
		} else {
			if (avoidCounter > 0) {
				if (canMoveForwards()) {
					moveForwards();
					avoidCounter--;
				} else {
					turnRight();
				}
			} else {
				 printf("NOT ahead, turning right\n");
				if (destinationIsRight(destX)) {
					turnRight();
				} else {
					printf("NOT ahead, turning left\n");
					turnLeft();
				}
			}
		}
	}
	hasAPuck();

}
void wander(){
	int randomNumber = rand() % 3;
	if(randomNumber == 0){
		turnLeft();
		ros::Duration(2.0).sleep();
	}else{
		if(randomNumber == 1){
			turnRight();
			ros::Duration(2.0).sleep();
		}else{
			moveForwards();
			ros::Duration(2.0).sleep();
		}
	}

}
bool canSeeBeacon(){
	return false;
}
void pickUpPuck(){

	int min = 1000;
	for(int i=0; i<detectedRedColours.size(); i++){
		//if(distance)
	}
	//moveTo(detectedRedColours[0].x, detectedRedColours[0].y);
}

void depositPuck(){

}
void abandonPuck(){

}
void reactToBeacon(){

}*/
// this method is called multiple times per spin
// setting states and operating on them in another method to take advantage of that
void begin() {
	//stop();
	/*
	 * colourDetection.distance = sqrt(pow((imgCenterX - colourDetection.x), 2)+pow((imgCenterY - colourDetection.y), 2));
	   colourDetection.bearing = atan2((colourDetection.y - imgCenterY), (colourDetection.x - imgCenterX)) * 180 / PI ;
	 
	printf("purple x : %d red y : %d\n", detectedPurpleColours[0].x, detectedPurpleColours[0].y);
	printf("distance: %d\n", getDistance(10,40,60,30));
	printf("beginning\n");
	if(canSeeBeacon()){
		if(hasPuck){
			abandonPuck();
		}else{
			//reactToBeacon
		}
			return;
	}

	if((!canSeeRed() && !canSeeBlue()) || (hasPuck && !canSeeBlue())){
		wander();
		return;
	}
	if(canSeeRed() && !hasPuck){
		pickUpPuck();
		return;
	}
	if(hasPuck && canSeeBlue()){
		depositPuck();
		return;
	}

*/

	/*if (canSeeRed()) {
		printf("can see red\n");
		// go to beacon
		printf("red x : %d red y : %d\n", detectedRedColours[0].x, detectedRedColours[0].y);
		moveTo(detectedRedColours[0].x, detectedRedColours[0].y);
	} else {
		// flock to nearest robot
		if (canSeeBlue()) {
			//printf("HERE\n");
			int nearestBlueX, nearestBlueY;
			getClosestBlue(nearestBlueX, nearestBlueY);
			printf("blue x : %d red y : %d\n", detectedRedColours[0].x, detectedRedColours[0].y);
			moveTo(nearestBlueX, nearestBlueY);
		}else{
			stop();
		}
	}*/
//moveForwards();


}

void coloursCb(const colour_detector::ColourDetectionArray::ConstPtr& msg){
	
		/*for(std::vector<colour_detector::ColourDetectionArray>::const_iterator it = msg->detections.begin(); it != msg->detections.end(); it++){
		std::cout << "vector" << std::endl;
}*/
std::vector<colour_detector::ColourDetection> detectionsArray = msg -> detections;
for (int i=0; i<detectionsArray.size(); i++){
std:cout << detectionsArray[i].colour << std::endl;
}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "main_node");
	ros::NodeHandle n;
	ros::Subscriber imageSubscriber = n.subscribe("coloursDetected", 10, coloursCb);
	motorPublisher = n.advertise<std_msgs::String>("wheelSpeeds", 1);
	ros::spin();
	return(0);
}