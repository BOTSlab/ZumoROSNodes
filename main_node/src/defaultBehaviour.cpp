#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
#include <colour_detector/ColourDetection.h>
#include <colour_detector/ColourDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf/transform_broadcaster.h>
#include <pixy_node/PixyData.h>
#include <pixy_node/PixyBlock.h>
#include <pixy_node/Servo.h>
#include <fstream>
#include <ctime>
using namespace std;

ros::Publisher motorPublisher;


int state = 0;
int subState = 0;

int factor = 2;
int proximityReading = 12;
int puckYCoor = 0;//////////////////////////
int puckXLCoor = 0;
int puckXRCoor = 0;

string beaconPriorities [3] = {"0","1","2"};

bool turnedLeft = false;
bool turnedRight = false;

std::vector<colour_detector::ColourDetection> detectedColours;
std::vector<colour_detector::ColourDetection> blueColours;
std::vector<colour_detector::ColourDetection> orangeColours;
std::vector<colour_detector::ColourDetection> greenColours;
std::vector<pixy_node::PixyBlock> pixyBlocks;
std::vector<apriltags_ros::AprilTagDetection> detectedTags;

bool canTurnRight = true;
bool canTurnLeft = true;
bool canMoveForwards = true;

int wanderCounterDefault = 3;
int wanderCounter = wanderCounterDefault;
int wanderDirection = 0;

ofstream simulationLog;
string simulationNumber = "2";
string simulationHSINumber = "7";  //REMEMBER to update the same variable in beaconControl.cpp!!!!
int logState = -1;

/****************** LOGGING ******************/

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

/****************** Issuing Movement Commands To Motors ******************/

void publishMotorSpeed(string wheelSpeed) {

	std_msgs::String msg;	
	std::stringstream ss;
	ss << wheelSpeed;
	msg.data = ss.str();
	motorPublisher.publish(msg);
}

void moveForwards() {
	publishMotorSpeed("F");
}
void turnRight() {
	publishMotorSpeed("R");
}
void turnRightSlow() {
	publishMotorSpeed("E");
}
void turnLeft() {
	publishMotorSpeed("L");

}
void turnLeftSlow() {
	publishMotorSpeed("K");

}
void stop() {
	publishMotorSpeed("S");
}

void moveBackwards(){
	publishMotorSpeed("B");
}

/****************** Colour Detections ******************/
/*
	For each robot adjust pixy signatures
*/

bool pixyCanSeeGreen(){
	for(int i=0; i<pixyBlocks.size(); i++){
		if(pixyBlocks[i].signature == 1){
			return true;
		}
	}
	return false;
}

bool piCamCanSeeGreen(){
	if(greenColours.size()>0){
		return true;
	}
}

bool canSeeGreen() {
	if(piCamCanSeeGreen()){
		return true;
	}
	if(pixyCanSeeGreen()){
		return true;
	}
	return false;
}

bool pixyCanSeeOrange(){
	for(int i=0; i<pixyBlocks.size(); i++){
		if(pixyBlocks[i].signature == 2){
			return true;
		}
	}
	return false;
}

bool piCamCanSeeOrange(){
	if(orangeColours.size()>0){
		return true;
	}
}

bool canSeeOrange() {
	if(piCamCanSeeOrange()){
		return true;
	}
	if(pixyCanSeeOrange()){
		return true;
	}
	return false;
}

bool pixyCanSeeBlue(){
	for(int i=0; i<pixyBlocks.size(); i++){
		if(pixyBlocks[i].signature == 2){
			return true;
		}
	}
	return false;
}

bool piCamCanSeeBlue(){
	if(blueColours.size()>0){
		return true;
	}
}

// green is 0, orange is 1? for now
bool canSeeBlue() {
	if(pixyCanSeeBlue()){
		return true;
	}
	if(piCamCanSeeBlue()){
		return true;
	}
	return false;
}

/****************** Landmark Detections ******************/
/*
	For each robot adjust detection distances
*/

bool canSeeNest() {
	/*for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 0){
			return true;
		}
	}*/
	for(int i=0; i<detectedTags.size(); i++){
	if(detectedTags[i].id == 0 && detectedTags[i].distance > 170){ //edit
		return true;
	} //if assigned an extra colour then use that too
	return false;
}


//range is limited so just seeing the tag means the robot is where it needs to be?
bool atNest(){
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 0 && detectedTags[i].distance <= 170){
			return true;
		}
	}
	return false;
}


bool canSeeSource() {
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 1 && detectedTags[i].distance > 170){
			return true;
		}
	}
	return false;
}

bool atSource(){
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 1 && detectedTags[i].distance <= 170){
			return true;
		}
	}
	return false;
}


bool atOrangeDestination() { //update values
	for(int i=0; i<orangeColours.size();i++){
		if(orangeColours[i].distance < 170){
			return true;
		}
	}
	return false;
}

bool atGreenDestination() { //update values
	for(int i=0; i<greenColours.size();i++){
		if(greenColours[i].distance < 170){
			return true;
		}
	}
	return false;
}

bool canSeeBeacon(){
	if(canSeeOrange()){
		return true;
	}
	return false;
}

/****************** Target Locations ******************/
/*
	For each robot adjust detection distances and bearings
*/

bool destinationIsAhead(int bearing) {
	if (bearing <= 30 && bearing >= -35) {
		return true;
	}
	return false;
}

bool destinationIsAheadPixy(int x_offset){
	if (x_offset <= 190 && x_offset >= 110) {
		return true;
	}
	return false;
}

bool destinationIsRight(int bearing) {
	if (bearing > 30) {
		return true;
	}
	return false;
}
bool destinationIsRightPixy(int x_offset) {
	if (x_offset > 190) {
		return true;
	}
	return false;
}
bool destinationIsLeft(int bearing) {
	if (bearing < -35.0) {
		return true;
	}
	return false;
}
bool destinationIsLeftPixy(int x_offset) {
	if (x_offset < 110) {
		return true;
	}
	return false;
}




bool hasPuck(){
	if(proximityReading == 12){
		return false;
	}
	return true;
}

/****************** Movement Functions ******************/

void avoid() {
if (canTurnRight) {
		turnRight();
		//printf("Avoiding Right...");
	} else {
		if (canTurnLeft) {
			turnLeft();
			//printf("Avoiding Forwards...");
		} else {
			if (canMoveForwards) {
				moveForwards();
				//printf("Avoiding Left...");
			}
			else{
				if(hasPuck){
					stop();
				}else{
					moveBackwards();
				}
			}
		}
	}
	//ros::Duration(1.0).sleep();

}

void wander(){
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

void moveToPixy(int x_offset, int y_offset) {
	
	if (destinationIsAheadPixy(x_offset) && y_offset > 190) {
		std::cout << "picking up puck" << std::endl;
		moveForwards();
		ros::Duration(2.0).sleep();
		return;
	}
	if (destinationIsAheadPixy(x_offset)) {
		std::cout << "destination IS ahead" << std::endl;
		if (canMoveForwards {
			moveForwards();
		} else {
			//avoidCounter = 5;
			avoid();
		}
		return;
	}

	if (destinationIsRightPixy(x_offset)) {
		//if (canTurnRight {
			std::cout << "NOT ahead, turning right" << std::endl;
			turnRightSlow();
		//} else {
		//	avoidCounter = 5;
		//	avoid();
		//}
		return;
	}
	if (destinationIsLeftPixy(x_offset)) {
		//if (canTurnLeft {
			std::cout << "NOT ahead, turning left" << std::endl;
			turnLeftSlow();
		//} else {
		//	avoidCounter = 5;
		//	avoid();
		//}

	}

}
void moveTo(int distance, int bearing) {

	if (destinationIsAhead(bearing)) {
		std::cout << "destination IS ahead" << std::endl;
		if (canMoveForwards) {
			moveForwards();
		} else {
			//avoidCounter = 5;
			avoid();
		}
		return;
	}

	if (destinationIsRight(bearing)) {
	//	if (canTurnRight {
		std::cout << "bearing is " << bearing << " turning right" << std::endl;
		turnRight();
	//	} else {
	//		avoidCounter = 5;
	//		avoid();
	//	}
		return;
	}
	if (destinationIsLeft(bearing)) {
		//if (canTurnLeft {
			std::cout << "bearing is " << bearing << " turning left" << std::endl;
			turnLeft();
		//} else {
		//	avoidCounter = 5;
		//	avoid();
		//}

	}

}

/****************** GoTo Functions ******************/

void goToNest() {
	/*if (canSeeYellow()) {
		moveTo(detectedYellowColours[0].x, detectedYellowColours[0].y);
	}*/
		for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 0){
			moveTo(detectedTags[i].distance, detectedTags[i].bearing);
		}
	}
}

void goToSource() {
		for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 1){
			moveTo(detectedTags[i].distance, detectedTags[i].bearing);
		}
	}
}

/****************** Puck Functions ******************/
/*
	For each robot adjust detection distances and bearings
*/
bool canSeePuck() {
	if(greenColours.size()>0){
		return true;
	}else{
		return false;
	}
}

void pickUpPuck(){
// TODO: add signature to differentiate pucks from others
	int max = 0;
	int maxX = 0;
	for (int i=0; i<greenColours.size(); i++){
		//std::cout << greenColours[i].roi.x_offset << "\n" <<std::endl;
		if(pixyBlocks[i].roi.x_offset < 110){
			std::cout << "turning left\n" << std::endl;
			turnLeftSlow();
		}else{
			if(pixyBlocks[i].roi.x_offset > 190){
				std::cout << "turning right\n" << std::endl;
				turnRightSlow();
			}else{
				std::cout << "moving forwards\n" << std::endl;
				moveForwards();
			}
		}
	}
}

void piCamGoToNearestPuck(){
	int bearing, minDistance;
	minDistance = 1000;
	for(int i=0; i<greenColours.size();i++){
		if(greenColours[i].distance < minDistance){
			minDistance = greenColours[i].distance;
			bearing = greenColours[i].bearing;
		}
	}
	moveTo(minDistance, bearing);
}
void depositPuck(){
	
	if(hasPuck()){
		moveBackwards();
		ros::Duration(2.0).sleep();
	}
	avoid();
	ros::Duration(1.5).sleep();
	return;
}
void pixyPickUpPuck(){

	int destX, minDestY;
	minDestY = 1000;
	for(int i=0; i<pixyBlocks.size();i++){
		if(pixyBlocks[i].signature == 1 && pixyBlocks[i].roi.y_offset < minDestY){
			destX = pixyBlocks[i].roi.x_offset;
			minDestY = pixyBlocks[i].roi.y_offset;
		}
	}
	moveToPixy(destX, minDestY);
}


/****************** Callback Functions ******************/

void coloursCb(const colour_detector::ColourDetectionArray::ConstPtr& msg){

	detectedColours = msg -> detections;
	canMoveForwards = msg -> canMoveForwards;
	canTurnLeft = msg -> canTurnLeft;
	canTurnRight = msg -> canTurnRight;
	blueColours.clear();
	greenColours.clear();
	orangeColours.clear();
	for(int i=0; i<detectedColours.size(); i++){
		if(detectedColours[i].colour == "blue"){
			blueColours.push_back(detectedColours[i]);
		}
		if(detectedColours[i].colour == "green"){
			greenColours.push_back(detectedColours[i]);
		}
		if(detectedColours[i].colour == "orange"){
			orangeColours.push_back(detectedColours[i]);
		}
	}

	/*std::cout << blueColours.size() << std::endl;
	std::cout << greenColours.size() << std::endl;
	std::cout << orangeColours.size() << std::endl;
	*/

}

void pixyCb(const pixy_node::PixyData::ConstPtr& msg){
	pixyBlocks = msg -> blocks;
}

void aprilTagsCb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg){
	detectedTags = msg -> detections;
}

void sensorsCb(const std_msgs::String::ConstPtr& msg){
	if(msg -> data == ""){
		proximityReading = 12;
	}else{
		std::stringstream reading(msg -> data);
		reading >> proximityReading;
	}

}
/****************** Main Behaviour Functions ******************/

void begin(){
	/*
	 * Log states are: 0 for wandering, 1 for going to nest, 2 for going to source, 3 for picking up puck, 4 for depositing puck, 5 for beacon action
	 */
	
	if (hasPuck && atNest()) {
		//std::cout << "Depositing Puck at nest." << std::endl;
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
			//std::cout << "going to nest\n" << std::endl ;
			logState = 1;
			writeToLog("Going to Nest");
		}

		goToNest();
		return;
	}

	//changed for can see source to at source
	if (atSource() && !hasPuck && canSeePuck()) {

		if (logState != 3) {
			//std::cout >> "Picking up a puck\n" << std::endl;
			logState = 3;
			writeToLog("Picking up a Puck");
		}
		pickUpPuck();
		return;
	}

	// might need to remove this one
	if (canSeeSource() && !hasPuck) {

		if (logState != 2) {
			//std::cout >> "Approaching Source\n" << std::endl;
			logState = 2;
			writeToLog("Going to Source");
		}
		goToSource();
		return;
	}

if ((!hasPuck && !canSeeSource()) || (hasPuck && !canSeeNest())) {
		//std::cout >> "Wandering...\n" << std::endl;
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
			//std::cout >> "Can't find pucks at source, wandering!\n" << std::endl;
			logState = 0;
			writeToLog("Wandering without a Puck");
		}
		wander();
		return;
	}

	//std::cout >> "don't know what to do\n" << std::endl;
	stop();


}

int main(int argc, char **argv) {
	ros::init(argc, argv, "main_node");
	ros::NodeHandle n;
	ros::Subscriber coloursSubscriber = n.subscribe("coloursDetected", 1, coloursCb);
	ros::Subscriber pixySubscriber = n.subscribe("block_data", 1, pixyCb);
	ros::Subscriber aprilTagsSubscriber = n.subscribe("aprilTags", 1, aprilTagsCb);
	ros::Subscriber proximitySensorsSubscriber = n.subscribe("Sensors", 1, sensorsCb);
	motorPublisher = n.advertise<std_msgs::String>("wheelSpeeds", 1);
	ros::Rate r(10);
	while(ros::ok()){
		begin();
		ros::spinOnce();
		r.sleep();
	}
	
	return(0);
}
