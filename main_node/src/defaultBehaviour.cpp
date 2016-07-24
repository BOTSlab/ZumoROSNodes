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
using namespace std;

ros::Publisher motorPublisher;


int state = 0;
int subState = 0;

int factor = 2;
int proximityReading = 12;
int puckYCoor = 0;//////////////////////////
int puckXLCoor = 0;
int puckXRCoor = 0;
int avoidCounter = 0;
string beaconPriorities [3] = {"0","1","2"};
bool turnedLeft = false;
bool turnedRight = false;
bool firstSourceDepleted = false;
bool secondSourceDepleted = false;
std::vector<colour_detector::ColourDetection> detectedColours;
std::vector<colour_detector::ColourDetection> blueColours;
std::vector<colour_detector::ColourDetection> orangeColours;
std::vector<colour_detector::ColourDetection> greenColours;
std::vector<pixy_node::PixyBlock> pixyBlocks;
std::vector<apriltags_ros::AprilTagDetection> detectedTags;

void publishMotorSpeed(string wheelSpeed) {

	std_msgs::String msg;	
	std::stringstream ss;
	ss << wheelSpeed;
	msg.data = ss.str();
	motorPublisher.publish(msg);
	

}
// green is 0, orange is 1? for now
bool canSeeBlue() {
	if(blueColours.size()>0){
		return true;
	}

	return false;
}

bool pixyCanSeeGreen(){
	for(int i=0; i<pixyBlocks.size(); i++){
		if(pixyBlocks[i].signature == 5){
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

bool canSeeOrange() {
	if(orangeColours.size()>0){
		return true;
	}
	if(pixyCanSeeOrange()){
		return true;
	}

	return false;
}
/*
	This method returns 0 if there is an obstacle to the left, 1 if there is no obstacle, and 2 if there is an obstacle to the right
*/
int canMoveForwards() {

	//the pixy can't see the blue colour around the robots because it's mounted too high
	for (uint i = 0; i < blueColours.size(); i++) {
		if (blueColours[i].distance < 190.0 && blueColours[i].bearing > -60.0 && blueColours[i].bearing < 60.0){
			if(blueColours[i].bearing < -3){
				std::cout << "Obstacle on the left!" << std::endl;
				return 0;
			}else{
				std::cout << "Obstacle on the right!" << std::endl;
				return 2;
			}
		}
	}
	std::cout << "All clear move fowards!" << std::endl;
	return 1;
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


//range is limited so just seeing the tag means the robot is where it needs to be
bool atNest(){
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 0){
			return true;
		}
	}
	return false;
}

bool atFirstSource(){
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 1){
			return true;
		}
	}
	return false;
}

bool atSecondSource(){
	for(int i=0; i<detectedTags.size(); i++){
		if(detectedTags[i].id == 2){
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
bool hasPuck(){
	if(proximityReading == 12){
		return false;
	}
	return true;
}


void avoid(int direction) {
	if(direction == 0){
	std::cout << "avoiding by going right" << std::endl;
		turnRight();
	}else{
		if(direction == 1){
			std::cout << "avoiding by going forwards" << std::endl;
			moveForwards();
		}else{
			std::cout << "avoiding by going left" << std::endl;
			turnLeft();	
		}
	}

}

void wander(){
	int randomNumber = rand() % 4;
	if(randomNumber == 0){
		std::cout << "turning left\n" << std::endl;
		turnLeft();
		ros::Duration(2.0).sleep();
	}else{
		if(randomNumber == 1){
			std::cout << "turning right\n" << std::endl;
			turnRight();
			ros::Duration(2.0).sleep();
		}else{
			std::cout << "moving forwards\n" << std::endl;
			if(canMoveForwards() == 1){
				moveForwards();
				ros::Duration(2.0).sleep();
			}else{
				/*randomNumber = rand() % 2;
				if(randomNumber == 0){
					turnLeft();
				}else{
					turnRight();
				}*/
				avoidCounter = 10;
				avoid(canMoveForwards());
			}
			
		}
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
		if (canMoveForwards() == 1) {
			moveForwards();
		} else {
			avoidCounter = 10;
			avoid(canMoveForwards());
		}
		return;
	}

	if (destinationIsRightPixy(x_offset)) {
		//if (canTurnRight()) {
			std::cout << "NOT ahead, turning right" << std::endl;
			turnRightSlow();
		//} else {
		//	avoidCounter = 5;
		//	avoid(canMoveForwards());
		//}
		return;
	}
	if (destinationIsLeftPixy(x_offset)) {
		//if (canTurnLeft()) {
			std::cout << "NOT ahead, turning left" << std::endl;
			turnLeftSlow();
		//} else {
		//	avoidCounter = 5;
		//	avoid(canMoveForwards());
		//}

	}

}
void moveTo(int distance, int bearing) {

	if (destinationIsAhead(bearing)) {
		std::cout << "destination IS ahead" << std::endl;
		if (canMoveForwards() == 1) {
			moveForwards();
		} else {
			avoidCounter = 10;
			avoid(canMoveForwards());
		}
		return;
	}

	if (destinationIsRight(bearing)) {
	//	if (canTurnRight()) {
		std::cout << "bearing is " << bearing << " turning right" << std::endl;
		turnRight();
	//	} else {
	//		avoidCounter = 5;
	//		avoid(canMoveForwards());
	//	}
		return;
	}
	if (destinationIsLeft(bearing)) {
		//if (canTurnLeft()) {
			std::cout << "bearing is " << bearing << " turning left" << std::endl;
			turnLeft();
		//} else {
		//	avoidCounter = 5;
		//	avoid(canMoveForwards());
		//}

	}

}


void pickUpPuck(){
// TODO: add signature to differentiate pucks from others
	if(pixyBlocks.size()>0){
		for (int i=0; i<pixyBlocks.size(); i++){
			std::cout << pixyBlocks[i].roi.x_offset << "\n" <<std::endl;
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
	}else{
		stop();
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

}
void pixyPickUpPuck(){

	int destX, minDestY;
	minDestY = 1000;
	for(int i=0; i<pixyBlocks.size();i++){
		if(pixyBlocks[i].signature == 5 && pixyBlocks[i].roi.y_offset < minDestY){
			destX = pixyBlocks[i].roi.x_offset;
			minDestY = pixyBlocks[i].roi.y_offset;
		}
	}
	moveToPixy(destX, minDestY);
}

void coloursCb(const colour_detector::ColourDetectionArray::ConstPtr& msg){

	detectedColours = msg -> detections;
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
void begin(){

	/*
	Signal a go to nest command with april tag?
	if(firstSourceDepleted && secondSourceDepleted && atNest()){
		std::cout << "Task Completed: Stopping." << std::endl;
		stop();
		return;
	}*/
	std::cout << avoidCounter << std::endl;
	if(avoidCounter > 0){
		std::cout << "AVOIDING!!!" << std::endl;
		avoid(canMoveForwards());
		avoidCounter--;
		return;
	}
	if(firstSourceDepleted && secondSourceDepleted && !atNest()){
		std::cout << "All sources depleted. Looking for nest." << std::endl;
		wander();
		return;
	}
	if(hasPuck() && !atNest()){
		std::cout << "Looking for nest to deposit puck." << std::endl;
		wander();
		return;
	}
	if(hasPuck() && atNest()){
		std::cout << "Depositing Puck at nest." << std::endl;
		depositPuck();
		return;
	}
	if(!hasPuck() && atFirstSource()){
		if(pixyCanSeeGreen()){
			std::cout << "Picking up puck at first source via pixy." << std::endl;
			pixyPickUpPuck();
			return;
		}
		if(piCamCanSeeGreen()){
			std::cout << "Picking up puck at first source via picamera." << std::endl;
			piCamGoToNearestPuck();
			return;
		}
		// in this case there is no green around the first source so we assume it ran out
		std::cout << "First source has no pucks, is depleted. Wandering." << std::endl;
		wander();
		return;
	}
	if(!hasPuck() && atSecondSource()){
		if(pixyCanSeeGreen()){
			std::cout << "Picking up puck at second source via pixy." << std::endl;
			pixyPickUpPuck();
			secondSourceDepleted = false;
			return;
		}
		if(piCamCanSeeGreen()){
			std::cout << "Picking up puck at second source via picamera." << std::endl;
			piCamGoToNearestPuck();
			return;
		}
		// in this case there is no green around the first source so we assume it ran out
		std::cout << "Second source has no pucks. Wandering." << std::endl;
		wander();
		return;
	}

	if(!hasPuck() && !atFirstSource() && !atSecondSource()){
		std::cout << "Finding pucks, wandering." << std::endl;
		wander();
		return;
	}


/*
		if(pixyCanSeeGreen()){
			std::cout << "Picking up puck at second source via pixy." << std::endl;
			pixyPickUpPuck();
			secondSourceDepleted = false;
			return;
		}
*/
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
