#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cvwimage.h>
#include <colour_detector/ColourDetection.h>
#include <colour_detector/ColourDetectionArray.h>
#include <math.h>

#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <string>


#define PI 3.14159265
ros::Publisher detections_pub;

using namespace cv;
using namespace std;
float imgCenterX = 477.0; //370.0; //1288.0;
float imgCenterY = 259.0; //350.0; //737.0;



int moveForwardsCheckX11 = 672;
int moveForwardsCheckX12 = 695;
int moveForwardsCheckY11 = 160;
int moveForwardsCheckY12 = 365;

int moveForwardsCheckX21 = 620;
int moveForwardsCheckX22 = 678;
int moveForwardsCheckY21 = 145;
int moveForwardsCheckY22 = 170;

int moveForwardsCheckX31 = 662;
int moveForwardsCheckX32 = 690;
int moveForwardsCheckY31 = 330;
int moveForwardsCheckY32 = 368;

int turnLeftCheckX1 = 435;
int turnLeftCheckX2 = 630;
int turnLeftCheckY1 = 110;
int turnLeftCheckY2 = 128;

int turnRightCheckX1 = 435;
int turnRightCheckX2 = 630;
int turnRightCheckY1 = 438;
int turnRightCheckY2 = 450;


bool canTurnRight;
bool canTurnLeft;
bool canMoveForwards;


float getDistance(float )
{

}

colour_detector::ColourDetectionArray getDetectedColours(const cv_bridge::CvImageConstPtr &cvSegmentationImage, Scalar lowerBound, Scalar higherBound, string colour){
    
	Mat HSV;
	cvtColor(cvSegmentationImage->image, HSV, COLOR_BGR2HSV);
	//cv::imshow("image",cvSegmentationImage->image);
	//cv::waitKey(10);
	cv::Mat threshold;
	inRange(HSV, lowerBound, higherBound, threshold);


	colour_detector::ColourDetectionArray colourDetectionArray;
	if (colour == "black" /*|| colour == "cyan" || colour == "green"*/) {
		canTurnRight = true;
		canTurnLeft = true;
		canMoveForwards = true;
		for (int i = turnRightCheckX1; i <= turnRightCheckX2 ; i++) {
			for (int j = turnRightCheckY1; j < turnRightCheckY2; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canTurnRight = false;
					//std::cout << "Can't move right at x: " << i << " y: " << j << std::endl;
				}
			}
		}
		for (int i = turnLeftCheckX1; i <= turnLeftCheckX2 ; i++) {
			for (int j = turnLeftCheckY1; j < turnLeftCheckY2; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canTurnLeft = false;
					//std::cout << "Can't move left at x: " << i << " y: " << j << std::endl;
				}
			}
		}
		for (int i = moveForwardsCheckX11; i < moveForwardsCheckX12 + 1; i++) {
			for (int j = moveForwardsCheckY11; j < moveForwardsCheckY12; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canMoveForwards = false;
					//std::cout << "Can't move forwards at x: " << i << " y: " << j << std::endl;
				}
			}
		}

		for (int i = moveForwardsCheckX21; i < moveForwardsCheckX22 + 1; i++) {
			for (int j = moveForwardsCheckY21; j < moveForwardsCheckY22; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canMoveForwards = false;
					canTurnLeft = false;
					//std::cout << "Can't move forwards or left at x: " << i << " y: " << j << std::endl;
				}
			}
		}

		for (int i = moveForwardsCheckX31; i < moveForwardsCheckX32 + 1; i++) {
			for (int j = moveForwardsCheckY31; j < moveForwardsCheckY32; j++) {
				if (threshold.at<bool>(j, i) == 255) {
					canMoveForwards = false;
					canTurnRight = false;
					//std::cout << "Can't move forwards or right at x: " << i << " y: " << j << std::endl;
				}
			}
		}
		/*if(canMoveForwards){
			std::cout << "CAN move forwards" << std::endl;
		}
		if(canTurnRight){
			std::cout << "CAN turn right" << std::endl;
		}
		if(canTurnLeft){
			std::cout << "CAN turn left" << std::endl;
		}*//*//else{
		//	std::cout << "can NOT move forwards" << std::endl;
		//}

		*/
		//colour_detector::ColourDetectionArray colourDetectionArray;

		
	}else{



	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Get the moments
	// mu[i].m00 represents how big the blob of color is
	vector<Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
	///  Get the mass centers
	for (int i = 0; i < contours.size(); i++)
	{
		colour_detector::ColourDetection colourDetection;
		colourDetection.colour = colour;
		colourDetection.blobSize = mu[i].m00;
		colourDetection.x = mu[i].m10 / mu[i].m00;
		colourDetection.y = mu[i].m01 / mu[i].m00;
		colourDetection.distance = sqrt(pow((imgCenterX - colourDetection.x), 2)+pow((imgCenterY - colourDetection.y), 2));
		colourDetection.bearing = atan2((colourDetection.y - imgCenterY), (colourDetection.x - imgCenterX)) * 180 / PI ;
		// the min blob size depends on the size of the image being used. larger images will yield larger blobs and vice versa
		if(colourDetection.blobSize > 50){
			colourDetectionArray.detections.push_back(colourDetection);
		}
		
	}
}

	return colourDetectionArray;
}
void imageCb(const sensor_msgs::ImageConstPtr &image){

	colour_detector::ColourDetectionArray colourDetectionArray;

	cv_bridge::CvImageConstPtr cvSegmentationImage = cv_bridge::toCvCopy(image,"bgr8");

        //colourDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(65,0,0), Scalar(90,255,255), "blue");
	
	//colour_detector::ColourDetectionArray pinkDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(4, 50, 50), Scalar(10, 255, 255), "pink");

	//colour_detector::ColourDetectionArray redDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(0, 50, 50), Scalar(0, 255, 255), "red");

	colour_detector::ColourDetectionArray greenDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(52, 50, 50), Scalar(57, 255, 255), "green");
	
	//colour_detector::ColourDetectionArray orangeDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(8, 0, 0), Scalar(14, 255, 255), "orange");	
	
	//colour_detector::ColourDetectionArray mauveDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(165, 0, 0), Scalar(185, 255, 255), "mauve");

	colour_detector::ColourDetectionArray blackDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(0, 50, 0), Scalar(40, 255, 57), "black");

	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), pinkDetectionArray.detections.begin(), pinkDetectionArray.detections.end());
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), greenDetectionArray.detections.begin(), greenDetectionArray.detections.end());
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), orangeDetectionArray.detections.begin(), orangeDetectionArray.detections.end());
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), mauveDetectionArray.detections.begin(), mauveDetectionArray.detections.end());
	colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), blackDetectionArray.detections.begin(), blackDetectionArray.detections.end());
	std::cout << "COLOR CMF: " << canMoveForwards << " CTR: " << canTurnRight << " CTL: " << canTurnLeft << std::endl;	
	colourDetectionArray.canMoveForwards = canMoveForwards;
	colourDetectionArray.canTurnRight = canTurnRight;
	colourDetectionArray.canTurnLeft = canTurnLeft;
	detections_pub.publish(colourDetectionArray);
	//detections_pub.publish(redDetectionArray);
	
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "colour_detector");
	ros::NodeHandle n;
	ros::Subscriber imageSubscriber = n.subscribe("image_topic", 10, imageCb);
	detections_pub = n.advertise<colour_detector::ColourDetectionArray>("coloursDetected", 1);
	ros::Rate r(10);
	while(ros::ok()){
		//begin();
		ros::spinOnce();
		r.sleep();
	}
	return(0);
}
