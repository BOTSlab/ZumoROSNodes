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
#define PI 3.14159265
ros::Publisher detections_pub;

using namespace cv;
using namespace std;
float imgCenterX = 477.0; //370.0; //1288.0;
float imgCenterY = 259.0; //350.0; //737.0;

int rightTurnCheckX1 = 80;
int rightTurnCheckX2 = 210;
int rightTurnCheckY = 180;

int leftTurnCheckX1 = 300;
int leftTurnCheckX2 = 430;
int leftTurnCheckY = 185;

int moveForwardsCheckX1 = 200;
int moveForwardsCheckX2 = 310;
int moveForwardsCheckY = 190;


bool canTurnRight;
bool canTurnLeft;
bool canMoveForwards;


float getDistance(float )
{

}

colour_detector::ColourDetectionArray getDetectedColours(const cv_bridge::CvImageConstPtr &cvSegmentationImage, Scalar lowerBound, Scalar higherBound, string colour){
    
	Mat HSV;
	cvtColor(cvSegmentationImage->image, HSV, COLOR_BGR2HSV);
	//cv::imshow("image",HSV);

	cv::Mat threshold;
	inRange(HSV, lowerBound, higherBound, threshold);



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
	colour_detector::ColourDetectionArray colourDetectionArray;
	colourDetectionArray.canMoveForwards = canMoveForwards;
	colourDetectionArray.canTurnRight = canTurnRight;
	colourDetectionArray.canTurnLeft = canTurnLeft;
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
	return colourDetectionArray;
}
void imageCb(const sensor_msgs::ImageConstPtr &image){

	colour_detector::ColourDetectionArray colourDetectionArray;

	cv_bridge::CvImageConstPtr cvSegmentationImage = cv_bridge::toCvCopy(image,"bgr8");

        colourDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(65,0,0), Scalar(90,255,255), "blue");
	
	//colour_detector::ColourDetectionArray pinkDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(4, 50, 50), Scalar(10, 255, 255), "pink");

	//colour_detector::ColourDetectionArray redDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(0, 50, 50), Scalar(0, 255, 255), "red");

	colour_detector::ColourDetectionArray greenDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(52, 50, 50), Scalar(57, 255, 255), "green");
	
	//colour_detector::ColourDetectionArray orangeDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(8, 0, 0), Scalar(14, 255, 255), "orange");	
	
	//colour_detector::ColourDetectionArray mauveDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(165, 0, 0), Scalar(185, 255, 255), "mauve");

	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), pinkDetectionArray.detections.begin(), pinkDetectionArray.detections.end());
	colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), greenDetectionArray.detections.begin(), greenDetectionArray.detections.end());
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), orangeDetectionArray.detections.begin(), orangeDetectionArray.detections.end());
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), mauveDetectionArray.detections.begin(), mauveDetectionArray.detections.end());

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
