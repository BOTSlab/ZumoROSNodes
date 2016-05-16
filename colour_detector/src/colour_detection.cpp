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
float imgCenterX = 370.0; //1288.0;
float imgCenterY = 350.0; //737.0;
float getDistance(float )
{

}

colour_detector::ColourDetectionArray getDetectedColours(const cv_bridge::CvImageConstPtr &cvSegmentationImage, Scalar lowerBound, Scalar higherBound, string colour){
    
	Mat HSV;
	cvtColor(cvSegmentationImage->image, HSV, COLOR_BGR2HSV);
	//cv::imshow("image",HSV);

	cv::Mat threshold;
	inRange(HSV, lowerBound, higherBound, threshold);
	//cv::Mat resizedthreshold;
	//cv::resize(threshold, resizedthreshold, cv::Size(), 0.5,0.5, cv::INTER_LINEAR);
	cv::waitKey(10);
	cv::imshow("image",cvSegmentationImage->image);

	//Morphological operators are far too slow
	//morphological opening (removes small objects from the foreground)
	//erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	//dilate(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//erode(threshold, threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

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
		if(colourDetection.blobSize > 100){
			colourDetectionArray.detections.push_back(colourDetection);
		}
		
	}
	return colourDetectionArray;
}
void imageCb(const sensor_msgs::ImageConstPtr &image){

	colour_detector::ColourDetectionArray colourDetectionArray;

	cv_bridge::CvImageConstPtr cvSegmentationImage = cv_bridge::toCvCopy(image,"bgr8");

        colourDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(84,0,0), Scalar(150,255,255), "blue");
	
	//colour_detector::ColourDetectionArray redDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(0, 50, 50), Scalar(0, 255, 255), "red");

	//colour_detector::ColourDetectionArray greenDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(50, 50, 50), Scalar(65, 255, 255), "green");
	//colour_detector::ColourDetectionArray greenDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(47, 50, 50), Scalar(58, 255, 255), "green");
	//colour_detector::ColourDetectionArray orangeDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(10, 0, 0), Scalar(18, 255, 255), "orange");
	//colour_detector::ColourDetectionArray mauveDetectionArray = getDetectedColours(cvSegmentationImage, Scalar(165, 0, 0), Scalar(185, 255, 255), "mauve");
	//colourDetectionArray.detections.insert(colourDetectionArray.detections.end(), greenDetectionArray.detections.begin(), greenDetectionArray.detections.end());
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
