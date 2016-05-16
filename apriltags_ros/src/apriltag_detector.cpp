#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
#include <cv_bridge/cv_bridge.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>


float imgCenterX = 185.0; //370.0;
float imgCenterY = 172.0; //350.0;

double camera_focal_length_x(700); // in pixels. late 2013 macbookpro retina = 700
double camera_focal_length_y(700); // in pixels
double tag_size(0.029); // tag side length of frame in meters
ros::Publisher detections_pub;
const double PI = 3.14159265358979323846;
const double TWOPI = 2.0 * PI;
inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t + PI, TWOPI) - PI;
	} else {
		t = fmod(t - PI, -TWOPI) + PI;
	}
	return t;
}
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
		double& roll) {
	yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
	roll = standardRad(
			atan2(wRo(0, 2) * s - wRo(1, 2) * c,
					-wRo(0, 1) * s + wRo(1, 1) * c));
}
void imageCb(const sensor_msgs::ImageConstPtr &image){
// may need to resize image for performance
	AprilTags::TagCodes tag_codes(AprilTags::tagCodes16h5);	
	AprilTags::TagDetector* tag_detector = new AprilTags::TagDetector(tag_codes);

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
	cv::Mat image_gray;
	cv::cvtColor(cv_ptr->image, image_gray, CV_BGR2GRAY);
	
	//image_gray.convertTo(image_gray, -1, 2, 0);
	cv:threshold(image_gray, image_gray, 75, 255, cv::THRESH_BINARY);    
	cv::imshow("image",image_gray);    
	cv::waitKey(10);
	vector<AprilTags::TagDetection> detections = tag_detector->extractTags(image_gray);
	apriltags_ros::AprilTagDetectionArray tag_detection_array;
	//std::cout << detections.size() << std::endl;
	for (int i = 0; i < detections.size(); i++) {
		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)
		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detections[i].getRelativeTranslationRotation(tag_size, camera_focal_length_x, camera_focal_length_y, cv_ptr->image.cols / 2, cv_ptr->image.rows / 2, translation, rotation);
		Eigen::Matrix3d F;
		F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
		Eigen::Matrix3d fixed_rot = F * rotation;
		double yaw, pitch, roll;
		wRo_to_euler(fixed_rot, yaw, pitch, roll);
		apriltags_ros::AprilTagDetection tag_detection;
		tag_detection.id = detections[i].id;
		tag_detection.x = detections[i].cxy.first; //pixel x-location
		tag_detection.y = detections[i].cxy.second; //pixel y-location
		tag_detection.yaw = yaw;
		tag_detection.pitch = pitch;
		tag_detection.roll = roll;
		tag_detection.code = detections[i].code;
		tag_detection.distance = sqrt(pow((imgCenterX - tag_detection.x), 2)+pow((imgCenterY - tag_detection.y), 2));
		tag_detection.bearing = atan2((tag_detection.y - imgCenterY), (tag_detection.x - imgCenterX)) * 180 / PI ;
		tag_detection_array.detections.push_back(tag_detection);

	}
	detections_pub.publish(tag_detection_array);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apriltag_detector");
	ros::NodeHandle n;
	ros::Subscriber imageSubscriber = n.subscribe("small_image_topic", 1, imageCb);
	detections_pub = n.advertise<apriltags_ros::AprilTagDetectionArray>("aprilTags", 1);
	ros::Rate r(10);
	while(ros::ok()){
		//begin();
		ros::spinOnce();
		r.sleep();
	}
	return(0);
}
