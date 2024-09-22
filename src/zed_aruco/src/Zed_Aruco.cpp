// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "utils.hpp"
#include "aruco.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_aruco/msg/aruco_pose_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

// ROS2  Params
#define TOPIC_NAME "pose_data"
#define QUEUE_LENGTH 10
#define TIMER_DURATION 10ms

// Program Params
#define DEBUG true
// Activate point cloud based position and rotation
// Default is openCV
#define ZED_POSE true	

// Zed SDK Runtime Paramiters
#define CONFIDENCE_THRESHOLD 					95
#define TEXTURE_CONFIDENCE_THRESHOLD 	95
#define ENABLE_FILL_MODE							true

// Zed SDK Init Paramiters
#define MINIMUM_DEPTH_DISTANCE				50	// In camera units
#define CAMERA_RESOLUTION 						sl::RESOLUTION::HD1080
#define CAMERA_UNITS 									sl::UNIT::MILLIMETER
#define DEPTH_MODE										sl::DEPTH_MODE::ULTRA
#define ENABLE_SENSORS								true
#define ENABLE_DEPTH_STABILIZATION		true

// OpenCV Aruco Paramiters
#define ERROR_CORRECTION_RATE					0
#define MARKER_SIZE										50
#define DELAY 												1
#define DICTIONARY										cv::aruco::DICT_4X4_5

// Zed SDK Tracking Params
#define ENABLE_IMU_FUSION							true
#define SET_GRAVITY_AS_ORIGIN					true
#define ENABLE_AREA_MEMORY						true
#define ENABLE_POSE_SMOOTHING					true
#define ENABLE_SET_AS_STATIC 				  true
#define PLANE_ORIENTATION 						false

using namespace std::chrono_literals;
using std::placeholders::_1;

// Draws aruco code position and orientation on display
void drawPoseData(cv::Mat& image, const cv::Point3f& position, const cv::Point3f& eulerAngles) {
	std::string positionText = "Position: x=" + std::to_string(position.x) + 
																			" y=" + std::to_string(position.y) + 
																			" z=" + std::to_string(position.z);
	std::string eulerText = "Euler: x=" + std::to_string(eulerAngles.x) + 
																" y=" + std::to_string(eulerAngles.y) + 
																" z=" + std::to_string(eulerAngles.z);
	
	// Display the position data on the image
	cv::putText(image, positionText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
	
	// Display the orientation data on the image
	cv::putText(image, eulerText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

// Function to display the image from camera
void detectAndDisplay(cv::Mat& image, std::vector<std::vector<cv::Point2f>>& corners, std::vector<int>& ids) {

	// Draw detected markers and corners on the image
	cv::aruco::drawDetectedMarkers(image, corners, ids);

	// Pose estimation (this is a placeholder - replace with actual pose data retrieval)
	/*
	for (size_t i = 0; i < ids.size(); ++i) {
    cv::Point3f position(100.0f, 50.0f, 30.0f);  // Placeholder for actual position
    cv::Point3f eulerAngles(0.1f, 0.2f, 0.3f);   // Placeholder for actual orientation
    
    drawPoseData(image, position, eulerAngles);
	}*/

	// Display the image with the detected markers and pose data
	cv::imshow("Detected Markers and Pose", image);
	cv::waitKey(DELAY);  // Wait for a key press
}

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher()
  : Node("_publisher")
  {

		// Init Zed camera params
		init_params.camera_resolution = CAMERA_RESOLUTION;
		init_params.coordinate_units = CAMERA_UNITS;
		init_params.sensors_required = ENABLE_SENSORS;
		//init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
		init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
		init_params.depth_minimum_distance = MINIMUM_DEPTH_DISTANCE;
		init_params.depth_stabilization = ENABLE_DEPTH_STABILIZATION;
		init_params.depth_mode = DEPTH_MODE; 

		sl::ERROR_CODE err = zed.open(init_params);
		if (err != sl::ERROR_CODE::SUCCESS)
		{
		  std::cerr << "Error, unable to open ZED camera: " << sl::toString(err).c_str() << "\n";
		  zed.close();
		  exit(EXIT_FAILURE); // Quit if an error occurred
		}

		// Init Zed runtime params
		runtime_params.confidence_threshold = CONFIDENCE_THRESHOLD; // Increase this value to reduce noise
		runtime_params.texture_confidence_threshold = TEXTURE_CONFIDENCE_THRESHOLD; // Adjust as necessary
		runtime_params.enable_fill_mode = ENABLE_FILL_MODE;

		// Image initialisation
		auto cameraInfo = zed.getCameraInformation();
		image_size = cameraInfo.camera_configuration.resolution;
		image_zed.alloc(image_size, sl::MAT_TYPE::U8_C4);
		image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
		
		// Camera Calibration
		auto calibInfo = cameraInfo.camera_configuration.calibration_parameters.left_cam;
		camera_matrix = cv::Matx33d::eye();
		camera_matrix(0, 0) = calibInfo.fx;
		camera_matrix(1, 1) = calibInfo.fy;
		camera_matrix(0, 2) = calibInfo.cx;
		camera_matrix(1, 2) = calibInfo.cy;

		// OpenCV init
		dist_coeffs = cv::Vec4f::zeros();	//Used for opencv position estimation
		dictionary = cv::aruco::getPredefinedDictionary(DICTIONARY);
		//dictionary = cv::aruco::generateCustomDictionary(1, 4, cv::aruco::getPredefinedDictionary(DICTIONARY));
    
		std::cout << "Custom dictionary contains " << dictionary.bytesList.rows << " markers." << std::endl;

    // Set the error correction rate
    parameters.errorCorrectionRate = ERROR_CORRECTION_RATE;
		parameters.cornerRefinementWinSize = 5;    // Size of the window for the corner refinement
		parameters.cornerRefinementMaxIterations = 30;  // Maximum number of iterations for corner refinement
		parameters.cornerRefinementMinAccuracy = 0.05;   // Minimum accuracy for corner refinement
		parameters.adaptiveThreshWinSizeMin = 3;
		parameters.adaptiveThreshWinSizeMax = 30;
		parameters.adaptiveThreshWinSizeStep = 9;
		parameters.minMarkerPerimeterRate = 0.01;
		parameters.maxMarkerPerimeterRate = 4.0;


		std::cout << "Make sure the ArUco marker is a 4x4 (250), measuring " << actual_marker_size_meters << " mm" << std::endl;

		// Zed Camera tracking params init
		tracking_params.enable_imu_fusion = ENABLE_IMU_FUSION; // for this sample, IMU (of ZED-M) is disable, we use the gravity given by the marker.
		tracking_params.set_gravity_as_origin = SET_GRAVITY_AS_ORIGIN;
		tracking_params.enable_area_memory = ENABLE_AREA_MEMORY;
		tracking_params.enable_pose_smoothing = ENABLE_POSE_SMOOTHING; 
		tracking_params.set_as_static = ENABLE_SET_AS_STATIC;
		
		err = zed.enablePositionalTracking(tracking_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
			std::cerr << "Error enabling positional tracking: " << sl::toString(err) << std::endl;
			zed.close();
			exit(EXIT_FAILURE);
    }
		
		// Kalman Filtering ***NOT USED YET***
		/*
		KF = cv::KalmanFilter(4, 2, 0);
		KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		KF.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 1.0;  // Start with a higher value
		
		cv::setIdentity(KF.measurementMatrix);
		cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
		cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
		cv::setIdentity(KF.errorCovPost, cv::Scalar::all(0.1));
		*/

		rotation_matrix = camera_pose.getRotationMatrix();
		if (DEBUG) printRotationAsEulerAngles(rotation_matrix);

    publisher_ = this->create_publisher<zed_aruco::msg::ArucoPoseArray>(TOPIC_NAME, QUEUE_LENGTH);
    timer_ = this->create_wall_timer(TIMER_DURATION, std::bind(&PosePublisher::publish_pose, this));
  }

private:
  sl::Camera zed;
	sl::Rotation rotation_matrix;
  sl::Mat image_zed;
	sl::Mat point_cloud;
  sl::InitParameters init_params;
	sl::RuntimeParameters runtime_params;
  sl::Resolution image_size;
  sl::PositionalTrackingParameters tracking_params;
  
  sl::POSITIONAL_TRACKING_STATE tracking_state;
  
	sl::Transform pose;
	sl::Pose camera_pose;
	sl::Transform plane_transform;
	sl::Plane plane;
	sl::Orientation orientation;
	
	cv::Mat image_ocv;
	cv::Mat image_ocv_rgb;
	cv::aruco::Dictionary dictionary;
	cv::Matx<float, 4, 1> dist_coeffs;
	cv::Matx33d camera_matrix;
	
	std::vector<cv::Vec3d> rvecs, tvecs;
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	std::string position_txt;
  
  float actual_marker_size_meters = MARKER_SIZE*0.001; // real marker size in meters -> IT'S IMPORTANT THAT THIS VARIABLE CONTAINS THE CORRECT SIZE OF THE TARGET

	rclcpp::Publisher<zed_aruco::msg::ArucoPoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

	//cv::KalmanFilter KF;
	cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

  void publish_pose(){
		
		if (zed.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) return;
		
		zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);

		// Gets updated rotation matrix of camera if not set as static
		if (!ENABLE_SET_AS_STATIC){
			rotation_matrix = camera_pose.getRotationMatrix();
			if (DEBUG) printRotationAsEulerAngles(rotation_matrix);
		}
		
		// Retrieve the left image
		zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
		zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, image_size);
		// convert to RGB
		cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_BGRA2BGR);
		cv::Mat grayImage;
		//cv::cvtColor(image_ocv, grayImage, cv::COLOR_BGR2GRAY);
		// detect marker
		//cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);
		cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids, parameters);

		zed_aruco::msg::ArucoPoseArray message = zed_aruco::msg::ArucoPoseArray();

		// corner pixel refinement
		/*
		for (size_t i = 0; i < corners.size(); ++i) {
			cv::cornerSubPix(grayImage, corners[i], cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.05));
		}
		*/

		if (DEBUG) detectAndDisplay(image_ocv_rgb, corners, ids);

		if (ids.size() > 0)
		{
			cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);
			int ids_size = ids.size();
			
			for (int i = 0; i < ids_size; i++)
			{
				// Kalman Filtering, no need
				/*
				for (int j = 0; j < 4; ++j) {
					cv::Mat prediction = KF.predict();
					cv::Mat measurement = (cv::Mat_<float>(2, 1) << corners[i][j].x, corners[i][j].y);
					cv::Mat estimated = KF.correct(measurement);
					//corners[i][j] = cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
				}
				*/

				zed_aruco::msg::ArucoPose element = zed_aruco::msg::ArucoPose();
				element.id = ids[i];

				cv::Point2f center;
				cv::Point3f euler_angles;
				std::array<double, 4> quaternions;
				if (DEBUG) std::cout<< "Corners: " <<corners[i]<<"\n";
				center = std::accumulate(corners[i].begin(), corners[i].end(), cv::Point2f(0.0f,0.0f), addPoints);
				center.x = center.x/4;
				center.y = center.y/4;
				cv::Point3f point3D = retrieveValid3DPoint(static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y)), point_cloud);
				if (DEBUG) std::cout<< "3D Points: " <<point3D<<"\n";
				point3D = getCorrectedPosition(point3D ,rotation_matrix);

				std::array<cv::Point3f, 3> markerVectors;
				sl::ERROR_CODE find_plane_status = zed.findPlaneAtHit(sl::uint2(static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y))), plane);
				
				// Use Zed SDK's plane detection for pose
				if (find_plane_status == sl::ERROR_CODE::SUCCESS && PLANE_ORIENTATION) {
					// Get the transform of the plane according to the global reference frame
					orientation = plane.getPose().getOrientation();
					sl::Matrix3f plane_rotation_matrix = plane.getPose().getOrientation().getRotationMatrix();
					sl::Matrix3f corrected_plane_rotation_matrix = rotation_matrix * plane_rotation_matrix;
					orientation = convertArrayToOrientation(rotationMatrixToQuaternion(convertMatrix3fToArray(corrected_plane_rotation_matrix)));

					if (DEBUG) std::cout<<orientation<<std::endl;
					euler_angles = quaternionToEuler(orientation);

					quaternions[0] = orientation.w; // w component
					quaternions[1] = orientation.x; // x component
					quaternions[2] = orientation.y; // y component
					quaternions[3] = orientation.z; // z component
					
				}

				else {
					markerVectors = getMarkerVector(corners[i], zed, point_cloud, rotation_matrix);
					//std::array<cv::Point3f, 4> cornerPosition = getCornerPosition(corners[i], point_cloud);
					//std::cout<<"ID: "<<ids[i]<<std::endl;
					//printArrayAsVectors(cornerPosition);
					//std::array<cv::Point3f, 4> squareCorners = fitSquareToPoints(cornerPosition, MARKER_SIZE, point3D);
					//std::array<cv::Point3f, 3> markerVectors = calculateRotationalVector(squareCorners);
					//printArrayAsVectors(squareCorners);
					euler_angles = rotationMatrixToEulerAngles(markerVectors);
					quaternions = rotationMatrixToQuaternion(markerVectors);
				}
				
				if(ZED_POSE){
					element.position.x = point3D.z;
					element.position.y = point3D.x;
					element.position.z = point3D.y;
					element.orientation.euler.x = euler_angles.z;
					element.orientation.euler.y = euler_angles.x;
					element.orientation.euler.z = euler_angles.y;
					element.orientation.quaternion.w = quaternions[0];
					element.orientation.quaternion.i = quaternions[3];
					element.orientation.quaternion.j = quaternions[1];
					element.orientation.quaternion.k = quaternions[2];
				}
				else{
					element.position.x = tvecs[i](2);
					element.position.y = tvecs[i](0);
					element.position.z = tvecs[i](1);
					element.orientation.euler.x = rvecs[i](2);
					element.orientation.euler.y = rvecs[i](0);
					element.orientation.euler.z = rvecs[i](1);
				}
				message.data.push_back(element);
			}

		}
		publisher_->publish(message);
		
  }

	
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}
