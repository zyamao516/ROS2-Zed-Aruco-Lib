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

#include <memory>

#include "utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_aruco/msg/aruco_pose_array.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;


#define TIMER_DURATION    50ms
#define QUEUE_LENGTH      10
#define PUBLISHER_TOPIC   "grid_data"
#define SUBSCRIBER_TOPIC  "pose_data"

#define TOP_LEFT          1
#define TOP_RIGHT         2
#define BOTTOM_RIGHT      3
#define BOTTOM_LEFT       4

#define SIDE_LENGTH       90

#define MARKER_ID         0

class Aruco_Processor : public rclcpp::Node
{
public:
  Aruco_Processor()
  : Node("Aruco_Processor")
  {
    pose_publisher_ = this->create_publisher<zed_aruco::msg::ArucoPose>(PUBLISHER_TOPIC, QUEUE_LENGTH);
    pose_timer_ = this->create_wall_timer(TIMER_DURATION, std::bind(&Aruco_Processor::publish_pose_values, this));
    subscription_ = this->create_subscription<zed_aruco::msg::ArucoPoseArray>(SUBSCRIBER_TOPIC, QUEUE_LENGTH, std::bind(&Aruco_Processor::read_aruco_position, this, std::placeholders::_1));
  }


private:

  rclcpp::Publisher<zed_aruco::msg::ArucoPose>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::Subscription<zed_aruco::msg::ArucoPoseArray>::SharedPtr subscription_;

  int cornerCodes[4] = {TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT};
  std::array<cv::Point3f, 3> grid_vector;
  cv::Point3f position;
  cv::Point3f euler;
  std::array<double, 4> quaternions;

  int marker_id;


  void read_aruco_position(const zed_aruco::msg::ArucoPoseArray::SharedPtr msg)
  {
    marker_id = MARKER_ID;
    if (msg->data.size()<3) {
      std::cout<<"err"<<std::endl;
      marker_id = -1;
      return;
    }
    std::array<cv::Point3f, 4> point3D;
    std::array<cv::Point3f, 4> estimatedPoints;
    for (auto& point : point3D) {
      point = cv::Point3f(std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN());
    }

    for (const auto& marker : msg->data){
      int* found = std::find(std::begin(cornerCodes), std::end(cornerCodes), marker.id);
      if (found == std::end(cornerCodes)) continue;

      int index;
      switch(marker.id){
        case TOP_LEFT:
          index = 0;
          break;
        case TOP_RIGHT:
          index = 1;
          break;
        case BOTTOM_RIGHT:
          index = 2;
          break;
        case BOTTOM_LEFT:
          index = 3;
          break;
        default:
          continue;
      }
      point3D[index].x = marker.position.y;
      point3D[index].y = marker.position.z;
      point3D[index].z = marker.position.x;
    }

    cv::Point3f sum(0.0f, 0.0f, 0.0f);
    int validPoints = 0;

    // Array to keep track of which points are valid
    std::array<bool, 4> valid = { 
      !std::isnan(point3D[0].x),  // TOP_LEFT
      !std::isnan(point3D[1].x),  // TOP_RIGHT
      !std::isnan(point3D[2].x),  // BOTTOM_RIGHT
      !std::isnan(point3D[3].x)   // BOTTOM_LEFT
    };

    for (bool val : valid)  if (val) ++validPoints;

    // If less than 3 valid points, return early
    if (validPoints < 3) {
      std::cerr << "Not enough valid points. Exiting function." << std::endl;
      marker_id = -1;
      return;
    }

    validPoints = 0;

    // Exclude opposing corners if one is invalid
    if (!valid[0] || !valid[2]) {  // TOP_LEFT or BOTTOM_RIGHT invalid
      valid[0] = valid[2] = false;
    }
    if (!valid[1] || !valid[3]) {  // TOP_RIGHT or BOTTOM_LEFT invalid
      valid[1] = valid[3] = false;
    }

    // Calculate the sum of valid points
    for (int i = 0; i < 4; ++i) {
      if (valid[i]) {
        sum += point3D[i];
        ++validPoints;
      }
    }

    // Calculate the average point
    
    if (validPoints > 0) {
      position.x = sum.x / validPoints;
      position.y = sum.y / validPoints;
      position.z = sum.z / validPoints;
    } else {
      // If no valid points, set average to NaN
      position = cv::Point3f( std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::quiet_NaN());
    }


    //estimatedPoints = fitSquareToPoints(point3D, SIDE_LENGTH, position);
    //printArrayAsVectors(point3D);
    //printArrayAsVectors(estimatedPoints);

    grid_vector = calculateRotationalVector(point3D);

    /*
    std::cout<<"Rotation Matrix"<<std::endl;
    for (const auto& point : grid_vector) {
      std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }*/
    
    euler = rotationMatrixToEulerAngles(grid_vector);
    quaternions = rotationMatrixToQuaternion(grid_vector);

    std::cout<<"Rotation (Euler)"<<std::endl;
    std::cout << "X: " << radToDeg(euler.z) << std::endl;
    std::cout << "Y: " << radToDeg(euler.x) << std::endl;
    std::cout << "Z: " << radToDeg(euler.y) << std::endl;
  }
  
  void publish_pose_values(){
    zed_aruco::msg::ArucoPose message = zed_aruco::msg::ArucoPose();
    message.id = marker_id;
    message.position.x = position.z;
    message.position.y = position.x;
    message.position.z = position.y;
    message.orientation.euler.x = euler.z;
    message.orientation.euler.y = euler.x;
    message.orientation.euler.z = euler.y;
    message.orientation.quaternion.w = quaternions[0];
    message.orientation.quaternion.i = quaternions[3];
    message.orientation.quaternion.j = quaternions[1];
    message.orientation.quaternion.k = quaternions[2];
    pose_publisher_->publish(message);
  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aruco_Processor>());
  
  rclcpp::shutdown();
  return 0;
}
