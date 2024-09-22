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
#define MTM_DIST          220

#define MARKER_ID         0
#define IDS               {1,2}

struct Position{
  double x;
  double y;
  double z;
};

struct Euler{
  double x;
  double y;
  double z;
};


struct Quaternion {
  double w, i, j, k;

  // Normalize the quaternion
  void normalize() {
    double magnitude = std::sqrt(w * w + i * i + j * j + k * k);
    w /= magnitude;
    i /= magnitude;
    j /= magnitude;
    k /= magnitude;
  }

  // SLERP function between two quaternions
  static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
    // Compute the cosine of the angle between the two quaternions
    double dot_product = q1.w * q2.w + q1.i * q2.i + q1.j * q2.j + q1.k * q2.k;

    // If the dot product is negative, the quaternions have opposite handedness
    // and slerp won't take the shorter path. Fix this by reversing one quaternion.
    Quaternion q2_mod = q2;
    if (dot_product < 0.0) {
      q2_mod.w = -q2_mod.w;
      q2_mod.i = -q2_mod.i;
      q2_mod.j = -q2_mod.j;
      q2_mod.k = -q2_mod.k;
      dot_product = -dot_product;
    }

    // If the quaternions are very close, use linear interpolation
    const double THRESHOLD = 0.9995;
    if (dot_product > THRESHOLD) {
      Quaternion result = {
        q1.w + t * (q2_mod.w - q1.w),
        q1.i + t * (q2_mod.i - q1.i),
        q1.j + t * (q2_mod.j - q1.j),
        q1.k + t * (q2_mod.k - q1.k)
      };
      result.normalize();
      return result;
    }

    // Calculate the angle between the quaternions
    double theta_0 = std::acos(dot_product);  // theta_0 = angle between input vectors
    double theta = theta_0 * t;               // theta = angle between q1 and result

    double sin_theta = std::sin(theta);       // Compute this only once
    double sin_theta_0 = std::sin(theta_0);   // Compute this only once

    double s1 = std::cos(theta) - dot_product * sin_theta / sin_theta_0;
    double s2 = sin_theta / sin_theta_0;

    return {
      (s1 * q1.w + s2 * q2_mod.w),
      (s1 * q1.i + s2 * q2_mod.i),
      (s1 * q1.j + s2 * q2_mod.j),
      (s1 * q1.k + s2 * q2_mod.k)
    };
  }
};

struct Aruco_Code{
  int id;
  Position position;
  Euler euler;
  Quaternion quaternion;
};

class Aruco_Processor : public rclcpp::Node
{
public:
  Aruco_Processor()
  : Node("Aruco_Processor_Ears")
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
  int docking_ids[2] = IDS;


  void read_aruco_position(const zed_aruco::msg::ArucoPoseArray::SharedPtr msg)
  {
    marker_id = MARKER_ID;
    if (msg->data.size()<1) {
      std::cout<<"err"<<std::endl;
      marker_id = -1;
      return;
    }

    Aruco_Code marker_1;
    Aruco_Code marker_2;
    marker_1.id = -1;
    marker_2.id = -1;

    for (auto marker : msg->data){
      if (marker.id == 1){
        marker_1.id = 1;
        marker_1.position.x = marker.position.x;
        marker_1.position.y = marker.position.y;
        marker_1.position.z = marker.position.z;
        marker_1.euler.x = marker.orientation.euler.x;
        marker_1.euler.y = marker.orientation.euler.y;
        marker_1.euler.z = marker.orientation.euler.z;
        marker_1.quaternion.w = marker.orientation.quaternion.w;
        marker_1.quaternion.i = marker.orientation.quaternion.i;
        marker_1.quaternion.j = marker.orientation.quaternion.j;
        marker_1.quaternion.k = marker.orientation.quaternion.k;
      }
      else if (marker.id == 2){
        marker_2.id = 2;
        marker_2.position.x = marker.position.x;
        marker_2.position.y = marker.position.y;
        marker_2.position.z = marker.position.z;
        marker_2.euler.x = marker.orientation.euler.x;
        marker_2.euler.y = marker.orientation.euler.y;
        marker_2.euler.z = marker.orientation.euler.z;
        marker_2.quaternion.w = marker.orientation.quaternion.w;
        marker_2.quaternion.i = marker.orientation.quaternion.i;
        marker_2.quaternion.j = marker.orientation.quaternion.j;
        marker_2.quaternion.k = marker.orientation.quaternion.k;
      }
    }

    if (marker_1.id < 0 || marker_2.id < 0){
      std::cout<<"err"<<std::endl;
      marker_id = -1;
      return;
    }

    marker_id = 0;

    position.x = (marker_1.position.x + marker_2.position.x) * 0.5;
    position.y = (marker_1.position.y + marker_2.position.y) * 0.5;
    position.z = (marker_1.position.z + marker_2.position.z) * 0.5;

    /*
    std::cout<<"Rotation Matrix"<<std::endl;
    for (const auto& point : grid_vector) {
      std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }*/
    
    Quaternion avg = Quaternion::slerp(marker_1.quaternion, marker_2.quaternion, 0.5);
    quaternions[0] = avg.w;
    quaternions[1] = avg.i;
    quaternions[2] = avg.j;
    quaternions[3] = avg.k;

    euler.x = (marker_1.euler.x + marker_2.euler.x) * 0.5;
    euler.y = (marker_1.euler.y + marker_2.euler.y) * 0.5;
    euler.z = (marker_1.euler.z + marker_2.euler.z) * 0.5;

    std::cout<<"Rotation (Euler)"<<std::endl;
    std::cout << "X: " << radToDeg(euler.z) << std::endl;
    std::cout << "Y: " << radToDeg(euler.x) << std::endl;
    std::cout << "Z: " << radToDeg(euler.y) << std::endl;
  }
  
  void publish_pose_values(){
    zed_aruco::msg::ArucoPose message = zed_aruco::msg::ArucoPose();
    message.id = marker_id;
    message.position.x = position.x;
    message.position.y = position.y;
    message.position.z = position.z;
    message.orientation.euler.x = euler.x;
    message.orientation.euler.y = euler.y;
    message.orientation.euler.z = euler.z;
    message.orientation.quaternion.w = quaternions[0];
    message.orientation.quaternion.i = quaternions[1];
    message.orientation.quaternion.j = quaternions[2];
    message.orientation.quaternion.k = quaternions[3];
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
