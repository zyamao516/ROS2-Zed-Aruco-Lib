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
#include <cmath>

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "zed_aruco/msg/aruco_pose_array.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;


#define TIMER_DURATION 50ms
#define QUEUE_LENGTH 10
#define MOTOR_PUBLISHER_TOPIC "motor_pwr"
#define ROVER_PUBLISHER_TOPIC "/r4/cmd_vel"
#define SUBSCRIBER_TOPIC "pose_data"
#define MOTOR_MAX_VAL 400
#define MOTOR_MIN_VAL 80
#define MOTOR_ERROR   0.7
#define ROVER_MAX_VAL 0.09
#define TARGET_DEPTH  160
#define LINEAR_CUTOFF_DISTANCE 110
#define CUTOFF true
#define MOTOR_GAIN 15
#define ROVER_GAIN 0.02
#define ROVER_ERR 40

#define Y_OFFSET 30
#define ANGULAR_COMPENSATION_DIST 300
#define CODE_TO_CENTER_DIST 45
#define M 4.3/(3000-LINEAR_CUTOFF_DISTANCE)
#define B (1.5*M)/4.3
#define ANGULAR_COMP true

#define MOTOR_MAX_POS 950
#define MOTOR_MIN_POS 30

#define PORT_NAME "/dev/ttyACM0"

// Function to open the serial port
int openSerialPort(const char* portname)
{
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::cerr << "Error opening " << portname << ": " << strerror(errno) << std::endl;
    return -1;
  }
  return fd;
}

// Function to close the serial port
void closeSerialPort(int fd) { close(fd); }

// Function to configure the serial port
bool configureSerialPort(int fd, int speed)
{
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
    return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0; // no signaling chars, no echo, no
                  // canonical processing
  tty.c_oflag = 0; // no remapping, no delays
  tty.c_cc[VMIN] = 0; // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                   // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

// Function to read data from the serial port
inline int readFromSerialPort(int fd, char* buffer, size_t size)
{
  return read(fd, buffer, size);
}

// Function to write data to the serial port
inline int writeToSerialPort(int fd, const char* buffer, size_t size)
{
  return write(fd, buffer, size);
}


class Motor_Driver : public rclcpp::Node
{
public:
  Motor_Driver()
  : Node("motor_driver")
  {
    port = openSerialPort(PORT_NAME);
    if (port < 0) return;

    if (!configureSerialPort(port, B9600)) {
      closeSerialPort(port);
      return;
    }


    motor_publisher_ = this->create_publisher<std_msgs::msg::Int16>(MOTOR_PUBLISHER_TOPIC, QUEUE_LENGTH);
    rover_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(ROVER_PUBLISHER_TOPIC, QUEUE_LENGTH);
    motor_timer_ = this->create_wall_timer(TIMER_DURATION, std::bind(&Motor_Driver::publish_motor_values, this));
    rover_timer_ = this->create_wall_timer(TIMER_DURATION, std::bind(&Motor_Driver::publish_cmd_rover, this));
    subscription_ = this->create_subscription<zed_aruco::msg::ArucoPoseArray>(SUBSCRIBER_TOPIC, QUEUE_LENGTH, std::bind(&Motor_Driver::read_aruco_position, this, std::placeholders::_1));
  }


private:

  float ey;
  float pos_y;
  float uy;

  float ex;
  float pos_x;
  float ux_rover;

  float x_angular;
  float z_angular;
  float delta_y;

  bool pub;

  int port;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr motor_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rover_publisher_;
  rclcpp::TimerBase::SharedPtr motor_timer_;
  rclcpp::TimerBase::SharedPtr rover_timer_;
  rclcpp::Subscription<zed_aruco::msg::ArucoPoseArray>::SharedPtr subscription_;

  zed_aruco::msg::ArucoPose mkr;

  void read_aruco_position(const zed_aruco::msg::ArucoPoseArray::SharedPtr msg)
  {
    /*
    if (!mkr.data) {
      pos_y = 0;
      return;
    }*/
    //RCLCPP_INFO(this->get_logger(), "Aruco ID: %d", mkr.id);
    //RCLCPP_INFO(this->get_logger(), "Aruco Position: x = %g", mkr.position.x);
    //RCLCPP_INFO(this->get_logger(), "Aruco Position: y = %g", mkr.position.y);
    
    // temp condition
    mkr.id = -1;

    for (const auto marker : msg->data){
      if (marker.id == 0) mkr = marker; break;
    }

    char buffer[100];
    int n = readFromSerialPort(port, buffer, sizeof(buffer));
    if (n < 0) {
        std::cerr << "Error reading from serial port: "
                  << strerror(errno) << std::endl;
    } else {
      // Null-terminate the buffer (important to avoid undefined behavior)
      buffer[n] = '\0';

      try {
        // Attempt to convert the buffer to an integer
        int value = std::stoi(buffer);
        //RCLCPP_INFO(this->get_logger(), "Buffer: %d", value);
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid argument in buffer: %s", e.what());
      } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "Out of range error in buffer: %s", e.what());
      }
    }

    if (mkr.id == -1){
      pub = false;
      return;
    }

    pos_x = mkr.position.x;

    //double sinr_cosp = 2.0 * (mkr.orientation.quaternion.w * mkr.orientation.quaternion.i + mkr.orientation.quaternion.j * mkr.orientation.quaternion.k);
    //double cosr_cosp = 1.0 - 2.0 * (mkr.orientation.quaternion.i * mkr.orientation.quaternion.i + mkr.orientation.quaternion.j * mkr.orientation.quaternion.j);
    //x_angular = std::atan2(sinr_cosp, cosr_cosp);
    //std::cout<<"X angular: "<<x_angular<<std::endl; 

    // Using the formula: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    double siny_cosp = 2.0 * (mkr.orientation.quaternion.w * mkr.orientation.quaternion.k + mkr.orientation.quaternion.i * mkr.orientation.quaternion.j);
    double cosy_cosp = 1.0 - 2.0 * (mkr.orientation.quaternion.j * mkr.orientation.quaternion.j + mkr.orientation.quaternion.k * mkr.orientation.quaternion.k);
    z_angular = std::atan2(siny_cosp, cosy_cosp) * -1;

    //z_angular = mkr.orientation.euler.z;
    if (std::abs(mkr.position.y)<600 && mkr.id == 0){
      pos_y = mkr.position.y;
    }
    else{
      pos_y = 0;
    }
    pub = true;
    //RCLCPP_INFO(this->get_logger(), "Aruco Position: w = %g", mkr.orientation.w);
    //RCLCPP_INFO(this->get_logger(), "Aruco Position: x = %g; y = %g; z = %g", mkr.position.x, mkr.position.y, mkr.position.z);
    //RCLCPP_INFO(this->get_logger(), "Aruco Orientation: x = %g; y = %g; z = %g", mkr.orientation.x, mkr.orientation.y, mkr.orientation.z);
  }
  void publish_motor_values(){
    if (pos_x < LINEAR_CUTOFF_DISTANCE && CUTOFF) {
      uy = 0;
      return;
    }
    std_msgs::msg::Int16 message = std_msgs::msg::Int16();
    float multiplier = 1.0;

    //std::cout<<"Rotational Compensation: " << (100*sin(x_angular)) << std::endl;
    std::cout<<"Y offset: "<<(Y_OFFSET + (-1*sin(z_angular)*CODE_TO_CENTER_DIST))<<std::endl;
    // Enable angular compensation at set distance
    ey = (pos_x < ANGULAR_COMPENSATION_DIST && ANGULAR_COMP) ? (Y_OFFSET + (-1*sin(z_angular)*CODE_TO_CENTER_DIST))-pos_y : Y_OFFSET - pos_y;
    
    std::cout<<"X: "<<pos_x<<std::endl;
    std::cout<<"Y: "<<pos_y<<std::endl;
    std::cout<<"Err Y: "<<ey<<std::endl;
    uy = MOTOR_GAIN*ey*multiplier;
    float min_err= M*pos_x + B;
    //std::cout<<"Min Err: " << min_err << std::endl;
    if (uy < MOTOR_MIN_VAL && ey > min_err) uy = MOTOR_MIN_VAL;
    else if (uy > -MOTOR_MIN_VAL && ey < -min_err) uy = -MOTOR_MIN_VAL;
    else if (uy > MOTOR_MAX_VAL) uy = MOTOR_MAX_VAL;
    else if (uy < -MOTOR_MAX_VAL) uy = -MOTOR_MAX_VAL;
    if(!pub) {
      uy = 0;
    }
    message.data = static_cast<int16_t>(uy);
    motor_publisher_->publish(message);
  }
  void publish_cmd_rover(){
    
    geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
    ex = TARGET_DEPTH-pos_x;
    if (ex > 0 && ex < ROVER_ERR) ex = 0;
    ux_rover = -ROVER_GAIN * ex;
    if(ux_rover > ROVER_MAX_VAL) ux_rover = ROVER_MAX_VAL;
    else if(ux_rover < -ROVER_MAX_VAL) ux_rover = -ROVER_MAX_VAL;
    if(!pub) {
      ux_rover = 0;
    }
    message.linear.x = ux_rover;
    message.angular.z = 0;
    rover_publisher_->publish(message); 
  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motor_Driver>());
  
  rclcpp::shutdown();
  return 0;
}
