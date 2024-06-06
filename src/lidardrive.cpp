#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <cmath>
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)

const float L = 70.f;

static void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr error_publisher) {
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  Mat canvas(Size(500,500), CV_8UC3, Scalar(0,0,0));
  
  static Point center(250, 250);

  drawMarker(canvas, Point(250, 250), Scalar(0, 255, 0), MARKER_STAR, 20, 1, LINE_4);

  float min_angle_left = 0;
  float min_angle_right = 0;

  float min_dist_left = numeric_limits<float>::max();
  float min_dist_right = numeric_limits<float>::max();

  for (int i = 0; i < count; i++) {
    float angle = scan->angle_min + scan->angle_increment * i;
    float degree = RAD2DEG(angle);
    float distance = scan->ranges[i];

    int x = 250 + distance * 100 * cos(angle);
    int y = 250 + distance * 100 * sin(angle);
    
    drawMarker(canvas, Point(cvRound(x),cvRound(y)),Scalar(255,255,255),MARKER_SQUARE,2);
    
    if(degree >= -180 && degree < -90 && distance < 2){
      if(scan->ranges[i] < min_dist_left){
        min_dist_left = scan->ranges[i];
        min_angle_left = degree;
      }
    }
    else if(degree >= 90 && degree < 180 && distance < 2){
      if(scan->ranges[i] < min_dist_right){
        min_dist_right = scan->ranges[i];
        min_angle_right = degree;
      } 
    }
  }

  float cen_angle = (min_angle_left + min_angle_right) / 2;
  float error = cen_angle;

  printf("Central angle: %f, Error: %f\n", cen_angle, error);
  
  geometry_msgs::msg::Vector3 vel_data;
  
  int vel1 = 100 + error;
  int vel2 = -1 * (100 - error );
  
  vel_data.x = vel1;
  vel_data.y = vel2;

  imshow("win", canvas);
  waitKey(1);

  error_publisher->publish(vel_data);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lidardrive");
  auto error_publisher = node->create_publisher<geometry_msgs::msg::Vector3>("error_topic", 10);
  
  std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)> fn;
  fn = bind(scanCb, std::placeholders::_1, error_publisher);
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), fn);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}