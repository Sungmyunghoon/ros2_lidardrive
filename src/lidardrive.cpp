 #include "rclcpp/rclcpp.hpp"// ROS2의 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/laser_scan.hpp"// 라이다 센서 메시지를 포함
#include <math.h> 
#include <cmath> // 수학 함수들을 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/vector3.hpp" // 기하학적 벡터 메시지를 포함
#include "opencv2/opencv.hpp" // OpenCV 라이브러리를 포함
using namespace std;
using namespace cv;

// 라디안을 각도로 변환하는 매크로 정의
#define RAD2DEG(x) ((x)*180./M_PI)

const float L = 70.f; // 일정한 거리 L을 상수로 정의

// 라이다 데이터를 처리하는 콜백 함수
static void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr error_publisher) {
  int count = scan->scan_time / scan->time_increment; // 라이다 스캔 데이터의 수를 계산
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  Mat canvas(Size(500,500), CV_8UC3, Scalar(0,0,0));// 라이다 데이터를 시각화할 canvas를 초기화
  
  static Point center(250, 250); // 중심점을 정의

  drawMarker(canvas, Point(250, 250), Scalar(0, 255, 0), MARKER_STAR, 20, 1, LINE_4); // 중심점에 마커를 그림

  float min_angle_left = 0; // 왼쪽에서 가장 작은 각도를 저장할 변수
  float min_angle_right = 0; // 오른쪽에서 가장 작은 각도를 저장할 변수

  float min_dist_left = numeric_limits<float>::max(); // 왼쪽에서 가장 작은 거리를 초기화
  float min_dist_right = numeric_limits<float>::max(); // 오른쪽에서 가장 작은 거리를 초기화

  // 라이다 데이터를 반복하면서 처리
  for (int i = 0; i < count; i++) {
    float angle = scan->angle_min + scan->angle_increment * i; // 각도를 계산
    float degree = RAD2DEG(angle); // 각도를 도(degree)로 변환
    float distance = scan->ranges[i]; // 거리를 편하게 변수로 지정

    // 캔버스 상에 그리기 위한 좌표를 계산
    int x = 250 + distance * 100 * cos(angle);
    int y = 250 + distance * 100 * sin(angle);

    // 계산된 좌표에 마커를 그림
    drawMarker(canvas, Point(cvRound(x),cvRound(y)),Scalar(255,255,255),MARKER_SQUARE,2);

    // 각도와 거리 조건에 따라 최소 거리를 갱신
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

  float cen_angle = (min_angle_left + min_angle_right) / 2; // 왼쪽과 오른쪽 최소 각도의 중간 각도를 계산
  float error = cen_angle; // 중앙 각도를 오류 값으로 사용

  printf("Central angle: %f, Error: %f\n", cen_angle, error); // 화면에 에러값 출력
  
  geometry_msgs::msg::Vector3 vel_data; // 속도 데이터를 저장할 메시지 객체를 생성

  // 오류 값에 따라 속도를 계산
  int vel1 = 100 + error;
  int vel2 = -1 * (100 - error );
  
  vel_data.x = vel1;
  vel_data.y = vel2;

  imshow("win", canvas); // 캔버스를 화면에 표시
  waitKey(1);

  error_publisher->publish(vel_data); // 계산된 속도 데이터를 퍼블리
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // ROS2 노드를 초기화
  auto node = rclcpp::Node::make_shared("lidardrive"); // "lidardrive"라는 이름의 노드를 생성
  auto error_publisher = node->create_publisher<geometry_msgs::msg::Vector3>("error_topic", 10); // "error_topic" 토픽에 퍼블리셔를 생성
  
  std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)> fn; // 콜백 함수를 위한 std::function 객체를 선언
  fn = bind(scanCb, std::placeholders::_1, error_publisher); // 콜백 함수를 바인딩
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), fn); // "scan" 토픽에 서브스크립션을 생성
  
  rclcpp::spin(node);
  rclcpp::shutdown(); // ROS2를 종료
  return 0;
}
