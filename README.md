# ros2_lidardrive
![image](https://github.com/Sungmyunghoon/Last_season/assets/112747810/eaaf1fc2-992b-4833-9312-541758f00c40)
## 1. Jetson 보드의 Publisher노드에서 토픽전송
## 2. WSL2의 Subscriber 노드에서 에러계산
## 3. WSL2의 Publisher노드에서 에러토픽을 전송
## 4. Jetson보드의 Subscriber 노드에서 양쪽바퀴의 속도명령을 계산하여 다이내믹셀로전송

![image](https://github.com/Sungmyunghoon/ros2_lidardrive/assets/112747810/db7ba450-3b2a-46cc-8b93-6aac31d42b15)
## 1. 스캔영상을 만들고 영상의 전방180도 영역에서 장애물의 위치를 인식
## 2. 좌측 ½영역에서 최단거리 장애물 검출
## 3. 우측1/2영역에서 최단거리 장애물 검출
## 4. 2개의 최단거리 장애물의 중앙방향을 구함
## 5. 정면방향과 각도차이를 에러(부호있는정수)로 정의함
