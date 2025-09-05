## Gazebo 시뮬레이션 환경 - 동적 장애물 회피
> 이 프로젝트는 Gazebo 환경에서 "Line Detection" 및 "Obstacle Detection" 기능 향상을 위한 프로젝트로 진행됐습니다.
> 현재 리드미 나를 위함
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;실제 환경에서 발생할 수 있는 **동적 장애물(보행자, 차량)을** 인식하고 회피하는 로봇 시스템을 구축합니다.
> 또한, **예기치 않은 환경 변화(방직턱)에** 로봇이 안정적으로 대응하는 능력을 테스트하고자 합니다.
<br />

## 📌 주요 기능
- **동적 장애물 회피** : 카메라 센서를 활용하여 색상 기반으로 보행자와 자동차를 인식하고 대응합니다.
  - **보행자(파란색)** : 접근 시 로봇을 정지시켜 충돌을 방지합니다.
  - **자동차(초록색)** : 접근 시 로봇을 정지시켜 경로를 양보합니다.
- **방지턱 통과** : 방지턱(빨간색)을 인식하면 속도를 줄여 부드럽게 통과합니다.
- **Gazebo 환경 모델링** : 보행자와 자동차가 앞뒤 또는 'ㄱ'자 형태로 움직이도록 설계했습니다.
<br />

## 🎥 시연 영상
- [🎞️ Demo (개인 시연 영상)](https://www.youtube.com/shorts/dJ0Hz2PkLn4)  
👉 클릭하시면 시연 영상을 시청하실 수 있습니다.
- [🖥️ 시뮬레이션 환경 발표 자료 (Canva)](https://www.canva.com/design/DAGt2sLx8RI/VkKIgs1l_i8HXuTRUbE8JQ/edit?utm_content=DAGt2sLx8RI&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ Canva에서는 언어를 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **로봇 시뮬레이터**: Gazebo
- **개발 언어**: Python 
- **백엔드**: rclpy
- **컴퓨터 비전** : OpenCV
<br />

## 🤔 트러블슈팅 및 해결 

- **문제 상황 1: 빛 반사로 인한 차선 검출 오류 발생**
  
  - **상황** : **햇빛, 그림자, 반사 등**으로 인해 차선 검출이 불안정했습니다.  
  👉 [초기 차선 검출 이미지](https://github.com/juntae02/go_dream-team/blob/main/resource/before_img.png)
  
  - **원인** : 조명 조건으로 도로와 차선 색상이 변해 발생한 **노이즈가 그대로 허프 변환으로 유입**되어 발생했습니다.
    
  - **해결** : **Grayscale 변환**을 적용하여 색상 의존도를 제거했고, **Gaussian Blur**와 **모폴로지 Opening 연산**을 사용해 조명 조건에 의한 노이즈를 제거했습니다. 
    그 결과, 노이즈를 제거하고 끊어진 선들을 연결해 안정적인 엣지를 추출했습니다.  
  👉 [이미지 전처리 코드](https://github.com/juntae02/go_dream-team/blob/main/realistic_env/lane_detect/brightness_lane_detect.py#L108-L125)  
---


## 💡 과정 속에서 배운 점
&nbsp;&nbsp;이번 프로젝트를 통해 **로봇 시스템이 실제 환경의 불확실성에 대응하는 능력**을 길렀습니다. 시뮬레이션에서 예상했던 장애물 외에, 빛 반사나 그림자 같은 예상치 못한 변수를 직접 경험하며 **현실과 시뮬레이션 간의 간극**을 이해했습니다. 이를 해결하기 위해 이미지 전처리와 직선 필터링 방식을 적용하며, 로봇 시스템의 안정성과 신뢰성을 확보하는 **실무 역량**을 키울 수 있었습니다.
<br />

## 🤝 팀원 정보
- ***준태***: 밝기 기반 Line Detection(본인)   
- 석환: 색상 기반 Line Detection 및 총괄 지휘
- 예은: Contour 기반 Line Detection 
- 요한: Aruco 마커 기반 Obstacle Detection 및 Pick & Place 
<br />

## 📚 참고 및 출처
- turtlebot3_simulations 폴더
> $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

- turtlebot3_autorace 폴더
> $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git

> 본 저장소는 교육 및 학습 목적으로만 사용됩니다.
