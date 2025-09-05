## Gazebo 시뮬레이션 환경 - 동적 장애물 회피  
> (제가 담당한 범위만 포함됐습니다)
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;실제 환경에서 발생할 수 있는 **동적 장애물(보행자, 차량)을** 인식하고 회피하는 로봇 시스템을 구축하고자 했습니다.  
> 또한, **방지턱과 같은 예기치 못한 환경 변화**에 로봇이 안정적으로 대응하는 능력을 테스트하고자 했습니다.
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

## 👨‍💻 구현 기능
- **동적 장애물 회피** : 카메라 센서를 활용하여 **HSV 색공간에서 특정 색상을 마스크 처리**하고, **윤각을 추출해** 장애물을 인식하고 대응합니다.
  - **보행자(파란색)** : 파란색 원형 객체 면적 변화를 추적하여 가까워지면 정지, 멀어지면 전진하도록 속도를 제어합니다.  
    👉 [보행자 대응 코드](https://github.com/juntae02/go_dream-team/blob/main/simulation_env/turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/avoid_dynamic_obstacle.py#L53-L97)
  - **자동차(초록색)** : 초록색 원형 객체의 원형도 및 면적을 비교해 가까워지면 정지, 멀어지면 전진하도록 속도를 제어합니다.  
    👉 [자동차 대응 코드](https://github.com/juntae02/go_dream-team/blob/main/simulation_env/turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/avoid_dynamic_obstacle.py#L99-L144)
- **방지턱(빨간색) 통과** : 빨간색 객체를 검출해 일정 크기 이상일 때 속도를 낮추고 직진하도록 명령합니다.  
  👉 [방지턱 대응 코드](https://github.com/juntae02/go_dream-team/blob/main/simulation_env/turtlebot3_autorace/turtlebot3_autorace_detect/turtlebot3_autorace_detect/avoid_dynamic_obstacle.py#L146-L178)
- **Gazebo 환경 모델링** : 보행자와 자동차가 앞뒤 또는 'ㄱ'자 형태로 움직이도록 설계했습니다.
<br />

## 🤔 트러블슈팅 및 해결 

- **문제 상황 1: 보행자 접근 여부 판단 불가로 충돌 발생**
  
  - **상황** : 보행자 객체가 멀리 있을 때 '내 앞에 없다'고 잘못 판단하여 로봇이 주행을 재개했고, 보행자가 다가오며 충돌하는 문제가 발생했습니다.
  
  - **원인** : 이전 프레임과의 비교 없이 현재 프레임의 거리만으로 보행자의 움직임 방향을 판단했기 때문입니다.
    
  - **해결** : 이전 프레임과 현재 프레임의 면적을 비교하여, 보행자가 로봇에게서 멀어지는지 가까워지는지를 판단하는 로직을 추가했습니다. 이를 통해 보행자가 완전히 멀어진 후에만 주행을 재개하도록 개선했습니다.  
    👉 [프레임 비교 코드]()
---

- **문제 상황 2: 후진 동작의 오작동 발생**
  
  - **상황** : 좁은 골목에서 다가오는 자동차(초록색)를 마주쳤을 때 후진 동작이 적용되지 않았습니다.
  
  - **원인** : Turtlebot4 모델의 특성상 후진을 지원하지 않는 것으로 추측했습니다.
    
  - **해결** : 다가오는 자동차를 발견하면 정지하도록 동작을 변경했습니다.
---

- **문제 상황 3: 역광이나 조명 변화로 인한 색상 인식 오류 발생**
  
  - **상황** : Gazebo 가상 환경에서도 역광이나 조명이 바뀌면 장애물이 흐릿하게 보이되거나 검출되지 않는 문제가 발생했습니다. 
  
  - **원인** : HSV 색공간의 고정된 임계값을 사용했기 때문에 조명 조건이 변하면 마스크가 제대로 생성되지 않았습니다.
    
  - **해결** : 시뮬레이션 모델의 장애물 색상을 실제보다 더 밝게 설정하여 검출이 쉽도록 변경했습니다.
<br />

## 💡 과정 속에서 배운 점
&nbsp;&nbsp;초기 시뮬레이션에서 예상했던 장애물 외에도 **빛 반사나 역광 같은 예상치 못한 변**수를 직접 경험했습니다. 특히, 가상 환경에서도 현실과 유사한 조명 조건이 존재할 수 있다는 점을 배웠고, 이를 반영하기 위해 장애물 색상을 더 밝게 설정하여 인식 성능을 개선했습니다. 이 경험은 시뮬레이션이 단순히 기능 구현을 넘어, **현실의 다양한 불확실성을 예측하고 극복하는 훈련 과정**이라는 것을 깨닫게 해주었습니다.
<br />

## 📚 참고 및 출처
- turtlebot3_simulations 폴더
> $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
- turtlebot3_autorace 폴더
> $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git

본 저장소는 교육 및 학습 목적으로만 사용됩니다.
