## 1. 🚗 출발~ 드림팀
- **차선 인식 기반 자율주행 실험** :  
  &nbsp;&nbsp;컴퓨터 비전 기술을 활용하여, 차선 인식 및 환경 인지 자율주행 기능을 구현 및 실험한 프로젝트입니다.
- 개발기간 : 2025.06.23-07.04(2주) 
- 개발인원 : 4명(팀원) 
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;해당 프로젝트는 **실제 환경에서 발생할 수 있는 다양한 문제**(예: 빛 반사로 인한 잘못된 차선 인식, 차선과 횡단보도 구별의 어려움)를 극복하고, 안정적이며 정확한 **차선 인식 기반의 자율주행 시스템**을 개발하고자 했습니다. 또한, 자율주행 중 **장애물**을 감지 및 Pick & Place를 통해, **다양한 환경 변화에 강인한 로봇 제어 시스템**을 구축하는 것을 목표로 했습니다.

> &nbsp;&nbsp;해당 프로젝트 이전에, **"Line Detection"** 및 **"Obstacle Detection"** 기능 향상을 위해 Gazebo 환경에서 프로젝트를 진행했습니다.  
👉 [시뮬레이션 환경]([https://github.com/juntae02/go_dream-](https://github.com/juntae02/go_dream-team/tree/main/simulation_env)
<br />

## 📌 주요 기능
- **밝기 기반 차선 감지** : 빛 반사 등 외부 환경 변화에 강인한 차선 인식을 위한, 밝기 기반 차선 검출 기능
- **색상 기반 차선 감지** : 특정 색상 정보를 활용하여 차선 검출 기능
- **Contour 기반 차선 감지** : 이미지 내 윤곽선(Contour) 분석을 통해 차선을 정밀하게 감지하는 기능
- **Aruco 마커 기반 장애물 감지** : Aruco 마커를 활용하여 장애물을 감지하고, 로봇 팔을 이용한 집기 및 놓기 동작 
- **횡단보도 감지 및 정지** : 횡단보도를 정확히 인식하고, 로봇이 안전하게 정지하는 기능
<br />

## 🎥 시연 영상
- [🎞️ Demo (개인 시연 영상)](https://www.youtube.com/watch?v=1DT9jmcWfok)  
👉 클릭해서 시연 영상 보기
- [🖥️ 시뮬레이션 환경 발표 자료 (Canva)](https://www.canva.com/design/DAGt2sLx8RI/VkKIgs1l_i8HXuTRUbE8JQ/edit?utm_content=DAGt2sLx8RI&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)
- [🖥️ 현실 환경 발표 자료 (Canva)](https://www.canva.com/design/DAGt2kunhJg/goZzTe1LCq2qbrLk0cxp_A/edit?utm_content=DAGt2kunhJg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)
⚠️ Canva에서는 언어를 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어**: Doosan Robotics M-series M0609
- **개발 언어**: Python 
- **백엔드**: rclpy
- **통신**: TCP CLIENT APP
- **협업 툴**: Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
&nbsp;&nbsp;주조 공정 
- **충돌 방지 로직** : amovej 이동 중 외력을 실시간 감지하여, 순응 제어로 충돌 피해를 최소화했습니다.
- **래들 감지** : 순응 제어 및 힘 제어를 통해 래들을 감지하고, 자동 좌표 보정으로 수거 위치를 정밀 제어했습니다.
<br />

**[담당 역할]**
> ***밝기***를 기반으로 차선을 자율주행하고, 횡단보도 감지 시 정지하는 동작 구현
>> - ***빛 반사***로 인한 잘못된 차선 인식을 방지하기 위해, 색상이 아닌 ***밝기 정보***로 차선 검출  
>> - ***밝기***만으로 횡단보도와 차선의 구별이 어려운 한계를 극복하기 위해, ***색상 정보***로 횡단보도 검출   
>> - [프로젝트 상세 설명] 괄호 치고 프로젝트 리드미 링크 넣기  
---

## 🤔 트러블슈팅 및 해결 

- **문제 상황 1: amovej 이동 중, stop() 명령어가 작동하지 않음**
  - **상황** :  
    &nbsp;&nbsp;Doosan API의 stop() 
  - **원인** :  
    &nbsp;&nbsp;외력 감지 기반의 실시간 정지 기능이 필요하다고 판단했습니다.
  - **해결** :  
    &nbsp;&nbsp;외력이 임계값을 초과
  👉 [밝기 기반 코드](https://github.com/juntae02/go_dream-team/blob/main/realistic_env/lane_detect/brightness_lane_detect.py)
---

- **문제 상황 1: amovej 이동 중, stop() 명령어가 작동하지 않음**
  - **상황** :  
    &nbsp;&nbsp;Doosan API의 stop() 
  - **원인** :  
    &nbsp;&nbsp;외력 감지 기반의 실시간 정지 기능이 필요하다고 판단했습니다.
  - **해결** :  
    &nbsp;&nbsp;외력이 임계값을 초과
  👉 [충돌 방지 코드]()
<br />

## 💡 과정 속에서 배운 점
&nbsp;&nbsp;이번 프로젝트를 통해 단순한 경로 제어가 아닌, 
<br />

## 🤝 팀원 정보
- ***준태***: 밝기 기반 Line Detection(본인)   
- 석환: 색상 기반 Line Detection 및 총괄 지휘
- 예은: Contour 기반 Line Detection 
- 요한: Aruco 마커 기반 Obstacle Detection 및 Pick & Place 
<br />

## 📚 참고 및 출처

- `simulation_env/turtlebot3_autorace/` 디렉토리는 아래 공개 저장소를 기반으로 하여,  
  **일부 기능을 수정하고 프로젝트 목적에 맞게 재구성**하였습니다.  
  > 🔗 https://github.com/ROBOTIS-GIT/turtlebot3_autorace

- `simulation_env/turtlebot3_simulations/` 디렉토리 역시 다음 저장소를 바탕으로 하여,  
  **시뮬레이션 환경 설정 등에 필요한 부분을 수정하여 사용**하였습니다.  
  > 🔗 https://github.com/ROBOTIS-GIT/turtlebot3_simulations

- 위 두 저장소 모두 [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0.html)를 따르며, 본 프로젝트 역시 해당 라이선스를 준수하여 사용하고 있습니다.

- `realistic_env/` 폴더 내 코드는 **직접 설계 및 구현한 실제 환경용 코드**로,  
  시뮬레이션 환경 코드를 바탕으로 참고한 부분이 있을 수 있으나, 본 프로젝트 목적과 환경에 맞게 독립적으로 작성되었습니다.

> 본 저장소는 교육 및 학습 목적으로만 사용됩니다.
