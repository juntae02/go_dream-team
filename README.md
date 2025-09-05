## 1. 🚗 출발~ 드림팀
- **차선 인식 기반 자율주행 실험** :  
  &nbsp;&nbsp;컴퓨터 비전 기술을 활용하여 차선 및 환경 인식 자율주행 프로젝트입니다.
- 개발기간 : 2025.06.23-07.04(2주) 
- 개발인원 : 4명(팀원) 
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;해당 프로젝트는 **실제 환경에서 발생할 수 있는 다양한 문제**를 극복하여 안전한 **자율주행 시스템**을 개발하고자 했습니다.
> 또한, **장애물**을 감지 및 Pick&Place를 하여 **다양한 환경 변화에 강인한 로봇 제어 시스템**을 구축하는 것을 목표로 했습니다.

> &nbsp;&nbsp;5일간 **Gazebo 환경**에서 **"Line Detection"** 및 **"Obstacle Detection"** 기능 향상을 위한 프로젝트를 진행했습니다.  
👉 [시뮬레이션 환경](https://github.com/juntae02/go_dream-team/tree/main/simulation_env)
<br />

## 📌 주요 기능
- **밝기 기반 차선 감지** : 빛 반사 등 외부 환경 변화에 강인한 밝기 기반 차선 검출 기능
- **색상 기반 차선 감지** : 특정 색상 정보를 활용하여 차선 검출 기능
- **Contour 기반 차선 감지** : 이미지 내 윤곽선(Contour) 분석을 통해 차선을 정밀하게 감지하는 기능
- **Aruco 마커 기반 장애물 감지** : Aruco 마커를 활용하여 장애물을 감지하고 로봇 팔을 이용한 집기 및 놓기 동작 
- **횡단보도 감지 및 정지** : 횡단보도를 정확히 인식하고 로봇이 안전하게 정지하는 기능
<br />

## 🎥 시연 영상
- [🎞️ Demo (개인 시연 영상)](https://www.youtube.com/watch?v=1DT9jmcWfok)  
👉 클릭하시면 시연 영상을 시청하실 수 있습니다.
- [🖥️ 시뮬레이션 환경 발표 자료 (Canva)](https://www.canva.com/design/DAGt2sLx8RI/VkKIgs1l_i8HXuTRUbE8JQ/edit?utm_content=DAGt2sLx8RI&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)
- [🖥️ 모의 도로 환경 발표 자료 (Canva)](https://www.canva.com/design/DAGt2kunhJg/goZzTe1LCq2qbrLk0cxp_A/edit?utm_content=DAGt2kunhJg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ Canva에서는 언어를 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어**: TurtleBot3
- **개발 언어**: Python 
- **백엔드**: rclpy
- **컴퓨터 비전** : OpenCV
- **협업 툴**: GitHub, Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
&nbsp;&nbsp;**밝기**를 기반으로 차선을 검출하여 자율주행하고, 횡단보도 감지 시 **정지**하는 동작 구현을 담당했습니다.

- **밝기 기반 차선 검출** :  
  &nbsp;&nbsp;**빛 반사**로 잘못된 차선 인식을 방지하기 위해 이미지의 **밝기 정보**만으로 차선을 검출하는 기능을 구현했습니다. 이를 통해 다양한 조명 조건에서도 안정적인 차선 인식이 가능하도록 기여했습니다.  
---

- **횡단보도 감지 및 정지** :  
  &nbsp;&nbsp;밝기 정보만으로는 횡단보도와 차선을 구변하기 어려운 한계를 극복하기 위해 **색상 정보**를 보조적으로 활용하여 **횡단보도를 감지**하는 로직을 개발했습니다. 횡단보도 감지 시 TurtleBot3가 안전하게 **정지**하도록 제어하는 기능을 구현했습니다.
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

- **문제 상황 2: 불필요한 직선 검출 발생**
  - **상황** : 허프 변환 후 **짧은 선, 수평·수직선, 어두운 선 등** 불필요한 직선이 검출되었습니다.
    
  - **원인** : 허프 변환은 엣지만 존재하면 모두 직선으로 검출하기 때문에 발생했습니다.
    
  - **해결** : 4가지의 직선 필터링 과정을 추가해 실제 차선만 추출하도록 개선했습니다.
    1. **선 길이 필터링**: 직선의 두 끝점 좌표를 기반으로 짧은 노이즈 선을 제거했습니다.
    2. **기울기 필터링**: 수평이나 수직에 가까운 선은 벽이나 다른 차량일 가능성이 높아 제외했습니다.
    3. **선 두께 필터링**: 선 중심부 픽셀 수를 스캔해 너무 얇은 선은 노이즈로 간주하여 제거했습니다.
    4. **밝기 필터링**: 선 주변의 밝기를 측정해 임계값보다 작다면 노이즈를 최종적으로 제거했습니다.  
  👉 [최종 차선 검출 이미지](https://github.com/juntae02/go_dream-team/blob/main/resource/after_img.jpg)  
  👉 [직선 필터링 코드](https://github.com/juntae02/go_dream-team/blob/main/realistic_env/lane_detect/brightness_lane_detect.py#L144-L206)
---

- **문제 상황 3: 횡단보도와 차선의 구별을 못하는 경우 발생**
  - **상황** : 횡단보도 또한 밝은 색상으로 이루어져 일반 차선과 구별하기 어려운 문제가 발생했습니다. 
    
  - **원인** : 단일 정보인 밝기만으로는 횡단보도와 차선의 시각적 특성을 완벽하게 분리하기 어려웠습니다. 두 객체 모두 밝은 색상을 가지고 있기 때문에 밝기만으로는 의미 있는 구별이 불가능했습니다.
    
  - **해결** : 밝기 정보의 한계를 보완하기 위해 색상 정보를 보조적인 판단 기준으로 활용하는 접근 방식을 선택했습니다.   
  👉 [횡단보도와 차선 구별 코드](https://github.com/juntae02/go_dream-team/blob/main/realistic_env/lane_detect/brightness_lane_detect.py#L245-L298)
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
