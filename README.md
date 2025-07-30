# digital_twin
디지털 트윈 기반 서비스 로봇 운영 시스템 구성 프로젝트
## 1. 🚗 출발~ 드림팀
> 차선 인식 기반 ***자율주행*** 실험  
> - 개발기간 : 2025.06.23-07.04(2주)  
> - 개발인원 : 4명(팀원)  

**[프로젝트 개요]**
> TurtleBot3를 활용하여, 다양한 방식의 ***Line Detection*** 및 ***환경 인지 기능*** 구현
>> - ***준태***: ***밝기*** 기반 Line Detection  
>> - 석환: ***색상*** 기반 Line Detection 및 총괄 지휘  
>> - 예은: ***Contour*** 기반 Line Detection  
>> - 요한: ***Aruco 마커*** 기반 Obstacle Detection 및 Pick & Place  

**[담당 역할]**
> ***밝기***를 기반으로 차선을 자율주행하고, 횡단보도 감지 시 정지하는 동작 구현
>> - ***빛 반사***로 인한 잘못된 차선 인식을 방지하기 위해, 색상이 아닌 ***밝기 정보***로 차선 검출  
>> - ***밝기***만으로 횡단보도와 차선의 구별이 어려운 한계를 극복하기 위해, ***색상 정보***로 횡단보도 검출   
>> - [프로젝트 상세 설명] 괄호 치고 프로젝트 리드미 링크 넣기  
<br />

- 발표 자료  
> 언어는 영어로 설정해야 폰트가 변형되지 않음  
>> - [🚗 출발~ 드림팀](https://www.canva.com/design/DAGt2sLx8RI/VkKIgs1l_i8HXuTRUbE8JQ/edit?utm_content=DAGt2sLx8RI&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton) (시뮬레이션)
>> - [🚗 출발~ 드림팀](https://www.canva.com/design/DAGt2kunhJg/goZzTe1LCq2qbrLk0cxp_A/edit?utm_content=DAGt2kunhJg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton) (실제)


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

@#@@

## 🦾 나의 완벽한 비서
- **장인 보조 협동로봇**, Part 1 - 대장장이 :  
  &nbsp;&nbsp;주조 및 단조 공정의 단순 반복 작업을 협동로봇이 수행하여, 장인들의 고유 기술을 지키는 프로젝트입니다.
- 개발기간 : 2025.05.23-06.05(2주)  
- 개발인원 : 4명(팀장)  
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;장인들의 **고령화**와 젊은 세대의 **높은 진입 장벽**으로 인해, 고유 기술이 사라진다는 문제의식에서 출발했습니다.  
저희는 **협동로봇의 도입**을 통해, 자동화 및 보조 기능으로 **장인분들의 신체적 부담을 줄이는 것을 목표**로 했습니다.   
또한, 자동화 및 사용자 친화적인 TCP 통신으로 기술의 진입 장벽을 낮춰, **고유 기술의 계승을 돕는 것을 목표**로 했습니다.  
<br />

## 📌 주요 기능
- **주조 공정 자동화** : 용탕 주입, 용탕 균일화 등 주조 과정에서의 반복적이고 위험한 작업을 협동로봇이 수행합니다.
- **단조 공정 지원** : 수평/수직 연마, 재련, 비틀기 등 단조 과정의 특정 작업을 협동로봇이 보조합니다.
- **안전 기능 구현** : 충돌 감지 및 용탕 튐 방지와 같은 위험 요소를 사전에 방지하는 기능을 수행합니다.
- **QC 무게 측정** : 생산된 제품의 무게를 측정하여 품질을 관리하는 기능을 수행합니다.
- **TCP 통신 기반 제어** : 휴대폰과의 TCP 통신을 통해 로봇을 제어하고, 공정 상태를 모니터링 합니다.
<br />

## 🎥 시연 영상
- 영상(본인) : [Demo_Me](https://www.youtube.com/watch?v=wulUciU5lNg)  
- 영상(전체) : [Demo_All](https://www.youtube.com/watch?v=4p3I4KdZMHU)  
👉 클릭해서 시연 영상 보기
- 발표 자료 : [Presentation](https://www.canva.com/design/DAGt2pGk8OM/XAmI-RrP8dZGNZCpIRM1vw/edit?utm_content=DAGt2pGk8OM&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ 언어는 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어**: Doosan Robotics M-series M0609
- **개발 언어**: Python 
- **백엔드**: rclpy
- **통신**: TCP CLIENT APP
- **협업 툴**: Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
&nbsp;&nbsp;"충돌" 및 "용탕 튐"과 같은 **위험 요소를 방지**하는 방향으로, ***주조 공정 자동화*** 설계 

- **amovej 이동 시 충돌 방지를 위한 정지 기능** :  
  > &nbsp;&nbsp;비동기 movej 이동 중 **로봇암에 가해지는 외력**을 힘 센서로 실시간 감지하고, 설정된 임계값을 초과할 경우 **즉시 정지**하는 안전 로직을 구현했습니다. 
  > 충돌이 감지되면 로봇은 **XYZ방향으로 위치 유연 및 자세 고정의 순응 제어**를 활성화하여 충격을 흡수하고, 작업자나 주변 환경에 가해지는 물리적 부담을 완화합니다. 
  > 이후 외력이 임계값 이하로 떨어지면 **순응 제어를 해제**하고, 이전 목표 지점으로 **재이동**하여 작업을 재개합니다.

  👉 [충돌 방지 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/stop_motion.py#L35-L68)
  
- **래들 감지를 위한 Compliance Control 및 Force Control 적용** :  
  > &nbsp;&nbsp;용탕을 담는 **래들**의 위치를 정밀하게 감지하기 위해 **task_compliance_ctrl() 및 set_desired_force()를** 적용하여, 로봇이 **Y축 방향**으로 부드럽게 접근하도록 설계했습니다. 
  > **check_force_condition()을** 통해 외력 변화로 래들의 존재를 감지하고, **get_current_posx()[0]으로** 좌표를 획득하여 해당 위치를 기반으로 래들을 수거하는 동작을 수행합니다.

  👉 [래들 감지 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L115-L145)

- **안정적인 용탕 이송을 위한 movesx 기능** :  
  > &nbsp;&nbsp;용탕 이송 중 **넘침**을 방지하기 위해 관절을 움직이는 **movej**는 사용하지 않았고, 직선 경로의 **movel** 대신 곡선 궤적의 **movesx**를 사용하여 보다 **부드럽고 신속하게** 이동하도록 구현했습니다. 
  > 특히 **Z축의 진동**은 넘침 위험이 있으므로, **Z축은 고정**한 채 **X-Y축 중심으로** 이송을 수행하도록 제어했습니다.

  👉 [용탕 이송 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L147-L169)
  
- **용탕 균일화를 위한 move_periodic 기능** :  
  > &nbsp;&nbsp;용탕 속 불순물이 바닥에 가라앉는 **침전 현상**과 **내부 온도 불균형**을 방지하기 위해, **move_periodic()** 함수를 활용해 **주기적 진동 기반의 교반 동작**을 구현했습니다. 
  > **X-Y축**에 진폭과 주기를 각각 적용하여 용탕을 일정 패턴으로 흔들며 **열 균일화**와 **품질 안정화**를 유도했고, 넘침 방지를 위해 **Z축 회전**은 적용하지 않았습니다.

  👉 [용탕 균일화 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L171-L182)

- **안정적인 용탕 주입을 위한 movel 명령 시퀀스 기능** :  
  > &nbsp;&nbsp;용탕 주입 시 튐 현상을 방지하기 위해, **두 단계의 movel() 직선 궤적 시퀀스**를 적용했습니다. 
  > 실제 물을 따르듯, 먼저 천천히 로봇의 회전 각도를 조절해 **용탕의 흐름을 유도**한 후, 깊은 각도로 주입을 완료하여 잔여 용탕까지 **모두 투입**되도록 구현했습니다.  

  👉 [용탕 주입 기능](https://github.com/juntae02/my_perfect_secretary/blob/main/blacksmith_robot/casting.py#L184-L198)
<br />

## 🤔 트러블슈팅 및 해결 과정 
- 문제 상황: 충돌 감지 시 정지 기능 모션 명령어의 stop 모션이 적용되지 않음
> - 시도했던 방법:
> - 해결 과정:
> - 결과
- 문제 상황: 래들 감지 위치가 다를 때, get_current_posx()[0] 로 위치 받아서 상대 좌표로 이동

<br />

## 💡 과정 속에서 배운 점 및 향후 계획
- 배운 점:
> - d:
- 향후 계획:
> - 긴급 정지 기능:
<br />


## 🤝 팀원 정보
- ***준태***: YOLO 모델 생성, 꽃 추천 및 씨앗 심기(본인)   
- 석환: 기능 모듈화, 전체 시나리오 관리 및 총괄 지휘  
- 예은: 기능 모듈화, 자동 관수 및 꽃 포장 
- 요한: Front-end, 음성 메시지 기록 및 QR코드 생성 
<br />

