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
