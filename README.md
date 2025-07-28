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




## 🌸 Bloom for you
- ***키움의 과정을 선물하는*** 꽃 선물 서비스 :  
&nbsp;&nbsp;선물의 목적과 기간에 적합한 꽃을 추천하고, 재배 과정에서 음성을 기록하여 진심을 전달합니다.  
또한, 로봇암 기반의 자동화 시스템으로 재배 실패 부담을 해소합니다.
- 개발기간 : 2025.06.09-06.20(2주)  
- 개발인원 : 4명(팀원)  
<br />

## 🔍 문제 정의
> &nbsp;&nbsp;이 프로젝트는 ***"선물의 종류는 많지만 감정은 없다"는*** 문제의식에서 출발했습니다.  
> 기존의 선물은 순간적인 감동에 그치는 경우가 많습니다. 우리는 받는 이가 꽃을 직접 기르지 않더라도, 꽃의 성장 과정을 ***공유***하며 지속적인 ***기쁨과 진심***이 전해지는, ***새로운 형태의 선물***을 만들고자 했습니다. 또한, 식물 재배의 어려움과 실패에 대한 부담을 ***로봇 기술***로 해결함으로써, ***누구나 꽃을 쉽게 기를 수 있도록 돕는 것***을 목표로 했습니다.
<br />

## 📌 주요 기능
- ***상황별 꽃 추천*** : 대화를 통해, 선물의 목적과 기간에 최적화된 꽃을 추천합니다.
- ***자동화된 씨앗 심기*** : 로봇암을 활용하여, 씨앗 심기 과정을 자동화합니다.
- ***음성 메시지 기록*** : 재배 과정에서 음성을 기록하고, 포장 시 QR코드로 전달합니다.
- ***자동 관수 시스템*** : 꽃의 관수 주기에 맞춰, 물을 자동으로 공급합니다. 
- ***YOLO 기반 객체 탐지*** : 프로젝트에 최적화된 객체 탐지 모델을 통해, 특정 객체를 인식하고 처리합니다.
<br />

## 🎥 시연 영상
- 영상(본인) : [Demo_Me](https://www.youtube.com/watch?v=E3JtXJ45bFk)  
- 영상(전체) : [Demo_All](https://www.youtube.com/watch?v=jA9iK2Lapts)  
👉 클릭해서 시연 영상 보기
- 발표 자료 : [Presentation](https://www.canva.com/design/DAGt2hxdEvg/8CEbznRIRrQc8xgAosH4Jg/edit?utm_content=DAGt2hxdEvg&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)  
⚠️ 언어는 영어로 설정해야 폰트가 변형되지 않습니다.  
<br />

## 🛠️ 기술 스택
- **하드웨어 제어**: Doosan Robot Arm
- **개발 언어**: Python 
- **프론트엔드**: Kivy
- **백엔드**: Flask, rclpy
- **AI / 데이터 처리**: YOLO, OpenAI API
- **협업 툴**: GitHub, Notion, Draw.io
<br />

## 👨‍💻 담당한 기능
- ***YOLO 모델 생성 및 학습*** :  
  &nbsp;&nbsp;프로젝트에 **최적화된 객체 탐지 모델을 구축하기 위해**, YOLO 모델을 생성 및 학습시켰습니다.  
  이를 통해, 특정 꽃의 종류나 씨앗을 탐지하는 데 기여했습니다.  
  👉 [모델 생성 및 학습 과정](https://github.com/juntae02/bloom_for_you/tree/main/yolo_models)
  
- ***상황별 꽃 추천 기능*** :  
  &nbsp;&nbsp;OpenAI API 기반 키워드 추출 함수(팀원)를 활용하여, 사용자의 **목적과 상황을 반영하는 프롬프트**를 설계했습니다.  
  추출된 키워드를 **JSON 파일과 매칭**하여, 적절한 꽃 정보를 자동으로 불러오는 **추천** 로직을 구성했습니다.  
  해당 꽃 정보는 **Kivy 기반의 GUI**에서 사용자가 결과를 확인하고, **"선택" 또는 "재선택"을** 요청할 수 있는 인터페이스를 구현했습니다.  
  👉 [꽃 추천 기능(flower_recommender.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L205-L250)  
  👉 [GUI(Kivy) 기능(flower_recommender.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/flower_recommender.py#L95-L203)  
  👉 [프롬프트 파일](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/prompt/recommender_prompt.txt)  
  👉 [JSON 파일](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/resource/flower_recommendations.json)
  
- ***자동화된 씨앗 심기 기능*** :  
  &nbsp;&nbsp;**ROS2 토픽 통신**을 통해 전달받을 꽃 정보를 해석하여, **로봇의 동작 흐름**을 설계했습니다.  
  해당 꽃 정보를 바탕으로, YOLO를 통해 씨앗 및 화분의 위치를 탐지하고, **"씨앗 집기 -> 운반 -> 이식 -> 재배 구역 이동"** 로직을 구현했습니다.  
  **씨앗 미탐지 예외 처리** 및 **순응 제어 기반의 정밀 배치 기능** 로직도 구성하여, 안정적인 동작을 보장했습니다.  
  👉 [씨앗 Pick&Place(seed_planting.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L76-L156)  
  👉 [화분 Compliance Control(seed_planting.py)](https://github.com/juntae02/bloom_for_you/blob/main/bloom_for_you/bloom_for_you/seed_planting.py#L158-L184)
<br />

## 🤔 트러블슈팅 및 해결 과정 
- 문제 상황: 화분에 꽃이 심어져 있을 때, 꽃은 인식하지 못하고 화분만 인식하는 문제 발생
> - 시도했던 방법:
> - 해결 과정:
> - 결과
- 문제 상황: 멀티 쓰레드
- 문제 상황: 씨앗 pick 실패의 경우
<br />

## 💡 과정 속에서 배운 점 및 향후 계획
- 배운 점:
> - d:
- 향후 계획:
> - d:
<br />

## 🤝 팀원 정보
- ***준태***: YOLO 모델 생성, 꽃 추천 및 씨앗 심기(본인)   
- 석환: 기능 모듈화, 전체 시나리오 관리 및 총괄 지휘  
- 예은: 기능 모듈화, 자동 관수 및 꽃 포장 
- 요한: Front-end, 음성 메시지 기록 및 QR코드 생성 
<br />

## 📚 참고 및 출처
- 본 프로젝트의 일부 코드는 **두산 로보틱스 부트캠프 교육과정**에서 제공된  
  `Tutorial.zip` 자료를 기반으로 작성되었습니다.
- `STT.py`, `wakeup_word.py`, `keyword_extraction.py` 파일은 해당 자료에서 가져와 **일부 기능을 수정하여 사용**하였습니다.
- 그 외의 파일은 프로젝트 목적에 맞게 **직접 구현된 코드**입니다.
- 원본 전체 코드는 포함되어 있지 않으며, 본 저장소에는 **필요한 일부 파일만 포함**되어 있습니다.

- `bloom_for_you/function_modules/onrobot_.py` 파일은 아래 공개 저장소의 코드를 기반으로 **일부 수정하여 사용**하였습니다:  
  > 🔗 https://github.com/takuya-ki/onrobot-rg

- 원본 코드는 [MIT License](https://github.com/takuya-ki/onrobot-rg/blob/main/LICENSE)를 따르며, 본 저장소 내 수정된 코드 역시 동일한 라이선스를 존중하여 사용하고 있습니다.

> 본 저장소는 **교육 및 학습 목적**으로만 사용됩니다.

