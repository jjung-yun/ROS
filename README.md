>![header](https://capsule-render.vercel.app/api?type=transparent&color=auto&height=90&section=header&text=2022-2%20SW%20Capstone%20Design&fontSize=50&fontColor=A9280C)
>
>## Field painting robot using ROS-based DQN reinforcement learning and PID control
>
> 2018100670 김정윤
> 2018100694 방지호

#0. 시현영상

https://

00:04~04:43 gazebo 시뮬레이션 환경에서 PID제어로 축구장 그리기
04:44~08:29 gazebo 시뮬레이션 환경에서 DQN 학습모델이 학습하는 모습
08:30~11:24 실제 turtlebot3 모델=burger가 사각형 트랙을 그리는 모습과 장애물 회피영상

# 1. 실행환경 
- Ubuntu 18.04 LTS버전, ROS-melodic, python2.7.17, tensorflow 2.1.0, numpy 1.16.4, keras 2.3.1, Anaconda 3
- Ubuntu 22.04, ROS2-humble, python 3.9, tensorflow 2.1.0, numpy 1.16.6, keras 2.3.1, Gazebo9

# 2. Hardware 스펙
모델 : turtlebot3 burger, Size (L x W x H)=138mm x 178mm x 192mm, Weight=1kg
SBC (Single Board Computers) : Raspberry Pi
LDS(Laser Distance Sensor) : 360 Laser Distance Sensor LDS-01
IMU센서 : Gyroscope 3 Axis, Accelerometer 3 Axis
위 2 가지 실행환경 중 1번째 ROS-melodic 위주로 과제를 수행하였음. ROS1에서 제작한 패키지 및 노드, 소스코드들을 활용하여 실제 turtlebot3(model=burger)를 작동하였음.

# 3. Waypoint 기능 구현
  Waypoint란 특정 point를 따라 이동하는 것을 의미한다. 도로의 차선과 축구장, 야구장과 같은 원하는 map을 그리기 위해선 정교한 waypoint를 수행할 수 있어야 한다. 터틀봇3에서는 Odometry 즉, 로봇의 위치 정보를 subscriber를 통해 받아올 수 있으며, cmd_vel 즉, 선속도와 각속도 정보를 publisher를 통해 로봇에게 줄 수 있다. 이 두가지를 가지고 터틀봇3의 waypoint 기능을 구현하였다.


![fig1](https://user-images.githubusercontent.com/104184349/204995168-e52a79d6-6049-4ab2-9c2c-3c58fecd04ed.png)

**Fig. 1** Conceptual schematic for waypoint implementation.


  앞서 언급하였듯이 Odometry를 통해 터틀봇3의 x, y, θ 값을 받아들일 수 있다. 이때 각각의 값을 self.x, self.y, self.θ라고 하자. 도착 지점의 좌표는 명령을 통해 넣어주는 값으로 x, y으로 정의하였다.


  이를 활용하면 (1)의 식과 같이 터틀봇3와 목표점까지의 벡터 를 구할 수 있으며, 터틀봇3와 목표점까지의 거리를 나눠주면 (2)의 식과 같이 크기가 1인 단위벡터 를 구할 수 있다. (1), (2) 식을 활용하여 이후 터틀봇3를 목표점까지 일직선으로 이동시킬 수 있다.

  터틀봇3를 안정적으로 이동시키기 위해선 먼저 self.θ 값 즉, 로봇이 바라보는 방향과 목표점까지의 θ값을 일치시켜야 한다. self.θ와 θ가 일치한다는 것은 angular가 0이 되도록 만든다는 것이다. 이를 위해선 cmd_vel으로 각속도를 조절해야 한다. 각속도는 (3) 식에서 볼 수 있듯 θ - self.θ를 계속 업데이트하는 방식으로 조절할 수 있다. 이후 PID 제어기를 추가하여 각속도의 성능과 안정성을 향상시킬 수 있다.

  angular 값을 0으로 맞춰주었다면 터틀봇3를 바라보고 있는 방향 즉, x축 성분의 선속도 명령만을 주어 목표점으로 이동시킬 수 있다. 그러나 이 과정에서 내부와 외부 요인으로 로봇이 일정한 방향으로 갈 수 없는 상황이 생길 수 있으므로 각속도 명령도 포함되어야 한다. 먼저, 선속도는 (4) 식과 같이 계산할 수 있다. 여기서 단위벡터 는 초기 로봇 위치와 목표점으로부터 계산한 값이며, 벡터 는 이동 중에 계속해서 변화하는 벡터 값이다. 이 두 벡터를 내적하여 로봇과 목표점 사이의 직선 성분만 남길 수 있다.
  위와 같은 과정으로 거쳐 터틀봇3의 waypoint 기능을 구상 및 구현하였다.

# 4. PID제어 구현
PID제어는 Proportional Integral Derivative control의 약자로, 얻고자 하는 출력 값 (controlled variable)이 명령을 받아 실제 System을 변화시키는 명령 값 (commanded variable)과 얼마나 차이가 나는지 (error term)를 계산해 0으로 수렴할 수 있게 해주는 것을 의미한다.

![pid](https://user-images.githubusercontent.com/104184349/204840458-fef330d6-dae7-4bde-b161-b13c38145957.png)

**Fig. 2** A generic closed-loop process-control system with PID controller.


위 그림에서 알 수 있듯, PID제어는 P I D 3종류의 제어를 받는다. 따라서 각 제어에 적절한 이득 값 (gain)을 주어 오차항을 줄일 수 있는데 이 적절한 이득 값을 계산하거나 구하는 과정을 튜닝 (tuning)이라고 한다. PID제어는 Closed loop에서 오차를 이용하여 조절하는 구조로 그 구조가 간단하고, 파라미터 조정을 비교적 쉽게 할 수 있으며, 제어 성능 또한 우수하여 가장 많이 사용된다. 하지만 각각의 Gain을 찾는 과정이 번거롭고 어렵다. 아래는 각 Gain값이 PID제어에서 어떤 역할을 하는지에 대한 설명이다.

아래는 PID제어이론을 기반으로 속도제어를 해 지나야 할 경로들에 대한 설계이다. 

![축구장설계](https://user-images.githubusercontent.com/104184349/204840821-ae1564a1-c480-4f5d-b124-ed839e349898.png)

**Fig. 3** Design for football field

PID제어에서 축구장, 야구장, 오징어 게임장 총 3개 종류의 field 설계 중 축구장만을 대표 예시로 들겠다.

추가적으로, field를 그리면서 생길 수 있는 돌발 상황으로 갑작스런 장애물이 생기면 감지된 라이더의 개수로 멈추는 소스코드를 제작하였다. 코드에 대한 설명은 다음과 같다. 왼쪽 15개 오른쪽 15개의 범위의 라이다를 센싱하여 3개씩 평균낸 값을 뽑아내면 10개 묶음의 라이다 값을 받아들인다. 그 중 40cm이내의 범위에 들어온 묶음이 4개 이상이 감지되는 동안 정지한다. 그 물체가 사라지면 다시 이동을 재개한다.

# 5. DQN학습 
DQN은 강화학습의 일종으로 주어진 환경에서 agent가 어떤 action을 취하게 했을 때 가장 큰 보상을 받았던 에피소드를 찾아내는 것을 뜻한다. 또한 ‘Greedy algorithm’을 따른다. Greedy Algorithm을 따르면 2 가지 문제점이 생기는데, 1번째로 ‘그 상황에서의 최적값이 전체에서의 최적값인지 알 수 없다.‘와 2번째로 ’지연된 보상‘이다.
1번째 문제점의 해결책으로 탐험지수 ’엡실론(ε)‘이다. 학습된 최적값을 로봇이 알고 있음에도 ε확률에 의해서 무작위 행동을 취하게 하여 또 다른 최적값을 찾을 수 있게 하는 것이다.
2번째 문제점의 해결책은 적절한 discount factor의 할당이다. 현재 좋은 값만 취하게 되면 즉각 보상이 높아지게 된다. 하지만 다른 루트로 갔을 때 잠재적인 미래 가치를 알 수 없다. 또한 좋은 길을 택했음에도 특정 값들을 얻지 못하여 보상이 없는 경우가 있다. 이런 경우들에 즉각보상과 할인율, 미래보상에 대한 DQN의 비용함수를 아래의 방정식과 같이 유도할 수 있다.

위 방정식을 참고하여, 터틀봇3에 적용한 parameter들은 아래와 같다.
 - Action parameter
직진, 우측 각속도, 우측 주행, 좌측 각속도, 좌측 주행 총 5개의 Action size임.
 - Reward
Collision 시 –1500, 목표 도달 시 3000, 
목표점과 가까울 시 (current pose/target pose) 200*2^distance rate 등을 주요 보상값으로 잡음.
 - 탐험지수 엡실론(ε)
시작 값=1, episode마다 감소하는 비율 0.99 최소값 0.0001 
 
위 맵을 기반으로 DQN학습으로 간단한 테니스 코트장을 구현하고자 한다.
좌우가 2m by 2m인 정사각형이고 총 4.5m by 2.5m인 직사각형 코트의 경로를 그리며 학습하기 위해서 아래와 같이 벽을 설계하였다.

 -학습할 map

![가제보맵](https://user-images.githubusercontent.com/104184349/204853755-cdadca60-55d8-4ddf-923c-1e90347ba971.png)

**Fig. 4** Maps used for dqn learning

# 6. Hardware 터틀봇3 모델=burger 구현
터틀봇의 SBC인 라즈베리 파이 PC를 고정 IP할당과 Remote PC를 연결해야 한다. 그 과정으로는 PC setup과 SBC setup을 마친 후 두 PC ubuntu에 ROS 의존성 패키지를 설치한 이후 remote PC에 필요한 ros 패키지들을 모두 설치하여야 한다. 이후 remote PC의 Ubuntu와 터틀봇의 라즈베리파이를 원격 연결할 수 있다. 다음으로 remote PC에서 turtlebot3_bringup 패키지를 실행할 수 있는데 이 패키지에 의해서 실제 터틀봇의 odometry와 battery state, IMU 등등의 터틀봇의 data값들을 가져올 수 있다. 이후에는 gazebo상에서 시뮬레이션 하던 것과 같이 소스코드와 패키지를 실행할 수 있었다.
실제 구현한 사진은 아래와 같고, simulation에서 움직이는 것과 같이 teleop 패키지를 이용해 키보드로 터틀봇을 간단히 조작할 수 있었다.

![더작음](https://user-images.githubusercontent.com/104184349/204842767-e07c0bb4-f934-471d-9bb3-4871a13502b8.jpg)

** Fig. 5** Connecting the actual Turtlebot 3 and the auxiliary battery


# 6. 결과
(1) PID제어를 통한 Field painting
ROS-melodic에서 gazebo로 구현하여 뽑은 trajectory는 아래와 같다.

![축구장 (1)](https://user-images.githubusercontent.com/104184349/204845108-ff631cb5-cd3a-494c-97d5-e1c666c05c74.png)

**Fig. 6** A soccer field implemented by the trajectory of turtlebot3

![오징어 (1)](https://user-images.githubusercontent.com/104184349/204847346-6220dc9e-6671-4645-ba6f-718b92abc160.png)

**Fig. 7** A baseballfield implemented by the trajectory of turtlebot3

![야구장 (1)](https://user-images.githubusercontent.com/104184349/204847483-2914155e-a39c-4b10-9a4f-f8ef44fa06cb.png)

**Fig. 8** A squid game field implemented by the trajectory of turtlebot3

(2) DQN학습을 통한 Field painting 

![화면 캡처 2022-11-30 213337](https://user-images.githubusercontent.com/104184349/204847565-1efba6ff-1955-4788-957d-a50b27b59976.png)

**Fig. 9** goal reward 3000 collision reward –1500 

gazebo 시뮬레이션 환경에서 터틀봇이 학습하며 맵을 나아가 경로를 그리는 모습이다. reward는 가장 크게는 도착 보상과 충돌 보상을 가지며, yaw축이 회전한 정도, goal box와 가까워지는 정도 생존한 시간 등을 종합적으로 고려해 총 보상값을 적용했으며 상단 그래프가 reward에 대한 값을 나타낸다.
또한 아래의 그래프는 DQN모델의 비용함수 값의 파라미터 중 하나이며, state와 action의 조합값인 Q-value이다.
학습으로 얻어진 경로는 아래와 같다.

![fig9](https://user-images.githubusercontent.com/104184349/204847629-f9666fb8-4388-420e-9c84-88632f8b03f0.png)

**Fig. 10** trajectory graph obtained from DQN learning


![image](https://user-images.githubusercontent.com/104184349/204995456-cc6f2be4-e455-4926-bd19-1636086fe12f.png)

**Fig. 11** Turtlebot trying to reach the target box through learning

https://user-images.githubusercontent.com/104184349/204995619-c6884f3d-05c7-46ef-9fa4-c029053a1f80.mp4


이후 굉장히 많은 episode를 통해 학습한 터틀봇이 마지막 목표상자를 타겟으로 접근하는 모습이다. 학습하면서 생긴 경로들은 1번 목표점부터 총 26번 목표점까지 모두 누적하여 아래의 그래프와 같이 나타내었다.


![image](https://user-images.githubusercontent.com/104184349/204995491-2470564d-bffe-47fe-8041-58d7cc095b1b.png)

**Fig. 12** Final trajectory of tennis court created by DQN learning

위 시뮬레이션 사진에서 (-0.5, 1.0) 점과 (-0.5, 1.5) 목표점을 한번에 통과하여 그 경로가 이산적으로 나타났다. 경로가 연속적이지 못한 한계가 있었지만, 원하는 목표점을 학습을 통해 모두 경유할 수 있었던 점에 큰 의의가 있었다.
위는 DQN학습을 통한 field painting으로 대략적인 테니스 코트를 trajectory를 통해 그려본 모습이다. 40000번이 넘는 episode를 통해 목표점에 도달한 경로들만 골라 뽑은 그래프이다. 학습하는 모델 특성상 굉장히 많은 시도를 해야하는데 실제 터틀봇에서 여러번의 시도에는 제한이 있어 시뮬레이션으로만 구현하였다.

(3) 실제 터틀봇 구현
waypoint를 따라 움직일 수 있음을 간단히 보여주기 위해 정사각형 트랙을 움직이며 그 trajectory를 그래프로도 보여줌을 확인하는 영상이다.

https://user-images.githubusercontent.com/104184349/204998809-32511aed-ed34-4f8a-af70-0e792af17b01.mp4

**Fig. 13** The actual Turtlebot drawing a square field

정사각형 트랙을 따라 움직이며 경로를 생성할 때, 레이저에서 스캔된 값들이 앞서 언급된 조건과 같은 경우 터틀봇이 정지하는 것을 확인한 영상이다.

https://user-images.githubusercontent.com/104184349/204998984-740a9857-6a62-4f91-86be-d3d8616ac77d.mp4

**Fig. 14** Turtlebot moving and stopping along the path
