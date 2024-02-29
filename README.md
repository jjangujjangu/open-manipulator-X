# open_manipulator_x_tutorial
본 리포지토리는 open Manipulator X의 ROS2 연습한 것을 기록한 리포지토리입니다.
전반적인 내용은 (https://github.com/jungsuyun/open_manipulator_x_tutorial)를 참고하였고, 환경설정 및 실행 결과와 IK푸는 것을 추가하였다.
![image](https://github.com/jjangujjangu/open-manipulator-X/assets/158059339/f3bc96bc-1de6-442c-b48a-72a1860d4c73)

## __1. 개발환경__
* [Ubuntu 20.04](https://ubuntu.com/)
* [ROS2 foxy](https://omorobot.gitbook.io/manual/product/omo-r1mini/ros/ros2-foxy/ros2-ubuntu-20.04)
* [Robotis Open Manipulator X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#opensoftware)
* Python 3.8.10

## __2. 노드 소개__
* [__hello_ros_pub__](https://github.com/jungsuyun/open_manipulator_x_tutorial#4-hello_ros_pub) : Python 기반의 ROS2 Topic Publishing 예제
* [__hello_ros_sub__](https://github.com/jungsuyun/open_manipulator_x_tutorial#5-hello_ros_sub) : Python 기반의 ROS2 Topic Subscribing 예제
* [__init_and_home__](https://github.com/jungsuyun/open_manipulator_x_tutorial#6-init_and_home) : 초기 위치, home 위치로 이동하기
* init_and_home2: 초기 위치, 원하는 좌표로 이동하기
* [__gripper_control__](https://github.com/jungsuyun/open_manipulator_x_tutorial#7-gripper_control) : Gripper 열고 닫기
* __jointstate_subscriber__ : 각 Joint별 각속도 값 읽어오기
* __kinematics_subscriber__ : Gripper를 기준으로 현재 위치를 X/Y/Z축 좌표값으로 읽어오기
* __joint_teleoperation__ : Joint 기반의 이동 명령 내리기
* __kinematics_teleoperation__ : X/Y/Z 좌표값 기반의 이동 명령 내리기

## __3. 개발환경 세팅하기__
### __3.1. Open Manipulator 패키지 설치하기__
Ubuntu 20.04가 깔려있다는 가정하에 ROS2 Foxy를 설치한다.
```bash
$ sudo apt update
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
$ chmod 755 ./install_ros2_foxy.sh
$ bash ./install_ros2_foxy.sh
```

다음으로는 open manipulator와 관련된 의존성 패키지들을 다운로드 및 빌드를 수행한다.
```bash
$ sudo apt install ros-foxy-rqt* ros-foxy-joint-state-publisher
$ cd ~/colcon_ws/src/
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone -b foxy-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/colcon_ws && colcon build --symlink-install
```

### __3.2. Hardware 구성하기__
OpenMANIPULATOR-X를 제어하는 데 사용할 수 있는 통신 인터페이스 하드웨어 옵션은 2가지 있다. 나의 경우 OpenCR을 communication interface로 이용하였고 이를 통해 Open Manipulator와 PC와 연결하였다.

![OpenManipulator_opencr_setup2](https://github.com/jjangujjangu/open-manipulator-X/assets/158059339/391c0d63-34e2-4c01-bb68-dc01f806edf2)


[OpenCR 연결법](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/) (Open Manipulator X homepage 4.quick start guide에서 4.1.4 Communication Interface 참조)

이후, 아래의 코드를 terminal에 입력했을 때 open manipulator X에 torque가 걸린다면 잘 설정한 것이다.
```bash
$ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/ttyACM0
```
[launch controller](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/)(open manipulator launch controller 참조)
### __3.3. Python 의존성 패키지 설치하기__
다양한 key값을 받아올 수 있도록 우리는 getkey()라는 함수가 필요하다. 이는 Windows의 C++ 코드에는 구현되어 있지만, Ubuntu Linux에는 구현되어 있지 않다. 이를 위해 우리는 getkey()와 관련된 python 패키지를 설치해준다.
terminal에 다음과 같이 입력한다.
```bash
pip3 install getkey
```
## __4. Workspace 생성 및 환경설정 __

Workspace를 생성하는 과정은 아래와 같다.


Workspace를 생성하기 위해 다음과 같이 terminal에 입력한다.
```bash
cd 
mkdir -p ~/colcon_ws/src
# workspace로 이동
cd ~/colcon_ws
# 초기 build 수행(build, install log 폴더가 추가 생성)
colcon build --symlink-install

```
이후 ROS 환경 변수 설정을 한다.
```bash
gedit ~/.bashrc
```
.bashrc 폴더 가장 아래에 해당 문구를 추가한다.
```bash
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
```bash
# .bashrc 폴더 저장 후 종료
source ~/.bashrc
```
source ~/.bashrc를 반드시 입력해야 변경사항이 반영됨을 유의하자.

Cmake package 생성
```bash
cd ~/colcon_ws/src
ros2 pkg create --build-type ament_cmake tutorial_python
```

Python package 생성
```bash
ros2 pkg create --build-type ament_python tutorial_python
```

## __5. hello_ros_pub__
본 패키지는 ROS2 기반의 프로그래밍 개발을 위해 가볍게 코드를 구현해보자는 의미에서 만든 패키지이다.
### 5.1. 전체 code

```bash
cd ~/colcon_ws/src/tutorial_python/tutorial_python
gedit hello_ros_publisher.py
```
gedit에서 script를 생성하고 아래의 코드를 작성한다.
```bash
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'talker', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.data)
        self.i += 1

def main(args = None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()
```
이후 src폴더 안에 있는package.xml에 dependency를 추가한다.
```bash
# rclpy와 std_msgs에 대한 dependency를 추가한다.
.....
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>open_manipulator_msgs</depend>
.....
```
또한 src폴더 안에 있는 setup.py의 console_scripts에 다음과 같이 추가한다.
```bash
.....
entry_points={
        'console_scripts': [
            'hello_ros_pub = tutorial_python.hello_ros_publisher:main',
            'hello_ros_sub = tutorial_python.hello_ros_subscriber:main',
            'init_and_home = tutorial_python.init_and_home_node:main',
            'init_and_home2 = tutorial_python.init_and_home_node2:main',
            'gripper_control = tutorial_python.gripper_control_node:main',
            'jointstate_subscriber = tutorial_python.get_joint_state_node:main',
            'kinematics_subscriber = tutorial_python.get_kinematics_node:main',
            'joint_teleoperation = tutorial_python.joint_teleoperation:main',
            'kinematics_teleoperation = tutorial_python.kinematics_teleoperation:main',
        ],
    },

```

### 5.2. Source code 설명
가장 먼저 의존성 패키지들을 import 해준다. ROS2 기반의 python 프로그래밍을 위해선 `rclpy` 패키지를 import 해주어야 한다. 또한 `String` 타입의 메시지를 발행하기 위해 `std_msgs/msg/String` 타입을 import 해준다.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

다음으로 클래스 선언 부분이다. 우리는 SimplePublisher라는 클래스를 새로 생성할 것이고 해당 클래스는 `rclpy.node`를 상속받게된다. 가장 먼저 부모 클래스의 `__init__` 함수를 통해 해당 Node 명을 선언해주고 `talker`라는 topic을 발행할 publisher를 선언해준다. 해당 topic은 `timer_callback` 함수를 통해 0.5초마다 반복 실행 될 것이다.
```python
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'talker', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```

`timer_callback` 함수에서는 `String`타입의 메시지를 선언해주고 해당 `msg`의 `data` 부분에 str 값을 입력해주게 된다.
```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello ROS %d' % self.i
    self.publisher.publish(msg)
    self.get_logger().info('Publishing: %s' % msg.data)
    self.i += 1
```

다음으로 메인에서는 `rclpy.init` 을 통해 노드 연결을 준비하고 `rclpy.spin()` 함수를 통해 무한루프 형태로 노드가 동작하도록 구현하였다.
```python
def main(args = None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)

    simple_publisher.destroy_node()
    rclpy.shutdown()

```
### 5.3. 구동하기
```bash
cd ~/colcon_ws 
colcon build --packages-select tutorial_python
```
이후 구동하면 다음과 같은 화면이 나타난다.
```bash
ros2 run open_manipulator_x_tutorial hello_ros_pub
```
![Screenshot from 2024-02-29 15-13-52](https://github.com/jjangujjangu/open-manipulator-X/assets/158059339/8d2d835b-c6d8-48c7-bb8f-02e31c3e8ee9)


## __6. hello_ros_sub__
본 노드는 앞의 5.에서 구현한 hello_ros_pub에서 발행되는 topic값을 subscribe하여 값을 terminal에 출력해보는 것이다.
### __6.1 전체 code__
```bash
cd ~/colcon_ws/src/tutorial_python/tutorial_python
gedit hello_ros_subscriber.py
```
gedit에서 script를 생성하고 아래의 코드를 작성한다.
```bash
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('hello_ros_subscriber')
        self.subscription = self.create_subscription(String, 'talker', self.talker_callback , 10)
        self.subscription
    def talker_callback(self, msg: String):
        self.get_logger().info('I heard : %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()
```
### 6.2. Sourcecode 설명
가장 먼저 의존성 패키지들을 import 해준다. ROS2 기반의 python 프로그래밍을 위해선 `rclpy` 패키지를 import 해주어야 한다. 또한 `String` 타입의 메시지를 발행하기 위해 `std_msgs/msg/String` 타입을 import 해준다.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

다음으로 클래스 선언 부분이다. 우리는 SimpleSubscriber라는 클래스를 새로 생성할 것이고 해당 클래스는 `rclpy.node`를 상속받게된다. 가장 먼저 부모 클래스의 `__init__` 함수를 통해 해당 Node 명을 선언해주고 `talker`라는 topic을 Subscribe할 `subscription`을 선언해준다.
```python
class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('hello_ros_subscriber')
        self.subscription = self.create_subscription(String, 'talker', self.talker_callback, 10)
        self.subscription
```

`talker_callback` 함수는 talker topic에서 값이 들어왔을 경우 수행되는 함수 부분으로 `get_logger().info()` 함수를 통해 terminal에 출력을 수행할 것이다.
```python
def talker_callback(self, msg: String):
    self.get_logger().info('I heard : %s' % msg.data)
```

다음으로 메인에서는 `rclpy.init` 을 통해 노드 연결을 준비하고 `rclpy.spin()` 함수를 통해 무한루프 형태로 노드가 동작하도록 구현하였다.
```python
def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()
```
### __6.3. 구동하기__

package를 build한 후
```bash
cd ~/colcon_ws
colcon build --packages-select tutorial_python
```
구동하면 다음과 같은 결과가 출력된다.
```bash
ros2 run tutorial_python hello_ros_sub
```

![Screenshot from 2024-02-29 15-16-00](https://github.com/jjangujjangu/open-manipulator-X/assets/158059339/64bb496e-8b1e-44f0-a57b-05addaee8143)

## __7. init_and_home__
본 노드는 Open Manipulator X를 초기위치, Home 위치로 이동하는 노드를 구현해본다.

### __7.1. 전체 code__
```bash
$ cd ~/colcon_ws/src/tutorial_python/tutorial_python
$ gedit init_and_home_node.py
```
gedit에서 script를 만든 다음 다음과 같이 코드를 작성한다.
```bash
import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition

PI = 3.14159265359
NUM_OF_JOINT = 4

class InitAndHome(Node):
    def __init__(self):
        self.future = None
        super().__init__('init_and_home_node')
        self.path_time = 2.0
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        self.request = SetJointPosition.Request()

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        command = getkey()
        self.send_command(str(command))

    def set_joint_space_path(self, joint_name: list, joint_angle: list):
        self.request.joint_position.joint_name = joint_name
        self.request.joint_position.position = joint_angle
        self.request.path_time = self.path_time

        future = self.client.call_async(self.request)

    def send_command(self, command: str):
        if command is '1':
            print("input : 1 \t init pose")
            joint_name = []
            joint_angle = []

            joint_name.append('joint1')
            joint_name.append('joint2')
            joint_name.append('joint3')
            joint_name.append('joint4')

            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            joint_angle.append(0.0)
            self.set_joint_space_path(joint_name, joint_angle)
        elif command is '2':
            print("input : 2 \t home pose")
            joint_name = []
            joint_angle = []

            joint_name.append('joint1')
            joint_name.append('joint2')
            joint_name.append('joint3')
            joint_name.append('joint4')

            joint_angle.append(0.0)
            joint_angle.append(-PI/3)
            joint_angle.append(PI/9)
            joint_angle.append(PI*2/9)

            self.set_joint_space_path(joint_name, joint_angle)
        else:
            print("input : %s" % command)

def main(args=None):
    rclpy.init(args=args)
    init_and_home = InitAndHome()
    while rclpy.ok():
        rclpy.spin_once(init_and_home)

    init_and_home.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 7.2. Sourcecode 설명
가장 먼저 의존성 패키지들을 import해준다. 가장 먼저 사용자의 key값을 받기 위한 `getkey`모듈을 import하고 ROS2 기반의 python 프로그래밍을 위해선 `rclpy` 패키지를 import 해주어야 한다. 마지막으로 Joint별 명령을 제어하기 위해 `open_manipulator_msgs/srv/SetJointPosition` 모듈을 import 해주도록 한다.

```python
import os
from getkey import getkey

import rclpy
from rclpy.node import Node

from open_manipulator_msgs.srv import SetJointPosition
```

다음으로 전역변수 선언이다. Open Manipulator X는 4개의 구동축으로 구성되어 있기 때문에 `NUM_OF_JOINT`라는 변수에 4의 값을 넣어주고, Home 위치 선언을 위해 `PI`변수를 선언해주었다.
```python
PI = 3.14159265359
NUM_OF_JOINT = 4
```

다음은 Class 선언 부분이다. `InitAndHome` Class는 `Node` Class를 상속받았다. `super().__init__()`함수를 통해 Node명을 선언해준다. `client` 변수는 `goal_joint_space_path` Service와 연동되는 client이고 해당 client는 `request`를 통해 값을 보내고 받아오게 된다.

해당 노드는 0.5초 간격으로 값을 보내는 역할을 수행하게 할 것이므로 `create_timer`함수를 통해 callback함수단에서 동작이 이루어질 수 있도록 한다.

```python
class InitAndHome(Node):
    def __init__(self):
        self.future = None
        super().__init__('init_and_home_node')
        self.path_time = 2.0
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        self.request = SetJointPosition.Request()

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
```

`timer_callback`함수에서는 `getkey()`함수를 통해 값을 입력받고 입력받은 값을 `send_command`함수로 보내게 된다.
```python
def timer_callback(self):
    command = getkey()
    self.send_command(str(command))
```

`send_command`함수는 입력된 값에 따라 동작이 이루어지도록 한다. 초기위치와 home 위치로 이동하기 위해서는 각 `Joint`를 제어해주어야 한다. Joint를 제어해주기 위해서는 각 Joint의 이름과 joint의 angle 값이 필요하다.

이를 위해 `joint_name`이라는 list와 `joint_angle`이라는 list를 선언해주고 각 list에 맞는 값을 입력해준다. 입력이 완료되면 `set_joint_space_path`라는 함수로 인자를 전달하여 `service call`이 이루어지도록 한다.
```python
def send_command(self, command: str):
    if command is '1':
        print("input : 1 \t init pose")
        joint_name = []
        joint_angle = []

        joint_name.append('joint1')
        joint_name.append('joint2')
        joint_name.append('joint3')
        joint_name.append('joint4')

        joint_angle.append(0.0)
        joint_angle.append(0.0)
        joint_angle.append(0.0)
        joint_angle.append(0.0)
        self.set_joint_space_path(joint_name, joint_angle)
    elif command is '2':
        print("input : 2 \t home pose")
        joint_name = []
        joint_angle = []

        joint_name.append('joint1')
        joint_name.append('joint2')
        joint_name.append('joint3')
        joint_name.append('joint4')

        joint_angle.append(0.0)
        joint_angle.append(-PI/3)
        joint_angle.append(PI/9)
        joint_angle.append(PI*2/9)

        self.set_joint_space_path(joint_name, joint_angle)
    else:
        print("input : %s" % command)
```

`set_joint_space_path`에서는 service 타입에 맞는 값을 입력해주도록 한다. `request`의 joint_name에는 인자로 전달받은 joint_name을, 축의 각 값은 joint_position에 입력해주도록 한다. `path_time`은 해당 동작을 몇초안에 수행해야 하는지를 표기한다. 여기에선 2.0초로 `__init__`함수에서 선언해주었다.

request에 인자를 정상적으로 입력하였다면 `client.call_async()` 함수를 통해 request 변수를 call 해주도록 한다.
```python
def set_joint_space_path(self, joint_name: list, joint_angle: list):
    self.request.joint_position.joint_name = joint_name
    self.request.joint_position.position = joint_angle
    self.request.path_time = self.path_time

    future = self.client.call_async(self.request)
```
### __7.3. 구동하기__

package를 build하는 방법은 다음과 같다.
```bash
cd ~/colcon_ws 
colcon build --packages-select tutorial_python
```
init_and_home_node를 구동하기 앞서서 Open Manipulator X의 launch controller를 구동해야한다. 

새로운 terminal을 열어 다음 코드를 실행시킨다. 
```bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/ttyACM0
```
이후 새로운 terminal에서 다음 코드를 실행시킨 결과는 다음과 같다.
```
ros2 run tutorial_python init_and_home

```


https://github.com/jjangujjangu/open-manipulator-X/assets/158059339/76bdbdf2-ef04-42b2-982d-0cac33073e51


