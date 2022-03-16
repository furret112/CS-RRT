# MoveIt +  customize planner


!build on ubuntu 18.04

### 1. install MoveIt from source

- remove MoveIt

```bash
sudo apt remove ros-melodic-moveit
```

- install some dependence

```bash
sudo apt-get update    
sudo apt-get dist-upgrade
sudo apt-get install python-wstool python-catkin-tools clang-format-3.9
```

- build MoveIt from source

```bash
mkdir ~/ws_moveit
cd ~/ws_moveit
source /opt/ros/melodic/setup.bash
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release 

sudo catkin build
```

- remove OMPL

```bash
sudo apt remove ros-melodic-ompl
```

- Clone source OMPL

```bash
cd ~/moveit_ws/src
git clone https://github.com/ompl/ompl
#cd ~/ws_moveit/src/ompl
#wget [https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml](https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml)

cd ~/ws_moveit

source ~/ws_moveit/devel/setup.bash
```

---

### 2. Add customize planner in OMPL

- at ~/workspace/srcompl/src/ompl/geometric/planners/rrt

```bash
1. copy RRT.h rename to my_RRT.h

2. in /src copy RRT.cpp rename my_RRT.cpp

3. change all RRT in two file to my_RRT
```

![Untitled](https://github.com/furret112/CS-RRT/blob/main/1.png)

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/53894306-3b6d-483a-b8a7-83cc466e832b/Untitled.png)

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/2888daef-01e3-41db-afb1-822a3d98e082/Untitled.png)

- at ~/workspace/src/moveit/moveit_planners/ompl/ompl_interface/src/**planning_context_manager.cpp**
1.  add my_RRT relate

```cpp
#include </home/<user>/<your workspace>/src/ompl/src/ompl/geometric/planners/rrt/my_RRT.h>
```

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/0fe0bd37-2085-4402-9954-96b04fbb52cb/Untitled.png)

1. register your planner in same file

```cpp
registerPlannerAllocator(  //
      "geometric::my_RRT",      //
      std::bind(&allocatePlanner<og::RRT>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
```

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/748f7205-a01b-41a1-88f9-843ccc8800b3/Untitled.png)

- at UR5_control/src/univeral_robot/ur5_moveit_config/config 的 **ompl_planning.yaml**
1.  add your planner

```yaml
myRRTkConfigDefault:
    type: geometric::my_RRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability? default: 0.05
```

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/636b56a1-8468-4bef-9927-6351afcb47df/Untitled.png)

1.  add your planner’s option below planner_configs:

```yaml
- my_RRTkConfigDefault
```

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/a240d071-2f71-4e91-bf3e-0f8ff7762720/Untitled.png)

### 3. Compile

```bash
cd ws_moveit
sudo catkin build
```

### 4. Test

- cd ur5 workspace

```bash
source ~/ws_moveit/devel/setup.bash

roslaunch ur5_moveit_config demo.launch
```

- in rviz
1.  choose  **Context** tab
2. check is there have **my_RRTkConfigDefault**
3. Plan arm trajectory

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/5e8775f1-5b52-4eb8-ab8f-bd343179a40f/Untitled.png)

---

### 5. Using CS-RRT on UR5

```bash
# connect to UR5 
1. roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.12

# connect to planner
2. roslaunch ur5_ft_gripper_cabinet_moveit_config ur5_ft_gripper_cabinet_moveit_planning_execution.launch

# open 2f_gripper
3. roslaunch robotiq_2f_gripper_control robotiq_2f_gripper_RtuNode.launch comport:=/dev/ttyUSB0

# open camera
4. rosrun obj_detect get_image.py

# Test
5. rosrun ur_move_test ur_strategy.py 
```

- about  camera

[https://github.com/furret112/azure_yolov4](https://github.com/furret112/azure_yolov4)
