                                                                                   Origin Version Project

1. $ cd moveit/src

2. #clone the ur driver
   $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

3. #retrieve the ur3 sources (replace '$ROS_DISTRO' with the ROS version you are using)
   $ git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

4. #clone the moveit model
   $ git clone -b $ROS_DISTRO-devel https://github.com/ros-planning/moveit.git src/moveit

5. #clone the robotiq model
   $ git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git

6. #clone the ros_control
   $ wstool init
   $ wstool merge https://raw.github.com/ros-controls/ros_control/melodic-devel/ros_control.rosinstall
   $ wstool update

7. #clone the actionlib
   $ git clone -b $ROS_DISTRO-devel https://github.com/ros/actionlib.git

7. #complete the project
   cd ..
   rosdep install --from-paths . --ignore-src --rosdistro melodic -y
   
8. $ catkin_make

