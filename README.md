drone-load-transportation
=========================
"drone-load-transportaion" is a series of modified ROS packages to implement autonomous control functionality on an AR.Drone 2.0 with a suspended load in order to stabilize it during the flight. It uses the "ardrone_autonomy" driver (from AutonomyLab) and "tum_ardrone" (from tum-vision) for implementing autonomous flight with PTAM-based visual navigation. For estimating the load's position, the package "ar_pose" (from LucidOne) is used. It contains AR Marker tools for ROS based on ARToolKit for publishing pose data (tf) from a camera and a marker. The idea is to integrate all trhee packages in order to obtain a platform capable of load transportation using the AR.Drone bottom camera to estimate its position.

###Installation (with catkin)

Download the original packages from:

ardrone_autonomy: https://github.com/tum-vision/ardrone_autonomy

tum_ardrone: https://github.com/tum-vision/tum_ardrone/tree/indigo-devel

ar_pose e libArToolKit: https://github.com/srv/ccny_vision

In case the following packages are not included in your ROS installation, get them from:

rosbag: https://github.com/ros/ros_comm.git

tf: https://github.com/ros/geometry.git

rviz: https://github.com/ros-visualization/rviz.git

dynamic_reconfigure: https://github.com/ros/dynamic_reconfigure.git

After download, unzip the files into the source folder of your workspace and execute:
``` bash
cd catkin_ws/src/<package name>
rosdep install <package name>
catkin_make
```

Download all the files in this repository, look into the original packages and replace the existing files with these ones (look for the same file name). Then build your workspace.
``` bash
cd catkin_ws
catkin_make
```

###Quick Start

Connect your AR.Drone battery and, in separate terminals, launch the nodes:

ardrone_driver: this may take a few seconds to build. Check the prompt messages for connection failures. The AR.Drone will best perform with full charge.
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ardrone_autonomy
rosmake ardrone_autonomy
roslaunch ardrone_autonomy ardrone_driver.launch
```
ar_pose: this node will perform the marker pose estimation (assuming you already attached the marker to the suspended load)
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ar_pose
rosmake ar_pose
rosmake rviz rosbag
roslaunch ar_pose ar_pose_single.launch
```
The [rviz](http://ros.org/wiki/rviz) window should pop on the screen, but you will not see the image until the Drone's bottom camera streaming is enabled (later).

tum_ardrone: this will launch three nodes. Do not proceed without checking [tum_ardrone](https://github.com/tum-vision/tum_ardrone/tree/indigo-devel) for information about the nodes functionalities.
``` bash
cd catkin_ws/src
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/tum_ardrone
rosmake tum_ardrone
roslaunch tum_ardrone tum_ardrone.launch
```
Your system is ready. Run the [rqt_graph](http://wiki.ros.org/rqt_graph) tool to make sure you have all nodes running:

``` bash
rosrun rqt_graph rqt_graph
```

![envio_tf](https://cloud.githubusercontent.com/assets/9382891/5175632/c8205962-7424-11e4-894b-19378d03720d.png)

###Flying

AR.Drone 2.0 does not allow streaming of both camera images simultaneously, which means that for the load transportation porpuse it is necessary to give up the PTAM functionality. The Drone will still be able to fly autonomously, but its pose estimation will be worse. Let's hope that Parrot changes that in the future. 

In order to disable PTAM:
``` bash
rosrun rqt_reconfigure rqt_reconfigure
```
This will open the [rqt_reconfigure](http://wiki.ros.org/dynamic_reconfigure) screen, which allows the user to dynamically configure node parameters without having to access the source code or stop running it.
> IMPORTANT: requires the node drone_stateestimation to be running.

Select the node [drone_stateestimation](http://wiki.ros.org/tum_ardrone/drone_stateestimation) and uncheck "Use PTAM". For further experiments, you can also uncheck "Use navdata", which will make the EKF use only the control gains to update.

In order to enable the load stabilization controller, select the node [drone_autopilot](http://wiki.ros.org/tum_ardrone/drone_autopilot) and check "Use Load Control". The controller parameters are set for a suspended load with approximately 10% of the Drone's weight, but feel free to experiment.

![UseLoadConfig](https://cloud.githubusercontent.com/assets/9382891/5175752/12f73fea-7426-11e4-89cc-1a1ac70605e5.png)

Open the [drone_gui](http://wiki.ros.org/tum_ardrone/drone_gui#Keyboard_Control) interface, click on "Toggle Cam". Now you should see the bottom image on the rviz screen.

![rviz_screen](https://cloud.githubusercontent.com/assets/9382891/5175742/0266cdf8-7426-11e4-966a-8efcbdc31c14.png)

On the top left box of [drone_gui](http://wiki.ros.org/tum_ardrone/drone_gui#Keyboard_Control) you can either select one of the flight plans included in the package or write your own. Here is a simple flight plan using only control gains:

``` bash
takeoff

goto 0 0 0.7 0

goto 0.8 0.8 0.7 0
goto 0 1.6 0.7 0
goto -0.8 0.8 0.7 0
goto 0 0 0.7 0

land
```
####Procedure
- Position the Drone on the ground with a lot of free space around it. If you are using PTAM (which means you are not monitoring the load's position), you should have enough key points in front of the Drone. Give preference to furnitured indoor environmnents.
- Load the flight plan or write one.
- Click on "Reset" then "Clear and Send"
- Always be ready to press "Land" in case of imminent crash.

####Recording Flight Data
``` bash
$Create a folder to store data
mkdir ~/bagfiles
cd ~/bagfiles
$Recording messages from all running topics
rosbag record -a
$..or from a specific topic (e.g.: navdata)
rosbag record -O subset /ardrone/navdata
$Verifying content
rosbag info subset.bag
```
In order to take the recorded date to other softwares, it is useful to convert the bag file to txt:
``` bash
rostopic echo -b subset.bag -p /ardrone/navdata > output.txt
```

