# swarm-robotics
1. This project is about swarm robotics with ros.
2. Main task is rescueing people from natural disasters or rescueing lost people.. 
3. In this project ros is used to control robots, gazebo is used for simulating scenario and hog feature for detecting people with camera. 
4. To use this project you need to install ubuntu 14.04 or Linux Mint 17.1, ROS Indigo Distribution, Gazebo 2.2 and Hector Quadrotor Packages.
5. You can use modeldownload.sh to install gazebo models, which is copied from [here.](http://machineawakening.blogspot.com.tr/2015/05/how-to-download-all-gazebo-models.html)
6. Project needs a walking human model which is named actor but hector quadrotor packages don't run properly in Gazebo 8.
7. Gazebo 2.2 has no actor also, so you need to change one of pioneers' chasis mesh from either xacro from the project or from gazebo's xacro file which is in the "gazebo_plugins/test/multi_robot_scenario".
8. And in this project we used an interval of angles which we choose to read laser values. But it can be calculated from Hog detection to detect where is the human. It is commented right before the line that we used the angle.
9. After installing all of the dependencies, project can be run with roslaunch command and using "demo.launch" file.
10. Swarm Intelligence wasn't implemented, but i will implement bee algorithm in nearly future and abstacle avoidance with vector field histogram. Feel free to contribute or use. 
