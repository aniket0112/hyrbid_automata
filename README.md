# hyrbid_automata
HybridAutomata with Go To Goal, Obstacle Avoidance and Follow Wall modes governed by SlidingModeSwitch for Differential Drive Robot in ROS Gazebo.
![Image Not Found](/images/dd_bot.png)
## Getting started

### Prerequisites
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) with Gazebo-9 (Desktop-Full Install)
* [Python 2.7](https://www.python.org/downloads/source/) with [numpy](https://askubuntu.com/questions/868599/how-to-install-scipy-and-numpy-on-ubuntu-16-04)
* [Control Of Mobile Robots: Geogia Tech](https://www.youtube.com/playlist?list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr) is a prerequisite for understanding the process of how this project works or you can visit this blog to get a quick crashcourse.

### Installing
* Clone the repository to the local machine
* Place the repository folder in ```/your/machine/catkin_ws/src```
* Navigate into ```catkin_ws``` folder and open terminal.
* Type ```catkin_make``` and press enter.
* Make sure your ```.bashrc``` file (located in Home Folder) has these lines added in them:
```source yourpath/ros/melodic/setup.bash```
```source yourpath/catkin_ws/devel/setup.bash```
* After catkin has completed making the workspace, ROS should be able to locate your new rospackage ```dd_bot```. To check it, type in terminal ```rospack find dd_bot```. This should print out the location of the ```dd_bot``` folder.
If the rospackage location is not displayed, please contact the collaborators.

### Simulating an Induced Mode Control
* Open a terminal and type ```roslaunch dd_bot gazebo.launch```. A Gazebo world should be loaded with a differential robot surrounded by a blue colored disc (Laser Sensor visualiztion) and a wall obstacle around it.
* Open another terminal inside the folder by entering command ```cd /your/machine/catkin_ws/src/hybrid_automata/dd_bot``` and type ```python induced_mode.py```.
* The robot will start to move from current position to (5,5) as per default value. To change the desired position, open ```induced_mode.py``` and make changes in line 18 accordingly.
*```hybrid_automata.py``` has all the constants and configuration parameters defined for the Python controller design.
*The folders ```urdf``` and ```config``` collectively constitute the model specifications for the physics engine of simulator. A good way to understand about editting them is following ROS Gazebo tutorials [here](http://gazebosim.org/tutorials) or follow up [my blog](https://medium.com/@aniket0112/twolinkman-3b326c1320eb) for a crashcourse
*NOTE* : If anything is logged in red color while running any of the above commands, there has been some error. Contact collaborators in such case with the log messages.

### Known issues
* Could not load controller 'right_wheel_controller' because controller type 'effort_controllers/JointVelocityController' does not exist.  
Instal ros-control package for ros-melodic  
```sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers```
* [REST.cc:205] Error in REST request  
Check this [Gazebo Answers thread](https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/)

## Built with
* [SolidWorks](http://www.solidworks.in/Default.htm) - 3D CAD model
* [ROS Meloidc](http://wiki.ros.org/melodic) - ROS control node
* [Gazebo 9](http://gazebosim.org/) - Simulation environment
* [Python 2.7](https://www.python.org/) - Python interface for easy UI

## Authors
* [Aniket Sharma](https://github.com/aniket0112)

## Acknowledgements
Hearty thanks to Georgia Tech for putting up and amazing walkthrough course on controlling mobile robots. Dr. Magnus Egerstedt has done a brilliant job in explaining each concept part by part and compiling them in the big picture with real world demonstration of each part. I definitely recommend completing that course for control system enthusiasts.
