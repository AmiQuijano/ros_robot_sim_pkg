# ROS robot simulator

### Table of Contents
1. [Robot simulator description](#robot-simulator-description)
2. [Assignment 2 - Research Track 1: ROS Nodes](#assignment-2---research-track-1-ros-nodes)
3. [Assignment 1 - Research Track 2: Software Documentation](#assignment-1---research-track-2-software-documentation)
4. [Assignment 2 - Research Track 2: Jupyter Notebook](#assignment-2---research-track-2-jupyter-notebook)

MSc Robotics Engineering at University of Genova

-------------------

# Robot Simulator description 
This ROS simulator encompasses the visualization of a mobile robot able to reach a desired target by avoiding obstacles. The obstacles are recognized by an onboard laser. 
The ROS package *assignment_2_2023* containing all the files for this simulator was obtained from this [repository](https://github.com/CarmineD8/assignment_2_2023.git)

### Visualization tools
* **RViz**: a ROS 3D visualization tool which allows the user to view the robot model as well as the output of its sensors (i.e. laser, camera). It allows to see what the robot "sees". In this case, it visualizes the robot, its motion on a grid surface and the output of the laser, i.e. the obstacles or walls detected by the robot's laser according to the laser's reach.
* **Gazebo**: a ROS 3D realistic simulation tool that serves as a physical simulator. It allows to see the robot as an external observer. In this case, it visualizes the robot, its motion and the world or arena where the robot navigates including the obstacles and walls in it.

### Package description
* **world**: Contains the .world file corresponding to the information to load the environment or arena where the robot will navigate in Gazebo.
* **urdf**: Contains files describing the control and structure of the robot including information regarding joints and links and their position, orientation, dynamics, among others.
* **scripts**: There are 3 already given Pyhton scripts, `wall_follow_service.py`, `bug_as.py` and `go_to_point_service.py`. These work together to make the robot move towards a goal sent by an action-client, making the robot head towards the goal but also follow obstacles until it can again heards the goal (surroung wall until there isn't wall anymore).
* **action**: Contains the .action file used by the action-server `bug_as.py`.
* **launch**: Contains the .launch file which executes the nodes (i.e. the .py scripts) as well as the simulation environments
* **config**: Contains .rviz files which store the configuration settings, layouts and displays used by RViz.

# Assignment 2 - Research Track 1: ROS Nodes
## Assignment description
The task carried out was to write and add to the previously described robot simulator the necessary scripts, message and service files for creating the following 3 nodes:
1. **Node (a)**: Create a node that implements an action-client, allowing the user to input in the terminal a target (x, y) or to cancel it. Use the feedback of the action-server to know when the target has been reached. The node also publishes the robot position and velocity as a custom message (x, y, vel_x, vel_z), by relying on the values published on the topic /odom.
2. **Node (b)**: Create a service node that, when called, returns the coordinates of the last target sent by the user. 
3. **Node (c)**: Create a service node that subscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed. The size of the averaging window is set as a parameter in the launch file.

As well as editing the .launch file to make these nodes run as well.

## Files developed
The developed files for the required nodes are the following:
* **scripts**: Folder containing Nodes scripts
  * `ac_Node_a.py`: Node (a) script
  * `srvNode_b.py`: Node (b) script
  * `srvNode_c.py`: Node (c) script
* **msg**: Folder with custom messages
  * `PosVel.msg`: Custom message for robot's position and velocity
* **srv**: Folder with custom service messages
  * `GetDistSpeed.srv`: Service for replying the distance of the robot from the target and the robot’s average speed
  * `GetLastTarget.srv`: Service for replying the coordinates of last target sent
  
## Installing and Running
The simlator and its files require Ubuntu 20.04, Python 3 (already installed within Ubuntu 20.04) and ROS Noetic. If ROS Noetic is not yet installed in your Ubuntu system follow the steps found in the [Ubuntu install of ROS Noetic ](http://wiki.ros.org/noetic/Installation/Ubuntu) making sure to install the *Desktop-full* version.

To download the simulator, install git and clone this repository in the *src* folder of your *ROS workspace*. Create it if you don't have one by following the [Create a Workspace tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
git clone https://github.com/AmiQuijano/ros_robot_sim_pkg.git
```
Once all the dependencies are installed,

1. Open a terminal and start ROS
```
roscore
```
2. Open another terminal and build the workspace
```
catkin_make
```
3. Launch the simulation with the roslaunch command to test the developed nodes
```
roslaunch ros_robot_sim_pkg ros_robot_sim.launch
```
## Assignment solution 
### Node (a)
The code in the file `acNode_a.py` contains the following functions and main explained as follows:

Inside the class `ActionClient` the implemented functions are:

#### `__init__(self)`
This function initialized the ROS action-client, subscriber, publisher and message as required

Arguments:
* `self`: Instance of the class `ActionClient`.

Pseudocode:
```
Function __initi__(self):
    Initialize action-client the /reaching_goal server with the PlanningAction action type
    Wait for the action-server to start
    Initialize goal variable as a PlanningGoal message type to send to the action-server
    Initialize subscriber for /odom topic with Odometry message type and odom_callback function
    Initialize publisher named to publish in /PosVel topic the message of PosVel type
```
#### `get_user_input(self)`
This function asks the user for a goal coordinate or allows user to cancel current goal. 

Arguments:
* `self`: Instance of the class `ActionClient`.

Pseudocode:
```
Function get_user_input(self):
    Print prompt for user input to enter goal coordinates or cancel the goal
    Begin infinite loop:
        If there is a user input:
            If 'c', cancel the current goal
        Else:
            Try to:
                Get goal coordinates from user input as two floats
                Set goal coordinates in the goal message
                Send the goal to the action-server with feedback_callback function
            Except:
                Display error message for invalid input
```
#### `odom_callback(self, msg)`
This function is a subscriber callback function that saves the x and y position and the x and z velocity from the `/odom` topic in a custom message and publishes it

Arguments:
* `self`: Instance of the class `ActionClient`.
* `msg`: message received in the `/odom` topic

Pseudocode:
```
Function odom_callback(self, msg):
    Initialize custom_msg variable as a PosVel message type
    Assign to custom_msg the x position, y position, the linear x velocity and angular z velocity from /odom
    Publish the custom message
```
#### `feedback_callback(self, feedback)`
This function is an action callback function that displays the feedback from the action-server 

Arguments:
* `self`: Instance of the class `ActionClient`.
* `feedback`: feedback received from the action-server

Pseudocode:
```
Funtion feedback_callback(self, feedback):
  Print feedback from the action-server
```
In the main,
#### `__main__`
Pseudocode:
```
Main section:
    Try to:
        Initialize ROS action-client node
        Create an object for the class ActionClient
        Wait for 2 seconds
        Display the prompt for the user to input target or cancel by calling get_user_input function
        Keep the script running until the node is shut down
    Except:
        Interrupt the ROS process
```
For running this node, the custom message `PosVel.msg` was created which contains 4 components
* **float64 x**: x-coordinate of the robot's position
* **float64 y**: y-coordinate of the robot's position
* **float64 vel_x**: linear velocity of the robot in the x-axis
* **float64 vel_z**: angular velocity of the robot in the z-axis

### Node (b)
The code in the file `srvNode_b.py` contains the following functions and main explained as follows:

Inside the class `GetLastTargetService` the implemented functions are:

#### `__init__(self)`
This function initializes the ROS service for getting the last target, the subscriber to the `/reaching_goal/goal` topic where the goal coordinates are sent and the variable `last_target` where the x and y coordinates of the last target sent will be saved and sent as a reponse from the service-server.

Arguments:
* `self`: Instance of the class `GetLastTargetService`

#### `target_callback(self, msg)`
This function is a subscriber callback function to update the `last_target` variable when a new target is received in the `/reaching_goal/goal` topic.

Arguments:
* `self`: Instance of the class `GetLastTargetService`
* `msg`: Message received in the `/reaching_goal/goal` topic

#### `handle_get_last_target(self, req)`
This function is a service callback function to handle requests for the last target coordinates. It sends back a responde the last target coordinates and a boolean to state if the request was handled successfully or not.

Arguments:
* `self`: Instance of the class `GetLastTargetService`
* `req`: Request message sent to the server

In the main,
#### `__main__`
In the main the node is initialized and the class `GetLastTargetService` is instantiated.

For running this node, the custom service message `GetLastTarget.srv` was created which contains no request message and 3 responses:
* **float64 x**: x-coordinate of the last target
* **float64 y**: y-coordinate of the last target
* **bool success**: Indicates if the request was successful

### Node (c)
The code in the file `srvNode_c.py` contains the following functions and main explained as follows:

Inside the class `GetDistSpeedService` the implemented functions are:

#### `__init__(self)`
This function initializes the ROS service for getting the distance from the target and the average velocity, the subscriber to the `/reaching_goal/goal` topic where the goal coordinates are sent, the subscriber to the `/PosVel` topic where the (x, y, vel_x, vel_y) custom messages arrive, and the variables `last_target`, `posvel` of PosVel message type, `window size` obtained from the parameter, and the queue variables to store the last N velocities.

Arguments:
* `self`: Instance of the class `GetLastTargetService`.

#### `target_callback(self, msg)`
This function is a subscriber callback function to update the `last_target` variable when a new target is received in the `/reaching_goal/goal` topic.

Arguments:
* `self`: Instance of the class `GetLastTargetService`
* `msg`: Message received in the `/reaching_goal/goal` topic

#### `posvel_callback(self, msg)`
This function is a subscriber callback function to update the robot's position and velocity variables when a new message is received in the /PosVel topic

Arguments:
* `self`: Instance of the class `GetLastTargetService`
* `msg`: Message received in the `/PosVel` topic

#### `handle_get_dist_speed(self, req)`
This function is a service callback function to handle requests to send as a response the robot's distance from the target and average speed.

Arguments:
* `self`: Instance of the class `GetLastTargetService`
* `req`: Request message sent to the server

In the main,
#### `__main__`
In the main the node is initialized and the class `GetDistSpeedService` is instantiated.

For running this node, the custom service message `GetDistSpeed.srv` was created which contains no request message and 7 responses:
* **float64 dist_x**: distance in x-axis from the robot's position to the target
* **float64 dist_y**: distance in y-axis from the robot's position to the target
* **float64 dist_eucl**: Euclidean distance from the robot's position to the target
* **bool success_dist**: Indicates if the request was successful for calculating the distance
* **float64 avg_vel_x**: average linear speed in x-axis
* **float64 avg_vel_z**: average angular speed in z-axis
* **bool success_avg_vel**: Indicates if the request was successful for calculating the velocity

### Launch file
The launch file was changed adding the following:
* Launch of **Node (a)** with the script `acNode_a.py` with screen output.
* Launch of **Node (b)** with the script `srvNode_b.py` with screen output.
* Launch of **Node (c)** with the script `srvNode_c.py` with screen output.
* Parameter `window size` to set the number of last velocity readings to averaging in **Node (c)**. Currently set as 100.

### Visualization/Confirmation of nodes functionality
To check that the nodes work optimally or to visualize the messages created:
* **Feedback**: `actual_pose` and `status` feedbacks are visible automatically in the same terminal where the package was launched. To confirm the values are correct, in another terminal run
```
$ rostopic echo /reaching_goal/feedback
```
* **(x, y, vel_x, vel_y) custom messages**: To visualize the custom messages `PosVel` published in the topic `/PosVel`, open another terminal and run
```
$ rostopic echo /PosVel
```
To confirm that these values are correctly obtained from the `/odom` topic, in another terminal run
```
$ rostopic echo /odom
```
* **Last target**: To visualize the last target sent by the user it is necessary to call the service `/get_last_target` in order to get the responds. For this, open a new terminal and run
```
$ rosservice call /get_last_target
```
The user itself can confirm if the displayed target coordinate is indeed the last one inputted.
* **Distance from target and average speed**: To visualize the distance from the target and average speed it is necessary to call the service `/get_dist_speed` in order to get the response. For this, open a new terminal and run
```
$ rosservice call /get_dist_speed
```

### Possible improvements
The solution implemented fullfills the requirements stated by the *Assignment description*. However, there are improvement opportunities to make the simulator clearer for the user or make it have more functionalities in case of user mistake:

* **Goal mark in Gazebo**: It would be useful to have a visualization of the goal that the user has inputted in order to have a clear idea of where the robot is heading or whether or not it is heading/arriving to the goal.
* **Error message for unreachable goals**: It would be more user-friendly and time-saving to have an error message in the terminal if the user inputs a goal coordinate that is unreachable by the robot given the used world's limits (in case the world is an enclosed arena like the one used in this simulation). This would avoid having the user cancel the goal until realizing that the robot cannot reach it. The cancel option should still be kept in case the user wants to cancel the current inputted goal.

# Assignment 1 - Research Track 2: Software Documentation
## Assignment description
The task consists on creating the documentation of the python scripts written for all three ROS nodes of [Assignment 2 - Research Track 2: ROS nodes]. The chosen tool was Sphinx given that the scripts were developed in Python 3.

## Documentation
Find here the documentation of the 3 ROS nodes:
[https://amiquijano.github.io/ros_robot_sim_pkg/] 

## Files developed
The developed files for the required task are the following:
* **scripts**: Folder containing documented Nodes scripts
  * `ac_Node_a.py`: Node (a) script
  * `srvNode_b.py`: Node (b) script
  * `srvNode_c.py`: Node (c) script
* **sphinx_docs**: Folder containing the files generated by compiling the HTML (**build** folder, **source** folder, `Makefile`, and `make.bat`)
* **docs**: Folder containing all the folders and files inside the **html** folder located at **sphinx_docs > build > html**. This folder is necessary for Github to give the html website.

# Assignment 2 - Research Track 2: Jupyter Notebook
## Assignment description 
The task consists on developing a Jupyter Notebook which has:
1. An interface to assign the coordinates of the goal or to cancel the current goal
2. The position of the robot
3. The targets that have been sent, reached and cancelled in the environment
4. A plot for the number of reached and cancelled goals
5. A plot with the robot and target's position in the environment

## Jupyter Notebook
Find the Jupyter Notebook in the following following folder:

* **jupyter_scripts**: Folder with the Jupyter Notebook file
  * `jupyter_node.ipynb`: Jupyter Notebook file
  
## Running
To download the simulator, install git and clone this repository in the *src* folder of your *ROS workspace*. Create it if you don't have one by following the [Create a Workspace tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
git clone https://github.com/AmiQuijano/ros_robot_sim_pkg.git
```
Once all the dependencies are installed,

1. Open a terminal and start ROS
```
roscore
```
2. Open another terminal and build the workspace
```
catkin_make
```
3. Launch the simulation with the roslaunch command
```
roslaunch ros_robot_sim_pkg ros_robot_sim.launch
```
4. Open Jupyter notebook in the directory where you have downloaded the .ipynb file, typically with
```
jupyter notebook
```
5. Once inside Jupyter Notebook enter the `jupyter_node.ipynb` file and click on Kernel > Restart & Run All

**_Note_**: The Jupyter node does not work with custom messages. Therefore, it can be ran without downloading the whole package if you already have it downloaded (i.e. just downloading and running the Jupyter Notebook will work if the robot simulator is already installed).

