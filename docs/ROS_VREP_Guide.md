Using TAPIR with ROS and V-REP
==========================================================================

--------------------------------------------------------------------------
ROS
--------------------------------------------------------------------------

Robot Operating System (ROS) is an open-source software framework designed for supporting collaboration and code re-usability in robotics research and development. ROS provides process management, message passing, hardware abstraction, and more. Users unfamiliar with ROS should go through the tutorials available on the ROS website: http://wiki.ros.org/ROS/Tutorials. This will provide an understanding of basic ROS concepts such as publishing/subscribing messages, custom message generation, the ROS build system called catkin, and so on.


--------------------------------------------------------------------------
V-REP
--------------------------------------------------------------------------

V-REP (Virtual Robot Experimentation Platform) is a versatile general-purpose robot simulation framework. V-REP provides realistic physics and sensor simulation, and is able to publish and subscribe to ROS topics. V-REP features a ”drag and drop” interface and allows embedded Lua scripts to set up a simulation scenario. The basic controls are: holding left click pans the view while holding right click rotates it. Middle clicking brings up the window for repositioning objects. Futher help can be found in the V-REP user manual, available online: http://www.coppeliarobotics.com/helpFiles. TAPIR V-REP examples are viewable in this [video](https://www.youtube.com/watch?v=5fdEgwSxGMg). Although the provided example uses V-REP, you are free to use TAPIR with other simulators or hardware.


--------------------------------------------------------------------------
TapirNode Template
--------------------------------------------------------------------------

TAPIR provides a class template for implementation with ROS. It is contained in a header file in [TapirNode]. The key virtual functions left to you to implement are getObservation, which returns an observation, and applyAction, which applies a given action. For example the getObservation function will parse sensor data into a POMDP model observation, and the applyAction will publish the actuator commands. These functions must be implemented unless the you intend to set internalSimulation\_ to true, in which case the [Simulator] class is used to generate the results. The [TapirNode] is initialised by calling initialiseBase, which loads the runtime configuration file, initialises TAPIR and generates the starting policy. The bulk of TAPIR processing is handled by a call to the processTapir function. This function follows this procedure:
1. Get observation
2. Replenish particles
3. Update current belief with observation
4. Prune the tree (optional)
5. Get the best action and apply it
6. Improve the policy
Improving the policy generally takes more time than the other steps, therefore it may be desirable to be able to run this step while the action is being applied, e.g. by having the actuator controlled by a separate ROS node. It is up to you how the processTapir function is called, e.g. a ROS timer loop could be set up to call it at regular  intervals, or the function can be called as soon as the action is completed. 


--------------------------------------------------------------------------
Tag Example
--------------------------------------------------------------------------

TAPIR has an implementation of the Tag problem, where the robot's goal is to tag (move to the same cell) as the human target. The problem is relatively simple in that the map is discretised into a grid, and the robot only gets an observation on the target’s position if it is in the same cell. As described in [the Quick Start README][../README.md], the tag problem can be viewed through a simple command line interface, but the problem is also implemented in ROS as an example.

### Initialise

The Tag ROS node is defined in [TagNode]. Before implementation in ROS, the problem model, observations, actions, etc. must be defined as described in [the guide on implementing a new model][../docs/Making_A_New_Model.md]. [TagNode] inherits from [TapirNode], which is a templated class, so the TagNode class declaration appears as 

    class TagNode : public TapirNode<TagOptions, TagModel, TagObservation, TagAction> {

The initialise function initialises TAPIR by calling initialiseBase, which is defined in [TapirNode]. The topics Tag node publishes/subscribes to are then registered. For example, the robot goal is published using the ROS point message. It is advertised as:

    robotPub_ = node_->advertise<geometry_msgs::Point>("robot_goal", 1);

Here the topic name is set to “robot_goal” and the maximum number of messages to hold in queue is set to 1. The callback function for a subscriber is called when  ROS “spins” and if a message has been received. The subscriber cannot be set as 

    selectSub_ = node_->subscribe("/selected_cells", 1,selectCallback);

(This is similar to the ROS tutorial). Instead, it is set as 

    selectSub_ = node_->subscribe("/selected_cells", 1,	&TagNode::selectCallback, this);

Since selectCallback is a member function. In this example, a ROS timer is created to call timerCallback every few seconds:

    timer_ = node_->createTimer(timerDuration, &TagNode::timerCallback, this);

Note that this is not a multi-threaded application, therefore callback functions are handled one after the other, so that subscriber callbacks never interrupt the timer callback.

### Timer Loop

In timerCallback, handleChanges (defined in [TapirNode]) is called with parameter changes\_, which is a vector of model changes to be applied. The Tag V-REP scenario was set up to publish the list of obstacle blocks selected by the user as a string message, and these are processed in selectCallback into the model changes. In this Tag example, which is not really an accurate reflection of a practical application, the human movement is not controlled by an external model or actual human. Instead the next cell the human moves to is generated by calling the [TagModel] sampleNextOpponentPosition function, and then published. The V-REP side then listens to this and moves the simulated human accordingly.

After processing the model changes, the timer callback calls the processTapir function, which is defined in [TapirNode] and handles the TAPIR solver process. processTapir calls getObservation, which  forms an observation based on data published by V-REP, and then calls applyAction, which calculates the next robot position from taking the action and publishes it over ROS. The map and belief about target position is then published over ROS. Both are published in a string message as comma separated values.

### Main Function

Since we intend to register [TagNode] as a ROS executable, a main function is required. The main function calls ros::init, creates a [TagNode], calls initialise, then calls ros::spin. The spin function blocks until ROS is shutdown. 

### Compiling

ROS uses a build system called catkin, which extends CMake. Source files and package dependencies are listed in CMakeLists.txt. The necessary source files (solver, Tag model, etc) are listed as libraries. The TagNode.cpp is listed as an executable in the line

    add_executable(tag_node src/problems/tag/ros/TagNode.cpp)

This allows it to be started through ROS terminal commands (i.e. rosrun tapir tag\_node). A launch file in the abt/launch folder also allows it to be started by roslaunch tapir tag.launch. Note: if these commands cannot be run even after a successful compilation, you may need to source the relevant catkin workspace by navigating to the catkin\_ws/devel folder and calling source setup.bash. The program can be compiled by navigating to the catkin workspace folder and calling catkin_make.

### Running

TAPIR provides a bash script for launching the tag example in one script, as described in [the Quick Start README][../README.md]. However, it is often preferable to run the programs manually. The process is as follows:
1. Start a ROS server by running roscore
2. Start V-REP by navigating to the V-REP folder and running ./vrep.sh Then open the scenario file tag.ttt located in abt/problems/tag/vrep_scenes. Note that V-REP must be started after roscore, otherwise the ROS plugins for V-REP will not load. 
3. Start tag by calling rosrun tapir tag_node	



[TapirNode]: ../src/problems/shared/ros/TapirNode.hpp
[Simulator]: ../src/solver/Simulator.hpp
[TagNode]: ../src/problems/tag/ros/TagNode.hpp
[TagModel]: ../src/problems/tag/TagModel.cpp
[../README.md]: ../README.md
[../docs/Making_A_New_Model.md]: ../docs/Making_A_New_Model.md
