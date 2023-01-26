# ROS Package bt_plugins

## General Description
This package has been developped to implement new plugins for the nav2 behavior tree. The nav2 behavior tree is used by the nav2 navigation stack to decide on its behavior. The standard tree used is `navigate_to_pose_w_replanning_and_recovery.xml` which can be found on the jetson on the directory `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/` along other standard behavior trees. 

The reason this package has been developed is to be able to add more functionalities to behavior trees to make a custom one. In the case of our project, we wanted to design a behavior tree that made a sound when the goal set at the beginning has been reach as well as detect when a package has been dropped. 
A description of our one almost functional custom tree is described and explained on th GitHub project on the folder  [`/jetsonNano/ros2_ws/src/car_description/config`](./../car_description/config). 

## The plugins
### BipAction 
**Action Node** : This plugin will just turn on the buzzer connected on the RaspberryPi. It should get the actual state and inverse it.

**What works** :  The publisher does work and the first time the node is called it makes a sound. When we put it in a behavior tree, the plugin is correctly loaded and returns succes.

**Problems** : The subscriber to the topic does not work properly and the callback function does not get the state.

### WaitPackage
**Condition Node** : This plugin should check the state of the topic `/detect_package` to get the state of the micro switch on the car. It has an input port called `fetch_drop` where we can set if what we want to verify, if its either true or false. If it matches it returns SUCCESS, if does not it returns FAILURE. (A condition node cannot return RUNNING). Should be used in conjunction with a ReactiveFallback control Node and a Wait action node. 

**What works** :  The plugin is correctly loaded and enters the tick() function. 

**Problems** : The subscriber to the topic does not work properly and the callback function does not get the state. It is never called. An idea might be to add the function `config().blackboard->get<interfaces::msg::Package>("detect_package", drop);` on the `tick()` function to get the state. Instead of declaring a subscriber. 

## Building the package
### CLI command
To build the package (alone) we use the regular colcon command :
```
colcon build --packages-select bt_plugins
```
It is better to build the plugins before compiling everything else but should work fine even if it is not the case. 
### MakeFile

## Adding a new nav2_behavior_tree plugin
### Header Files
### Source Files
### Add to CMake

## Additionnal useful infprmation

### Loading and registering plugins in the nav2_behavior_tree
### Implementation of a new tree on the jetson


