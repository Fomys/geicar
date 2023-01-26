# General organization

Our team focused on the navigation2 stack, so you will found here a lot of resources we consider as important and needed to understand how to work with it.

## General ressources

To bring up you can read the navigation2 part of the Olivier Stasse's course. It is not complete, but you will understand the basics.

You can found the official tutorial here, the documentation is not complete and very messy, but it is the only official ressource.

## What are the differents nodes

Navigation2 comes with a lot of modules, and they need each other for working (otherwise it will not be fun).

### The lifecycle manager

The lifcycle manager is used by ALL of the modules, you must keep it. It ensure that all Nav2 services are lunched in the correct order. In fact its algorithm is realy simple: you give a list of nodes, and it launch them one by one, in the order of the list. It means that if the node "planner" need the node "map", you MUST put the node "map" before the node "planner"

### Robot state publisher, joint state publisher

Those two nodes are used to publish the model of the robot in ROS. The first publish all the components and the second publish all the joints between them (for example: "from the main body, the lidar is 24 cm up and rotated by 45°")

### RViz

Rviz is not needed to make the car working, but it is the visualization tool for everything in ros, and is very useful when dealing with a map, a robot representation and the navigation.

### Laser scan matcher

The laser scan matcher node is a node which create a LIDAR odometry. In our project it is the only source of odometry and it will be a good idea to merge it with an IMU and/or wheel odometry.

### Lidar

The lidar node is only here to transform the information from the lidar to a ros topic.

### System check nodes

All of the system check nodes are used to create a report of the different components status. We didn't tried to keep it working, so it may not work.

### Mapserver

This node has the role to publish a map in ROS. It can change the map at the runtime by calling a service.

### Planner server

The planner server has the role to create the path from the car localisation to the destination. You can check all the comments of the configuration file to understand what it can do. The current config allows you to create a path feasable by the car.

### Controller

The controller has the role to transform path to real instructions. You can also see in the config file a detailled description of each parameters. A good amelioration will be to create a controller which is aware of the response speed of the direction. Actually, the controller thinks that the rotation of the wheels is instant, but in fact it can takes five seconds.

### AMCL, SLAM localization

AMCL is a localisation node, as you can see, it is not started. That because we didn't manage to configure it properly. This is the only algorithm we found for ROS2, but you can search and implement your own based on those from ROS1.

### Smoother

The smoother take a path from the planner, and smooth it to create curves feasable by our car.

### Behaviors servers

Please refer to the next section

## How everything works in navigation2


In navigation 2, there is a lot of complex concepts: the way to locate the car, the way to create and execute a path and the way to manage the behaviors.

### Localisation of the car

This step is important and the goal is to create a transformation "map to base_link" which locate the car in the right place on the map

In an ideal world, to locate the car you just have to :
- Take a reference, often it will be the map
- Compute the localisation of the car on the map.

In reality, the localisation of the car is very complex to compute and can not be done fast enough to get a real time position. That why you have to use a third step: the odometry.

In navigation2, there is two main transformations:
- map to odom, which is the offset between the map and the "starting point" of the odometry. If the odometry is perfect, this value will remain constant, but as the odometry is never perfect, someone has to adapt this offset to create the correct position. That is the role of AMCL for example. As it was not working in our project, we publish manually a static position here, and only rely on odometry. It was correct but not enough to enter the elevator.
- odom to baselink, which is the computed odometry from the start of the robot. That the role of the lidar odometry in our project.

### Path and execution

To compute the path, the localisation of the car must be functional ! The path is computed by the planner and executed by the executor (what a surprise !). You can read the configuration file to understand all the configuration.

There is a small subltility here, the executor publish on the topic cmd_vel, and we have to transform the order to the correct kind on our car. That is done by the node {TODO @madeline}. 

### Behaviors
The nav2 stack needs to decide what to do when it receives a new target position. What it should do next, at what frequency, etc. Nav2 implements something called a behavior tree which does exactly that. When a new position is set, the first step is to compute a path then to follow it. However, sometimes things don't go as planned. We can add more behavior to our tree to take into accounts this cases to try to solve. 

The default tree, which is loaded when not a specific one is given corresponds to one called `navigate_to_pose_w_replanning_and_recovery.xml` You can find a more detailed description of this tree on the documentation website of  navigation2. 

We can also modify this tree and add more nodes. You can read how a behavior tree works on the `BehaviorTree3.CPP` GitHub. Navigation2 is based on this library to implement its tree and some specific nodes. We can also code our own nodes (in our case for example to make a sound when a destination is reached). 

For further information refer to the README on the directory `jetsonNano/ros2_ws/src/bt_plugins` as well as the comments on the code. You can also see some examples of custom trees on the folder `jetsonNano/ros2_ws/src/car_description/config`. They are not fully functionnal but they will give an idea of how they are done. You can visualize the trees using a tool called Groot which you can download [here](https://github.com/BehaviorTree/Groot). 



