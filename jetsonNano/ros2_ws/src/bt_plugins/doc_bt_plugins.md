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
The make file let's us add a new library executable (.so) that can be found by the behavior tree when loading. The library executables will be compiled to the directory `<your_workspace>/install/bt_plugins/lib` In the CMake file we also have to add all the new dependencies for our new plugins as well as in the package .xml, specially if new types of messages have to be added. 

## Adding a new nav2_behavior_tree plugin
You need to write 2 new files and a add some lines to the CMake file to add a new plugin. Also, you will have to add your new plugin in the file `car_description/config/nav2_params` where we set the configuration. You just have to add the name of the plugin as it is written on your CMake file. 
### Header Files
Copy one the files present as an example. You first have to decide which type of node you want for your derived class (action, condition, some special nav2 node, etc.) Then define the constructor, the tick function (which might be called `tick()`or `on_tick()`depending on the type) and any other function you may want. You also have to define all the class variables.  
Do not forget that you also need to define the porst of your node. You can have a node with no input or output ports as seen on the BipAction node. 
### Source Files
Here you write the code you want to execute for all the functions you defined in the .hpp. Don't forget to initialize the node variable if you want to communicate with the rest of ROS. Look at the nodes here and on nav2 (in the folder nav2_behavior_tree) for inspiration. 

Don't forget to add the function extern `"C"
{void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)` at the end to register your behavior tree. The name of the function might change depending on the version of nav2


### Add to CMake
To compile your new plugin you have to add the following lines to the CMake file :

```
add_library(<new_plugin_name> SHARED plugins/<name_cpp_file>)

ament_target_dependencies(<new_plugin_name>
        ${dependencies}
        )

install(TARGETS
        bt_plugin_new_bip_action_node
        bt_plugin_new_wait_package_node
        <new_plugin_name>
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )
```
The name `<new_plugin_name>` is the name you have to add to the configuration .yaml for the bt_navigator configuration. 



## Additional useful information

### Loading and registering plugins in the nav2_behavior_tree
You have to make sure that your library is loaded. For this the easiest way is to add a `std::cerr` in you function. This way a message will be printed to the screen. 
If you have an error like `cannot load <Symbol_name` this means it was looking for something and did not find it. You can use the `nm` command on the terminal to analyse the .so and see if its really missing or if the name is not what is looking for. If the symbol is missing it might mean that the compiler is doing some optimization and not compiling your function. It was the case for us and that is why if you look at the actual documentation, you will that we do not use the same macro to register our node to the behavior tree. We have done this because we encountered this problem. 

### Implementation of a new tree on the jetson
When you implement a new tree for nav2 check the version of the downloaded trees because they do not match up with the last ones (in our case it did not at least). Even though we had downloaded everything only 3 months ago. This meant that the default trees on the last commit on GitHub could not be used on our system. When writing a new one or design it with Groot, check that you are using the right version of the nodes. 

