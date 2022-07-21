## Code Templates

This package contains standard templates for code components such as launch files, nodes, and so on

**Available Templates:**
 - [x] Package template
 - [x] Parameter config file
 - [x] Launchfile
 - [x] template_node.cpp script
 - [x] template.cpp script
 - [x] template.hpp script
 - [x] TemplateMsg.msg script
 - [x] CmakeLists.txt
 - [x] package.xml
 - [x] README.md template
 - [x] parameter type protection/ dynamic parameter usage examples
 - [x] ros2 bag api example for bag reading

### How to build and run the examples
**Package: template_package**
A package to show basic node contruction and provide templates for associated files and scripts

*Dependencies*
ROS2 Foxy
std_msgs

*Build*

```
colcon build --packages-select template_package
```

*Run*

```
source install/setup.bash
ros2 launch template_package template.launch.py
```

**Package: ros2_api_examples**
A package containing usage examples of the rosbag2 api, for reading, writing, and manipulating ros2 bags from within ros2 nodes

*Dependencies*
ROS2 Foxy
rosbag2
std_msgs
sensor_msgs

*Usage*
Update the bag_params.yaml file to reflect the bagfile and topic names that you'd like to use

In this slim example, only std_msgs::msg::Float32 and sensor_msgs::msg::PointCloud2 types are supported

This repository is set up to include a bags folder within it. This is not recommended for normal practice! Large bag files should not be pushed to git, as they tend to slow down pulls and pushes to the remote. To reference bag files in a different location, either add a relative path to the beginning of the bagfile parameter, or edit the launch file to use a different bag path with respect to the package share location (ros2_templates/install/rosbag2_api_examples/share/rosbag2_api_examples) 

*Build*

```
colcon build --symlink-install --packages-select rosbag2_api_examples
```

*Run*

```
source install/setup.bash
ros2 launch rosbag2_api_examples bag_reader.launch.py
```

 ### How to Use the Template Node to Make Your Own Package
 1. Make a copy of the template package inside the src/ of a desired ros workspace
 2. Change the package name
    - update the package folder name (change src/template_package to be src/your_package_name)
    - update the package name in CMakeLists.txt (line 2)
    - update the package name in package.xml (line 4)
 3. Add your own code
    - update the folders under include/ to reflect your node names
    - add your header files (.hpp's) to the appropriate include folders - use "agnostic_logic_class.hpp" as a guideline/basis 
    - add your source code (.cpp's) to the src/ folder (include node subfolders as desired) - use "agnostic_logic_class.cpp" and "ros_node.cpp" as a guideline/basis
    - create a launch file for your node(s) in the launch/ folder - use the template.launch.py as a guideline/basis
    - create a config/param file (.yaml) for your node(s) in the config/ folder - use the template_params.yaml as a guideline/basis
 4. Update CMakeLists.txt to build and install your node(s)
    - add your package dependencies at the top of the file (find_package()) - also add your dependencies to the package.xml file
    - Add/change the add_executable() line to reflect your node name and source files
    - Add/change the ament_target_dependencies() line to reflect your node name and package dependencies
    - Add your node name to the install call at the bottom of the file
    - Repeat the above 4 sub-steps for as many nodes as you'd like to create
 5. Build and run! Debug as necessary


## Suggested Code Standandards
- **variable_names** are lower-case with underscores for spaces
- **ClassNames** are camel case
- **functionOrMemberNames** begin lower case and continue in camel case, and are named to indicate their action or functionality

- ROS2 parameters are used to provide tunable/configurable settings for the node

**C++ Standards**

- .hpp files will be wrapped in #ifndef protection according to the following format:
```
#ifndef FILE_NAME_HPP
#define FILE_NAME_HPP
<code>
#endif //end define FILE_NAME_HPP
```

- Function documentation block (c++):
```
    /**
     * @function    <function_name>
     * @brief       <short description>
     * @param       <param_name> - <description>
     * @return      <return type>
     */
```

- Python standards: *to come*

## ROS2 Standards

- ros2 nodes are defined using at least three scripts (c++): 
    - a _node.cpp script which establishes a ros node class and handles all subscriptions, ros parameters, and publishing (this is the "ros" part of the node, or the "ros wrapper"), and which has an instance of your functionality class
    - a class.cpp script which contains ros-agnostic code that completes your actual functionality, usually through defining a class
    - a class.hpp script which includes the #includes and class declarations to accompany the class.cpp script

- Python ROS2 standards: *to come*

- ROS2 launchfiles are defined using the python method established by the ROS2 community

- ROS2 parameters are defined in .yaml config files separate from the python launchfile


## Documentation Standards
- Each pacakge will contain a README.md with at least:
    - List and brief description of nodes and launchfiles
    - Launch syntax
    - Package dependencies

- Each function should have a documentation block where declared and defined, except get/set functions and class constructors/destructors - the names/syntax of these functions are sufficient for documentation

- Each script will contain a comment-block header with 
    - The primary coder's name, email, and code creation date
    - Brief description of the script purpose or functionality
    - name of the file (ie class.hpp) - to confirm that the header belongs to the current file

- Variables will have a brief description where declared, including uints in [] if applicable (ie [m/s])

- Config/yaml files will have a brief description, including units in [] if applicable, of each parameter

## Gitlab Standards
- **run/** branches are ready to execute on the car, full-stack functional. A new /run branch should only be created when a new version of the software stack with significantly altered functionality or intentions is developed
- **dev-** branches are primary development branches for different sections of code (ie dev-perception, dev-control, and so on)
- **feature/** branches are created off of the dev- branches to develop specific functionalities
- Merge strategy: develop in feature/ branches, then merge **feature/ -> dev-**, then **dev- -> run/**
