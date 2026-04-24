# RT2 Assignment 1: Containerized Navigation Stack

**Done by** : Bouhraoua Hani . <br>
**ID** : 8314923 . <br>
**Course:** Research Track 2 (Master's in Robotics Engineering). <br>
**University of genoa** .

## Project Overview
This repository contains a complete ROS 2 navigation stack built entirely using C++ components. The objective of this assignment is to dynamically load an Action Server and an Action Client (User Interface) into the same `component_container` to achieve highly efficient, zero-copy intra-process communication. 

The system commands a robot in a Gazebo simulation to navigate to user-defined coordinates (x, y, theta) using proportional control kinematics, while allowing for asynchronous goal cancellation.


## Final directory architecture of entire ROS_2 workspace.

```
~/ros2-navigation-action-server-main/

├── README.md                
└── src/
    ├── bme_gazebo_sensors/  <-- (The professor's simulation repository to be cloned)
    │   ├── launch/
    │   │   └── spawn_robot_ex.launch.py
    │   ├── models/
    │   ├── rviz/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── nav_interfaces/      <-- (Phase 1: custom action blueprint)
    │   ├── action/
    │   │   └── Navigate.action
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── nav_project/         <-- (Phases 2-5: main C++ logic)
        ├── launch/
        │   └── navigation.launch.py              <-- (The background launch script)
        ├── src/
        │   ├── navigation_server_component.cpp   <-- (The math and control loop)
        │   └── ui_client_component.cpp           <-- (The multi-threaded terminal menu)
        ├── CMakeLists.txt                        <-- (Configured for shared libraries)
        └── package.xml                           <-- (Includes tf2 dependency)
```


## Implementation Phases

The project architecture was developed systematically across five phases:

* **Phase 1: Custom Action Interface (`nav_interfaces`)**
    * Created a standalone package to define the communication protocol.
    * Defined `Maps.action` comprising the Goal (target x, y, theta), Feedback (current distance and angle errors), and Result (success state and final coordinates).
* **Phase 2: Navigation Action Server Component**
    * Developed `navigation_server_component.cpp` as a ROS 2 node inheriting from `rclcpp::Node`.
    * Implemented an odometry subscriber (`/odom`) to track current pose using `tf2` quaternion-to-Euler conversions.
    * Engineered a proportional control loop running in an isolated thread to calculate heading and distance errors, publishing velocities to `/cmd_vel` while continuously monitoring for cancellation requests.
* **Phase 3: User Interface Action Client Component**
    * Developed `ui_client_component.cpp` to send goals and handle server callbacks.
    * Utilized C++ `std::thread` to isolate standard terminal input (`std::cin`). This ensures the ROS 2 executor remains unblocked to process incoming feedback while the user navigates the interactive menu.
* **Phase 4: Component Containerization**
    * Configured `CMakeLists.txt` to compile both the server and client as shared libraries (`.so` plugins).
    * Registered the plugins using the `RCLCPP_COMPONENTS_REGISTER_NODE` macro, enabling dynamic loading at runtime.
* **Phase 5: Deployment and Execution**
    * Prepared execution methodologies utilizing both Python launch scripts (`ComposableNodeContainer`) and manual terminal loading to accommodate interactive standard input requirements.


## Prerequisites

Before compiling the workspace, the specific simulation environment must be downloaded into the `src` directory.

```bash
cd ~/Desktop/ros2-navigation-action-server-main/src
git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors.git
```

## How to Build

Navigate to the root of the workspace and compile the packages using `colcon`:

```bash
cd ~/Desktop/ros2-navigation-action-server-main
colcon build
source install/setup.bash
```

## How to Run

To properly interact with the C++ `std::cin` prompts while utilizing component containers, launch the system across three separate terminals. Ensure you run `source install/setup.bash` in every new terminal.

**Terminal 1: Start the Gazebo & RViz Simulation**
This launches the walled arena and the robot.
```bash
ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
```

**Terminal 2: Initialize the Component Container**
This terminal will host the shared process and act as the interactive user interface.
```bash
ros2 run rclcpp_components component_container
```

**Terminal 3: Load the Plugins**
Execute these commands sequentially to inject the custom C++ plugins into the running container from Terminal 2.
```bash
ros2 component load /ComponentManager nav_project nav_project::NavigationServer
ros2 component load /ComponentManager nav_project nav_project::UIClient
```

### Usage
Once the plugins are loaded from Terminal 3, return to **Terminal 2**. 
1. The terminal will display `=== Robot Navigation Menu ===`.
2. Enter the target `X`, `Y`, and `Theta` coordinates when prompted.
3. The robot will begin navigating in Gazebo (Terminal 1).
4. To halt the robot mid-navigation, type `c` and press Enter in Terminal 2 to trigger the cancellation sequence.
