# ME495 Sensing, Navigation and Machine Learning For Robotics
* Chenyu Zhu
* Winter 2025
# Package List
This repository consists of several ROS packages
- Nuturtle Description - describle and show multiple turtlerobots on rviz


# Nuturtle  Description
URDF files for Nuturtle RapidBot
* `ros2 launch nuturtle_description load_one.launch.xml` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)

# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.xml --show-args`
  ```bash
  Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Open rviz
        (default: 'true')

    'use_jsp':
        Use joint state publisher
        (default: 'true')

    'color':
        One of: ['red', 'green', 'blue', 'purple']
        (default: 'purple')
  ```
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`
  ```bash
  Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Open in rviz
        (default: 'true')

    'use_jsp':
        Use joint state publisher
        (default: 'true')

    'world_frame':
        Select world frame
        (default: 'nusim/world')

    'color':
        One of: ['red', 'green', 'blue', 'purple']
        (default: 'purple')
  ```