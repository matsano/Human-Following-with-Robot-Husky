# Person-Following-with-Robot-Husky
Project for the course Hardware and Software Architecture for Robotics

## 📋 Description
The idea behind this project is to transform a robot into an autonomous system so that it can move around without the user needing to control it manually. This technology could be very useful in the field of security, particularly in the military, where a robot for military use could follow an agent without the latter having to control it manually. In addition, this new technology could also be useful for transporting and storing goods in the industrial sector.

To facilitate the implementation and development of the autonomous system with the Husky robot, the Robot Operating System (ROS) was used. ROS is a set of open source software tools offering a variety of resources and libraries to facilitate the development of complex robotic systems. Implementing this project using ROS enabled the efficient integration of various components, such as the LiDAR sensor and the robot control system.

The project was divided into two parts: identification ([pipeline](https://github.com/matsano/Human-Following-with-Robot-Husky/tree/main/pc_pipeline-main)) and control ([controller](https://github.com/matsano/Human-Following-with-Robot-Husky/tree/main/husky_controller-main)).
The main aim of the identification part was to make the robot identify a person:
1. LiDAR operation
2. Point cloud processing
3. Clustering
4. Identification of individuals

The main objective of this part is to implement a controller that controls the robot so that it can follow a person:
1. Husky robot operation
2. Controller implementation



## 🛠️ Demonstrations

### Final result

![alt text](Demos/Final_result.mp4)

### Intermediary result

![alt text](Demos/Intermediary_result.mp4)

### Human identification and following ([pipeline](https://github.com/matsano/Human-Following-with-Robot-Husky/tree/main/pc_pipeline-main))

![alt text](Demos/Human_identification_following.mp4)

### LIDAR operation

![alt text](Demos/LIDAR.mp4)



## ✒️ Authors

- Arthur COELHO RUBACK:
    - [![GitHub](https://i.stack.imgur.com/tskMh.png) GitHub](https://github.com/arthur-ruback)

- Matheus SANTOS SANO:
    - [![GitHub](https://i.stack.imgur.com/tskMh.png) GitHub](https://github.com/matsano)

We would like to thank our professorS Mr. Emmanuel BATTESTI and Thibault TORALBA for the knowledge taught in course which were essential for the realization of this project.
