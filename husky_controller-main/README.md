# ROB314

The project has this module but also a main [pipeline](https://github.com/arthur-ruback/pc_pipeline).

### Instalation using Catkin

This repository shall be cloned into [your catktin workspace]/src.

```
git clone https://github.com/arthur-ruback/husky_controller
cd ..
catkin_make 
```

### Use

##### Parameters configuration

The config file defined in *config/params.yaml* contains the definition of several parameters.

- kd *[float]* : proportional gain in relation to linear speed

- kw *[float]* : proportional gain in relation to angular speed (rotation)

- target_distance *[float]* : goal distance to target

##### Subscribed topics:
- /cluster_center *[sensor_msgs/PointStamped]* : input from *pc_pipeline*

##### Published topics:
- /cmd_vel *[geometry_msgs/Twist]* : output to low level husky controller

##### Execution

The standard execution of this module is coupled with *pc_pipeline* but it can be executed by itself using:

```
roslaunch husky_controller controller.launch
```

