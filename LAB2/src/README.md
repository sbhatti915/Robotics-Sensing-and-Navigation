Install tf and tf2 for use in the imu driver from the link below:

https://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2

## Usage
clone the repository

move to LAB2 directory
```
catkin_make

source devel/setup.bash

rosrun imu_ros_driver imu_driver.py _port:="<path of device being used>"
```

Default of _port is /dev/pts/1