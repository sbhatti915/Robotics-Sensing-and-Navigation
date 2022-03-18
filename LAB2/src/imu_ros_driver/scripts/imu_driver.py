#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler
import rosbag

if __name__ == '__main__':
    SENSOR1_NAME = "imu"
    SENSOR2_NAME = "magnet"
    rospy.init_node('imu_driver', anonymous=True)
    serial_port = rospy.get_param('~port','/dev/pts/1')
    serial_baud = rospy.get_param('~baudrate',115200)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    rospy.logdebug("Using IMU sensor on port "+ serial_port +" at "+ str(serial_baud))
    
    rospy.sleep(0.2)

    imu_pub = rospy.Publisher(SENSOR1_NAME,Imu, queue_size=10000)
    magnet_pub = rospy.Publisher(SENSOR2_NAME,MagneticField, queue_size=10000)
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing IMU Information.")
    
    imu_msg = Imu()
    mag_msg = MagneticField()
    rate = rospy.Rate(40)

    initTime = rospy.get_time()
    #bag = rosbag.Bag('raw_imu_stationary.bag')
    #for topic, msg, t in bag.read_messages(topics=['/all_imu_lines']):
    try:
        while not rospy.is_shutdown():
            line = port.readline()

            #rate.sleep()
            #line = str(msg)
            if 'VNYMR' in line:
                try: 
                    data = line.split(",")
                    imu_msg.header.frame_id = 'imu'
                    imu_msg.header.stamp.secs = rospy.get_time() - initTime
                    #imu_msg.header.stamp = t

                    mag_msg.header.frame_id = 'mag'
                    mag_msg.header.stamp.secs = rospy.get_time() - initTime
                    #mag_msg.header.stamp = t

                    yaw = float(np.deg2rad(float(data[1])))
                    pitch = float(np.deg2rad(float(data[2])))
                    roll = float(np.deg2rad(float(data[3])))

                    rollPitchYawQuat = quaternion_from_euler(roll, pitch, yaw)

                    xMag = float(data[4])
                    yMag = float(data[5])
                    zMag = float(data[6])
                    xAccel = float(data[7])
                    yAccel = float(data[8])
                    zAccel = float(data[9])
                    xGyro = float(data[10])
                    yGyro = float(data[11])
                    zGyroRaw = data[12].split("*")
                    zGyro = float(zGyroRaw[0])


                    imu_msg.orientation.x = rollPitchYawQuat[0]
                    imu_msg.orientation.y = rollPitchYawQuat[1]
                    imu_msg.orientation.z = rollPitchYawQuat[2]
                    imu_msg.orientation.w = rollPitchYawQuat[3]

                    imu_msg.angular_velocity.x = xGyro
                    imu_msg.angular_velocity.y = yGyro
                    imu_msg.angular_velocity.z = zGyro

                    imu_msg.linear_acceleration.x = xAccel
                    imu_msg.linear_acceleration.y = yAccel
                    imu_msg.linear_acceleration.z = zAccel

                    mag_msg.magnetic_field.x = xMag
                    mag_msg.magnetic_field.y = yMag
                    mag_msg.magnetic_field.z = zMag

                    imu_pub.publish(imu_msg)
                    magnet_pub.publish(mag_msg)
                except:
                    rospy.logwarn("Data exception: " + line)
                    continue
        
    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu node...")
    #bag.close
