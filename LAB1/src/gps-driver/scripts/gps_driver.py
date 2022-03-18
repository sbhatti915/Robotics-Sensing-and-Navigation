#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Header
from gps.msg import gps_msg

# Converts latitude or longitude of gpgga data to proper decimals
def latlonConv(gpgga):
    gpgga = float(gpgga)*10000
    x = gpgga % 10000
    y = (x/10000*60)/100
    gpgga = ((gpgga-x)/10000 + y)/100
    return gpgga

# Converts latitude and longitude to utm 
def utmConv(lat, lon):
    utm_data = utm.from_latlon(lat, lon)
    return utm_data

# Converts time of gpgga data 
def timeConv(time):
    seconds = time % 100
    minToSec = ((time/100) % 100)*60
    hourToSec = ((time/10000) % 100)*3600
    ss = seconds + minToSec + hourToSec
    return ss


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_driver', anonymous=True)
    serial_port = rospy.get_param('~port','/dev/pts/1')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)
  
    port = serial.Serial(serial_port, serial_baud, timeout=3.0)
    rospy.logdebug("Using GPS sensor on port "+ serial_port +" at "+ str(serial_baud))
    
    rospy.sleep(0.2)

    gps_pub = rospy.Publisher(SENSOR_NAME,gps_msg, queue_size=10)
    
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing GPS Information.")

    sleep_time = 1/sampling_rate - 0.025
    
    gps_info = gps_msg()

    try:
        while not rospy.is_shutdown():
            line = port.readline()

            if line == '':
                rospy.logwarn("Port: No data")
            else:
                if '$GPGGA' in line:
                    data = line.split(",")

                    gps_info.header.stamp.secs = timeConv(float(data[1]))
                    gps_info.header.frame_id = 'gps'

                    try: 

                        eastWestID = 1
                        northSouthID = 1

                        if data[3] == 'S':
                            northSouthID = -1

                        if data[5] == 'W':
                            eastWestID = -1
                        
                        gps_info.latitude = float(northSouthID*latlonConv(data[2]))
                        gps_info.longitude = float(eastWestID*latlonConv(data[4]))
                        gps_info.altitude = float(data[9]) # (m)

                        utm_data = utmConv(gps_info.latitude,gps_info.longitude)
                        gps_info.utm_easting = float(utm_data[0])
                        gps_info.utm_northing = float(utm_data[1])
                        gps_info.Zone = utm_data[2]
                        gps_info.fields = utm_data[3]

                        print(float(northSouthID*latlonConv(data[2])))

                        gps_pub.publish(gps_info)

                    except:
                        rospy.logwarn("Data exception: " + line)
                        continue

            rospy.sleep(sleep_time)

            
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps node...")
