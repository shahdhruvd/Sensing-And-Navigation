#!/usr/bin/env python
# -*- coding: utf-8 -*-


from re import I
import rospy
import serial
#Importing sensor_msgs dependency#
from sensor_msgs.msg import Imu
#Importing the msg file from source#
from lab03.msg import imumsg
#Importing math functions to convert to quaternion# 
from tf.transformations import quaternion_from_euler
import math



if __name__ == '__main__':
    SENSOR_NAME = "IMU"       #Naming the Sensor#
    pub =rospy.Publisher("IMUTopic",imumsg , queue_size=1 )      #making a publisher
    rospy.init_node('imu_driver_node')                           #Initialising the node
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')          #Settings for serial connection   
    serial_baud = rospy.get_param('~baudrate',115200)           #Settings for serial connection
    port = serial.Serial(serial_port,serial_baud,timeout = 3)   #Settings for serial connection
    port.write(b"$VNWRG,07,40*XX")

    rospy.loginfo("Reading Data")
    # line = port.readline()
    # print(line)
    i = 1

    try:
        while(not rospy.is_shutdown()):
            line = port.readline()
            # print(line)
            if line.startswith(b'\r$VNYMR'):                #If Line Starts with the strinf then print line
                print(line)
            try:
                x = line.split(b',')                        #f Line Starts with the strinf then initialize an array and split components
                print(line)
                yaw = float(x[1])                           # declaring variable to array poistion
                pitch = float(x[2])                         # declaring variable to array poistion      
                roll = float(x[3])                          # declaring variable to array poistion

                MagX = float(x[4])                          # declaring variable to array poistion
                MagY = float(x[5])                          # declaring variable to array poistion
                MagZ = float(x[6])                          # declaring variable to array poistion
                
                AccX = float(x[7])                          # declaring variable to array poistion
                AccY = float(x[8])                          # declaring variable to array poistion
                AccZ = float(x[9])                          # declaring variable to array poistion

                GyroX = float(x[10])                        # declaring variable to array poistion
                GyroY = float(x[11])                        # declaring variable to array poistion
                GyroZ = float(x[12][0:10])                  # declaring variable to array poistion and added a condition to ignore the last string part in the array

                quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))  # Conversion to quaternion
                
                imudata = imumsg()                          #Declaring the msg 
                
                imudata.imudata.header.stamp = rospy.Time.now()     #Declaring the msg component to a variable 
                imudata.imudata.header.seq=i

                
                imudata.imudata.orientation.x = quaternion[0]       #Declaring the msg component to a variable
                imudata.imudata.orientation.y = quaternion[1]       #Declaring the msg component to a variable      
                imudata.imudata.orientation.z = quaternion[2]       #Declaring the msg component to a variable
                imudata.imudata.orientation.w = quaternion[3]       #Declaring the msg component to a variable

                imudata.imudata.linear_acceleration.x = AccX        #Declaring the msg component to a variable
                imudata.imudata.linear_acceleration.y = AccY        #Declaring the msg component to a variable  
                imudata.imudata.linear_acceleration.z = AccZ        #Declaring the msg component to a variable

                imudata.imudata.angular_velocity.x = GyroX          #Declaring the msg component to a variable
                imudata.imudata.angular_velocity.y = GyroY          #Declaring the msg component to a variable
                imudata.imudata.angular_velocity.z = GyroZ          #Declaring the msg component to a variable
                
                imudata.mag.magnetic_field.x = MagX                 #Declaring the msg component to a variable
                imudata.mag.magnetic_field.y = MagY                 #Declaring the msg component to a variable
                imudata.mag.magnetic_field.z = MagZ                 #Declaring the msg component to a variable
                
                pub.publish(imudata)                                #Publishing the Msg

                i = i +1
            except rospy.ROSInterruptException:
                port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")