#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#from matplotlib.backend_bases import LocationEvent

import rospy
import serial
import numpy as np 
#import utm
from lab4.msg import IMUMessage

def get_quaternion_from_euler(yaw, pitch, roll):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in degrees.
    :param pitch: The pitch (rotation around y-axis) angle in degrees.
    :param yaw: The yaw (rotation around z-axis) angle in degrees.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  yaw = yaw * .01745329251
  pitch = pitch * .01745329251
  roll = roll * .01745329251
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

if __name__ == '__main__':
    
    #Sensor parameter configuration
    SENSOR_NAME = "imu"
    rospy.init_node('my_imu_node', anonymous=True)
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    #Publisher with topic : imu_data and custom message type: IMUMessage
    imu_pub = rospy.Publisher('imu_data', IMUMessage, queue_size=20)
    msg = IMUMessage()

    rospy.loginfo("Using IMU sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.loginfo("Initialization complete")
    
    i = 0
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            if line == '':
                rospy.logwarn("Not reading any data from sensor")
            else:
                if line.startswith(b'$VNYMR'):
                    print("\nFound VNYMR in line")
                    line = line.split(b'*')
                    vnymr_array = line[0].split(b',')
                    #print(vnymr_array)
                    header = vnymr_array[0]
                    
                    yaw = float(vnymr_array[1].decode('utf-8'))
                    pitch = float(vnymr_array[2].decode('utf-8'))
                    roll = float(vnymr_array[3].decode('utf-8'))
                    
                    [qx, qy, qz, qw] = get_quaternion_from_euler(yaw , pitch, roll)
                    
                    magx = float(vnymr_array[4].decode('utf-8'))
                    magy = float(vnymr_array[5].decode('utf-8'))
                    magz = float(vnymr_array[6].decode('utf-8'))
                    accelx = float(vnymr_array[7].decode('utf-8'))
                    accely = float(vnymr_array[8].decode('utf-8'))
                    accelz = float(vnymr_array[9].decode('utf-8'))
                    gyrox = float(vnymr_array[10].decode('utf-8'))
                    gyroy = float(vnymr_array[11].decode('utf-8'))
                    gyroz = float(vnymr_array[12].decode('utf-8'))
                    
                    
                    msg.IMU.header.seq = i
                    i = i + 1
                    msg.IMU.header.stamp = rospy.Time.now()
                    msg.IMU.header.frame_id = "IMU nonmagnetic"

                    msg.IMU.orientation.x = qx
                    msg.IMU.orientation.y = qy
                    msg.IMU.orientation.z = qz
                    msg.IMU.orientation.w = qw            
                    msg.IMU.linear_acceleration.x = accelx
                    msg.IMU.linear_acceleration.y = accely
                    msg.IMU.linear_acceleration.z = accelz
                    msg.IMU.angular_velocity.x = gyrox
                    msg.IMU.angular_velocity.y = gyroy
                    msg.IMU.angular_velocity.z = gyroz
                    
                    msg.magnetic.header.seq = i
                    msg.magnetic.header.stamp = rospy.Time.now()
                    msg.magnetic.header.frame_id = "magnetic"
                    
                    msg.magnetic.magnetic_field.x = magx
                    msg.magnetic.magnetic_field.y = magy
                    msg.magnetic.magnetic_field.z = magz
                    
                    msg.yaw = yaw
                    msg.pitch = pitch
                    msg.roll = roll

                    imu_pub.publish(msg)
                    rospy.loginfo(msg)
                    
    except rospy.ROSInterruptException:
        print("Pressed CTRL+C")
        port.close()
            
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu sensor node...")
