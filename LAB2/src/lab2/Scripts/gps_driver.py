#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#from matplotlib.backend_bases import LocationEvent

import rospy
import serial
import utm
from lab2.msg import GPSMessageLab2

# Function to convert Degree Minute Second (DMS) to Decimal Degree(DD) notation
def dms_to_dd(d, min, sec):
    dd = d + float(min)/60 + float(sec)/3600
    return dd

if __name__ == '__main__':
    
    #Sensor parameter configuration
    SENSOR_NAME = "gps"
    rospy.init_node('my_gps_node', anonymous=True)
    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',57600)
    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    #Publisher with topic : gps_data and custom message type: GPSMessage
    gps_pub = rospy.Publisher('gps_data_lab2', GPSMessageLab2, queue_size=10)
    msg = GPSMessageLab2()

    rospy.loginfo("Using GPS sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.loginfo("Initialization complete")
    
    i = 0
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            if line == '':
                rospy.logwarn("Not reading any data from sensor")
            else:
                if line.startswith(b'$GNGGA'):
                    print("\nFound GNGGA in line")
                    gngga_array = line.split(b',')
                    print(gngga_array)
                    header = gngga_array[0]
                    
                    latitude = gngga_array[2].decode('utf-8')
                    latitude_direction = gngga_array[3].decode('utf-8')
                    latitude_dd = dms_to_dd(float(latitude[:2]) , float(latitude[2:9]), 0)
                    latitude_dd  = latitude_dd if latitude_direction == "N" else latitude_dd*-1 
                    gps_quality_indicator= int(gngga_array[6].decode('utf-8'))

                    longitude = gngga_array[4].decode('utf-8')
                    longitude_direction = gngga_array[5].decode('utf-8')
                    longitude_dd = dms_to_dd(float(longitude[:3]) , float(longitude[3:10]) , 0)                    
                    longitude_dd  = longitude_dd if longitude_direction == "E" else longitude_dd*-1 
                    

                    altitude = float(gngga_array[9].decode('utf-8'))
                    altitude_unit = gngga_array[10].decode('utf-8')
                    

                    utm_array = utm.from_latlon(latitude_dd, longitude_dd)
                    utm_easting = float(utm_array[0])
                    utm_northing = float(utm_array[1])
                    zone_number = int(utm_array[2])
                    zone_letter = utm_array[3]
                    
                    msg.header.seq = i
                    i = i + 1
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "gps_data"
                    msg.latitude = latitude_dd
                    msg.longitude = longitude_dd
                    msg.altitude = altitude
                    msg.utm_easting = utm_easting
                    msg.utm_northing = utm_northing
                    msg.zone = zone_number
                    msg.letter = zone_letter
                    msg.quality = gps_quality_indicator
                    gps_pub.publish(msg)
                    rospy.loginfo(msg)
                    
    except rospy.ROSInterruptException:
        print("Pressed CTRL+C")
        port.close()
            
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gps sensor node...")
