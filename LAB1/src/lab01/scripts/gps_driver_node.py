#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import utm
import serial
from std_msgs.msg import Float64
from std_msgs.msg import String
from lab01.msg import gpsdata

# from math import sin, pi
# from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    pub =rospy.Publisher("GpsTopic", gpsdata, queue_size=10 )
    rospy.init_node('gps_driver_node')
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)
    sampling_rate = rospy.get_param('~sampling_rate',5.0)

  
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using depth sensor on port "+serial_port+" at "+str(serial_baud))

    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    sampling_count = int(round(1/(sampling_rate*0.007913)))

    rospy.sleep(0.2)        
    line = port.readline()

    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Reading Data")
    

    sleep_time = 1/sampling_rate - 0.025
   
    try:
        while not rospy.is_shutdown():
            line = port.readline()
            #print(line)
            if line == '':
                rospy.logwarn("DEPTH: No data")
            else:
                if line.startswith(b'$GPGGA'):
                    print(line)                
                    try:
                        x = line.split(b',')
                        # print (x[1])
                        
                        lat=float(x[2])
                        latd=x[3]
                        lon=float(x[4])
                        lond=x[5]
                        # print(lond)
                        alt = float(x[9])
                        # lat_ft = 2
                        # lon_ft = 2
                        multiplierlat = +1
                        multiplierlon = +1


                        if latd == b'S':
                            # lat= -1*lat
                            # lat_ft = 3
                            multiplierlat = -1
                        
                    
                        if lond == b'W':
                            # lon = -1*lon
                            # lon_ft=3
                            multiplierlon = -1
                        
                        lat = str(lat)
                        Latdd = float(lat[:2])
                        # print(Latdd)
                        Latmmmmm = float(lat[2:])
                        # print(Latmmmmm)
                        Latddmmmm = multiplierlat*(Latdd+(Latmmmmm/60.0))
                        # print(Latddmmmm)

                        lon = str(lon)
                        Londd = float(lon[:2])
                        Lonmmmmm = float(lon[2:])
                        Londdmmmm = multiplierlon*(Londd+(Lonmmmmm/60.0))
                        # print(Londdmmmm)
                       
                        easting,northing,zone_number,zone_letter = utm.from_latlon(Latddmmmm, Londdmmmm)
                        # print(easting,northing,zone_number,zone_letter)
                        msg = gpsdata()
                        # msg.message = "My Position is :"
                        msg.Latitude = Latddmmmm
                        msg.Longitude = Londdmmmm
                        msg.Altitude = alt
                        msg.Easting = easting
                        msg.Northing = northing
                        msg.Zone_nos = zone_number
                        msg.Zone_let = zone_letter
                        pub.publish(msg)
                        





                    except: 
                        rospy.logwarn("Data exception: "+line)
                    continue
              
            rospy.sleep(sleep_time)
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")