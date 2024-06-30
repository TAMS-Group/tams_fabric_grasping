#! /usr/bin/env python 
#
import serial
import traceback
import sys

import rospy
from std_msgs.msg import Int32


def ForceSensorDriver():
    print( "force_sensor driver node init...\n" )

    rospy.init_node( 'force_sensor', anonymous=False )


    rate_hz = 500.0; 
    if rospy.has_param( '~rate' ):
        rate_hz = rospy.get_param( '~rate' )
    rate = rospy.Rate( rate_hz )
    print( '... loop rate is ' + str(rate_hz) + ' hz.' )
    rospy.logerr( '... loop rate is ' + str(rate_hz) + ' hz.' )


    # Serial-port stuff 
    # 
    port_name = sys.argv[1]
  
    if rospy.has_param( '~port_name' ):
        port_name = rospy.get_param( '~port_name' )
        print( '... serial port name is ' + port_name )

    baudrate = 115200
    if rospy.has_param( '~baudrate' ):
        baudrate = rospy.get_param( '~baudrate' )

    ser = serial.Serial( port_name, baudrate=baudrate )
    print( "... got the serial port ('" + port_name + ") baudrate '" + str(baudrate) )


    # ROS publishers and subscribers
    # 
    running = 1

    load_publisher = rospy.Publisher( 'load', Int32, queue_size=2 )
    current_publisher = rospy.Publisher( 'current', Int32, queue_size=2 )


    print( "... got the ROS node and publishers..." )

    # start ROS main loop
    # 
    while running and not rospy.is_shutdown():
        try:
            data = ""
            rawdata = ser.readline()
            try: 
                data = rawdata.decode( "utf-8" )
            except Exception:
                print(traceback.format_exc())


            if (data[0] == 'L'): 
                (str_I, l, c) = [t(s) for t,s in zip(( str, int, int), data.split()) ]


                load_msg = Int32()
                load_msg.data = l + 31
                load_publisher.publish(load_msg)
                current_msg = Int32()
                current_msg.data = int(round((((c/1023.0) * 10.0) - 5.0) * 1000 * 1.7))  # publish milliamps
                current_publisher.publish(current_msg)

            else: 
                if data == "'\r\n":
                    pass
                else:
                    print( "Unknown data/packet ignored: >" + str(data) + "< len: " + str(len(data)) )

            # rate.sleep() # drops serial data as the buffer fills up too quickly...
            # So, check buffer fill level and only wait if capacity remains
            if ser.in_waiting < 200:
                rospy.sleep( rospy.Duration( 0.002 ))
            elif ser.in_waiting < 500:
                rospy.sleep( rospy.Duration( 0.001 ))

        except ValueError:
            print( "Parsing data failed: '" + str(data) + "'" )
            continue

        except KeyboardInterrupt:
            running = False
            pass

    ser.close()
    print( "force_sensor node driver stopped." )
    


if __name__ == '__main__':
    try:
        ForceSensorDriver()
    except KeyboardInterrupt:
        print( "Received control-c, stopping..." )
        exit( 0 )
    except rospy.ROSInterruptException:
        pass