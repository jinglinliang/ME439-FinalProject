#!/usr/bin/env python
import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439WheelSpeeds
from mobrob_util.msg import ME439SensorsRaw

# Global Variables
vel_left = 0
vel_right = 0
far_flag = False
zone_previous = 1
def stagesettings(msg_in):
    global vel_left, vel_right, far_flag, zone_previous
    distance = msg_in.a0


    if distance < 15:
        far_flag = False
        vel_left = 0.1 + 0.1 * np.abs(distance - 15)/15
        vel_right = 0.1 - 0.1 * np.abs(distance - 15)/15
        zone_current = 1
    elif distance >= 15 and distance < 16:
        vel_left  = 0.1
        vel_right = 0.1
        zone_current = 2
    elif distance >= 16 and distance < 31:
        if far_flag:
            vel_left  = 0.1
            vel_right = 0.1
            zone_current = 3
        else:
            vel_left = 0.1 - 0.1 * np.abs(distance - 16)/15
            vel_right = 0.1 + 0.1 * np.abs(distance - 16)/15
            zone_current = 3.5
    elif distance >= 31 and distance < 41:
        vel_left = 0.5
        vel_right = 0.9
    else:
        far_flag = True
        vel_left  = 0.1
        vel_right = 0.1
        zone_current = 4

    if zone_current != zone_previous:
        print("zone changed to: {}".format(zone_current))
    zone_previous = zone_current
    

# Publish desired wheel speeds at the appropriate time. 
def talker(): 
    # Actually launch a node called "set_desired_wheel_speeds"
    rospy.init_node('set_desired_wheel_speeds', anonymous=False)
    pub_speeds = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=10)
    sub_distance = rospy.Subscriber('/sensors_data_raw', ME439SensorsRaw, stagesettings)
    msg_out = ME439WheelSpeeds()
    msg_out.v_left = 0
    msg_out.v_right = 0

    r = rospy.Rate(10) # N Hz
    try: 
        t_start = rospy.get_rostime()
        while not rospy.is_shutdown():
            msg_out.v_left = vel_left
            msg_out.v_right = vel_right
            pub_speeds.publish(msg_out)
            r.sleep()
        
    except Exception:
        traceback.print_exc()
        # When done or Errored, Zero the speeds
        msg_out.v_left = 0
        msg_out.v_right = 0
        pub_speeds.publish(msg_out)
        rospy.loginfo(pub_speeds)    
        pass
        
    # When done or Errored, Zero the speeds
    msg_out.v_left = 0
    msg_out.v_right = 0
    pub_speeds.publish(msg_out)
    rospy.loginfo(pub_speeds)    

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
