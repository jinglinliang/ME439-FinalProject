#!/usr/bin/env python

import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "mobrob") with an extension of .msg ...
# and actually import the message type by name (here "ME439WheelSpeeds")
from mobrob_util.msg import ME439WheelSpeeds
from mobrob_util.msg import ME439SensorsRaw

# =============================================================================
#     Set up a time course of commands
# =============================================================================
    
# Use a new structure: 
# structure: 
# np.array([[duration, left_wheel_speed, right_wheel_speed], [duration, left_wheel_speed, right_wheel_speed], ...]
# 
# Example: Move Forward and Back, 2s each, 0.3 meters per second: 
stage_settings = np.array( [ [0.0, 0.0, 0.0]] )
# Example: forward, turn, return to home, turn. 
#stage_settings = np.array( [ [0,0,0],[3,0.100,0.100],[1,0,0],[1,0.196,-0.196],[1,0,0],[3,0.100,0.100],[1,0,0],[1,-0.196,0.196],[1,0,0]] )

# Convert it into a numpy array
stage_settings_array = np.array(stage_settings)
# Convert the first column to a series of times (elapsed from the beginning) at which to switch settings. 
stage_settings_array[:,0] = np.cumsum(stage_settings_array[:,0],0)  # cumsum = "cumulative sum". The last Zero indicates that it should be summed along the first dimension (down a column). 
distance = 0
vel_left = 0
vel_right = 0
# =============================================================================
# # END of section on specifying movements with wheel speeds and durations. 
# =============================================================================
def stagesettings(msg_in):
    global distance, vel_left, vel_right
    desired_reading = 30 # target sensor reading for distance from the wall
    distance = desired_reading - msg_in.a0 # for sensor on left side of robot, robot distance from desired location
    vel_max = 0.05  # maximum wheel speed
    vel_control = 0.005  # velocity proportional control
    if distance > 5:  # robot is to the left of the desired path
        vel_left = vel_control * distance
        vel_right = vel_left * 0.5
        if (vel_left + vel_right) / 2 > vel_max:
            vel_left = 0.8 * vel_max
            vel_right = 0.2 * vel_max
        else:
            pass

    elif distance < -5:  # robot is to the right of the desired path
        vel_right = vel_control * distance
        vel_left = vel_right * 0.5
        if (vel_left + vel_right) / 2 > vel_max:
            vel_right = 0.8 * vel_max
            vel_left = 0.2 * vel_max
        else:
            pass

    else:
        vel_left = vel_max / 2
        vel_right = vel_max / 2

#    if np.abs(distance) > 10:
#        vel_left = 0.01
#        vel_right = 0
#    else:
#        vel_left = 0
#        vel_right = 0.01



# Publish desired wheel speeds at the appropriate time. 
def talker(): 
    # Actually launch a node called "set_desired_wheel_speeds"
    rospy.init_node('set_desired_wheel_speeds', anonymous=False)

    # Create the publisher. Name the topic "sensors_data", with message type "Sensors"
    pub_speeds = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=10)
    # create subscriber for raw sensor data, still need a function to turn msg_in data to wheel speeds
    sub_distance = rospy.Subscriber('/sensors_data_raw', ME439SensorsRaw, stagesettings)
    # Declare the message that will go on that topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    msg_out = ME439WheelSpeeds()
    msg_out.v_left = 0
    msg_out.v_right = 0
    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    try: 
        # start a loop 
        t_start = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            """future_stages = np.argwhere( stage_settings_array[:,0] >= (rospy.get_rostime()-t_start).to_sec() ) 
            if len(future_stages)>0:
                stage = future_stages[0]
                print stage
            else: 
                break
                """
            msg_out.v_left = vel_left
            msg_out.v_right = vel_right
            # Actually publish the message
            pub_speeds.publish(msg_out)
            # Log the info (optional)
#            rospy.loginfo(pub_speeds)    
            
            r.sleep()
        
#        # Here step through the settings. 
#        for stage in range(0,len(stage_settings_array)):  # len gets the length of the array (here the number of rows)
#            # Set the desired speeds
#            dur = stage_settings_array[stage,0]
#            msg_out.v_left = stage_settings_array[stage,1]
#            msg_out.v_right = stage_settings_array[stage,2]
#            # Actually publish the message
#            pub_speeds.publish(msg_out)
#            # Log the info (optional)
#            rospy.loginfo(pub_speeds)       
#            
#            # Sleep for just long enough to reach the next command. 
#            sleep_dur = dur - (rospy.get_rostime()-t_start).to_sec()
#            sleep_dur = max(sleep_dur, 0.)  # In case there was a setting with zero duration, we will get a small negative time. Don't use negative time. 
#            rospy.sleep(sleep_dur)
        
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
