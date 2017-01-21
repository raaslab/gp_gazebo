#!/usr/bin/python

import rospy
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool
from geometry_msgs.msg import *
import time
'''
x_pos = [1,1,-1,-1]
y_pos = [1,-1,-1,1]
z_pos = [1,1,1,1]

'''
x_pos = [0.5,-0.5,-0.5,0.5]
y_pos = [0.5,0.5,-0.5,-0.5]
z_pos = [0.5,0.5,0.5,0.5]
def state_cb(state):
    print state

def flight():
    # initialize the subscriber node
    rospy.init_node('flight', anonymous=True)
    # subcribe to the mavros State
    #rospy.Subscriber("mavros/state",State,state_cb)
    state = rospy.wait_for_message("mavros/state",State)
    #state_cb(state)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local",PoseStamped,queue_size=10)
    rospy.wait_for_service("mavros/cmd/arming");
    print " Arming service available"
    try:
        arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.wait_for_service("mavros/set_mode");
    print " SetMode service available"
    try:
        set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and not state.connected:
        state = rospy.wait_for_message("mavros/state",State)
        rate.sleep()


    pose = PoseStamped()
    last_request = rospy.get_rostime()

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;

    #send a few setpoints before starting
    set_mode_client(0,'OFFBOARD')
    print "******** OFFBOARD ENABLED *****"
    arming_client(True)
    print "****** ARMED ******"
    
    for i in range(0,100):
        local_pos_pub.publish(pose);
        time.sleep(0.1)

	
    i = 0
    while not rospy.is_shutdown() :
        state = rospy.wait_for_message("mavros/state",State)
        #if state.mode != "OFFBOARD" and (rospy.get_rostime() - last_request) > rospy.Duration(5.0):
        
         
        if state.mode != 'OFFBOARD':
            set_mode_client(0,'OFFBOARD')
            print "OFFBOARD ENABLED"
            
        if not state.armed:
            arming_client(True)
            print "ARMED"
            last_request = rospy.get_rostime()
        
         
        print i
        pose.pose.position.x = x_pos[i]
        pose.pose.position.y = y_pos[i]
        pose.pose.position.z = z_pos[i]
        local_pos_pub.publish(pose)
        #Use this with actual quad
        curr_pose = rospy.wait_for_message("/mavros/mocap/pose" ,PoseStamped)
        
        #Use this with Gazebo
        #curr_pose = rospy.wait_for_message("/mavros/local_position/pose" ,PoseStamped)
        if abs(curr_pose.pose.position.x - x_pos[i]) < 0.2 and  abs(curr_pose.pose.position.y - y_pos[i]) < 0.2 and  abs(curr_pose.pose.position.z - z_pos[i]) < 0.2:
            i = i + 1
        if i == 4:
            i = 0

	
    #rospy.spin()

if __name__ == '__main__':
    try:
        flight()
    except rospy.ROSInterruptException:
        pass
