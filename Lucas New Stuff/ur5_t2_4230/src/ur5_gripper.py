#!/usr/bin/env python



import rospy, sys, numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from std_msgs.msg import String

def gripper_status(msg):
    if msg.data:
        return True
        print('gripper status = {}'.format(msg.data))

def gripper_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper1_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper1/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper1/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper2_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper2/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper2/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper3_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper3/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper3/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper4_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper4/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper4/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper5_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper5/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper5/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper6_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper6/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper6/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper7_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper7/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper7/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper8_on():
    # Wait till the srv is available
    rospy.wait_for_service('/ur5/vacuum_gripper8/on')
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/ur5/vacuum_gripper8/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def gripper_off():
    rospy.wait_for_service('/ur5/vacuum_gripper/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper1_off():
    rospy.wait_for_service('/ur5/vacuum_gripper1/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper1/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper2_off():
    rospy.wait_for_service('/ur5/vacuum_gripper2/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper2/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper3_off():
    rospy.wait_for_service('/ur5/vacuum_gripper3/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper3/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper4_off():
    rospy.wait_for_service('/ur5/vacuum_gripper4/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper4/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper5_off():
    rospy.wait_for_service('/ur5/vacuum_gripper5/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper5/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper6_off():
    rospy.wait_for_service('/ur5/vacuum_gripper6/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper6/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper7_off():
    rospy.wait_for_service('/ur5/vacuum_gripper7/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper7/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def gripper8_off():
    rospy.wait_for_service('/ur5/vacuum_gripper8/off')
    try:
        turn_off = rospy.ServiceProxy('/ur5/vacuum_gripper8/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# class MyGripper:
#     def main(self):
#         rospy.Subscriber('Gripper', String, self.trigger)
#         self.value = 0
#         self.rate = rospy.get_param("~rate",10)
#         while not rospy.is_shutdown():

#             # print(self.value)
#             if self.value:
#                 print("gripper on")
#                 gripper_on()
#                 gripper1_on()
#                 gripper2_on()
#                 gripper3_on()
#                 gripper4_on()
#                 gripper5_on()
#                 gripper6_on()
#                 gripper7_on()
#                 gripper8_on()

#             else:
#                 print("gripper off")
#                 gripper_off()
#                 gripper1_off()
#                 gripper2_off()
#                 gripper3_off()
#                 gripper4_off()
#                 gripper5_off()
#                 gripper6_off()
#                 gripper7_off()
#                 gripper8_off()

#     def trigger(self, newValue):
#         self.value = int(newValue.data)
def trigger(msg):
    if int(msg.data):
        print("gripper on")
        gripper_on()
        gripper1_on()
        gripper2_on()
        gripper3_on()
        gripper4_on()
        gripper5_on()
        gripper6_on()
        gripper7_on()
        gripper8_on()

    else:
        print("gripper off")
        gripper_off()
        gripper1_off()
        gripper2_off()
        gripper3_off()
        gripper4_off()
        gripper5_off()
        gripper6_off()
        gripper7_off()
        gripper8_off()
        
rospy.Subscriber('Gripper', String, trigger)

rospy.init_node("ur5_gripper", anonymous=False)

gripper_status_sub = rospy.Subscriber('/ur5/vacuum_gripper/grasping', Bool, gripper_status, queue_size=1)

# if __name__ == '__main__':
#     try:
#         MyGripper().main()
#     except rospy.ROSInterruptException:
#         pass


rospy.spin()
