#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from markers import *
from abb_functions import *


if __name__ == '__main__':

  rospy.init_node("jointsNode")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  
  bmarker      = BallMarker(color['RED'])
  bmarker_des  = BallMarker(color['GREEN'])


  # Joint names
  jnames = ("joint_1", "joint_2", "joint_3","joint_4", "joint_4n", "joint_5", "joint_6")
  
  # Desired position
  xd = np.array([1.43, 0.89, 2.38])
  # Green marker shows the desired position
  bmarker_des.xyz(xd)

  # Initial configuration
  #[1,0.3,-0.5,-1,0.1,1.8,4.2]
  q0 = np.array([1.4, 2.0, -1.8, 1.7, 0.2, 0.1, 3.0])
  # Inverse kinematics
  q = ikine_irb_newton(xd, q0)


  # End effector with respect to the base
  T = fkine_irb(q)
  print( np.round(T, 3) )
  # Red marker shows the final efector postion
  bmarker.position(T)

  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q

  # Loop rate (in Hz)
  rate = rospy.Rate(20)
  # Continuous execution loop
  while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()