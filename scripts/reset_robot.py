#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Empty

messages_to_publish = 0
def callback(msg):
  rospy.loginfo('Resetting robot')
  global messages_to_publish
  messages_to_publish = 5

if __name__ == '__main__':

  rospy.init_node('reset_robot')
  r = rospy.Rate(10)

  state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  sub = rospy.Subscriber('/reset', Empty, callback=callback, queue_size=1)

  state_msg = ModelState()
  state_msg.model_name = 'texbot'
  state_msg.pose.position.x = 0
  state_msg.pose.position.y = 0
  state_msg.pose.position.z = 0

  vel_msg = Twist()
  vel_msg.linear.x = 0
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = 0

  try:
    while not rospy.is_shutdown():
      if messages_to_publish > 0:
        messages_to_publish -= 1
        state_pub.publish(state_msg)
        vel_pub.publish(vel_msg)
      r.sleep()
  except rospy.ROSInterruptException as e:
    pass
