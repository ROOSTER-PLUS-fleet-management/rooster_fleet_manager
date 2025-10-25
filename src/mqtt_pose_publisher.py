#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rooster_fleet_manager.msg import MexListInfo

# Subscriber class to the "/mex_sentinel/mex_list_info" topic
class MexListSubscriber:
   def __init__(self):
      self.setup = False
      self.mex_list = []
      self.subscriber = rospy.Subscriber('/mex_sentinel/mex_list_info', MexListInfo, self.callback)
      
   # Callback function for the rospy subscriber
   def callback(self,data):
      if not self.setup:
           if data.total_mex_number > 0:
               for mex_info in data.mex_list_info_array:
                   self.mex_list.append(mex_info.id)
               self.setup = True

# MQTT pose publisher class
class MqttPosePublisher:
   def __init__(self, mex_list):
      self.subscribers = {}
      self.publishers = {}
      self.mex_list = mex_list

      # Create a rospy subscriber and publisher
      for mex_id in self.mex_list:
         self.subscribers[mex_id] = rospy.Subscriber('/'+mex_id+'/amcl_pose', PoseWithCovarianceStamped, self.callback, callback_args = mex_id)
         self.publishers[mex_id] = rospy.Publisher('/'+mex_id+'/pose', Pose, queue_size=1)
   
   # Callback function for the rospy subcribers for each robot
   def callback(self,data,mex_id):
      # Extract robot pose from the PoseWithCovariance message from the /amcl_pose topic, and publish to the /pose topic
      pose = data.pose.pose
      self.publishers[mex_id].publish(pose)

rospy.init_node('mqtt_pose_publisher')

# Initialize subscriber to /mex_list_info to get list of robots
mex_sub = MexListSubscriber()
mex_list = mex_sub.mex_list
while mex_sub.setup != True:
   pass

# Create pose publisher object based on the list of robots
pose_publisher = MqttPosePublisher(mex_list)

rospy.loginfo("MQTT pose being published")
rospy.spin()
