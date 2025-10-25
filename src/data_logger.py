#! /usr/bin/env python
import yaml
import json
from datetime import datetime
import os
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
from rooster_fleet_manager.msg import MexListInfo

package_path = rospkg.RosPack().get_path("rooster_fleet_manager")
time_string = datetime.now().strftime("%Y-%m-%d_%H_%M_%S")

# Get configs from YAML file
with open(package_path + "/config/sim_configs.yaml", "r") as f:
    configs = yaml.safe_load(f)

# Extract config parameters
logs_folder_base_path = str(configs["data_logger"]["logs_folder_path"])
logging_interval = int(configs["data_logger"]["logging_interval"])

# Set directory for saving data logs and create it if it does not exist
if logs_folder_base_path == "": # If base path is empty, replace with the package path
   logs_folder_path = package_path + "/data_logs"
else:
   logs_folder_path = logs_folder_base_path
logs_folder_path = logs_folder_path + "/" + time_string + "/" # Add subfolder for the time of the simulation
if not os.path.exists(logs_folder_path):
   os.makedirs(logs_folder_path)

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

# Data logger class
class DataLogger:
   def __init__(self,mex_list,path):
      self.file_path = path
      self.last_print_time = {}
      self.current_time = {}
      self.subscribers = {}
      self.first_entry = {}
      self.mex_list = mex_list

      # Create a rospy subscriber, rospy publisher, current time, and last print time for each robot
      for mex_id in self.mex_list:
         self.subscribers[mex_id] = rospy.Subscriber('/'+mex_id+'/amcl_pose', PoseWithCovarianceStamped, self.callback, callback_args = mex_id)
         self.current_time[mex_id] = 0
         self.last_print_time[mex_id] = rospy.get_time()
         self.first_entry[mex_id] = True
   
   # Callback function for the rospy subcribers for each robot
   def callback(self,data,mex_id):
      # Update current time
      self.current_time[mex_id] = rospy.get_time()

      # Extract robot pose from the PoseWithCovariance message from the /amcl_pose topic
      pose = data.pose.pose

      # If the specified logging interval has passed since the last log, log the pose to the JSON file named after each robot in the folder
      if self.current_time[mex_id] - self.last_print_time[mex_id] >= logging_interval:
         # Create entry
         entry = {
            "timestamp": self.current_time[mex_id],
            "position": {
               "x": pose.position.x,
               "y": pose.position.y,
               "z": pose.position.z
            },
            "orientation": {
               "x": pose.orientation.x,
               "y": pose.orientation.y,
               "z": pose.orientation.z,
               "w": pose.orientation.w
            }
         }
         # Write to JSON file
         with open(self.file_path + mex_id + ".json", "a") as json_file:
            # If first entry, add open bracket, otherwise add comma
            if self.first_entry[mex_id]:
               json_file.write("[\n" + json.dumps(entry, indent = 2))
               self.first_entry[mex_id] = False
            else:
               json_file.write(",\n" + json.dumps(entry, indent = 2))
         self.last_print_time[mex_id] = self.current_time[mex_id]

   def shutdown_hook(self):
      # Close JSON file with a closed parentheses
      for file in os.listdir(logs_folder_path):
         if file.endswith(".json"):
            with open(logs_folder_path + file, "a") as json_file:
               json_file.write("\n]")
            
rospy.init_node('data_logger')

# Initialize subscriber to /mex_list_info to get list of robots
mex_sub = MexListSubscriber()
mex_list = mex_sub.mex_list
while mex_sub.setup != True:
   pass

# Create data logger object based on the list of robots
data_logger = DataLogger(mex_list, logs_folder_path)

rospy.on_shutdown(data_logger.shutdown_hook)
rospy.spin()
