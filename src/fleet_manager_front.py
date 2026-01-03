#! /usr/bin/env python
import json         # Used for reading JSON files (loading jobs to JobQueue)
import os           # Used to get base filename and file and directory handling
import sys

import rospy
import rospkg
from std_msgs.msg import Bool, String
from PyQt4 import QtGui, QtCore

from rooster_fleet_manager.srv import PlaceOrder, PlaceOrderRequest, GetPendingJobs, GetPendingJobsRequest, GetActiveJobs, GetActiveJobsRequest
from rooster_fleet_manager.msg import MexListInfo
from ui import fleet_manager_ui
from JobManager.Order import *
from JobManager.Job import JobStatus, Job, JobPriority

from num2word import word as n2w # for converting words to numbers

#region ################################### TO DO LIST #####################################
# DONE 1.  Perform simple call /job_manager/place_order service.
# DONE 2.  Load list of orders from JSON.
# DONE 3.  Call /job_manager/place_order(s?) service with the list of orders.
# DONE 4.  Add visually correct but non-functional GUI to node.
# DONE 5.  Replace JSON order list call and instead visualize it in the Order tab.
# DONE 6.  Connect MEx Sentinel to Mobile Executors treeWidget view.
# DONE 7.  Check if the order is viable before adding to order list, notify user if not.
# DONE 8.  Remove orders from Order list if they were placed succesfully.
# DONE 9.  Put in Placeholder text in the arguments field based on the order keyword.
# DONE 10. Connect Job Manager to the Jobs treeWidget view.
# DONE 11. Add the deletion of individual orders from the order list.
# DONE 12. Replace placeholders in the FILE ACTION MENU.
# DONE 13. Add logo in same style as GUI launcher.
# DONE 14. Automatically sort the Jobs list when new jobs have been added.
# DONE 15. Add KEYWORD to Job. 
# DONE 16. Fix issue where Job list does not update when <= 1 Jobs after Job completions.
#endregion #################################################################################


VERSION = "1.0"
APPLICATION_TITLE = "Fleet Manager"
NODE_NAME = "[fleet_manager_front] "
print(NODE_NAME + APPLICATION_TITLE + ". Version: "+VERSION)


#region         ### PyQt GUI ###
class GuiMainWindow(fleet_manager_ui.Ui_MainWindow, QtGui.QMainWindow):
    # Signal used to forward voice error messages from ROS thread to the Qt GUI thread
    voiceErrorSignal = QtCore.pyqtSignal(str)
    # Signal used to forward voice command strings from ROS thread to the Qt GUI thread
    voiceCommandSignal = QtCore.pyqtSignal(str)
    def __init__(self):
        """
        Initialise the ui widgets, items and varibles.
        Connect up all UI interactions to their methods.
        """
        super(GuiMainWindow, self).__init__()
        self.setWindowTitle(APPLICATION_TITLE)  #self.filename + " - " + 

        # Set up gui
        self.setupUi(self)

        #region FILE ACTION MENU
        self.actionAbout.triggered.connect(self.about)
        self.actionQuit_application.triggered.connect(self.close_application)
        #endregion

        #region ORDERS TAB
        self.pushButtonAddOrder.clicked.connect(self.add_order)
        self.pushButtonClearList.clicked.connect(self.clear_order_list)
        self.pushButtonPlaceOrder.clicked.connect(self.place_order_list)
        self.comboBoxKeyword.currentIndexChanged.connect(self.update_order_arguments_placeholder_text)
        self.lineEditArguments.setPlaceholderText("location")

        # TreeWidget context menu
        self.treeWidgetOrders.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeWidgetOrders.customContextMenuRequested.connect(self.open_context_menu)  

        self.treeMenu = QtGui.QMenu('Menu', self)
        deleteItem = QtGui.QAction("&Delete", self)
        deleteItem.setStatusTip("Delete item from Order list")
        deleteItem.triggered.connect(self.delete_orders_tree_item)
        deleteIcon = QtGui.QIcon()
        deleteIcon.addPixmap(QtGui.QPixmap(":/icons/Close.png"))
        deleteItem.setIcon(deleteIcon)
        self.treeMenu.addAction(deleteItem)

        #endregion

        #region Initializations for voice command functionality
        self.map_location_names_to_ids()
        self.voice_command_state_pub = rospy.Publisher('/voice_command/state', Bool, queue_size=10)
        self.pushButtonVoiceCommand.clicked.connect(self.toggle_voice_command)
        self.voice_command_output_sub = rospy.Subscriber('/voice_command/output', String, self.execute_voice_command)
        self.voice_command_error_sub = rospy.Subscriber('/voice_command/error', String, self.handle_voice_command_error)
        # Connect ROS->GUI signal to handler so GUI updates happen in main thread
        # Connect command signal to GUI handler
        try:
            self.voiceCommandSignal.connect(self._handle_voice_command)
        except Exception:
            pass
        try:
            self.voiceErrorSignal.connect(self._handle_voice_error)
        except Exception:
            # If signal/slot system unavailable for some PyQt builds, we'll still attempt
            # to call the handler directly from the ROS callback (best-effort).
            pass
        #endregion

    def open_context_menu(self):
        """ Opens the Right-Mouse-Button context menu, showing an option to delete the order tree item. """
        self.treeMenu.exec_(QtGui.QCursor.pos())

    def delete_orders_tree_item(self, item):
        """ Deletes the currently selected item from the orders treeWidget. """
        index = self.treeWidgetOrders.currentIndex()
        self.treeWidgetOrders.takeTopLevelItem(index.row())

    def update_order_arguments_placeholder_text(self):
        """
        Updates the light gray placeholder text of the Order arguments lineEdit
        input field to match with the new keyword.
        """
        keyword = str(self.comboBoxKeyword.currentText())
        if keyword == OrderKeyword.LOAD.name or keyword == OrderKeyword.UNLOAD.name:
            self.lineEditArguments.setPlaceholderText("No Arguments!")
        elif keyword == OrderKeyword.MOVE.name:
            self.lineEditArguments.setPlaceholderText("location")
        elif keyword == OrderKeyword.TRANSPORT.name:
            self.lineEditArguments.setPlaceholderText("location1 location2")
        
    def place_order_list(self):
        """ Place multiple orders from the Order tab Order list to the Job Manager. """
        # The order of the orders matter, so don't just loop over the dictionary, but check size and loop over order id's
        self.order_list = []        # Empty order list
        indices_to_remove = []      # Empty list for treeWidgetOrders indices.

        # Iterate over all the existing (top level) items (a.k.a. orders) in the treeWidgetOrders and add as order.
        root = self.treeWidgetOrders.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            item = root.child(i)
            order_keyword = str(item.text(0))
            order_priority = str(item.text(1))
            order_arguments = str(item.text(2)).split()
            order = [order_keyword, order_priority, order_arguments, i]
            self.order_list.append(order)
        
        # If there are orders to be placed, call the job_manager's place_order service.
        if len(self.order_list) != 0:
            rospy.wait_for_service('/job_manager/place_order')
            try:
                place_order = rospy.ServiceProxy('/job_manager/place_order', PlaceOrder)
                req = PlaceOrderRequest()
                for order in self.order_list:
                    req.keyword = order[0]
                    req.priority = order[1]
                    req.order_args = order[2]
                    resp = place_order(req)
                    print(NODE_NAME + "Response: ", resp)
                    if resp.error_status == OrderResponseStatus.SUCCES.name:
                        # The placement of this order was succesful, remove from Order list
                        indices_to_remove.append(order[3])
            except rospy.ServiceException as e:
                print(NODE_NAME + "Service call failed: %s"%e)
        
        if len(indices_to_remove) != 0:
            # Sort the indices list in descending order (from highest index to lowest index).
            indices_to_remove.sort(reverse=True)
            # Iterate over the sorted list, removing items from the treeWidgetOrders
            for index in indices_to_remove:
                self.treeWidgetOrders.takeTopLevelItem(index)
        
        if len(indices_to_remove) != len(self.order_list):
            # Show a notification box alerting the user not all orders were placed succesfully.
            QtGui.QMessageBox.warning(self, "Not all orders could be placed succefully!", "Out of the " + str(len(self.order_list)) + " orders, " + str(len(self.order_list) - len(indices_to_remove)) + " could not be placed succesfully. These orders have been kept in the order list, succesful orders have been removed.")
        else:
            # Show a notification informing the user that all orders were placed succesfully.
            QtGui.QMessageBox.information(self, "All orders placed succesfully!", "All " + str(len(self.order_list)) + " order(s) have been placed succesfully and have been removed from the order list.")


    def clear_order_list(self):
        """Clears the Order tab order list."""
        root = self.treeWidgetOrders.invisibleRootItem()
        child_count = root.childCount()
        if child_count > 0:
            self.treeWidgetOrders.clear()

    def add_order(self):
        """
        Add a order to the Order List based on the Order tab input field values.
        Before adding, a check is performaned to make sure the supplied fields are set correctly.
        If this is not the case the user is notified with a MessageBox.
        """
        order_keyword = self.comboBoxKeyword.currentText()
        order_priority = self.comboBoxPriority.currentText()
        order_arguments = self.lineEditArguments.text()
        
        # Check if the number of supplied arguments 
        supplied_args = len(str(order_arguments).split())
        expected_args = OrderTypeArgCount[str(order_keyword)].value

        if supplied_args == expected_args:
            # Add the order to the order list.
            self.lineEditArguments.clear()
            order_item = QtGui.QTreeWidgetItem([order_keyword, order_priority, order_arguments])
            self.treeWidgetOrders.addTopLevelItem(order_item)
        else:
            # Show a notification informing the user that the order is incorrect.
            QtGui.QMessageBox.information(self, "Incorrect number of order arguments!", "Incorrect number of arguments. Supplied " + str(supplied_args) + " argument(s) (" + order_arguments + "). Expected " + str(expected_args) + " argument(s).")

    def toggle_voice_command(self, checked):
        """
        Toggle the voice command listening state on or off.
        """
        # checked is True (Down) or False (Up)
        self.voice_command_state_pub.publish(checked)
        
        if checked:
            self.pushButtonVoiceCommand.setText("Listening...")
            self.pushButtonVoiceCommand.setStyleSheet("color: red;")
            # Change icon to Red
        else:
            self.pushButtonVoiceCommand.setText(" &Turn on voice command")
            self.pushButtonVoiceCommand.setStyleSheet("color: black;")
            # Change icon to Gray
    
    def execute_voice_command(self, msg):   
        """ROS callback: forward the raw command string to the GUI thread via signal."""
        try:
            command_text = str(msg.data).lower()
        except Exception:
            try:
                command_text = str(msg).lower()
            except Exception:
                command_text = ""

        # Emit signal to GUI thread for handling
        try:
            self.voiceCommandSignal.emit(command_text)
        except Exception:
            # Fallback: directly call handler (may be invoked from ROS thread)
            try:
                self._handle_voice_command(command_text)
            except Exception:
                pass

    def _handle_voice_command(self, command_text):
        """Handle a voice command in the GUI thread (this contains the original logic).
        `command_text` must be a lowercase string.
        """
        if not command_text:
            return

        command_text = str(command_text)
        print(NODE_NAME + "Received voice command: " + command_text)

        # Special command to place the order list.
        if command_text == "place order":
            self.place_order_list()
            return

        if command_text == "clear list":
            self.clear_order_list()
            return

        if command_text == "stop listening":
            # Ensure UI reflects stopped state
            self.toggle_voice_command(False)
            return

        if command_text == "cancel order":
            # remove first/top item if present
            root = self.treeWidgetOrders.invisibleRootItem()
            if root.childCount() > 0:
                self.treeWidgetOrders.takeTopLevelItem(0)
            return

        # Parse the command text into keyword, priority and arguments.
        command_parts = command_text.split(" ", 1) # Split into keyword and the rest (priority + arguments)
        if len(command_parts) <= 2:
            if len(command_parts) == 2:
                order_keyword = command_parts[0].strip().upper()
                # Find word "priority" in the rest of the command string.
                if "priority" in command_parts[1]:
                    priority_index = command_parts[1].find("priority")
                    order_priority = command_parts[1][priority_index + len("priority"):].strip().split(" ", 1)[0]
                    order_arguments = command_parts[1][:priority_index].strip()
                else:
                    order_priority = "LOW"   # Default priority if not specified.
                    order_arguments = command_parts[1].strip()
                order_priority = order_priority.upper()

                if order_keyword == "TRANSPORT":
                    # For transport, split arguments into two locations.
                    args_split = order_arguments.split(" to ", 1)
                    if len(args_split) == 2:
                        order_arguments = args_split
                        # Change location name to id
                        for i, arg in enumerate(order_arguments):
                            order_arguments[i] = self.location_mapping[arg]
                        order_arguments = " ".join(order_arguments)
                    else:
                        print(NODE_NAME + "Invalid number of arguments for TRANSPORT command.")
                        return
                elif order_keyword == "MOVE": 
                    # Change location name to id
                    order_arguments = self.location_mapping[order_arguments]
                else:
                    if order_keyword == "EMPTY": # unload is replaced with empty in voice command
                        order_keyword = "UNLOAD"
                    # Change location name to id
                    order_arguments = ""
            elif len(command_parts) == 1: # for cases like LOAD and UNLOAD with no arguments
                order_keyword = command_parts[0].strip().upper()
                if order_keyword == "EMPTY": # unload is replaced with empty in voice command
                    order_keyword = "UNLOAD"
                order_arguments = ""
                order_priority = "LOW"   # Default priority is not specified.
                
            # Check if the number of supplied arguments is correct
            supplied_args = len(str(order_arguments).split())
            expected_args = OrderTypeArgCount[str(order_keyword)].value

            # If correct, add to order list.
            if supplied_args == expected_args:
                # Add the order to the order list.
                order_item = QtGui.QTreeWidgetItem([order_keyword, order_priority, order_arguments])
                self.treeWidgetOrders.addTopLevelItem(order_item)
            else:
                print(NODE_NAME + "Invalid number of arguments for " + order_keyword.upper() + " command.")
        else:
            print(NODE_NAME + "Invalid voice command format received.")

    def map_location_names_to_ids(self):
        """ Create dictionary that maps location names to their corresponding IDs. """
        # Reads from locations.json file
        location_mapping = {}
        file_path = rospkg.RosPack().get_path("multi_robot_sim") +  "/scripts/JSONtoRosparam/locations.JSON"
        with open(file_path) as json_file:
            locations_data = json.load(json_file)
            for loc in locations_data:
                name = str(loc["name"])
                name = name.replace("#", "") # remove hashtags
                
                # convert letters to numbers
                words = []
                for word in name.split():
                    if word.isdigit():
                        words.append(n2w(int(word)))
                    else:
                        words.append(word)
                name = " ".join(words)
                name = name.lower()

                location_mapping[name] = loc["id"]
                
        self.location_mapping = location_mapping

    def handle_voice_command_error(self, msg):
        """ROS subscriber callback. Forward the error message to the GUI thread and
        ensure the voice command button is disabled after the user sees the popup.
        """
        text = None
        try:
            # msg may be a std_msgs/String
            text = str(msg.data)
        except Exception:
            try:
                text = str(msg)
            except Exception:
                text = "Unknown voice command error"

        # Emit signal to GUI thread if possible; otherwise call handler directly.
        try:
            self.voiceErrorSignal.emit(text)
        except Exception:
            # Fallback: try to call handler directly (may be unsafe from ROS thread)
            try:
                self._handle_voice_error(text)
            except Exception:
                pass

    def _handle_voice_error(self, text):
        """Runs in the GUI thread: show a small popup with the message and disable the voice button."""
        # Small popup with the error message
        QtGui.QMessageBox.warning(self, "Voice command error", text)

        # Disable the voice command push button to prevent further interaction
        try:
            self.pushButtonVoiceCommand.setEnabled(False)
            # Also reset checked state and text for clarity
            try:
                self.pushButtonVoiceCommand.setChecked(False)
            except Exception:
                pass
            try:
                self.pushButtonVoiceCommand.setText(" &Turn on voice command")
            except Exception:
                pass
        except Exception:
            # ignore GUI errors during shutdown
            pass

    def close_application(self):
        """Prompts the user if they are sure they which to quit the application before quitting."""
        choice = QtGui.QMessageBox.question(self, 
                                            'Quit application?',
                                            "Are you sure you want to quit? Any unsaved changed will be lost!", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        
        if choice == QtGui.QMessageBox.Yes:
            print(NODE_NAME + "Closing Fleet Manager node...")
            QtCore.QCoreApplication.instance().quit()
        else:
            pass
    
    def closeEvent(self, event):
        """Takes control of the close event, making sure the user cannot close the application before prompting them."""
        event.ignore()
        self.close_application()
    
    def about(self):
        """Display a MessageBox with the application title, version number and general information."""
        text = "<center>" \
            "<h2>"+APPLICATION_TITLE+"</h2>" \
            "</center>" \
            "The ROS package rooster_fleet_manager is created by the Human " \
            "Robot Co-production research group at the Industrial Design " \
            "Engineering faculty of the Delft University of Technology." \
            "<p>Version: "+VERSION+"<br/>" \
            "License: Apache License version 2.0</p>"
        QtGui.QMessageBox.about(self, "About - " + APPLICATION_TITLE + ".", text)

#endregion      ### PyQt GUI ###





def load_orders_from_JSON(filepath):
    """ Function; Add one or multiple orders from a JSON file. """
    # Load JSON file into dictionary
    loaddata_dict = None
    with open(filepath) as json_loadfile:
        loaddata_dict = json.load(json_loadfile)
    
    # Add orders from loaded JSON dictionary to the Order tab Order list.
    for order_id in range(len(loaddata_dict)):
        order_info_dict = loaddata_dict[str(order_id)]
        item_keyword = order_info_dict["keyword"]
        item_priority = order_info_dict["priority"]
        item_order_args = order_info_dict["order_args"]
        item_arguments = ""
        for arg in item_order_args:
            item_arguments = item_arguments + " " + arg
        order_item = QtGui.QTreeWidgetItem([item_keyword, item_priority, item_arguments])
        appGui.treeWidgetOrders.addTopLevelItem(order_item)

def job_list_cb(event):
    """
    Timer callback function, attempts to call Job Manager 'get_pending_jobs' & 
    'get_active_jobs' services for updating the Jobs treeWidget list.
    """
    combined_jobs_list = []     # Empty list for the jobs to be appened to.

    # Retrieve pending jobs.
    try:
        rospy.wait_for_service('/job_manager/get_pending_jobs', rospy.Duration(1))
        try:
            get_pending_jobs = rospy.ServiceProxy('/job_manager/get_pending_jobs', GetPendingJobs)
            req = GetPendingJobsRequest()
            resp = get_pending_jobs(req)
            if resp.jobs_count > 0:
                # Add jobs to combined_jobs_list
                for job in resp.jobs:
                    # [0 ID, 1 Priority, 2 Keyword, 3 Status, 4 MEx ID, 5 Task Count, 6 Current Task, 7 processed]
                    combined_jobs_list.append([job.job_id, job.priority, job.keyword, JobStatus.PENDING.name, None, job.task_count, 0, False])
        except rospy.ServiceException as e:
            print(NODE_NAME + "Service call failed: %s"%e)
    except rospy.ROSException:
        pass

    # Retrieve active jobs.
    try:
        rospy.wait_for_service('/job_manager/get_active_jobs', rospy.Duration(1))
        try:
            get_active_jobs = rospy.ServiceProxy('/job_manager/get_active_jobs', GetActiveJobs)
            req = GetActiveJobsRequest()
            resp = get_active_jobs(req)
            if resp.jobs_count > 0:
                # Add jobs to combined_jobs_list
                for job in resp.jobs:
                    # [0 ID, 1 Priority, 2 Keyword, 3 Status, 4 MEx ID, 5 Task Count, 6 Current Task, 7 processed]
                    combined_jobs_list.append([job.job_id, job.priority, job.keyword, job.status, job.mex_id, job.task_count, job.current_task+1, False])
        except rospy.ServiceException as e:
            print(NODE_NAME + "Service call failed: %s"%e)
    except rospy.ROSException:
        pass

    # if len(combined_jobs_list) > 0:
    update_jobs_list(combined_jobs_list)

def update_jobs_list(combined_jobs_list):
    """
    Add new/update existing items in the Jobs treeWidget with response information:
    1. Take in pending jobs and active jobs info into a single list.
    2. Loop over exisiting Job items in the treeWidget.
    3. If the job is in the list, update information.
    4. If it's not in the list, mark index for removal.
    5. If not all list items have been processed, this means it's new. Add new job items to the treeWidget.
    6. Loop over indices_for_removal list in descending order and remove all job items no longer existing.
    """
    # [2, 3, 4] First check if current Job items in the treeWidget require updating or removing.
    root = appGui.treeWidgetJobs.invisibleRootItem()
    child_count = root.childCount()
    indices_for_removal = []        # Empty list to which indices can be appended which can be removed after updating others.
    for i in range(child_count):    # Iterate over all the existing (top level) items (a.k.a. jobs) in the treeWidgetJobs.
        item = root.child(i)
        job_id = str(item.text(0))

        # Loop over all job information lists in the the combined_jobs_list, checking if the 'job_id' is in there.
        for job_info_list in combined_jobs_list:
            if job_id in job_info_list:
                # It's in there, so update the information and mark processed as True.
                item.setText(0, job_info_list[0])
                item.setText(1, str(JobPriority[str(job_info_list[1])].value)+" / "+job_info_list[1])
                item.setText(2, job_info_list[2])
                item.setText(3, job_info_list[3])
                item.setText(4, "\xA0" if job_info_list[4] == None else ""+str(job_info_list[4]) )
                item.setText(5, str(job_info_list[6])+" / "+str(job_info_list[5]) )
                job_info_list[7] = True
                break
        else:
            # The job item with it's job id is no longer in the Job Managers jobs lists, thus mark for removal.
            indices_for_removal.append(i)
    
    # [5] Check for unprocessed list items, adding them as new items to the jobs treeWidget
    for job_info_list in combined_jobs_list:
        if job_info_list[7] == False:
            job_item = QtGui.QTreeWidgetItem( [ 
                str(job_info_list[0]), 
                str(JobPriority[str(job_info_list[1])].value)+" / "+job_info_list[1], 
                str(job_info_list[2]), 
                str(job_info_list[3]), 
                "\xA0" if job_info_list[4] == None else ""+str(job_info_list[4]), 
                str(job_info_list[6])+"/"+str(job_info_list[5]) ] )
            appGui.treeWidgetJobs.addTopLevelItem(job_item)

    # [6] Remove items which were marked for removal
    if len(indices_for_removal) != 0:
        # Sort the indices list in descending order (from highest index to lowest index).
        indices_for_removal.sort(reverse=True)
        # Iterate over the sorted list, removing items from the treeWidgetJobs
        for index in indices_for_removal:
            appGui.treeWidgetJobs.takeTopLevelItem(index)
        
    
def mex_list_info_cb(data):
    """
    Subscription callback for the MEx Sentinel mex_list_info topic. 
    1. Take in MEx info.
    2. Loop over exisiting MEx items in the treeWidget.
    3. If the MEx is in the list, update information.
    4. If it's not in the list, mark index for removal.
    5. If not all list items have been processed, this means it's new. Add new MEx items to the treeWidget.
    6. Loop over indices_for_removal list in descending order and remove all MEx items no longer existing.
    """
    if data.total_mex_number > 0:
        # [1] Take in MEx info, add all MExs in list to a temporary dictionary.
        temp_dict = {}                  # Empty dictionary in which items will be marked as processed (True) or not (False).
        for mex_info in data.mex_list_info_array:
            temp_dict[str(mex_info.id)] = {
                "id" : mex_info.id,
                "job_id" : mex_info.job_id,
                "status" : mex_info.status,
                "processed" : False
            }   # Add incoming MEx info to temp_dict as dict and mark as not yet processed.

        # [2, 3, 4] First check if current MEx items in the treeWidget require updating or removing.
        root = appGui.treeWidgetMEx.invisibleRootItem()
        child_count = root.childCount()
        indices_for_removal = []        # Empty list to which indices can be appended which can be removed after updating others.

        for i in range(child_count):    # Iterate over all the existing (top level) items (a.k.a. MExs) in the treeWidgetMEx.
            item = root.child(i)
            mex_id = str(item.text(0))

            # Loop over all MEx information lists in the the mex_list_info_array, checking if the 'mex_id' is in there.
            for mex_info in data.mex_list_info_array:
                if mex_id == mex_info.id:
                    # It's in there, so update the information and mark processed as True.
                    item.setText(0, str(mex_info.id) )
                    item.setText(1, str(mex_info.status) )
                    item.setText(2, str(mex_info.job_id) )
                    temp_dict[str(mex_info.id)]["processed"] = True      # Mark MEx as processed.
                    break
            else:
                # The job item with it's job id is no longer in the Job Managers jobs lists, thus mark for removal.
                indices_for_removal.append(i)

        # [5] Check for unprocessed list items, adding them as new items to the jobs treeWidget
        for mex_key in temp_dict:
            if temp_dict[mex_key]["processed"] == False:
                mex_item = QtGui.QTreeWidgetItem( [ 
                    str(temp_dict[mex_key]["id"]), 
                    str(temp_dict[mex_key]["status"]),
                    str(temp_dict[mex_key]["job_id"]) ] )
                appGui.treeWidgetMEx.addTopLevelItem(mex_item)
        
        #  [6] Remove items which were marked for removal
        if len(indices_for_removal) != 0:
            # Sort the indices list in descending order (from highest index to lowest index).
            indices_for_removal.sort(reverse=True)
            # Iterate over the sorted list, removing items from the treeWidgetJobs
            for index in indices_for_removal:
                appGui.treeWidgetMEx.takeTopLevelItem(index)

if __name__ == '__main__':
    try:     
        # Initialize the node.
        rospy.init_node('fleet_manager_front')

        # Set up information update connections.
        rospy.Subscriber('/mex_sentinel/mex_list_info', MexListInfo, mex_list_info_cb)      # Subscription to MEx Sentinel for updating Mobile Executors list.
        rospy.Timer(rospy.Duration(1), job_list_cb)                                         # Timer for updating Jobs list.
        
        #region --- GUI ---
        app = QtGui.QApplication(sys.argv)
        appGui = GuiMainWindow()
        windowIcon = QtGui.QIcon()
        windowIcon.addPixmap(QtGui.QPixmap(":/icons/Fleet ManagerIcon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        appGui.setWindowIcon(windowIcon)

        # Add orders from example_orders.JSON
        filename = "example_orders.JSON"
        if not os.path.isfile(filename):
            # File does not exist yet (first time usage). Create it first from dictionary.
            order_data = {
                "0" : {
                    "keyword" : "MOVE",
                    "priority" : "LOW",
                    "order_args" : ["loc03"]
                },
                "1" : {
                    "keyword" : "MOVE",
                    "priority" : "MEDIUM",
                    "order_args" : ["loc02"]
                },
                "2" : {
                    "keyword" : "TRANSPORT",
                    "priority" : "LOW",
                    "order_args" : ["loc01", "loc02"]
                },
                "3" : {
                    "keyword" : "LOAD",
                    "priority" : "MEDIUM",
                    "order_args" : []
                },
                "4" : {
                    "keyword" : "UNLOAD",
                    "priority" : "HIGH",
                    "order_args" : []
                },
                "5" : {
                    "keyword" : "TRANSPORT",
                    "priority" : "CRITICAL",
                    "order_args" : ["loc04", "loc01"]
                }
            }
            with open(filename, 'w') as outfile:
                json.dump(order_data, outfile, indent=4)
            pass
        load_orders_from_JSON(filename)

        appGui.show()
        app.exec_()
        #endregion
    except rospy.ROSInterruptException:
        pass