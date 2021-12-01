#! /usr/bin/env python
"""
File         : user_control.py
Information  : This code file is used to handle the different package nodes and functions in a neat fashion.
Author       : Mohamed Tolba
Last Modified: 28 Nov 2021
"""
import os
import subprocess
import time

import rospy
from move_robot import move_jackal
from finders_keepers.msg import ctrl_msg
from finders_keepers.msg import tag_msg

class ctrl_jackal():
    def __init__(self):

        # Message objects
        self.tag = tag_msg()
        self.ctrl = ctrl_msg()
        self.move = move_jackal()

        # Reading the tag_ids.txt file
        self.tag_ids = []
        stream = os.popen('rospack find finders_keepers')
        directory = stream.readline()
        self.package_directory = directory[:-1]
        self.txt_file_path = str(self.package_directory)+"/src/tag_ids.txt"
        with open(self.txt_file_path, 'r') as f:
            for line in f:
                self.tag_ids.append(int(line))
        self.tag_ids.sort()

        # Initialize Publisher(s)
        self.ctrl_pub = rospy.Publisher('/ctrl', ctrl_msg, queue_size=1)

        # Initialize Subscriber(s)
        self.tag_sub = rospy.Subscriber('/april_tags', tag_msg, self.tag_sub_cb, queue_size=1)
        self.ctrl_sub = rospy.Subscriber('/ctrl', ctrl_msg, self.ctrl_sub_cb, queue_size=1)

        # Initial values
        self.ctrl_cmd = ''
        self.explore_node_flag = False

        # Other Stuff
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz
        rospy.on_shutdown(self.shutdownhook)

        # Launching /move_base and /slam_gmapping nodes
        stream = os.popen('rosnode list | grep /slam_gmapping')
        is_running = stream.readline()
        if not is_running:
            rospy.loginfo("Launching /move_base and /slam_gmapping nodes")
            os.popen("gnome-terminal --tab -- bash -c 'roslaunch finders_keepers odom_nav.launch ; exec bash' ")
            time.sleep(2)
            os.popen("gnome-terminal --tab -- bash -c 'roslaunch finders_keepers gmapping.launch ; exec bash' ")
            time.sleep(2)
        else:
            stream = os.popen('rosnode list | grep /move_base')
            is_running = stream.readline()
            if not is_running:
                rospy.loginfo("Launching /move_base node")
                os.popen("gnome-terminal --tab -- bash -c 'roslaunch finders_keepers odom_nav.launch ; exec bash' ")
                time.sleep(2)

        rospy.loginfo("/move_base and /slam_gmapping nodes have been launched in two new different tabs")
        time.sleep(2)

    def tag_sub_cb(self,msg):
        self.tag = msg

    def ctrl_sub_cb(self,msg):
        self.ctrl = msg.explore_find_status
        if self.ctrl == "no_frontiers":
            rospy.loginfo("No more frontiers....Exploration finished")
            rospy.loginfo("Go to the explore_find terminal tab for more options")
            print("\n")
            time.sleep(1)
            print("\n")
            print "The list of control commands are:"
            print "\t 'explore':   starts /explore_find node (if it is not already running)"
            print "\t 'tags':      shows the ids of the identified april tags"
            print "\t 'save_map':  saves the map constructed so far"
            print "\t 'stop':      kills the exploration process and stops the robot"
            print "\t 'restart':   restarts /move_base and /explore_find nodes"
            print "\t 'shutdown':  stops the robot and shuts down"
            print "\t 'exit_ctrl': kills the ctrl node only"
            print "\t 'clear':     clears the terminal"
            print "\n"
            print "Enter your command: "

            self.ctrl = ''

    def shutdownhook(self):
        self.ctrl_c = True

    def command(self):
        while not self.ctrl_c:
            self.ctrl_cmd = ''
            time.sleep(1)
            print("\n")
            print "The list of control commands are:"
            print "\t 'explore':   starts /explore_find node (if it is not already running)"
            print "\t 'tags':      shows the ids of the identified april tags"
            print "\t 'save_map':  saves the map constructed so far"
            print "\t 'stop':      kills the exploration process and stops the robot"
            print "\t 'restart':   restarts /move_base and /explore_find nodes"
            print "\t 'shutdown':  stops the robot and shuts down"
            print "\t 'exit_ctrl': kills the /ctrl_jackal node only"
            print "\t 'clear':     clears the terminal"
            print "\n"

            self.ctrl_cmd = input("Enter your command: ")

            if self.ctrl_cmd == 'explore':
                stream = os.popen('rosnode list | grep explore_find')
                is_running = stream.readline()
                if is_running:
                    self.explore_node_flag = True
                    subprocess.call(["clear"], shell=True)
                    rospy.loginfo("/explore_find node is running!")
                else:
                    self.explore_node_flag = False
                    os.popen(
                        "gnome-terminal --tab -- bash -c 'roslaunch finders_keepers explore_find.launch ; exec bash' ")
                    subprocess.call(["clear"], shell=True)
                    rospy.loginfo("/explore_find node has been launched in a new tab")
                    time.sleep(2)
            ############################################################################################################
            elif self.ctrl_cmd == 'tags':
                self.tag_ids = []
                with open(self.txt_file_path, 'r') as f:
                    for line in f:
                        self.tag_ids.append(int(line))
                self.tag_ids.sort()
                rospy.loginfo("The total number of april tags identified is " + str(
                    len(self.tag_ids)) + " with tag-id number(s): " + str(self.tag_ids))
            ############################################################################################################
            elif self.ctrl_cmd == 'save_map':
                map_directory = self.package_directory + "/map"
                subprocess.call(["cd " + map_directory + " && rosrun map_server map_saver -f map"], shell=True)
            ############################################################################################################
            elif self.ctrl_cmd == 'stop':
                self.move.stop_jackal()
                self.kill_node('/explore_find')
            ############################################################################################################
            elif self.ctrl_cmd == 'restart':
                os.popen("gnome-terminal --tab -- bash -c 'roslaunch finders_keepers odom_nav.launch ; exec bash' ")
                time.sleep(2)
                os.popen("gnome-terminal --tab -- bash -c 'roslaunch finders_keepers explore_find.launch ; exec bash' ")
                time.sleep(2)
            ############################################################################################################
            elif self.ctrl_cmd == 'shutdown':
                ans = input("Do you want to clear the april tag ids identified ('Y'/'N')? :")
                if ans == 'Y':
                    rospy.loginfo('Clearing the tag_ids.txt file...')
                    self.tag_ids = []
                    stream = os.popen('rospack find finders_keepers')
                    directory = stream.readline()
                    self.package_directory = directory[:-1]
                    self.txt_file_path = str(self.package_directory) + "/src/tag_ids.txt"
                    with open(self.txt_file_path, 'w') as f:
                        f.write('')

                    rospy.loginfo('Deleting the taken pictures for the tags...')
                    pics_directory = str(self.package_directory) + "/pics"
                    subprocess.call(["cd " + pics_directory + " && rm *"], shell=True)

                else: pass
                ans = input("Do you want to save the created map ('Y'/'N')? :")
                if ans == 'Y':
                    map_directory = self.package_directory + "/map"
                    subprocess.call(["cd " + map_directory + " && rosrun map_server map_saver -f map"], shell=True)
                else: pass

                rospy.loginfo("Shutting down!")
                self.move.stop_jackal()
                self.kill_node('/explore_find')
                self.kill_node('/move_base')
                self.kill_node('/slam_gmapping')
                rospy.signal_shutdown("Shutdown request")
            ############################################################################################################
            elif self.ctrl_cmd == 'exit_ctrl':
                self.kill_node('/ctrl_jackal')
            ############################################################################################################
            elif self.ctrl_cmd == 'clear':
                subprocess.call(["clear"], shell=True)
            ############################################################################################################
            else:
                rospy.loginfo("Please enter a command from the command list")
    def kill_node(self, node):
        subprocess.call(["rosnode kill " + node], shell=True)
        subprocess.call(["rosnode cleanup"], shell=True)

def main():
    rospy.init_node('ctrl_jackal')
    start = ctrl_jackal()
    start.command()
    try:
        rospy.spin()
    except (rospy.ROSInterruptException,KeyboardInterrupt):
        pass

if __name__ == "__main__":
    main()








