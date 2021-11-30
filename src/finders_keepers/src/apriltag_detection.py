#! /usr/bin/env python
"""
File         : apriltag_detection.py
Information  : This code file subscribes to the camera-taken image. Then it detects and returns the id of the april tag
               in the image if there is one in it. It also saves the pictures where the april tags are found.
Author       : Mohamed Tolba
Last Modified: 27 Nov 2021
Credit       : This algorithm is the result of my understanding to the tutorial provided by Adrian Rosebrock
               (https://www.pyimagesearch.com/2020/11/02/apriltag-with-python/)
"""
import os
import subprocess
import time

import apriltag
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from finders_keepers.msg import tag_msg
from sensor_msgs.msg import Image

class AprilTagDetect(object):
    def __init__(self):

        # Message objects
        self.bridge_obj = CvBridge()
        self.tag = tag_msg()

        # Initialize Publisher(s)
        self.tag_pub = rospy.Publisher('/april_tags', tag_msg, queue_size=1)

        # Initialize Subscriber(s)
        self.cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.cam_sub_cb)

        # Initial values
        self.tag_ids = []
        stream = os.popen('rospack find finders_keepers')
        directory = stream.readline()
        self.txt_file_path = str(directory[:-1])+"/src/tag_ids.txt"
        with open(self.txt_file_path, 'r') as f:
            for line in f:
                self.tag_ids.append(int(line))

        self.cam_img = []
        self.flag = False

        # Other Stuff
        self.rate = rospy.Rate(10)  # 10hz

    def cam_sub_cb(self, data):
        try:
            self.cam_img = self.bridge_obj.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.flag = True
        except CvBridgeError as e:
            rospy.logwarn(str(e))

    def find_apriltag(self, show_picture = False):
        if show_picture:
            cv2.imshow("Camera_view", self.cam_img)
            cv2.waitKey(1)

        if self.flag:
            gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
            options = apriltag.DetectorOptions(families='tag36h11',
                                           border=1,
                                           nthreads=4,
                                           quad_decimate=1.0,
                                           quad_blur=0.0,
                                           refine_edges=True,
                                           refine_decode=False,
                                           refine_pose=False,
                                           debug=False,
                                           quad_contours=True)
            detector = apriltag.Detector(options)
            result = detector.detect(gray)
            if len(result) >= 1:
                tag_id = result[0].tag_id
                new_id = True
                for i in self.tag_ids:
                    if i == tag_id:
                        new_id = False
                        break
                if new_id == True:
                    self.tag_ids.append(tag_id)
                    self.tag.number_of_tags = len(self.tag_ids)
                    self.tag.tag_id = self.tag_ids
                    self.tag_pub.publish(self.tag)
                    self.tag_logger(self.tag_ids)

                    rospy.loginfo("Saving the picture")
                    stream = os.popen('rospack find finders_keepers')
                    directory = stream.readline()
                    tags_dir_path = str(directory[:-1]) + "/pics"
                    subprocess.call(["cd " + tags_dir_path], shell=True)
                    pic_name = tags_dir_path + '/' + 'tag_' + str(tag_id) + '.jpg'
                    cv2.imwrite(pic_name, gray)

                    return tag_id
                else:
                    return -1

            self.flag = False
        else:
            self.rate.sleep()
            return -1

    def tag_logger(self, tag_ids):
        with open(self.txt_file_path, 'w') as f:
            for tag_id in tag_ids:
                f.write(str(tag_id))
                f.write('\n')

def main():
    rospy.init_node("detect_april_tag", anonymous=True)
    obj = AprilTagDetect()
    time.sleep(1)
    obj.find_apriltag()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down !!")
    cv2.destroyAllWindows()
if __name__=='__main__':
    main()