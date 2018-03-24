#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import numpy as np
import sys
import rospy
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from bargraphs import Bargraphs

import screeninfo


class Projector:

    def __init__(self):
        self.bridge = CvBridge()

        self.window_name = "window"
        # get the size of the screen
        # screen_id = 1
        # screen = screeninfo.get_monitors()[screen_id]
        # cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
        # cv2.moveWindow(self.window_name, screen.x - 1, screen.y - 1)
        # cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.level_image = self.load_level(1)
        self.health = 100.0
        self.progress = 0.0

        self.health_loss_per_second = -1.0

        self.last_update_time = None

        self.image_sub = rospy.Subscriber("/CloudGateWay/flat/image", Image, self.input_callback)
        self.image_pub = rospy.Publisher("/game_window", Image, queue_size=1)
        self.image_pub_compressed = rospy.Publisher("/game_window/compressed", CompressedImage, queue_size=1)

    def load_level(self, number):
        # filename = "/home/laurenz/sketchbook/projector/levels/"+str(number)+".png"
        filename = 'level.png'
        img = cv2.imread(filename)
        if img is None:
            rospy.logerr('level img from file \''+filename+'\' is None')
            return np.zeros((300,480), dtype=np.uint8)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img_gray

    def input_callback(self, data):
        try:
            input_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        now = rospy.get_time()
        if self.last_update_time is None:
            self.last_update_time = now

        # blur and threshold input image
        input_image_gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.blur(input_image_gray, (6, 6))
        ret,input_image_gray = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

        # level_or_input = cv2.bitwise_or(self.level_image, thresholded)
        level_and_input = cv2.bitwise_and(self.level_image, input_image_gray)
        inside_pixels_missed = self.level_image - level_and_input
        outside_pixels_toomuch = input_image_gray - level_and_input

        num_inside_pixels_missed = cv2.countNonZero(inside_pixels_missed)
        num_outside_pixels_toomuch = cv2.countNonZero(outside_pixels_toomuch)
        num_level_and_input = cv2.countNonZero(level_and_input)
        sum = (num_level_and_input + num_inside_pixels_missed + num_outside_pixels_toomuch)
        percent_matched = int(round(100.0 * num_level_and_input / sum)) if sum != 0 else 0

        # color these pixels
        level_and_input = cv2.cvtColor(level_and_input, cv2.COLOR_GRAY2BGR)
        inside_pixels_missed = cv2.cvtColor(inside_pixels_missed, cv2.COLOR_GRAY2BGR)
        outside_pixels_toomuch = cv2.cvtColor(outside_pixels_toomuch, cv2.COLOR_GRAY2BGR)
        inside_pixels_missed[np.where((inside_pixels_missed == [255, 255, 255]).all(axis=2))] \
            = [255,0,0]
        outside_pixels_toomuch[np.where((outside_pixels_toomuch == [255, 255, 255]).all(axis=2))] \
            = [0,0,255]
        level_and_input[np.where((level_and_input == [255, 255, 255]).all(axis=2))] \
            = [0,255,0]

        composite_image = cv2.add(inside_pixels_missed, outside_pixels_toomuch)
        composite_image = cv2.add(composite_image, level_and_input)

        composite_image_with_text = cv2.putText(composite_image, str(percent_matched)+"%", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255))

        # calculate health and progress
        time_since_last_update = now - self.last_update_time
        self.health += self.health_loss_per_second * time_since_last_update
        self.last_update_time = now

        # draw health and progress bars
        composite_image_with_text_and_bargraphs = Bargraphs.drawHealthBar(composite_image_with_text, self.health)
        composite_image_with_text_and_bargraphs = Bargraphs.drawProgressBar(composite_image_with_text_and_bargraphs, self.progress)


        # publish as ros topic
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(composite_image_with_text_and_bargraphs, encoding="bgr8"))
        self.image_pub_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(composite_image_with_text_and_bargraphs))


        composite_image_fullscreen = cv2.resize(composite_image_with_text_and_bargraphs, (1920, 1200))
        cv2.imshow(self.window_name, composite_image_fullscreen)

        key = cv2.waitKey(1)
        if key == 32:
            print('save')
            cv2.imwrite('level.png', input_image_gray)
            self.level_image = input_image_gray
        elif key == 27:
            exit(0)
        elif key != -1:
            print(key)

def main(args):
    rospy.init_node('game')
    rospy.loginfo('started node')
    p = Projector()
    # try:
    rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
