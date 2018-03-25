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

# import screeninfo
from os import listdir
from os.path import isfile, join


class Projector:

    def __init__(self):

        self.GS_WAITING = 1
        self.GS_GAME_RUNNING = 2
        self.GS_GAME_LOST = 3
        self.GS_GAME_WON = 4

        self.game_state = self.GS_WAITING
        # Load first Level


        self.bridge = CvBridge()

        self.level_base_directory = "/home/laurenz/catkin_ws/src/projector/levels/"
        self.window_name = "window"
        # get the size of the screen
        # screen_id = 1
        # screen = screeninfo.get_monitors()[screen_id]
        # cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
        # cv2.moveWindow(self.window_name, screen.x - 1, screen.y - 1)
        # cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # self.level_image = None  # self.load_level(1)  #  for Waiting state
        self.health = 100.0
        self.progress = 0.0
        self.levels = list()
        self.current_level_id = -1
        self.health_boost_level = 10
        self.level_start_time = None
        self.good_match_percentage = 40
        self.progress_step = 20
        self.max_progress = 100.0
        self.initial_health = 100.0

        self.health_loss_per_second = -1.0
        # self.time_since_last_update = None

        self.last_update_time = None

        self.image_sub = rospy.Subscriber("/CloudGateWay/flat/image", Image, self.input_callback)
        self.image_pub = rospy.Publisher("/game_window", Image, queue_size=1)
        self.image_pub_compressed = rospy.Publisher("/game_window/compressed", CompressedImage, queue_size=1)

        self.initial_level = cv2.imread(self.level_base_directory+'/single/0.png')
        self.level_image = cv2.cvtColor(self.initial_level, cv2.COLOR_BGR2GRAY)

    def total_level_count(self):
        return len(self.levels)

    def read_levels_from_file(self, single_player=True):
        level_directory = self.level_base_directory + "single/" if single_player else "multi/"
        onlyfiles = [f for f in listdir(level_directory) if isfile(join(level_directory, f))]

        self.levels = list()
        for local_name in onlyfiles:
            full_path = join(level_directory, local_name)
            img = cv2.imread(full_path)
            if img is None:
                rospy.logerr('level img from file \'' + full_path + '\' is None')
                continue
                # return np.zeros((300, 480), dtype=np.uint8)
            print(full_path)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #return img_gray
            self.levels.append(img_gray)
            print ("Loaded %i levels from %s" % (self.total_level_count(), level_directory))

        return self.total_level_count()


    def start_game(self, single_player=True):
        assert self.game_state == self.GS_WAITING
        self.game_state = self.GS_GAME_RUNNING
        self.health = self.initial_health
        self.read_levels_from_file()
        # load levels from correct folder (single or multi player)
        self.current_level_id = -1
        self.start_next_level()


    def start_next_level(self):
        assert self.game_state == self.GS_GAME_RUNNING
        self.current_level_id += 1
        if self.current_level_id == self.total_level_count():
            print ("GAME WAS FINISHED AFTER %i LEVELS" % self.current_level_id)
            self.game_state = self.GS_GAME_WON
            return False

        print("Loading level %i" % self.current_level_id)
        self.level_image = self.levels[self.current_level_id]
        self.health += min(self.health_boost_level, self.initial_health)
        self.level_start_time = rospy.Time.now()
        self.progress = 0

        return True

    def input_callback(self, data):

        try:
            input_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            assert False
            return

        now = rospy.Time.now()
        if self.last_update_time is None:
            self.last_update_time = now

        # blur and threshold input image
        input_image_gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.blur(input_image_gray, (6, 6))
        ret, input_image_gray = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

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
            = [255, 0, 0]
        outside_pixels_toomuch[np.where((outside_pixels_toomuch == [255, 255, 255]).all(axis=2))] \
            = [0, 0, 255]
        level_and_input[np.where((level_and_input == [255, 255, 255]).all(axis=2))] \
            = [0, 255, 0]

        composite_image = cv2.add(inside_pixels_missed, outside_pixels_toomuch)
        composite_image = cv2.add(composite_image, level_and_input)

        composite_image_with_text = cv2.putText(composite_image, str(percent_matched)+"%", (10,100), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255))

        draw_health_bars = True
        if self.game_state == self.GS_WAITING:
            draw_health_bars = False
            # checking for filled shape:
            if percent_matched > 40:
                # TODO: single or multi player
                print("Starting Game, counting players")
                self.start_game(single_player=True)

        if self.game_state == self.GS_GAME_RUNNING:
            if (now-self.level_start_time).to_sec() < 3.0:
                print("3sec grace period")
            else:
                time_since_last_update = (now - self.last_update_time).to_sec()
                self.health += self.health_loss_per_second * time_since_last_update
                self.last_update_time = now

                good_state = percent_matched > self.good_match_percentage
                # print("Good state: %i" % good_state)

                self.progress += (1 if good_state else -1) * self.progress_step * time_since_last_update
                print("Progress: %f" % self.progress)
                if self.progress > self.max_progress:
                    print("Level won")
                    self.start_next_level()

                if self.health < 0:
                    print("Game LOST")
                    draw_health_bars = False
                    self.game_state = self.GS_GAME_LOST


        if self.game_state == self.GS_GAME_LOST:
            print("SHOW LOST SCREEN")

        if self.game_state == self.GS_GAME_WON:
            print("SHOW WINNING SCREEN")


        # calculate health and progress
        if draw_health_bars:
            # draw health and progress bars
            composite_image_with_text_and_bargraphs = Bargraphs.drawHealthBar(composite_image_with_text, self.health)
            composite_image_with_text_and_bargraphs = Bargraphs.drawProgressBar(composite_image_with_text_and_bargraphs, self.progress)
        else:
            composite_image_with_text_and_bargraphs = composite_image_with_text

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
