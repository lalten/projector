#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import numpy as np
import sys
import rospy
import cv2
import random

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from projector.bargraphs import Bargraphs

from os import listdir, environ
from os.path import isfile, join


class Projector:

    def __init__(self):

        self.GS_WAITING = 1
        self.GS_GAME_RUNNING = 2
        self.GS_GAME_LOST = 3
        self.GS_GAME_WON = 4

        self.game_state = self.GS_WAITING
        # self.game_state = self.GS_GAME_LOST

        self.bridge = CvBridge()

        self.level_base_directory =environ['HOME']+"/catkin_ws/src/projector/levels/"
        self.window_name = "window"
        # cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        # cv2.startWindowThread()

        self.levels = list()
        self.current_level_id = -1
        self.level_start_time = rospy.Time.now()
        self.initial_level = cv2.imread(self.level_base_directory+'/single/0.png')
        self.level_image = cv2.cvtColor(self.initial_level, cv2.COLOR_BGR2GRAY)

        self.start_game_screen = cv2.imread(self.level_base_directory+'startgame.png')
        self.game_lost_screen = cv2.imread(self.level_base_directory+'gameover.png')
        self.game_lost_display_time = 5.0
        self.you_win_screen = cv2.imread(self.level_base_directory+'youwin.png')
        self.you_win_display_time = 20.0
        self.heart_icon = cv2.imread(self.level_base_directory+'heart.png')
        self.flash_icon = cv2.imread(self.level_base_directory+'flash.png')

        self.progress = 0.0
        self.good_match_percentage = 50
        self.progress_step = 0
        self.progress_step_max = 30
        self.max_progress = 100.0
        self.percent_matched = 0

        self.draw_health_bars = False
        self.health = 100.0
        self.initial_health = 100.0
        self.health_loss_per_second = -5.0
        self.health_boost_level = 20.0

        self.last_update_time = None

        self.composite_image = None
        self.input_image_gray = None
        self.input_image_background = None

        self.image_sub = rospy.Subscriber("/CloudGateWay/flat/image", Image, self.input_callback)
        self.image_pub = rospy.Publisher("/game_window", Image, queue_size=1)
        self.image_pub_compressed = rospy.Publisher("/game_window/compressed", CompressedImage, queue_size=1)

        rospy.loginfo("Projector was created")

    def total_level_count(self):
        return len(self.levels)

    def read_levels_from_file(self, single_player=True):
        level_directory = self.level_base_directory + "single/" if single_player else "multi/"
        onlyfiles = [f for f in listdir(level_directory) if isfile(join(level_directory, f))]

        onlyfiles = sorted(onlyfiles)
        print(onlyfiles)

        self.levels = list()
        for local_name in onlyfiles:
            full_path = join(level_directory, local_name)
            img = cv2.imread(full_path)
            if img is None:
                rospy.logerr('level img from file \'' + full_path + '\' is None')
                continue
            print(full_path)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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

    def stop_game(self):
        self.game_state = self.GS_WAITING
        self.health = self.initial_health
        self.current_level_id = -1
        self.start_next_level()


    def start_next_level(self):
        self.current_level_id += 1
        self.level_start_time = rospy.Time.now()
        if self.current_level_id == self.total_level_count():
            print ("GAME WAS FINISHED AFTER %i LEVELS" % self.current_level_id)
            self.draw_health_bars = False
            self.game_state = self.GS_GAME_WON
            return False

        print("Loading level %i" % self.current_level_id)
        self.level_image = self.levels[self.current_level_id]
        self.health = min(self.health + self.health_boost_level, self.initial_health)
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
        ret, self.input_image_gray = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        self.input_image_background = cv2.inRange(blurred, 1, 60)

        # level_or_input = cv2.bitwise_or(self.level_image, thresholded)
        level_and_input = cv2.bitwise_and(self.level_image, self.input_image_gray)
        inside_pixels_missed = self.level_image - level_and_input
        outside_pixels_toomuch = self.input_image_gray - level_and_input

        num_inside_pixels_missed = cv2.countNonZero(inside_pixels_missed)
        num_outside_pixels_toomuch = cv2.countNonZero(outside_pixels_toomuch)
        num_level_and_input = cv2.countNonZero(level_and_input)
        sum = (num_level_and_input + num_inside_pixels_missed + num_outside_pixels_toomuch)
        self.percent_matched = int(round(100.0 * num_level_and_input / sum)) if sum != 0 else 0

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

        # background first:
        self.composite_image = cv2.cvtColor(self.input_image_background, cv2.COLOR_GRAY2BGR)

        # add color layers
        self.composite_image = cv2.add(self.composite_image, outside_pixels_toomuch)
        self.composite_image = cv2.add(self.composite_image, inside_pixels_missed)
        self.composite_image = cv2.add(self.composite_image, level_and_input)

        if self.game_state == self.GS_WAITING:
            self.draw_health_bars = False
            # checking for filled shape:
            if self.percent_matched > 40:
                # TODO: single or multi player
                print("Starting Game, counting players")
                self.start_game(single_player=True)

        self.redraw()

    def redraw(self):

        if self.composite_image is None:
            # we're not ready yet
            return

        final_image = self.composite_image

        now = rospy.Time.now()
        if self.last_update_time is None:
            self.last_update_time = now
        time_since_last_update = (now - self.last_update_time).to_sec()

        if self.game_state == self.GS_WAITING:
            blink_state_visible = ((now - self.level_start_time).to_sec() % 1.0) < 0.7
            if blink_state_visible:
                ret, game_screen_mask = cv2.threshold(self.start_game_screen, 127, 255, cv2.THRESH_BINARY_INV)
                final_image = cv2.bitwise_and(self.composite_image, game_screen_mask)
                final_image = cv2.add(final_image, self.start_game_screen)
                # final_image = cv2.putText(final_image, str(self.percent_matched)+"%", (50,30), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255))

        if self.game_state == self.GS_GAME_RUNNING:
            blink_state_visible = ((now - self.level_start_time).to_sec() % 0.4) < 0.2
            if (now-self.level_start_time).to_sec() < 2.0:
                self.draw_health_bars = blink_state_visible
            else:
                self.draw_health_bars = True
                self.progress_step = \
                    min( max(self.percent_matched - self.good_match_percentage, -self.progress_step_max), self.progress_step_max)
                self.health += self.health_loss_per_second * time_since_last_update

            self.progress += self.progress_step * time_since_last_update
            self.progress = max(self.progress, 0)

            final_image = cv2.putText(final_image, "Cover: "+str(self.percent_matched) + "%", (50, 30), cv2.FONT_HERSHEY_COMPLEX,
                                      0.5, (255, 255, 255))
            final_image = cv2.putText(final_image, "Level: "+str(self.current_level_id+1)+"/"+str(self.total_level_count()),
                                      (50,50), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255))

            if self.progress > self.max_progress:
                print("Level won")
                self.start_next_level()

            if self.health < 0:
                print("Game LOST")
                self.draw_health_bars = False
                self.level_start_time = now
                self.game_state = self.GS_GAME_LOST

        # calculate health and progress
        if self.draw_health_bars:
            # draw health and progress bars
            final_image = Bargraphs.drawHealthBar(final_image, self.health)
            final_image = Bargraphs.drawProgressBar(final_image, self.progress)
            y_offset = 15
            x_offset = 11
            final_image[y_offset:y_offset+self.heart_icon.shape[0],
                x_offset:x_offset + self.heart_icon.shape[1]] = self.heart_icon
            y_offset = 15
            x_offset = 31
            final_image[y_offset:y_offset+self.flash_icon.shape[0],
                x_offset:x_offset + self.flash_icon.shape[1]] = self.flash_icon

        if self.game_state == self.GS_GAME_LOST:
            max_rand = 4
            game_lost_screen_shifted = self.game_lost_screen
            (channel_b, channel_g, channel_r) = cv2.split(game_lost_screen_shifted)
            rows, cols = channel_b.shape
            tx = random.randint(-max_rand, max_rand)
            ty = random.randint(-max_rand, max_rand)
            translation_matrix = np.float32([[1, 0, tx], [0, 1, ty]])
            channel_b = cv2.warpAffine(channel_b, translation_matrix, (cols, rows))
            tx = random.randint(-max_rand, max_rand)
            ty = random.randint(-max_rand, max_rand)
            translation_matrix = np.float32([[1, 0, tx], [0, 1, ty]])
            channel_g = cv2.warpAffine(channel_g, translation_matrix, (cols, rows))
            tx = random.randint(-max_rand, max_rand)
            ty = random.randint(-max_rand, max_rand)
            translation_matrix = np.float32([[1, 0, tx], [0, 1, ty]])
            channel_r = cv2.warpAffine(channel_r, translation_matrix, (cols, rows))
            game_lost_screen_shifted = cv2.merge((channel_b, channel_g, channel_r))
            final_image = game_lost_screen_shifted
            if now > self.level_start_time + rospy.Duration(self.game_lost_display_time):
                self.stop_game()

        if self.game_state == self.GS_GAME_WON:
            game_won_shifted = cv2.cvtColor(self.you_win_screen, cv2.COLOR_BGR2HSV)
            (channel_h, channel_s, channel_v) = cv2.split(game_won_shifted)
            hue = ((now - self.level_start_time).to_sec() * 120.0) % 256
            channel_h[:] = hue
            game_won_shifted = cv2.merge((channel_h, channel_s, channel_v))
            final_image = cv2.cvtColor(game_won_shifted, cv2.COLOR_HSV2BGR)
            if now > self.level_start_time + rospy.Duration(self.game_lost_display_time):
                self.stop_game()

        self.last_update_time = now


        # publish as ros topic
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(final_image, encoding="bgr8"))
        self.image_pub_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(final_image))

        # resize and show bigger on local screen
        # final_image_fullscreen = cv2.resize(final_image, (1920, 1200))
        # cv2.imshow(self.window_name, final_image_fullscreen)

        # check if we want a save
        do_store = rospy.get_param("~store", False)
        if do_store:
            rospy.set_param("~store", False)
            print('save')
            cv2.imwrite('level_save_'+str(rospy.Time.now().to_sec())+'.png', self.input_image_gray)
            self.level_image = self.input_image_gray

    def timer_callback(self, event):
        self.redraw()

def main(args):
    rospy.init_node('game')
    rospy.loginfo('Started Game')
    p = Projector()
    rospy.Timer(rospy.Duration(1.0/60), p.timer_callback, oneshot=False)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
