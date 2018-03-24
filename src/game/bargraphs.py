
import cv2

class Bargraphs:

    bargraph_width = 10
    bargraph_height = 280
    border_color = (255,255,255)
    border_thickness = 1

    @staticmethod
    def drawProgressBar(img, level):
        # draw white border
        tl = (10,10)
        br = (tl[0] + Bargraphs.bargraph_width, tl[1] + Bargraphs.bargraph_height)
        img = cv2.rectangle(img, tl, br, Bargraphs.border_color, thickness=Bargraphs.border_thickness)
        return img

    @staticmethod
    def drawHealthBar(img, level):
        return img