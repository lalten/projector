
import cv2

class Bargraphs:

    bargraph_width = 10
    bargraph_height = 280
    border_color = (255,255,255)
    health_color = (0,255,0)
    progress_color = (255,255,0)
    border_thickness = 1

    @staticmethod
    def drawHealthBar(img, level):
        # extents
        tl = (10,10 + int(round((100.0-level)/100.0 * Bargraphs.bargraph_height)))
        br = (tl[0] + Bargraphs.bargraph_width, 10 + Bargraphs.bargraph_height)

        # draw actual bar
        img = cv2.rectangle(img, tl, br, Bargraphs.health_color, thickness=cv2.FILLED)

        # draw white border on top
        tl = (10, 10)
        img = cv2.rectangle(img, tl, br, Bargraphs.border_color, thickness=Bargraphs.border_thickness)

        return img

    @staticmethod
    def drawProgressBar(img, level):
        tl = (30,10 + int(round((100.0-level)/100.0 * Bargraphs.bargraph_height)))
        br = (tl[0] + Bargraphs.bargraph_width, 10 + Bargraphs.bargraph_height)

        # draw actual bar
        img = cv2.rectangle(img, tl, br, Bargraphs.progress_color, thickness=cv2.FILLED)


        # draw white border on top
        tl = (30, 10)
        img = cv2.rectangle(img, tl, br, Bargraphs.border_color, thickness=Bargraphs.border_thickness)

        return img
