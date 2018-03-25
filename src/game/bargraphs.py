
import cv2

class Bargraphs:

    bargraph_width = 10
    bargraph_height = 250
    border_color = (255,255,255)
    health_color_perfect = (50,226,0)
    health_color_good= (39,231,255)
    health_color_medium = (0,163,255)
    health_color_bad = (77,0,255)
    progress_color = (255,173,41)
    border_thickness = 1

    @staticmethod
    def drawHealthBar(img, level):
        # extents
        tl = (10,30 + int(round((100.0-level)/100.0 * Bargraphs.bargraph_height)))
        br = (tl[0] + Bargraphs.bargraph_width, 30 + Bargraphs.bargraph_height)

        color = Bargraphs.health_color_bad
        if level > 25:
            color = Bargraphs.health_color_medium
        if level > 50:
            color = Bargraphs.health_color_good
        if level > 75:
            color = Bargraphs.health_color_perfect

        # draw actual bar
        img = cv2.rectangle(img, tl, br, color, thickness=cv2.FILLED)

        # draw white border on top
        tl = (10, 30)
        img = cv2.rectangle(img, tl, br, Bargraphs.border_color, thickness=Bargraphs.border_thickness)

        return img

    @staticmethod
    def drawProgressBar(img, level):
        tl = (30,30 + int(round((100.0-level)/100.0 * Bargraphs.bargraph_height)))
        br = (tl[0] + Bargraphs.bargraph_width, 30 + Bargraphs.bargraph_height)

        # draw actual bar
        img = cv2.rectangle(img, tl, br, Bargraphs.progress_color, thickness=cv2.FILLED)


        # draw white border on top
        tl = (30, 30)
        img = cv2.rectangle(img, tl, br, Bargraphs.border_color, thickness=Bargraphs.border_thickness)

        return img
