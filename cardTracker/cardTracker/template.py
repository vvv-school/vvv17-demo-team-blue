import numpy as np
import cv2


class Template(object):
    
    def __init__(self, tid, contour):
        self.tid        = tid
        self.contour    = contour


        # some processing
        self.peri    = cv2.arcLength(contour, True)
        self.approx  = cv2.approxPolyDP(contour, 0.02 * self.peri, True)

        # bounding box
        x, y, w, h = cv2.boundingRect(self.approx)
        self.tl    = ( x,    y)
        self.br    = ( x +w, y+h  )



    def highlite_marker(self, image, text_color=(255, 0, 0), linewidth=5):
        cv2.polylines(image, [np.int32(self.bbox)], True, 255, linewidth)
        center = 100, 100
        cv2.putText(image, str(self.tid), center, cv2.FONT_HERSHEY_SIMPLEX, 2, text_color)


    def getBBox(self):
        return self.bbox