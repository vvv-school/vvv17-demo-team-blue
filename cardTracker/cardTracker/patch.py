import numpy as np
import cv2
from scipy.spatial import distance as dist

from collections import OrderedDict


class Patch(object):
    """ The Patch class provides an image patch container with additional information. """

    LINE_WIDTH = 1
    HIST_SIZE  = 20


    def __init__(self, contour, image, rChannel, gChannel, bChannel):
        self.tid        = id(self) % 10000
        self.contour    = contour
        self.image      = image
        self.r_channel  = rChannel
        self.g_channel  = gChannel
        self.b_channel  = bChannel

        # some processing
        self.peri    = cv2.arcLength(contour, True)
        self.approx  = cv2.approxPolyDP(contour, 0.02 * self.peri, True)
        self.edges   = len(self.approx)

        # bounding box
        x, y, w, h   = cv2.boundingRect(self.approx)
        self.tl      = ( x,     y     )
        self.br      = ( x + w, y + h )
        self.center  = (x + (w / 2), y + (h / 2))
        self.bbox    = (self.tl, self.br)
        self.shape   = (w, h)
        self.x       = x
        self.y       = y
        
#         self.label   = ''
        self.w       = w
        self.h       = h
        self.size    = w * h
        self.area    = cv2.contourArea(self.contour)
        
        self.card    = self.isRectangle() 
        self.label   = ''

        # more expensive processing
        if self.isRectangle():
            self.label = 'r' if self.countPixels() < 5 else 'b'


    def highlite(self):
        """ Just a convenience method that calls some draw methods."""
        self.drawCenter()
#         self.drawBBox()
        self.drawContours()
        cv2.putText(self.image, str(self.label), self.center, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0))


    def drawCenter(self):
        """ Draws the center of the template on an image. """
        cv2.circle(self.image, self.center, self.LINE_WIDTH, (0, 255, 0), -1)


    def drawBBox(self):
        """ Draws the bounding box of the template on an image. """
        cv2.rectangle(self.image, self.tl, self.br, (255, 0, 0), self.LINE_WIDTH)


    def drawContours(self):
        """ Draws the bounding box of the template on an image. """
        cv2.drawContours(self.image, [self.approx], -1, (0, 255, 0), self.LINE_WIDTH)


    def isRectangle(self):
        """ Returns whether the template is a rectangle. """
        return self.edges == 4


    def isConvex(self):
        """ Returns whether the image patch contains a convex contour."""
        return cv2.isContourConvex(self.contour)


    def isCard(self):
        return self.isRectangle()


    def countPixels(self):

        black = 0
            
        for x in range(self.w):
            for y in range(self.h):
                relX = self.x + x
                relY = self.y + y
                
                if cv2.pointPolygonTest(self.approx, (relX, relY), 1) > 0.0:
                    px = self.r_channel[relY][relX]
                    if px < 10:
                        self.image[relY][relX] = ( 0, 0, 0 )
                        black += 1

        return black
        

    def estimateColor(self):
        pass