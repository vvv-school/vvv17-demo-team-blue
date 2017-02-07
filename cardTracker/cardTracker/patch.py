import numpy as np
import cv2



class Patch(object):
    """ The Patch class provides an image patch container with additional information. """

    LINE_WIDTH = 3
    

    def __init__(self, contour, image):
        self.tid     = 0
        self.contour = contour
        self.image   = image

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

        # more expensive processing
        if self.isRectangle() and self.isConvex():

            
            self.patch   = self.image[ x : x + w, y : y + h  ]
            print self.patch.shape


    def highlite(self):
        """ Just a convenience method that calls some draw methods."""
        self.drawCenter()
        self.drawBBox()
        self.drawContours()


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