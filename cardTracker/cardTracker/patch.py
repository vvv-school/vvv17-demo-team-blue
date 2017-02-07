import numpy as np
import cv2
from scipy.spatial import distance as dist

from collections import OrderedDict


class Patch(object):
    """ The Patch class provides an image patch container with additional information. """

    LINE_WIDTH = 3
    HIST_SIZE  = 20

    def __init__(self, contour, image):
        self.tid     = id(self) % 10000
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
        self.shape   = (w, h)
#         self.label   = ''
        self.w       = w
        self.h       = h
        self.size    = w * h

        # more expensive processing
        if self.isRectangle():


            # initialize the colors dictionary, containing the color
            # name as the key and the RGB tuple as the value
            colors = OrderedDict({
                "red": (255, 0, 0),
                "green": (0, 255, 0),
                "blue": (0, 0, 255)})
     
            # allocate memory for the L*a*b* image, then initialize
            # the color names list
            self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
            self.colorNames = []
     
            # loop over the colors dictionary
            for (i, (name, rgb)) in enumerate(colors.items()):
                # update the L*a*b* array and the color names list
                self.lab[i] = rgb
                self.colorNames.append(name)
     
            # convert the L*a*b* array from the RGB color space
            # to L*a*b*
            self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

            self.patch_color = self.image[ x : x + w, y : y + h  ].copy()
            self.patch_gray  = cv2.cvtColor(self.patch_color, cv2.COLOR_BGR2GRAY)
            self.mask        = np.zeros(self.patch_color.shape[:2], dtype="uint8")
            
            self.hist_b      = cv2.calcHist([self.patch_color], [0], None, [self.HIST_SIZE], [0, 256])
            self.hist_g      = cv2.calcHist([self.patch_color], [1], None, [self.HIST_SIZE], [0, 256])
            self.hist_r      = cv2.calcHist([self.patch_color], [2], None, [self.HIST_SIZE], [0, 256])
            self.hist_0      = cv2.calcHist([self.patch_gray],  [0], None, [self.HIST_SIZE], [0, 256])
            print self.shape
            print self.hist_0
#             print 'r:', np.sum(self.hist_r[10:])
#             print 'b:', np.sum(self.hist_g[10:])
#             print 'g:', np.sum(self.hist_b[10:])
            x = ((np.sum(self.hist_b[10:]) + np.sum(self.hist_g[10:]) + np.sum(self.hist_r[10:])) / 2) / (self.w * self.h)
#             print 'r->', self.hist_r[3] > self.hist_b[3] + self.hist_g[3]
#             print 'b->', self.hist_r[3] < (self.hist_b[3] + self.hist_g[3])

            r = np.sum((self.hist_r / self.size)[10:])
            g = np.sum((self.hist_g / self.size)[10:])
            b = np.sum((self.hist_b / self.size)[10:])
            red_ratio = (g + b) / 2 * r
                        
#             self.label = 'b' if (red_ratio > 0.01) else 'r'

            print self.label(), red_ratio, np.sum(self.hist_0[:10]) / np.sum(self.hist_0[:10])


    def highlite(self):
        """ Just a convenience method that calls some draw methods."""
#         self.drawCenter()
#         self.drawBBox()
#         self.drawContours()
        cv2.putText(self.image, str(self.label()), self.center, cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0))


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

    
    def label(self):
        # construct a mask for the contour, then compute the
        # average L*a*b* value for the masked region
        mask = np.zeros(self.patch_color.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [self.contour], -1, 255, -1)
        mask = cv2.erode(mask, None, iterations=2)
        mean = cv2.mean(self.patch_color, mask=mask)[:3]
 
        # initialize the minimum distance found thus far
        minDist = (np.inf, None)
 
        # loop over the known L*a*b* color values
        for (i, row) in enumerate(self.lab):
            # compute the distance between the current L*a*b*
            # color value and the mean of the image
            d = dist.euclidean(row[0], mean)
 
            # if the distance is smaller than the current distance,
            # then update the bookkeeping variable
            if d < minDist[0]:
                minDist = (d, i)
 
        # return the name of the color with the smallest distance
        return self.colorNames[minDist[1]]