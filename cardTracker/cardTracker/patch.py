import numpy as np
import cv2



class Patch(object):
    """ The Patch class provides an image patch container with additional information. """

    LINE_WIDTH = 1
    HIST_SIZE  = 20

    CLAHE        = cv2.createCLAHE( clipLimit = 2.0, tileGridSize = (8, 8) )


    def __init__(self, contour, image, rChannel, gChannel, bChannel):
        self.tid        = id(self) % 100000
        self.number     = -1
        self.num_hist   = []
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
        self.x, self.y, self.w, self.h   = cv2.boundingRect(self.approx)
        self.tl      = ( self.x,   self.y )
        self.br      = ( self.x +  self.w,      self.y +  self.h      )
        self.center  = ( self.x + (self.w / 2), self.y + (self.h / 2) )
        self.bbox    = ( self.tl, self.br )
        self.shape   = ( self.w,  self.h )
        
        self.size    = self.w * self.h
        self.area    = cv2.contourArea(self.contour)
        
        # assume the proper cards are of rectangle shape and we discard the cards that are put
        # horizontal
        self.card    = self.isRectangle() and self.h > self.w and self.area
        self.label   = ''

        # more expensive processing
        if self.isRectangle():
            self.findBlobs()
            self.label  = 'h' if self.countPixels() < 5 else 'p'
            self.label  = '%s%s' % (self.label, self.getEstimatedNumber())


    def highlite(self, image):
        """ Just a convenience method that calls some draw methods."""
        self.drawCenter(image)
        self.drawBBox(image)
        self.drawContours(image)
        cv2.putText(image, str(self.label), (self.x, self.y), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0))


    def drawCenter(self, image):
        """ Draws the center of the template on an image. """
        cv2.circle(image, self.center, self.LINE_WIDTH, (0, 255, 0), -1)


    def drawBBox(self, image):
        """ Draws the bounding box of the template on an image. """
        cv2.rectangle(image, self.tl, self.br, (255, 0, 0), self.LINE_WIDTH)


    def drawContours(self, image):
        """ Draws the bounding box of the template on an image. """
        cv2.drawContours(image, [self.approx], -1, (0, 255, 0), self.LINE_WIDTH)


    def isRectangle(self):
        """ Returns whether the template is a rectangle. """
        return self.edges == 4


    def belongsTo(self):
        return 'icub' if self.center[1] > 160 else 'human'

    
    def isConvex(self):
        """ Returns whether the image patch contains a convex contour."""
        return cv2.isContourConvex(self.contour)


    def isCard(self):
        return self.card


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
        

    def findBlobs(self):

        cropped   = self.image[self.y:self.y+self.h, self.x:self.x+self.w]

        rChannel = self.CLAHE.apply(cropped[:,:,2])
        gChannel = self.CLAHE.apply(cropped[:,:,1])
        bChannel = self.CLAHE.apply(cropped[:,:,0])

        _, rThreshed  = cv2.threshold(rChannel, 190, 255, cv2.THRESH_BINARY)
        _, gThreshed  = cv2.threshold(gChannel, 127, 255, cv2.THRESH_BINARY)
        _, bThreshed  = cv2.threshold(bChannel, 127, 255, cv2.THRESH_BINARY)


        cropped     = (gThreshed + bThreshed) / 2
        _, gray     = cv2.threshold(cropped, 10, 255, cv2.THRESH_BINARY)

        # filter
        gray        = cv2.bilateralFilter(gray, 11, 17, 17)
        edged       = cv2.Canny(gray, 30, 200)

        # get contours
        (cnts, _) = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # reverse sorting to find the 10 most outer ones
        contours  = sorted(cnts, key = cv2.contourArea, reverse = True)[1:10]
        
        # kill everything below threshold
        contours  = [contour for contour in contours if cv2.contourArea(contour) > 5.0]

        # get the areas for refinement
        areas     = [cv2.contourArea(contour) for contour in contours]

        # normalize areas
        areas     = [area / areas[0] for area in areas]

        self.number = len([area for area in areas if area > 0.3])
        self.num_hist.append(self.number)
        
        if self.isCard():
            print self.number, [cv2.contourArea(contour) for contour in contours], areas


    def overlaps(self, patch):
        assert isinstance(patch, Patch), patch
        
        return  self.range_overlap(self.x, self.x + self.w, patch.x, patch.x + patch.w) and \
                self.range_overlap(self.y, self.y + self.h, patch.y, patch.y + patch.h)    
    

    @staticmethod
    def range_overlap(a_min, a_max, b_min, b_max):
        return (a_min <= b_max) and (b_min <= a_max)
    
    
    def getEstimatedNumber(self):
        return np.argmax(np.bincount(np.array(self.num_hist)))

