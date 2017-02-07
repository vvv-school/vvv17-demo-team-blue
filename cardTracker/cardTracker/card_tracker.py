import argparse
import os.path as op
import time
import yarp

try:
    import cv2
except ImportError:
    print '[CardTracker] Can not import cv2. This module will raise a RuntimeException.'

try:
    import numpy as np
    from numpy import argmax
except ImportError:
    print '[CardTracker] Can not import numpy. This module will raise a RuntimeException.'


from cardTracker.BaseModule  import BaseModule, main
from cardTracker.template    import Template


class CardTracker(BaseModule):
    """ The CardTracker class provides a yarp module for recognizing play cards.
    """

    D_WIDTH  = 320
    D_HEIGHT = 240

    O_HORIZONTAL = 0
    O_VERTICAL   = 1


    def __init__(self, args):
        BaseModule.__init__(self, args)
        self.memory_length  = args.memory
        self.translation    = args.translation
        self.translation_db = {}

        if self.translation:

            # check that we got a file
            if not op.isfile(self.translation):
                raise ValueError, 'Path [%s] does not point to a file.' % self.translation

            # read translation database
            with open(self.translation, 'r') as _file:
                for line in _file.read().split('\n'):
                    mid, word = line.split()
                    self.translation_db[int(mid)] = word


    def configure(self, rf):

        BaseModule.configure(self, rf)

        self.cardsPort       = self.createOutputPort('cards')
        self.orderPort       = self.createOutputPort('order')
        self.translationPort = self.createOutputPort('translation')

        self.imgInPort       = self.createInputPort('image')
        self.imgOutPort      = self.createOutputPort('image')

        self.bufImageIn,  self.bufArrayIn  = self.createImageBuffer(320, 240, 3)
        self.bufImageOut, self.bufArrayOut = self.createImageBuffer(320, 240, 3)

        self.order           = CardTracker.O_HORIZONTAL
        self.orderIsReversed = False

        self.memory          = {}

        return True


    def updateModule(self):

        if self.imgInPort.read(self.bufImageIn):

            # Make sure the image has not been re-allocated
            assert self.bufArrayIn.__array_interface__['data'][0] == self.bufImageIn.getRawImage().__long__()

            # convert image to be usable in OpenCV
            image  = cv2.cvtColor(self.bufArrayIn, cv2.COLOR_BGR2RGB)

            # and convert image back to something the yarpview can understand
            self.bufArrayOut[:,:] = cv2.cvtColor(self.onImage(image), cv2.COLOR_RGB2BGR)

            # Send the result to the output port
            self.imgOutPort.write(self.bufImageOut)

        return True


    def sendOrder(self, cards):
        """ This method sends the order information to the order port.

        Message: <cardid-1> <cardid-2> ... <cardid-n>

        All values are integer values.

        @param cards - list of Template objects
        """

        cards.sort(key = lambda x: x.center[self.order], reverse = self.orderIsReversed)

        bottle = yarp.Bottle()
        bottle.clear()

        for card in cards:
            bottle.addInt(card.tid)

        self.orderPort.write(bottle)


    def sendTranslation(self, cards):
        """ This method sends the translated card IDs to the translation port. Order depends on
            the order settings. In case no translation for a card ID is provided the card ID
            will be returned as string.

        Message: "<translation-1> <translation-2> ... <translation-n>"

        The message is one string containing the words separated by a space.

        @param cards - list of Template objects
        """

        cards.sort(key = lambda x: x.center[self.order], reverse = self.orderIsReversed)

        bottle = yarp.Bottle()
        bottle.clear()

        # create translation string list
        translation = []
        for card in cards:
            translation.append(str(self.translation_db.get(card.tid, card.tid)))

        # transmit the joined strings
        bottle.addString(' '.join(translation))

        self.translationPort.write(bottle)


    def sendCards(self, cards):
        """ This method sends the card information to the cards port.

        Message: <number of cards> ( ( <id> <center-x>  <center-y> <contour> )* )
                 <contour> = (<p1-x> <p1-y> <p2-x> <p2-y> <p3-x> <p3-y> <p4-x> <p4-y>)

        All values are integer values.

        @param cards - list of Template objects
        """

        bottle  = yarp.Bottle()
        bottle.clear()

        bottle.addInt(len(cards))
        cards_list = bottle.addList()

        for card in cards:

            center_x, center_y  = card.center
            card_values         = cards_list.addList()
            card_values.addInt(card.tid)
            card_values.addInt(center_x)
            card_values.addInt(center_y)
            card_contour = card_values.addList()

            for value in card.contours:
                card_contour.addInt(int(value[0][0]))
                card_contour.addInt(int(value[0][1]))

        self.cardsPort.write(bottle)


    def onImage(self, cv2_image):
        """ This method gets called upon receiving an input image given by cv2_image.

        The method detects the cards. Then it chooses one Template object for each recognized
        card id and draws it on the output image. Afterwards the additional information is send to
        the corresponding ports.

        @param cv2_image - an OpenCV image object
        """

        # we only care for one contour
        cards = {}
        for card in [self.detect_cards(cv2_image)]:
            if card:
                cards[card.tid] = card        
        
        card_list = [ cards[mid] for mid in cards ]

        # handle memory
        if self.memory_length > 0:

            # set all current marker information
            cur_time = time.time()
            for card in card_list:
                self.memory[card.tid] = ( card, cur_time )

            # create new card list and only include card which are within the time frame
            card_list = [ self.memory[mid][0] for mid in self.memory if cur_time - self.memory[mid][1] < self.memory_length ]

        # highlight markers in output image
        for card in card_list:
            card.highlite_marker(cv2_image)

#         self.sendCards(card_list)
#         self.sendOrder(card_list)
#         self.sendTranslation(card_list)

        return self.image# cv2_image


    def respond(self, command, reply):

        success = False
        command = command.toString().split(' ')

        if command[0] == 'set':

            if command[1] == 'order':

                if command[2] == 'horizontal':
                    self.order = CardTracker.O_HORIZONTAL
                    success = True

                elif  command[2] == 'vertical':
                    self.order = CardTracker.O_VERTICAL
                    success = True

                elif command[2] == 'reverse':
                    self.orderIsReversed = not self.orderIsReversed
                    success = True

        elif command[0] == 'get':

            if command[1] == 'order':
                reply.add('horizontal' if self.order == CardTracker.O_HORIZONTAL else 'vertical')
                reply.add('normal' if not self.orderIsReversed else 'reversed')
                success = True

        elif command[0] == 'memory':

            self.memory_length = float(command[1])
            success = True


        reply.addString('ack' if success else 'nack')
        return True


    def detect_cards(self, image):
        gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray    = cv2.bilateralFilter(gray, 11, 17, 17)
        edged   = cv2.Canny(gray, 30, 200)
        (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    
        # loop over our contours
        found_cards = []
        for c in cnts:

            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) == 4:
                found_cards.append(approx)

                # draw contour
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 3)

                x,y,w,h = cv2.boundingRect(approx)
                
                # draw bounding box
                cv2.rectangle(image, (x, y), (x+w, y+h) , (255,0,0), 2)

                # draw center
                center = (x + (w / 2), y + (h / 2)) 
                cv2.circle(image, center, 5, (0,255,0), -1)
    
        self.image = image


def createArgParser():
    """ This method creates a base argument parser.

    @return Argument Parser object
    """
    parser = argparse.ArgumentParser(description='Create a SensorModule for Yarp.')
    parser.add_argument( '-n', '--name',
                         dest       = 'name',
                         default    = '',
                         help       = 'Name prefix for Yarp port names')

    parser.add_argument( '-m', '--memory',
                         dest       = 'memory',
                         type       = type(0),
                         default    = 0,
                         help       = 'Defines how long the marker positions are kept in memory.')

    parser.add_argument( '-t', '--translation',
                         dest       = 'translation',
                         default    = '',
                         help       = 'Give a file path to a translation file.')

    return parser.parse_args()


if __name__ == '__main__':
    main(CardTracker, createArgParser())