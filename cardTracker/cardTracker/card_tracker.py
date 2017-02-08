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
from cardTracker.patch       import Patch


class CardTracker(BaseModule):
    """ The CardTracker class provides a yarp module for recognizing play cards.
    """

    D_WIDTH  = 320
    D_HEIGHT = 240

    O_HORIZONTAL = 0
    O_VERTICAL   = 1

    CLAHE        = cv2.createCLAHE( clipLimit = 2.0, tileGridSize = (8, 8) )

    # 10cm below (0, 0, 0)
    DEPTH        = -0.1

    debug = False


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
        self.simplePort      = self.createOutputPort('simple')
        self.orderPort       = self.createOutputPort('order')
        self.translationPort = self.createOutputPort('translation')
        self.sfmPort         = self.createRpcClientPort('convert2D')

        self.imgInPort       = self.createInputPort('image')
        self.imgOutPort      = self.createOutputPort('image')
        self.dbgOutPort      = self.createOutputPort('debug')

        self.bufImageIn,  self.bufArrayIn  = self.createImageBuffer(self.D_WIDTH, self.D_HEIGHT, 3)
        self.bufImageOut, self.bufArrayOut = self.createImageBuffer(self.D_WIDTH, self.D_HEIGHT, 3)
        self.bufImageDbg, self.bufArrayDbg = self.createImageBuffer(self.D_WIDTH, self.D_HEIGHT, 1)

        self.order           = CardTracker.O_HORIZONTAL
        self.orderIsReversed = False

        self.memory          = {}

        return True


    def updateModule(self):

        if self.imgInPort.read(self.bufImageIn):

            print 'received'

#             assert self.bufImageIn.width()  == self.D_WIDTH
#             assert self.bufImageIn.height() == self.D_HEIGHT

            # Make sure the image has not been re-allocated
#            assert self.bufArrayIn.__array_interface__['data'][0] == self.bufImageIn.getRawImage().__long__()

            # convert image to be usable in OpenCV
            image  = cv2.cvtColor(self.bufArrayIn, cv2.COLOR_BGR2RGB)

            # and convert image back to something the yarpview can understand
            self.bufArrayOut[:,:] = cv2.cvtColor(self.onImage(image), cv2.COLOR_RGB2BGR)

            # Send the result to the output port
            self.imgOutPort.write(self.bufImageOut)

            # Send the result to the debug output port
            self.bufArrayDbg[:,:] = self.image
            self.dbgOutPort.write(self.bufImageDbg)

        return True


    def _sendMessage(self, msg):
        
        # check we send a bottle with the correct input
        assert isinstance(msg, yarp.Bottle)
        
        ans = yarp.Bottle()     
        ans.clear()

        self.sfmPort.write(msg, ans)

        # check we get a return and has content        
        assert ans

        return ans

    
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


    def sendSimpleBottle(self, cards):
        """ This method sends the simple card information to the simple port.

        Message: ( ( <center-x> <center-y>  <center-z> <owner> <number>)* )

        All values are integer values.

        @param cards - list of Template objects
        """
        bottle  = yarp.Bottle()
        bottle.clear()


        # this is a quick fix for the message format
        card_values = bottle

        # send all cards
        for card in cards:

            point3D = self.convert(card.center[0], card.center[1])

            # id and center
            card_values.addDouble(point3D[0])
            card_values.addDouble(point3D[1])
            card_values.addDouble(point3D[2])
            card_values.addString(card.belongsTo())
            card_values.addInt(card.getEstimatedNumber())

        self.simplePort.write(card_values)

    
    def sendCards(self, cards):
        """ This method sends the card information to the cards port.

        Message: <number of cards> ( ( <id> <center-x>  <center-y> <bounding box> )* )
                 <bounding box> = (<tl-x> <tl-y> <br-x> <br-y>)

        All values are integer values.

        @param cards - list of Template objects
        """

        bottle  = yarp.Bottle()
        bottle.clear()

        bottle.addInt(len(cards))
        cards_list = bottle.addList()

        # send all cards
        for card in cards:
            

            # id and center
            card_values = cards_list.addList()
            card_values.addInt(card.tid)
            card_values.addInt(card.center[0])
            card_values.addInt(card.center[1])

            # bounding box
            card_contour = card_values.addList()
            card_contour.addInt(int(card.tl[0]))
            card_contour.addInt(int(card.tl[1]))
            card_contour.addInt(int(card.br[0]))
            card_contour.addInt(int(card.br[1]))

        self.cardsPort.write(bottle)


    def convert(self, x, y):
        
        msg = yarp.Bottle()
        msg.addString('get')
        msg.addString('3D')
        msg.addString('proj')
        _list = msg.addList()
        _list.addString('left')
        _list.addDouble(x)
        _list.addDouble(y)
        _list.addDouble(0.0)
        _list.addDouble(0.0)
        _list.addDouble(1.0)
        _list.addDouble(-1.0 * self.DEPTH)

        answer = self._sendMessage(msg)
        
        _list  = answer.get(1).asList()
        if _list:
            return (_list.get(0).asDouble(), _list.get(1).asDouble(), self.DEPTH)

        else:
            return (0.0, 0.0, 0.0)


    def onImage(self, cv2_image):
        """ This method gets called upon receiving an input image given by cv2_image.

        The method detects the cards. Then it chooses one Template object for each recognized
        card id and draws it on the output image. Afterwards the additional information is send to
        the corresponding ports.

        @param cv2_image - an OpenCV image object
        """

        # we only care for one contour
        cards     = dict([ (card.tid, card) for card in self.detect_cards(cv2_image) if card ])
        card_list = [ cards[tid] for tid in cards ]

        # handle memory
        if self.memory_length > 0:

            # set all current marker information
            cur_time = time.time()
            for card in card_list:

                # rectify patch tids
                for mcard, _ in self.memory.values():
                    if card.overlaps(mcard):
                        card.tid = mcard.tid
                        card.num_hist += mcard.num_hist[:20]
                        print 'found match', card.num_hist, card.getEstimatedNumber()
                        break
                
                self.memory[card.tid] = ( card, cur_time )

            # create new card list and only include card which are within the time frame
            card_list = [ self.memory[tid][0] for tid in self.memory if cur_time - self.memory[tid][1] < self.memory_length ]

        # highlight markers in output image
        _ = [card.highlite(cv2_image) for card in card_list]

        if len(card_list):
            print 'send'
            self.sendCards(card_list)
            self.sendOrder(card_list)
            self.sendTranslation(card_list)
            self.sendSimpleBottle(card_list)

        return cv2_image


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
        """ Return the detected cards 

        @result list of Patch objects
        """
        
        rChannel = CardTracker.CLAHE.apply(image[:,:,2])
        gChannel = CardTracker.CLAHE.apply(image[:,:,1])
        bChannel = CardTracker.CLAHE.apply(image[:,:,0])

        _, rThreshed  = cv2.threshold(rChannel, 190, 255, cv2.THRESH_BINARY)
        _, gThreshed  = cv2.threshold(gChannel, 127, 255, cv2.THRESH_BINARY)
        _, bThreshed  = cv2.threshold(bChannel, 127, 255, cv2.THRESH_BINARY)

        # convert to gray
        gray      = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # filter
        rThreshed = cv2.bilateralFilter(rThreshed, 11, 17, 17)
        edged     = cv2.Canny(rThreshed, 30, 200)

        # get contours
        (cnts, _) = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # reverse sorting to find the 10 most outer ones
        contours  = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]


        # convert to patches
        patches   = [Patch(contour, image, rThreshed, gThreshed, bThreshed) for contour in contours]

            


        # return patches that represent a card
        result      = [patch for patch in patches if patch.isCard()]

        # find
        for patch in result:
            print patch.belongsTo(), patch.label, patch.getEstimatedNumber()


        self.image = (gThreshed + bThreshed) / 2
        _, self.image  = cv2.threshold(self.image, 10, 255, cv2.THRESH_BINARY)
        
        
        return result


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
                         default    = 2,
                         help       = 'Defines how long the marker positions are kept in memory.')

    parser.add_argument( '-t', '--translation',
                         dest       = 'translation',
                         default    = '',
                         help       = 'Give a file path to a translation file.')

    return parser.parse_args()


if __name__ == '__main__':
    main(CardTracker, createArgParser())
