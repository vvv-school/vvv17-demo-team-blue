####################################################################################################
#    Copyright (C) 2016 by Ingo Keller                                                             #
#    <brutusthetschiepel@gmail.com>                                                                #
#                                                                                                  #
#    This file is part of SPY (A Sensor Module Package for Yarp).                                  #
#                                                                                                  #
#    SPY is free software: you can redistribute it and/or modify it under the terms of the         #
#    GNU Affero General Public License as published by the Free Software Foundation, either        #
#    version 3 of the License, or (at your option) any later version.                              #
#                                                                                                  #
#    SPY is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;              #
#    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.     #
#    See the GNU Affero General Public License for more details.                                   #
#                                                                                                  #
#    You should have received a copy of the GNU Affero General Public License                      #
#    along with SPY.  If not, see <http://www.gnu.org/licenses/>.                                  #
####################################################################################################
import numpy as np
import yarp


EMSG_YARP_NOT_FOUND = "Could not connect to the yarp server. Try running 'yarp detect'."


class YarpFactory(object):
    """ The YarpFactor class provides convenience methods for handling Yarp related method calls
        such as port or buffer creations.

        Additionally it maintains a naming convention on the ports it creates. Suffixes are added
        to the port names to distinguish their intended use:
            ':i'   - input ports
            ':o'   - output ports
            ':rpc' - RPC ports
    """


    def __createPort(self, name, target = None, mode = 'unbuffered'):
        """ This method returns a port object.

        @param name     - yarp name for the port
        @param obj      - object for which the port is created
        @param buffered - if buffered is True a buffered port will be used otherwise not;
                          default is True.
        @result port
        """
        # create port
        if mode == 'buffered':
            port = yarp.BufferedPortBottle()

        elif mode == 'rpcclient':
            port = yarp.RpcClient()

        elif mode == 'rpcserver':
            port = yarp.RpcServer()

        else:
            port = yarp.Port()

        # build port name
        port_name = ['']

        # prefix handling
        if hasattr(self, 'prefix') and self.prefix:
            port_name.append(self.prefix)

        port_name.append(self.__class__.__name__)
        port_name.append(name)

        # open port
        port.open('/'.join(port_name))
#             raise RuntimeError, EMSG_YARP_NOT_FOUND

        # add output if given
        if target:
            port.addOutput(target)

        if hasattr(self, '_ports'):
            self._ports.append(port)

        return port


    def createInputPort(self, name, mode = 'unbuffered'):
        """ This method returns an input port.

        @param obj      - the object that the port is created for
        @param name     - if a name is provided it gets appended to the modules name
        @param buffered - if buffered is True a buffered port will be used otherwise not;
                          default is True.
        @result port
        """
        return self.__createPort(name + ':i', None, mode)


    def createOutputPort(self, name, target = None, mode = 'unbuffered'):
        """ This method returns an output port.

        @param obj      - the object that the port is created for
        @param name     - if a name is provided it gets appended to the modules name
        @param buffered - if buffered is True a buffered port will be used otherwise not;
                          default is True.
        @result port
        """
        return self.__createPort(name + ':o', target, mode)


    def createRpcClientPort(self, name, target = None):
        """ This method returns an output port.

        @param obj      - the object that the port is created for
        @param name     - if a name is provided it gets appended to the modules name
        @param buffered - if buffered is True a buffered port will be used otherwise not;
                          default is True.
        @result port
        """
        if name:
            name += ':rpc'
        else:
            name = 'rpc'
        return self.__createPort(name, target, 'rpcclient')


    def createRpcServerPort(self, name, obj = None, target = None):
        """ This method returns an output port.

        @param obj      - the object that the port is created for
        @param name     - if a name is provided it gets appended to the modules name
        @param buffered - if buffered is True a buffered port will be used otherwise not;
                          default is True.
        @result port
        """
        if name:
            name += ':rpc'
        else:
            name = 'rpc'
        return self.__createPort(name, target, 'rpcserver')


    @staticmethod
    def createImageBuffer(width = 320, height = 240, channels = 3):
        """ This method creates image buffers with the specified \a width, \a height and number of
            color channels \a channels.

        @param width    - integer specifying the width of the image   (default: 320)
        @param height   - integer specifying the height of the image  (default: 240)
        @param channels - integer specifying number of color channels (default: 3)
        @return image, buffer array
        """

        if channels == 1:
            buf_image = yarp.ImageFloat()
            buf_image.resize(width, height)

            buf_array = np.zeros((height, width), dtype = np.float32)

        else:
            buf_image = yarp.ImageRgb()
            buf_image.resize(width, height)

            buf_array = np.zeros((height, width, channels), dtype = np.uint8)

        buf_image.setExternal( np.getbuffer(buf_array),
                               buf_array.shape[1],
                               buf_array.shape[0] )

        return buf_image, buf_array


    @staticmethod
    def connect(source_port, target_port):
        """ This method connects two ports.

        If given a port it resolves the port names on its own.

        @param source_port - can be either a string or an object with a getName method
        @param target_port - can be either a string or an object with a getName method
        @result boolean - returns whether the port could be connected or not
        """

        # get the port name in case of a none-string argument
        if not isinstance(source_port, type('')) and hasattr(source_port, 'getName'):
            source_port = source_port.getName()

        # get the port name in case of a none-string argument
        if not isinstance(target_port, type('')) and hasattr(target_port, 'getName'):
            target_port = target_port.getName()

        return yarp.Network.connect(source_port, target_port)
