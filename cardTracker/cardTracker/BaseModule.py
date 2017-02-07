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
import argparse
import sys
import time
import yarp

from cardTracker.YarpFactory import YarpFactory

EMSG_YARP_NOT_FOUND  = "Could not connect to the yarp server. Try running 'yarp detect'."


class BaseModule(yarp.RFModule, YarpFactory):


    def __init__(self, args):
        yarp.RFModule.__init__(self)
        self.prefix = args.name
        self._ports = []


    def configure(self, rf):

        name = self.__class__.__name__
        if self.prefix:
            name = self.prefix + '/' + name

        self.setName(name)

        # RPC Port
        self.rpc_port = yarp.RpcServer()

        # name settings
        port_name = '/%s/%s' % (name, 'rpc')

        self.rpc_port.open(port_name)
#         if not self.rpc_port.open(port_name):
#             raise RuntimeError, EMSG_YARP_NOT_FOUND

        self.attach_rpc_server(self.rpc_port)
        self._ports.append(self.rpc_port)
        return True


    def interruptModule(self):
        for port in reversed(self._ports):
            port.interrupt()
        self.rpc_port.interrupt()
        return True


    def close(self):
        for port in reversed(self._ports):
            port.close()
        self.rpc_port.close()
        return True


    def getPeriod(self):
        return 0.01


    def updateModule(self):
        # XXX: I do not know why we need that, but if method is empty the module gets stuck
        time.sleep(0.000001)
        return True


####################################################################################################
#
# Default methods for running the modules standalone
#
####################################################################################################
def createArgParser():
    """ This method creates a base argument parser.

    @return Argument Parser object
    """
    parser = argparse.ArgumentParser(description='Create a SensorModule for Yarp.')
    parser.add_argument( '-n', '--name',
                         dest       = 'name',
                         default    = '',
                         help       = 'Name prefix for Yarp port names')

    return parser.parse_args()


def main(module_cls, args = None):
    """ This is a main method to run a module from command line.

    @param module_cls - a SensorModule based class that can be started as a standalone module.
    """
    if args is None:
        args = createArgParser()

    yarp.Network.init()

    resource_finder = yarp.ResourceFinder()
    resource_finder.setVerbose(True)

    resource_finder.configure(sys.argv)

    module = module_cls(args)
    module.runModule(resource_finder)
