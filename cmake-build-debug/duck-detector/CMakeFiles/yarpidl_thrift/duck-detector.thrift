# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# closest-blob.thrift

/**
* closest-blob_IDL
*
* IDL Interface to \ref Closest Blob Module.
*/
service duckDetector_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
