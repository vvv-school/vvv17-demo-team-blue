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
     * Set the lower bound of the colour threshold
     * @param r: Red value of the lower bound
     * @param g: Green value of the lower bound
     * @param b: Blue value of the lower bound
     * @return true/false on success/failure
     */
    bool setLowerBound(1:i32 r, 2:i32 g, 3:i32 b);

    /**
     * Set the upper bound of the colour threshold
     * @param r: Red value of the lower bound
     * @param g: Green value of the lower bound
     * @param b: Blue value of the lower bound
     * @return true/false on success/failure
     */
    bool setUpperBound(1:i32 r, 2:i32 g, 3:i32 b);
    
    /**
     * Gets the lower bound of the colour threshold
     * @return list of RBG values
     */
    list<i32> getLowerBound();
    
    /**
     * Gets the higher bound of the colour threshold
     * @return list of RBG values
     */
    list<i32> getUpperBound();
    
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
