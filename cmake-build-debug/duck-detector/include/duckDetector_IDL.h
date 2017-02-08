// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_duckDetector_IDL
#define YARP_THRIFT_GENERATOR_duckDetector_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class duckDetector_IDL;


/**
 * closest-blob_IDL
 * IDL Interface to \ref Closest Blob Module.
 */
class duckDetector_IDL : public yarp::os::Wire {
public:
  duckDetector_IDL();
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
