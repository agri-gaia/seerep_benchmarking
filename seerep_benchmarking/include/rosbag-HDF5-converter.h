#ifndef ROSBAG_HDF5_CONVERTER_H
#define ROSBAG_HDF5_CONVERTER_H

#include "seerep_hdf5_ros/hdf5_ros.h"


#include <filesystem>
#include <fstream>

// ros
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// logging
#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

class RosbagHDF5Converter
{
public:
  RosbagHDF5Converter(const std::string& bagPath,
               const std::string& hdf5FilePath,
               const std::string& topicPointcloud,
               const int iterations
               );
  ~RosbagHDF5Converter();

private:
  void iterateAndDumpPointclouds(const std::string& bagPath,
               const std::string& outputDir,
               const std::string& topicPointcloud,
               const int iterations);
  void calcStats(std::vector<std::chrono::nanoseconds> durations);

};

#endif  // ROSBAG_HDF5_CONVERTER_H
