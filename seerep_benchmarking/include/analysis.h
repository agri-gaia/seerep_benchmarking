#ifndef ANALYSIS_H
#define ANALYSIS_H

#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file

#include <cstdlib>
#include <mcap/types.hpp>
#include <mcap/writer.hpp>
#include <string>

#include "message-generation.h"
#include "seerep_hdf5_ros/hdf5_ros.h"

using hrc = std::chrono::high_resolution_clock;
using nanoseconds = std::chrono::nanoseconds;
using runResult = std::pair<nanoseconds, double>;

mcap::Timestamp now();

runResult summerize_run(const std::vector<nanoseconds>& durations,
                                                          std::vector<double>& written_bytes);

void save_run(runResult run, const std::string& file_type, const std::string& label, const std::string& csv_path);

template <typename T>
runResult saveInMCAP(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

template <typename T>
runResult saveInHdf5(const std::vector<T>& messages, const std::string& outputDir, const std::string& label);

#endif  // ANALYSIS_H
