#include "rosbag-HDF5-converter.h"

RosbagHDF5Converter::RosbagHDF5Converter(const std::string& bagPath,
                                         const std::string& outputDir,
                                         const std::string& topicPointcloud,
                                         const int iterations)
{
  boost::log::add_common_attributes();
  boost::log::register_simple_formatter_factory<
      boost::log::trivial::severity_level, char>("Severity");
  boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                      boost::log::trivial::error);

  iterateAndDumpPointclouds(bagPath, outputDir, topicPointcloud, iterations);
}

RosbagHDF5Converter::~RosbagHDF5Converter()
{
}

void RosbagHDF5Converter::iterateAndDumpPointclouds(
    const std::string& bagPath, const std::string& outputDir,
    const std::string& topicPointcloud, const int iterations)
{
  std::vector<std::chrono::nanoseconds> durations;
  std::string hdf5filepath = outputDir + "/rosbagHDFConverter.hdf5";
  for (int i = 0; i < iterations; i++)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    rosbag::Bag bag;
    bag.open(bagPath);

    seerep_hdf5_ros::Hdf5Ros hdf5RosIO(hdf5filepath, "conversion-test", "map");

    for (const rosbag::MessageInstance& m :
         rosbag::View(bag, rosbag::TopicQuery(topicPointcloud)))
    {
      sensor_msgs::PointCloud2::ConstPtr message =
          m.instantiate<sensor_msgs::PointCloud2>();
      hdf5RosIO.dump(*message);
    }

    auto duration = std::chrono::high_resolution_clock::now() - start_time;
    auto duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    durations.push_back(duration_ns);
    std::filesystem::remove(hdf5filepath);
  }

  calcStats(durations);
}

void RosbagHDF5Converter::calcStats(
    std::vector<std::chrono::nanoseconds> durations)
{
  const size_t sz = durations.size();

  std::chrono::nanoseconds mean =
      std::accumulate(durations.begin(), durations.end(),
                      std::chrono::nanoseconds(0)) /
      sz;

  uint64_t var = 0;
  for (size_t n = 0; n < sz; n++)
  {
    var += (durations.at(n).count() - mean.count()) *
           (durations.at(n).count() - mean.count());
  }
  var /= sz;
  uint64_t sd = sqrt(var);

  std::cout << "mean: " << mean.count() << " ns" << std::endl;
  std::cout << "mean: " << mean.count() / 1e9 << " s" << std::endl;

  std::cout << "standard deviation: " << sd << " ns" << std::endl;
  std::cout << "standard deviation: " << sd / 1e6 << " ms" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_rosbagDumper");
  ros::NodeHandle privateNh("~");

  std::string bagPath, outputDir, topicPointcloud;
  int iterations;

  privateNh.getParam("bagPath", bagPath);
  privateNh.getParam("outputDir", outputDir);
  privateNh.getParam("topicPointcloud", topicPointcloud);
  privateNh.param<int>("iterations", iterations, 10);

  if (!std::filesystem::exists(bagPath))
  {
    std::cout << std::endl
              << std::endl
              << "The rosbag file does not exist!" << std::endl
              << std::endl;
    return 0;
  }

  if (!std::filesystem::exists(outputDir))
  {
    std::cout << std::endl
              << std::endl
              << "The output folder does not exist!" << std::endl
              << std::endl;
    return 0;
  }

  RosbagHDF5Converter converter(bagPath, outputDir, topicPointcloud, iterations);
}
