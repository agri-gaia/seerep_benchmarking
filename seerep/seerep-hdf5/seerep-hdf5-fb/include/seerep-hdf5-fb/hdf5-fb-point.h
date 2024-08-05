#ifndef SEEREP_HDF5_FB_Point_H_
#define SEEREP_HDF5_FB_Point_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-point.h>
#include <seerep-hdf5-fb/hdf5-fb-general.h>

// seerep-com
#include <seerep-com/point_service.grpc.fb.h>

// std
#include <flatbuffers/grpc.h>
#include <grpcpp/grpcpp.h>

#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_fb
{
class Hdf5FbPoint : public Hdf5FbGeneral
{
public:
  Hdf5FbPoint(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writePoint(const std::string& id, const seerep::fb::PointStamped* point);
  void writeAdditionalPointAttributes(const seerep::fb::AttributesStamped& attributeStamped);

  std::optional<flatbuffers::grpc::Message<seerep::fb::PointStamped>> readPoint(const std::string& id);

private:
  std::string getHdf5DatasetRawDataPath(const std::string& id);
};

}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_Point_H_ */
