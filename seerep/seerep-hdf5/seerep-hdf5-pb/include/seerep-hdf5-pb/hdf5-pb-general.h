#ifndef SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_
#define SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// seerep-msgs
#include <seerep-msgs/boundingbox2d_labeled_with_category.pb.h>
#include <seerep-msgs/boundingbox_labeled_with_category.pb.h>
#include <seerep-msgs/header.pb.h>
#include <seerep-msgs/labels_with_instance_with_category.pb.h>

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_pb
{
class Hdf5PbGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  // ################
  //  Attributes
  // ################
  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::pb::Header& header);

  template <class T>
  seerep::pb::Header readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id);

  // ################
  //  BoundingBoxes
  // ################
  void writeBoundingBoxLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<::seerep::pb::BoundingBoxLabeledWithCategory>& boundingboxLabeled);

  void writeBoundingBox2DLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<seerep::pb::BoundingBox2DLabeledWithCategory>& boundingbox2DLabeled);

  std::optional<google::protobuf::RepeatedPtrField<::seerep::pb::BoundingBox2DLabeledWithCategory>>
  readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid);

  // ################
  //  Labels General
  // ################
  void writeLabelsGeneral(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<seerep::pb::LabelsWithInstanceWithCategory>& labelsGeneralWithInstances);

  std::optional<google::protobuf::RepeatedPtrField<seerep::pb::LabelsWithInstanceWithCategory>>
  readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid);
};

}  // namespace seerep_hdf5_pb

#include "impl/hdf5-pb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_ */
