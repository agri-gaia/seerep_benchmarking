#include "seerep-hdf5-fb/hdf5-fb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbGeneral::Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : seerep_hdf5_core::Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5FbGeneral::writeAttributeMap(
    const std::shared_ptr<HighFive::DataSet> dataSetPtr,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>* attributes)
{
  if (attributes)
  {
    for (auto attribute : *attributes)
    {
      if (attribute->value_type() == seerep::fb::Datatypes_Boolean)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Boolean*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Integer)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Integer*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Double)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Double*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_String)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::String*>(attribute->value())->data()->str());
      }
      else
      {
        std::stringstream errorMsg;
        errorMsg << "type " << attribute->value_type() << " of attribute " << attribute->key()->c_str()
                 << " not implemented in hdf5-io.";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << errorMsg.str();
        throw std::invalid_argument(errorMsg.str());
      }
    }
  }
}

void Hdf5FbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const BoundingBoxesLabeledWithCategoryFb* boundingBoxLabeledWithCategoryVector)
{
  for (auto boundingBoxLabeledWithCategory : *boundingBoxLabeledWithCategoryVector)
  {
    writeBoundingBoxLabeled(datatypeGroup, uuid, boundingBoxLabeledWithCategory->boundingBoxLabeled(),
                            boundingBoxLabeledWithCategory->category()->c_str());
  }
}

void Hdf5FbGeneral::writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                            const BoundingBoxesLabeledFb* boundingboxLabeled,
                                            const std::string& category)
{
  if (boundingboxLabeled && boundingboxLabeled->size() != 0)
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingboxLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_min()->z(), label->bounding_box()->point_max()->x(),
                               label->bounding_box()->point_max()->y(), label->bounding_box()->point_max()->z() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes =
        m_file->createDataSet<double>(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category,
                                      HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category,
        HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

void Hdf5FbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const BoundingBoxes2dLabeledWithCategoryFb* boundingbox2DLabeledWithCategoryVector)
{
  for (auto boundingBox2DLabeledWithCategory : *boundingbox2DLabeledWithCategoryVector)
  {
    writeBoundingBox2DLabeled(datatypeGroup, uuid, boundingBox2DLabeledWithCategory->boundingBox2dLabeled(),
                              boundingBox2DLabeledWithCategory->category()->c_str());
  }
}

void Hdf5FbGeneral::writeBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                              const BoundingBoxes2dLabeledFb* boundingbox2DLabeled,
                                              const std::string& category)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (boundingbox2DLabeled && boundingbox2DLabeled->size() != 0)
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingbox2DLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_max()->x(), label->bounding_box()->point_max()->y() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
    }

    if (m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB) ||
        m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES) ||
        m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES))
    {
      throw std::invalid_argument(datatypeGroup + " " + uuid + " already has bounding box based labels");
    }
    else
    {
      HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
          id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category, HighFive::DataSpace::From(labels));
      datasetLabels.write(labels);

      HighFive::DataSet datasetBoxes =
          m_file->createDataSet<double>(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category,
                                        HighFive::DataSpace::From(boundingBoxes));
      datasetBoxes.write(boundingBoxes);

      HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
          id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category,
          HighFive::DataSpace::From(instances));
      datasetInstances.write(instances);
    }

    m_file->flush();
  }
}

void Hdf5FbGeneral::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                       const GeneralLabelsWithCategoryFb* labelsWithInstanceWithCategoryGeneralVector)
{
  if (labelsWithInstanceWithCategoryGeneralVector && labelsWithInstanceWithCategoryGeneralVector->size() != 0)
  {
    std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory> labelsWithCategoryVector;
    for (auto labelsWithInstanceWithCategoryGeneral : *labelsWithInstanceWithCategoryGeneralVector)
    {
      std::vector<std::string> labels;
      std::vector<std::string> instances;
      for (auto label : *labelsWithInstanceWithCategoryGeneral->labelsWithInstance())
      {
        labels.push_back(label->label()->str());

        instances.push_back(label->instanceUuid()->str());
      }
      seerep_core_msgs::LabelsWithInstanceWithCategory labelsWithCategory;
      labelsWithCategory.category = labelsWithInstanceWithCategoryGeneral->category()->c_str();
      labelsWithCategory.labels = labels;
      labelsWithCategory.instances = instances;
      labelsWithCategoryVector.push_back(labelsWithCategory);
    }
    Hdf5CoreGeneral::writeLabelsGeneral(datatypeGroup, uuid, labelsWithCategoryVector);
  }
}

flatbuffers::Offset<GeneralLabelsWithCategoryFb>
Hdf5FbGeneral::readGeneralLabels(const std::string& datatypeGroup, const std::string& uuid,
                                 flatbuffers::grpc::MessageBuilder& builder)
{
  std::vector<std::string> labelCategories;
  std::vector<std::vector<seerep_core_msgs::LabelWithInstance>> labelsWithInstancesGeneralPerCategory;

  seerep_hdf5_core::Hdf5CoreGeneral::readLabelsGeneral(datatypeGroup, uuid, labelCategories,
                                                       labelsWithInstancesGeneralPerCategory);

  std::vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>> labelsWithInstanceWithCategoryVector;
  labelsWithInstanceWithCategoryVector.reserve(labelCategories.size());

  // loop categories
  for (size_t icat = 0; icat < labelCategories.size(); icat++)
  {
    std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelGeneralVector;
    labelGeneralVector.reserve(labelsWithInstancesGeneralPerCategory.at(icat).size());

    // loop labels
    for (size_t i = 0; i < labelsWithInstancesGeneralPerCategory.at(icat).size(); i++)
    {
      auto labelOffset = builder.CreateString(labelsWithInstancesGeneralPerCategory.at(icat).at(i).label);
      auto instanceOffset = builder.CreateString(
          boost::lexical_cast<std::string>(labelsWithInstancesGeneralPerCategory.at(icat).at(i).uuidInstance));

      seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
      labelBuilder.add_label(labelOffset);
      labelBuilder.add_instanceUuid(instanceOffset);
      labelGeneralVector.push_back(labelBuilder.Finish());
    }

    auto labelsGeneralOfCategoryOffset =
        builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>(labelGeneralVector);

    auto categoryOffset = builder.CreateString(labelCategories.at(icat));

    seerep::fb::LabelsWithInstanceWithCategoryBuilder labelsWithInstanceWithCategoryBuilder(builder);
    labelsWithInstanceWithCategoryBuilder.add_category(categoryOffset);
    labelsWithInstanceWithCategoryBuilder.add_labelsWithInstance(labelsGeneralOfCategoryOffset);
    auto labelsOfCategory = labelsWithInstanceWithCategoryBuilder.Finish();
    labelsWithInstanceWithCategoryVector.push_back(labelsOfCategory);
  }

  return builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>>(
      labelsWithInstanceWithCategoryVector);
}

flatbuffers::Offset<BoundingBoxes2dLabeledWithCategoryFb>
Hdf5FbGeneral::readBoundingBoxes2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                          flatbuffers::grpc::MessageBuilder& builder)
{
  std::vector<std::string> labelCategories;
  std::vector<std::vector<std::string>> labelsPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;
  readBoundingBoxLabeled(datatypeGroup, uuid, labelCategories, labelsPerCategory, boundingBoxesPerCategory,
                         instancesPerCategory);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << "creating the bounding boxes 2d with label fb msgs";

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>> boundingBox2DLabeledVector;
  boundingBox2DLabeledVector.reserve(labelCategories.size());

  // loop categories
  for (size_t icat = 0; icat < labelCategories.size(); icat++)
  {
    std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> bblabeledVector;
    bblabeledVector.reserve(labelsPerCategory.at(icat).size());
    // loop labels
    for (size_t i = 0; i < labelsPerCategory.at(icat).size(); i++)
    {
      auto InstanceOffset = builder.CreateString(instancesPerCategory.at(icat).at(i));
      auto labelOffset = builder.CreateString(labelsPerCategory.at(icat).at(i));

      seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
      labelBuilder.add_instanceUuid(InstanceOffset);
      labelBuilder.add_label(labelOffset);
      auto labelWithInstanceOffset = labelBuilder.Finish();

      auto pointMin = seerep::fb::CreatePoint2D(builder, boundingBoxesPerCategory.at(icat).at(i).at(0),
                                                boundingBoxesPerCategory.at(icat).at(i).at(1));
      auto pointMax = seerep::fb::CreatePoint2D(builder, boundingBoxesPerCategory.at(icat).at(i).at(2),
                                                boundingBoxesPerCategory.at(icat).at(i).at(3));

      seerep::fb::Boundingbox2DBuilder bbBuilder(builder);
      bbBuilder.add_point_min(pointMin);
      bbBuilder.add_point_max(pointMax);
      auto bb = bbBuilder.Finish();

      seerep::fb::BoundingBox2DLabeledBuilder bblabeledBuilder(builder);
      bblabeledBuilder.add_bounding_box(bb);
      bblabeledBuilder.add_labelWithInstance(labelWithInstanceOffset);

      bblabeledVector.push_back(bblabeledBuilder.Finish());
    }
    auto bblabeledVectorOffset = builder.CreateVector(bblabeledVector);
    auto categoryOffset = builder.CreateString(labelCategories.at(icat));

    seerep::fb::BoundingBox2DLabeledWithCategoryBuilder boundingBox2DLabeledWithCategoryBuilder(builder);
    boundingBox2DLabeledWithCategoryBuilder.add_category(categoryOffset);
    boundingBox2DLabeledWithCategoryBuilder.add_boundingBox2dLabeled(bblabeledVectorOffset);
    auto boundingBox2DLabeledWithCategory = boundingBox2DLabeledWithCategoryBuilder.Finish();
    boundingBox2DLabeledVector.push_back(boundingBox2DLabeledWithCategory);
  }
  return builder.CreateVector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>(
      boundingBox2DLabeledVector);
}

flatbuffers::Offset<BoundingBoxesLabeledWithCategoryFb>
Hdf5FbGeneral::readBoundingBoxesLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                        flatbuffers::grpc::MessageBuilder& builder)
{
  std::vector<std::string> labelCategories;
  std::vector<std::vector<std::string>> labelsPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;
  readBoundingBoxLabeled(datatypeGroup, uuid, labelCategories, labelsPerCategory, boundingBoxesPerCategory,
                         instancesPerCategory);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << "creating the bounding boxes 3d with label fb msgs";

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>> boundingBoxLabeledVector;
  boundingBoxLabeledVector.reserve(labelCategories.size());

  // loop categories
  for (size_t icat = 0; icat < labelCategories.size(); icat++)
  {
    std::vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>> bblabeledVector;
    bblabeledVector.reserve(labelsPerCategory.at(icat).size());
    // loop labels
    for (size_t i = 0; i < labelsPerCategory.at(icat).size(); i++)
    {
      auto InstanceOffset = builder.CreateString(instancesPerCategory.at(icat).at(i));
      auto labelOffset = builder.CreateString(labelsPerCategory.at(icat).at(i));

      seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
      labelBuilder.add_instanceUuid(InstanceOffset);
      labelBuilder.add_label(labelOffset);
      auto labelWithInstanceOffset = labelBuilder.Finish();

      auto pointMin = seerep::fb::CreatePoint(builder, boundingBoxesPerCategory.at(icat).at(i).at(0),
                                              boundingBoxesPerCategory.at(icat).at(i).at(1),
                                              boundingBoxesPerCategory.at(icat).at(i).at(2));
      auto pointMax = seerep::fb::CreatePoint(builder, boundingBoxesPerCategory.at(icat).at(i).at(3),
                                              boundingBoxesPerCategory.at(icat).at(i).at(4),
                                              boundingBoxesPerCategory.at(icat).at(i).at(5));

      seerep::fb::BoundingboxBuilder bbBuilder(builder);
      bbBuilder.add_point_min(pointMin);
      bbBuilder.add_point_max(pointMax);
      auto bb = bbBuilder.Finish();

      seerep::fb::BoundingBoxLabeledBuilder bblabeledBuilder(builder);
      bblabeledBuilder.add_bounding_box(bb);
      bblabeledBuilder.add_labelWithInstance(labelWithInstanceOffset);

      bblabeledVector.push_back(bblabeledBuilder.Finish());
    }
    auto bblabeledVectorOffset = builder.CreateVector(bblabeledVector);
    auto categoryOffset = builder.CreateString(labelCategories.at(icat));

    seerep::fb::BoundingBoxLabeledWithCategoryBuilder boundingBoxLabeledWithCategoryBuilder(builder);
    boundingBoxLabeledWithCategoryBuilder.add_category(categoryOffset);
    boundingBoxLabeledWithCategoryBuilder.add_boundingBoxLabeled(bblabeledVectorOffset);
    auto boundingBoxLabeledWithCategory = boundingBoxLabeledWithCategoryBuilder.Finish();
    boundingBoxLabeledVector.push_back(boundingBoxLabeledWithCategory);
  }
  return builder.CreateVector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>(boundingBoxLabeledVector);
}
}  // namespace seerep_hdf5_fb
