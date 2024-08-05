#include "seerep-hdf5-core/hdf5-core-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreGeneral::Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::vector<std::string> Hdf5CoreGeneral::getGroupDatasets(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::vector<std::string> rootObjects = m_file->listObjectNames();

  if (id.empty())
  {
    return rootObjects;
  }
  else
  {
    // check if rootObjects contains the group id
    if (std::find(rootObjects.begin(), rootObjects.end(), id) != rootObjects.end())
    {
      return m_file->getGroup(id).listObjectNames();
    }
    else
    {
      return std::vector<std::string>();
    }
  }
}

void Hdf5CoreGeneral::writeProjectname(const std::string& projectname)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTNAME))
  {
    m_file->createAttribute<std::string>(PROJECTNAME, projectname);
  }
  else
  {
    m_file->getAttribute(PROJECTNAME).write(projectname);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectname()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5CoreGeneral::writeProjectFrameId(const std::string& frameId)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->createAttribute<std::string>(PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file->getAttribute(PROJECTFRAMEID).write(frameId);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectFrameId()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

std::optional<std::string> Hdf5CoreGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;

  checkExists(hdf5DatasetRawDataPath);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "get dataset " << hdf5DatasetRawDataPath;
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  if (data_set_ptr->hasAttribute(HEADER_FRAME_ID))
  {
    std::string frameId;
    data_set_ptr->getAttribute(HEADER_FRAME_ID).read(frameId);
    return frameId;
  }
  else
  {
    return std::nullopt;
  }
}

void Hdf5CoreGeneral::readBoundingBoxLabeledAndAddToLabelsWithInstancesWithCategory(
    const std::string& datatypeGroup, const std::string& uuid,
    std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory)
{
  boost::uuids::string_generator gen;

  std::vector<std::string> labelCategoriesBB;
  std::vector<std::vector<std::string>> labelsBBPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;

  readBoundingBoxLabeled(datatypeGroup, uuid, labelCategoriesBB, labelsBBPerCategory, boundingBoxesPerCategory,
                         instancesPerCategory, false);

  // loop the label categories
  for (std::size_t i = 0; i < labelCategoriesBB.size(); i++)
  {
    auto& labelsBB = labelsBBPerCategory.at(i);
    auto& instances = instancesPerCategory.at(i);

    // check if category already exists in map
    // create new entry if it doesn't exist
    auto labelsWithInstanceOfCategory = labelsWithInstancesWithCategory.find(labelCategoriesBB.at(i));
    if (labelsWithInstanceOfCategory == labelsWithInstancesWithCategory.end())
    {
      auto emplaceResult = labelsWithInstancesWithCategory.emplace(labelCategoriesBB.at(i),
                                                                   std::vector<seerep_core_msgs::LabelWithInstance>());
      labelsWithInstanceOfCategory = emplaceResult.first;
    }

    // add labels with instance to this label category
    for (std::size_t i = 0; i < labelsBB.size(); i++)
    {
      boost::uuids::uuid instanceUuid;
      try
      {
        instanceUuid = gen(instances.at(i));
      }
      catch (std::runtime_error&)
      {
        instanceUuid = boost::uuids::nil_uuid();
      }
      labelsWithInstanceOfCategory->second.push_back(
          seerep_core_msgs::LabelWithInstance{ .label = labelsBB.at(i), .uuidInstance = instanceUuid });
    }
  }
}

void Hdf5CoreGeneral::readBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                             std::vector<std::string>& labelCategories,
                                             std::vector<std::vector<std::string>>& labelsPerCategory,
                                             std::vector<std::vector<std::vector<double>>>& boundingBoxesPerCategory,
                                             std::vector<std::vector<std::string>>& instancesPerCategory,
                                             bool loadBoxes)
{
  std::string id = datatypeGroup + "/" + uuid;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << "reading the bounding box with labels of " << id;

  getLabelCategories(id, LABELBB, labelCategories);

  labelsPerCategory.resize(labelCategories.size());
  boundingBoxesPerCategory.resize(labelCategories.size());
  instancesPerCategory.resize(labelCategories.size());

  for (std::size_t i = 0; i < labelCategories.size(); i++)
  {
    readLabel(id, LABELBB + "_" + labelCategories.at(i), labelsPerCategory.at(i));
    readInstances(id, LABELBBINSTANCES + "_" + labelCategories.at(i), instancesPerCategory.at(i));

    if (loadBoxes)
    {
      readBoundingBoxes(id, LABELBBBOXES + "_" + labelCategories.at(i), boundingBoxesPerCategory.at(i));
    }

    if (labelsPerCategory.at(i).size() != instancesPerCategory.at(i).size() ||
        (loadBoxes && labelsPerCategory.at(i).size() != boundingBoxesPerCategory.at(i).size()))
    {
      std::string errorMsg = "size of labels (" + std::to_string(labelsPerCategory.at(i).size()) +
                             "), size of bounding boxes (" + std::to_string(boundingBoxesPerCategory.at(i).size()) +
                             ") and size of instances (" + std::to_string(instancesPerCategory.at(i).size()) +
                             ") do not fit.";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << errorMsg;
      throw std::runtime_error(errorMsg);
    }
  }
}

void Hdf5CoreGeneral::readLabelsGeneralAndAddToLabelsWithInstancesWithCategory(
    const std::string& datatypeGroup, const std::string& uuid,
    std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory)
{
  std::vector<std::string> labelCategoriesGeneral;
  std::vector<std::vector<seerep_core_msgs::LabelWithInstance>> labelsWithInstancesGeneralPerCategory;
  readLabelsGeneral(datatypeGroup, uuid, labelCategoriesGeneral, labelsWithInstancesGeneralPerCategory);

  for (std::size_t i = 0; i < labelCategoriesGeneral.size(); i++)
  {
    labelsWithInstancesWithCategory.emplace(labelCategoriesGeneral.at(i), labelsWithInstancesGeneralPerCategory.at(i));
  }
}

void Hdf5CoreGeneral::readLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid, std::vector<std::string>& labelCategories,
    std::vector<std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesGeneralPerCategory)
{
  boost::uuids::string_generator gen;

  std::string id = datatypeGroup + "/" + uuid;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading labels general of " << id;

  getLabelCategories(id, LABELGENERAL, labelCategories);

  for (std::string category : labelCategories)
  {
    std::vector<std::string> labels, instances;
    readLabel(id, LABELGENERAL + "_" + category, labels);
    readInstances(id, LABELGENERALINSTANCES + "_" + category, instances);

    if (labels.size() != instances.size())
    {
      std::string errorMsg = "size of labels (" + std::to_string(labels.size()) + ") and size of instances (" +
                             std::to_string(instances.size()) + ") do not fit.";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << errorMsg;
      throw std::runtime_error(errorMsg);
    }

    std::vector<seerep_core_msgs::LabelWithInstance> labelsWithInstancesGeneral;
    for (long unsigned int i = 0; i < labels.size(); i++)
    {
      boost::uuids::uuid instanceUuid;
      try
      {
        instanceUuid = gen(instances.at(i));
      }
      catch (std::runtime_error&)
      {
        instanceUuid = boost::uuids::nil_uuid();
      }
      labelsWithInstancesGeneral.push_back(
          seerep_core_msgs::LabelWithInstance{ .label = labels.at(i), .uuidInstance = instanceUuid });
    }

    labelsWithInstancesGeneralPerCategory.push_back(labelsWithInstancesGeneral);
  }
}

void Hdf5CoreGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory>& labelsWithInstanceWithCategory)
{
  for (auto labels : labelsWithInstanceWithCategory)
  {
    std::string id = datatypeGroup + "/" + uuid;

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + labels.category,
        HighFive::DataSpace::From(labels.labels));
    datasetLabels.write(labels.labels);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" + labels.category,
        HighFive::DataSpace::From(labels.instances));
    datasetInstances.write(labels.instances);
  }
  m_file->flush();
}

void Hdf5CoreGeneral::writeAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);

  std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
                                 aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "write AABB as attribute";
  if (!group.hasAttribute(AABB_FIELD))
  {
    group.createAttribute(AABB_FIELD, aabbPoints);
  }
  else
  {
    group.getAttribute(AABB_FIELD).write(aabbPoints);
  }

  m_file->flush();
}

void Hdf5CoreGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  if (group.hasAttribute(AABB_FIELD))
  {
    std::vector<float> aabbPoints;
    group.getAttribute(AABB_FIELD).read(aabbPoints);

    aabb.min_corner().set<0>(aabbPoints.at(0));
    aabb.min_corner().set<1>(aabbPoints.at(1));
    aabb.min_corner().set<2>(aabbPoints.at(2));
    aabb.max_corner().set<0>(aabbPoints.at(3));
    aabb.max_corner().set<1>(aabbPoints.at(4));
    aabb.max_corner().set<2>(aabbPoints.at(5));
  }
}

bool Hdf5CoreGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(AABB_FIELD);
}

void Hdf5CoreGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void Hdf5CoreGeneral::checkExists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    // BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
    //     << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
}

bool Hdf5CoreGeneral::exists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    return false;
  }
  return true;
}

void Hdf5CoreGeneral::readLabel(const std::string& id, const std::string labelType, std::vector<std::string>& labels)
{
  checkExists(id + "/" + labelType);
  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + labelType);
  datasetLabels.read(labels);
}
void Hdf5CoreGeneral::readBoundingBoxes(const std::string& id, const std::string boundingBoxType,
                                        std::vector<std::vector<double>>& boundingBoxes)
{
  checkExists(id + "/" + boundingBoxType);
  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + boundingBoxType);
  datasetBoxes.read(boundingBoxes);
}
void Hdf5CoreGeneral::readInstances(const std::string& id, const std::string instanceType,
                                    std::vector<std::string>& instances)
{
  checkExists(id + "/" + instanceType);
  HighFive::DataSet datasetInstances = m_file->getDataSet(id + "/" + instanceType);
  datasetInstances.read(instances);
}

std::shared_ptr<HighFive::Group> Hdf5CoreGeneral::getHdf5Group(const std::string& hdf5GroupPath, bool create)
{
  try
  {
    checkExists(hdf5GroupPath);
    // BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
    //     << "hdf5 group" << hdf5GroupPath << " already exists!";
    return std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  }
  catch (std::invalid_argument const& e)
  {
    // BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
    //     << "hdf5 group " << hdf5GroupPath << " does not exist! Creating a new group";
    if (create)
    {
      return std::make_shared<HighFive::Group>(m_file->createGroup(hdf5GroupPath));
    }
    else
    {
      return nullptr;
    }
  }
}

void Hdf5CoreGeneral::writeGeodeticLocation(const seerep_core_msgs::GeodeticCoordinates geocoords)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(GEODETICLOCATION_COORDINATESYSTEM) || !m_file->hasAttribute(GEODETICLOCATION_ELLIPSOID) ||
      !m_file->hasAttribute(GEODETICLOCATION_ALTITUDE) || !m_file->hasAttribute(GEODETICLOCATION_LATITUDE) ||
      !m_file->hasAttribute(GEODETICLOCATION_LONGITUDE))
  {
    m_file->createAttribute<std::string>(GEODETICLOCATION_COORDINATESYSTEM, geocoords.coordinateSystem);
    m_file->createAttribute<std::string>(GEODETICLOCATION_ELLIPSOID, geocoords.ellipsoid);
    m_file->createAttribute<double>(GEODETICLOCATION_ALTITUDE, geocoords.altitude);
    m_file->createAttribute<double>(GEODETICLOCATION_LATITUDE, geocoords.latitude);
    m_file->createAttribute<double>(GEODETICLOCATION_LONGITUDE, geocoords.longitude);
  }
  else
  {
    m_file->getAttribute(GEODETICLOCATION_COORDINATESYSTEM).write(geocoords.coordinateSystem);
    m_file->getAttribute(GEODETICLOCATION_ELLIPSOID).write(geocoords.ellipsoid);
    m_file->getAttribute(GEODETICLOCATION_ALTITUDE).write(geocoords.altitude);
    m_file->getAttribute(GEODETICLOCATION_LATITUDE).write(geocoords.latitude);
    m_file->getAttribute(GEODETICLOCATION_LONGITUDE).write(geocoords.longitude);
  }
  m_file->flush();
}

std::optional<seerep_core_msgs::GeodeticCoordinates> Hdf5CoreGeneral::readGeodeticLocation()
{
  const std::scoped_lock lock(*m_write_mtx);

  seerep_core_msgs::GeodeticCoordinates geocoords;
  if (m_file->hasAttribute(GEODETICLOCATION_COORDINATESYSTEM))
  {
    m_file->getAttribute(GEODETICLOCATION_COORDINATESYSTEM).read(geocoords.coordinateSystem);
  }
  if (m_file->hasAttribute(GEODETICLOCATION_ELLIPSOID))
  {
    m_file->getAttribute(GEODETICLOCATION_ELLIPSOID).read(geocoords.ellipsoid);
  }
  if (m_file->hasAttribute(GEODETICLOCATION_ALTITUDE))
  {
    m_file->getAttribute(GEODETICLOCATION_ALTITUDE).read(geocoords.altitude);
  }
  if (m_file->hasAttribute(GEODETICLOCATION_LATITUDE))
  {
    m_file->getAttribute(GEODETICLOCATION_LATITUDE).read(geocoords.latitude);
  }
  if (m_file->hasAttribute(GEODETICLOCATION_LONGITUDE))
  {
    m_file->getAttribute(GEODETICLOCATION_LONGITUDE).read(geocoords.longitude);
  }
  return geocoords;
}

std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath)
{
  if (exists(hdf5DataSetPath))
  {
    return std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DataSetPath));
  }
  else
  {
    return nullptr;
  }
}

void Hdf5CoreGeneral::getLabelCategories(std::string id, std::string labelType,
                                         std::vector<std::string>& matchingLabelCategory)
{
  std::vector<std::string> leafObjects = m_file->getGroup(id).listObjectNames();

  std::string searchString = labelType + "_";

  for (auto obj : leafObjects)
  {
    // check if obj name starts with labelType
    if (obj.rfind(searchString, 0) == 0)
    {
      // get category from postfix
      matchingLabelCategory.push_back(obj.substr(searchString.size()));
    }
  }
}

}  // namespace seerep_hdf5_core
