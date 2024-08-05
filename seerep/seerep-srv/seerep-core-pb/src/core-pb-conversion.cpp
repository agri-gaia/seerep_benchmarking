#include "seerep-core-pb/core-pb-conversion.h"

namespace seerep_core_pb
{
seerep_core_msgs::Query CorePbConversion::fromPb(const seerep::pb::Query& query, seerep_core_msgs::Datatype datatype)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = datatype;

  fromPbBoundingBox(query, queryCore);
  fromPbTime(query, queryCore);
  fromPbLabel(query, queryCore);

  fromPbMustHaveAllLabels(query, queryCore);
  fromPbProject(query, queryCore);
  fromPbInstance(query, queryCore);
  fromPbDataUuids(query, queryCore);
  fromPbWithOutData(query, queryCore);
  fromFbQueryMaxNumData(query, queryCore);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CorePbConversion::fromPb(const seerep::pb::Image& img)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (img.header().uuid_msgs().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(img.header().uuid_msgs());
  }

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.datatype = seerep_core_msgs::Datatype::Image;
  dataForIndices.header.frameId = img.header().frame_id();
  dataForIndices.header.timestamp.seconds = img.header().stamp().seconds();
  dataForIndices.header.timestamp.nanos = img.header().stamp().nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(img.header().uuid_project());
  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  if (!img.labels_general().empty())
  {
    for (auto labelsCategories : img.labels_general())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.labelwithinstance().empty())
      {
        for (auto label : labelsCategories.labelwithinstance())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.instanceuuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label.label(), .uuidInstance = uuidInstance });
        }
        dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                               labelWithInstanceVector);
      }
    }
  }

  if (!img.labels_bb().empty())
  {
    for (auto labelsCategories : img.labels_bb())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.boundingbox2dlabeled().empty())
      {
        for (auto label : labelsCategories.boundingbox2dlabeled())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.labelwithinstance().instanceuuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(seerep_core_msgs::LabelWithInstance{
              .label = label.labelwithinstance().label(), .uuidInstance = uuidInstance });
        }
      }
      dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                             labelWithInstanceVector);
    }
  }

  return dataForIndices;
}

seerep_core_msgs::QueryTf CorePbConversion::fromPb(const seerep::pb::TransformStampedQuery& query)
{
  boost::uuids::string_generator gen;
  seerep_core_msgs::QueryTf queryTf;
  queryTf.childFrameId = query.child_frame_id();
  queryTf.parentFrameId = query.header().frame_id();
  queryTf.project = gen(query.header().uuid_project());
  queryTf.timestamp.seconds = query.header().stamp().seconds();
  queryTf.timestamp.nanos = query.header().stamp().nanos();

  return queryTf;
}

void CorePbConversion::fromPbProject(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.projectuuid().empty())
  {
    queryCore.projects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : query.projectuuid())
    {
      queryCore.projects.value().push_back(gen(projectuuid));
    }
  }
}

void CorePbConversion::fromPbLabel(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (!query.labelswithcategory().empty())
  {
    queryCore.label = std::unordered_map<std::string, std::vector<std::string>>();
    for (auto labelWithCategory : query.labelswithcategory())
    {
      std::vector<std::string> labels;
      for (auto label : labelWithCategory.labels())
      {
        labels.push_back(label);
      }
      queryCore.label.value().emplace(labelWithCategory.category(), labels);
    }
  }
}

void CorePbConversion::fromPbTime(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.timeinterval().has_time_min() && query.timeinterval().has_time_max())
  {
    queryCore.timeinterval = seerep_core_msgs::Timeinterval();
    queryCore.timeinterval.value().timeMin.seconds = query.timeinterval().time_min().seconds();
    queryCore.timeinterval.value().timeMax.seconds = query.timeinterval().time_max().seconds();
    queryCore.timeinterval.value().timeMin.nanos = query.timeinterval().time_min().nanos();
    queryCore.timeinterval.value().timeMax.nanos = query.timeinterval().time_max().nanos();
  }
}

void CorePbConversion::fromPbBoundingBox(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.boundingboxstamped().has_header() && query.boundingboxstamped().has_boundingbox() &&
      query.boundingboxstamped().boundingbox().has_point_min() &&
      query.boundingboxstamped().boundingbox().has_point_max())
  {
    queryCore.header.frameId = query.boundingboxstamped().header().frame_id();
    queryCore.boundingbox = seerep_core_msgs::AABB();
    queryCore.boundingbox.value().min_corner().set<0>(query.boundingboxstamped().boundingbox().point_min().x());
    queryCore.boundingbox.value().min_corner().set<1>(query.boundingboxstamped().boundingbox().point_min().y());
    queryCore.boundingbox.value().min_corner().set<2>(query.boundingboxstamped().boundingbox().point_min().z());
    queryCore.boundingbox.value().max_corner().set<0>(query.boundingboxstamped().boundingbox().point_max().x());
    queryCore.boundingbox.value().max_corner().set<1>(query.boundingboxstamped().boundingbox().point_max().y());
    queryCore.boundingbox.value().max_corner().set<2>(query.boundingboxstamped().boundingbox().point_max().z());
  }
}

void CorePbConversion::fromPbMustHaveAllLabels(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.mustHaveAllLabels = query.musthavealllabels();
}

void CorePbConversion::fromPbInstance(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.instanceuuid().empty())
  {
    queryCore.instances = std::vector<boost::uuids::uuid>();
    for (auto instance : query.instanceuuid())
    {
      queryCore.instances.value().push_back(gen(instance));
    }
  }
}

void CorePbConversion::fromPbDataUuids(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.datauuid().empty())
  {
    queryCore.dataUuids = std::vector<boost::uuids::uuid>();
    for (auto data : query.datauuid())
    {
      queryCore.dataUuids.value().push_back(gen(data));
    }
  }
}

void CorePbConversion::fromPbWithOutData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.withoutData = query.withoutdata();
}

void CorePbConversion::fromFbQueryMaxNumData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.maxNumData = query.maxnumdata();
}
}  // namespace seerep_core_pb
