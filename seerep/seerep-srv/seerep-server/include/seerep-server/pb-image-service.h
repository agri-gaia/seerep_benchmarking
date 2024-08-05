#ifndef SEEREP_SERVER_IMAGE_SERVICE_H_
#define SEEREP_SERVER_IMAGE_SERVICE_H_

// seerep
#include <seerep-com/image-service.grpc.pb.h>
#include <seerep-core-pb/core-pb-image.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class PbImageService final : public seerep::pb::ImageService::Service
{
public:
  PbImageService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetImage(grpc::ServerContext* context, const seerep::pb::Query* request,
                        grpc::ServerWriter<seerep::pb::Image>* writer);

  grpc::Status TransferImage(grpc::ServerContext* context, const seerep::pb::Image* image,
                             seerep::pb::ServerResponse* response);

private:
  std::shared_ptr<seerep_core_pb::CorePbImage> imagePb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_IMAGE_SERVICE_H_
