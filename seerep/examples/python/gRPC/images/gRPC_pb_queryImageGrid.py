#!/usr/bin/env python3

import os
import sys

from google.protobuf import empty_pb2
from seerep.pb import image_service_pb2_grpc as imageService
from seerep.pb import labels_with_category_pb2 as labels_with_category
from seerep.pb import meta_operations_pb2_grpc as metaOperations
from seerep.pb import query_pb2 as query

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

channel = util.get_gRPC_channel()

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "LabeledImagesInGrid":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = query.Query()
theQuery.projectuuid.append(projectuuid)
theQuery.boundingboxStamped.header.frame_id = "map"

theQuery.boundingboxStamped.boundingbox.point_min.z = -1.0
theQuery.boundingboxStamped.boundingbox.point_max.z = 1.0

# since epoche
theQuery.timeinterval.time_min.seconds = 1638549273
theQuery.timeinterval.time_min.nanos = 0
theQuery.timeinterval.time_max.seconds = 1938549273
theQuery.timeinterval.time_max.nanos = 0

# labels
label = labels_with_category.LabelsWithCategory()
label.category = "0"
label.labels.extend(["testlabel1"])
theQuery.labelsWithCategory.append(label)

for x in range(3):
    for y in range(3):
        theQuery.boundingboxStamped.boundingbox.point_min.x = x - 0.5
        theQuery.boundingboxStamped.boundingbox.point_min.y = y - 0.5
        theQuery.boundingboxStamped.boundingbox.point_max.x = x + 0.5
        theQuery.boundingboxStamped.boundingbox.point_max.y = y + 0.5
        for img in stub.GetImage(theQuery):
            print("General label of transferred img: " + img.labels_general[0].labelWithInstance[0].label)
