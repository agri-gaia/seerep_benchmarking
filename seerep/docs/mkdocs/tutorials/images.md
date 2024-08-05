# Sending & Querying images

## Sending images

In this example we want to send images with labeled bounding boxes as well as general labels to SEEREP. Additionally
we add some coordinate transformations at the end.

Source: `examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py`

```python
#!/usr/bin/env python3

import os
import sys
import time
import uuid

import boundingbox2d_labeled_pb2 as bb
import image_pb2 as image
import image_service_pb2_grpc as imageService
import label_with_instance_pb2 as labelWithInstance
import meta_operations_pb2_grpc as metaOperations
import numpy as np
import projectCreation_pb2 as projectCreation
import tf_service_pb2_grpc as tfService
import transform_stamped_pb2 as tf
from google.protobuf import empty_pb2

script_dir = os.path.dirname(__file__)
util_dir = os.path.join(script_dir, '..')
sys.path.append(util_dir)
import util

# Default server is localhost !
channel = util.get_gRPC_channel(target = "local")

# 1. Get gRPC service objects
stub = imageService.ImageServiceStub(channel)
stubTf = tfService.TfServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

# 2. Get all projects from the server
response = stubMeta.GetProjects(empty_pb2.Empty())

# 3. Check if we have an existing test project, if not, one is created.
found = False
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid
        found = True

if not found:
    creation = projectCreation.ProjectCreation(name="testproject", mapFrameId="map")
    projectCreated = stubMeta.CreateProject(creation)
    projectuuid = projectCreated.uuid


theTime = int(time.time())

# 4. Create ten images
for n in range(10):
    theImage = image.Image()

    rgb = []
    lim = 256 # 256 x 256 pixels
    for i in range(lim):
        for j in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(j) / lim
            r = np.ubyte((x * 255.0 + n) % 255)
            g = np.ubyte((y * 255.0 + n) % 255)
            b = np.ubyte((z * 255.0 + n) % 255)
            rgb.append(r)
            rgb.append(g)
            rgb.append(b)

    # Add image meta-data
    theImage.header.frame_id = "camera"
    theImage.header.stamp.seconds = theTime + n
    theImage.header.stamp.nanos = 0
    theImage.header.uuid_project = projectuuid
    theImage.height = lim
    theImage.width = lim
    theImage.encoding = "rgb8"
    theImage.step = 3 * lim
    # Add image data
    theImage.data = bytes(rgb)

    # 5. Create bounding boxes with labels
    bb1 = bb.BoundingBox2DLabeled()
    for i in range(0, 2):
        bb1.labelWithInstance.label = "testlabel" + str(i)
        bb1.labelWithInstance.instanceUuid = str(uuid.uuid4())
        bb1.boundingBox.point_min.x = 0.01 + i / 10
        bb1.boundingBox.point_min.y = 0.02 + i / 10
        bb1.boundingBox.point_max.x = 0.03 + i / 10
        bb1.boundingBox.point_max.y = 0.04 + i / 10
        theImage.labels_bb.append(bb1)

    # 6. Add general labels to the image
    for i in range(0, 2):
        label = labelWithInstance.LabelWithInstance()
        label.label = "testlabelgeneral" + str(i)
        # assuming that that the general labels are not instance related -> no instance uuid
        # label.instanceUuid = str(uuid.uuid4())
        theImage.labels_general.append(label)

    # 7. Send image to the server
    uuidImg = stub.TransferImage(theImage)

    print("uuid of transfered img: " + uuidImg.message)

# 8. Add coordinate transformations and send them to the server
theTf = tf.TransformStamped()
theTf.header.frame_id = "map"
theTf.header.stamp.seconds = theTime
theTf.header.uuid_project = projectuuid
theTf.child_frame_id = "camera"
theTf.transform.translation.x = 1
theTf.transform.translation.y = 2
theTf.transform.translation.z = 3
theTf.transform.rotation.x = 0
theTf.transform.rotation.y = 0
theTf.transform.rotation.z = 0
theTf.transform.rotation.w = 1
stubTf.TransferTransformStamped(theTf)

theTf.header.stamp.seconds = theTime + 10
theTf.transform.translation.x = 100
theTf.transform.translation.y = 200
theTf.transform.translation.z = 300
stubTf.TransferTransformStamped(theTf)
```

Output:

```bash
uuid of transfered img: 5836f989-adbb-46a0-a689-9e0527d457fe
uuid of transfered img: 5b39487d-238a-43e4-a8a4-0f7efb876e8b
uuid of transfered img: 00ced216-40b1-4d54-817f-11c413b228c6
uuid of transfered img: 5d330208-e534-4bfb-b242-00295e6b027d
uuid of transfered img: 18f00f33-0ac5-4221-a428-13e2c37b27cd
uuid of transfered img: ed5134c8-da18-476f-9c1b-60cea04c5627
uuid of transfered img: 22c995b0-c2b6-4c94-85a3-b1118ae3ac7d
uuid of transfered img: 9d980608-85ff-4e62-a6be-01c05bd59de9
uuid of transfered img: 24204e3f-1cbd-4cab-99b6-803b3944b450
uuid of transfered img: 797f2e1a-f9e8-4ed3-94b3-a5d101aa0697
```

## Query images

Now we want to query the previously send images with some criteria. Possible query parameters are:

* Bounding Boxes (spatial query)
* A time interval (temporal query)
* Labels (semantic query)

=== "Protocol Buffers"

    Source: `examples/python/gRPC/images/gRPC_pb_queryImage.py`

    ```python
    #!/usr/bin/env python3

    import os
    import sys

    import image_service_pb2_grpc as imageService
    import meta_operations_pb2_grpc as metaOperations
    import query_pb2 as query
    from google.protobuf import empty_pb2

    # importing util functions. Assuming that this file is in the parent dir
    # examples/python/gRPC/util.py
    script_dir = os.path.dirname(__file__)
    util_dir = os.path.join(script_dir, '..')
    sys.path.append(util_dir)
    import util

    # Default server is localhost !
    channel = util.get_gRPC_channel()

    # 1. Get gRPC service objects
    stub = imageService.ImageServiceStub(channel)
    stubMeta = metaOperations.MetaOperationsStub(channel)

    # 2. Get all projects from the server
    response = stubMeta.GetProjects(empty_pb2.Empty())

    # 3. Check if we have an existing test project, if not, we stop here
    projectuuid = ""
    for project in response.projects:
        print(project.name + " " + project.uuid + "\n")
        if project.name == "testproject":
            projectuuid = project.uuid

    if projectuuid == "":
        sys.exit()

    # 4. Create a query with parameters
    theQuery = query.Query()
    theQuery.projectuuid.append(projectuuid)
    theQuery.boundingboxstamped.header.frame_id = "map"

    theQuery.boundingboxstamped.boundingbox.point_min.x = 0.0
    theQuery.boundingboxstamped.boundingbox.point_min.y = 0.0
    theQuery.boundingboxstamped.boundingbox.point_min.z = 0.0
    theQuery.boundingboxstamped.boundingbox.point_max.x = 100.0
    theQuery.boundingboxstamped.boundingbox.point_max.y = 100.0
    theQuery.boundingboxstamped.boundingbox.point_max.z = 100.0

    # since epoche
    theQuery.timeinterval.time_min.seconds = 1638549273
    theQuery.timeinterval.time_min.nanos = 0
    theQuery.timeinterval.time_max.seconds = 1938549273
    theQuery.timeinterval.time_max.nanos = 0

    # labels
    theQuery.label.extend(["testlabel0"])

    # 5. Query the server for images matching the query and iterate over them
    for img in stub.GetImage(theQuery):
        print(f"uuidmsg: {img.header.uuid_msgs}")
        print(f"first label: {img.labels_bb[0].labelWithInstance.label}")
        print(
            "First bounding box (Xmin, Ymin, Xmax, Ymax): "
            + str(img.labels_bb[0].boundingBox.point_min.x)
            + " "
            + str(img.labels_bb[0].boundingBox.point_min.y)
            + " "
            + str(img.labels_bb[0].boundingBox.point_max.x)
            + " "
            + str(img.labels_bb[0].boundingBox.point_max.y)
            + "\n"
        )
    ```

    Output:

    ```
    testproject 3af70ba8-1e81-4f60-86d2-a4257d88f01e

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04

    first label: testlabel0
    First bounding box (Xmin, Ymin, Xmax, Ymax): 0.01 0.02 0.03 0.04
    ```

=== "Flatbuffers"

    Source: `examples/images/gRPC/images/gRPC_fb_queryImage.py`

    ```python
    #!/usr/bin/env python3

    import os
    import sys

    import flatbuffers
    from fb import Image
    from fb import image_service_grpc_fb as imageService

    # importing util functions. Assuming that these files are in the parent dir
    # examples/python/gRPC/util.py
    # examples/python/gRPC/util_fb.py
    script_dir = os.path.dirname(__file__)
    util_dir = os.path.join(script_dir, '..')
    sys.path.append(util_dir)
    import util
    import util_fb

    builder = flatbuffers.Builder(1024)
    # Default server is localhost !
    channel = util.get_gRPC_channel()

    # 1. Get all projects from the server
    projectuuid = util_fb.getProject(builder, channel, 'testproject')

    # 2. Check if the defined project exist; if not exit
    if not projectuuid:
        exit()

    # 3. Get gRPC service object
    stub = imageService.ImageServiceStub(channel)


    # Create all necessary objects for the query
    header = util_fb.createHeader(builder, frame="map")
    pointMin = util_fb.createPoint(builder, 0.0, 0.0, 0.0)
    pointMax = util_fb.createPoint(builder, 100.0, 100.0, 100.0)
    boundingboxStamped = util_fb.createBoundingBoxStamped(builder, header, pointMin, pointMax)

    timeMin = util_fb.createTimeStamp(builder, 1610549273, 0)
    timeMax = util_fb.createTimeStamp(builder, 1938549273, 0)
    timeInterval = util_fb.createTimeInterval(builder, timeMin, timeMax)


    projectUuids = [builder.CreateString(projectuuid)]
    labels = [builder.CreateString("testlabel0")]
    dataUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]
    instanceUuids = [builder.CreateString("3e12e18d-2d53-40bc-a8af-c5cca3c3b248")]

    # 4. Create a query with parameters
    # all parameters are optional
    # with all parameters set (especially with the data and instance uuids set) the result
    # of the query will be empty. Set the query parameters to adequate values or remove
    # them from the query creation
    query = util_fb.createQuery(
        builder,
        # boundingBox=boundingboxStamped,
        # timeInterval=timeInterval,
        # labels=labels,
        # projectUuids=projectUuids,
        # instanceUuids=instanceUuids,
        # dataUuids=dataUuids,
        withoutData=True,
    )
    builder.Finish(query)
    buf = builder.Output()

    # 5. Query the server for images matching the query and iterate over them
    for responseBuf in stub.GetImage(bytes(buf)):
        response = Image.Image.GetRootAs(responseBuf)

        print(f"uuidmsg: {response.Header().UuidMsgs().decode('utf-8')}")
        print("first label: " + response.LabelsBb(0).LabelWithInstance().Label().decode("utf-8"))
        print(
            "first bounding box (Xmin,Ymin,Xmax,Ymax): "
            + str(response.LabelsBb(0).BoundingBox().PointMin().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMin().Y())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().Y())
            + "\n"
        )

    print("done.")
    ```

    Output:

    ```
    testproject 3af70ba8-1e81-4f60-86d2-a4257d88f01e

    uuidmsg: 00ced216-40b1-4d54-817f-11c413b228c6
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5836f989-adbb-46a0-a689-9e0527d457fe
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5b39487d-238a-43e4-a8a4-0f7efb876e8b
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    uuidmsg: 5d330208-e534-4bfb-b242-00295e6b027d
    first label: testlabel0
    first bounding box (Xmin,Ymin,Xmax,Ymax): 0.01 0.02 0.03 0.04

    done.
    ```
