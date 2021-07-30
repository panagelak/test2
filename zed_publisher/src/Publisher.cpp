#include <Publisher.hpp>

Publisher::Publisher(ros::NodeHandle *nh)
{

    // Load and apply the configuration of the publishers
    std::string TopicImage;
    nh->param<std::string>("TopicImage", TopicImage, "image");
    PubImage = nh->advertise<sensor_msgs::Image>(TopicImage, 1);

    // Load and set the parameters of the camera
    sl::InitParameters ZEDParam;
    int Resolution;
    nh->param<int>("Resolution", Resolution, 3); // Default VGA
    ZEDParam.camera_resolution = static_cast<sl::RESOLUTION>(Resolution);
    int Mode;
    nh->param<int>("Mode", Mode, 2); // Default Quality
    ZEDParam.depth_mode = static_cast<sl::DEPTH_MODE>(Mode);
    ZEDParam.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    ZEDParam.coordinate_units = sl::UNIT::METER;

    // Open the camera
    sl::ERROR_CODE returned_state = ZED.open(ZEDParam);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        ROS_ERROR("Cannot open camera. Make sure another process is not using it.");
        Shutdown();
    }

    // Enable positional tracking (mandatory for object detection)
    // sl::PositionalTrackingParameters positional_tracking_parameters;
    // If the camera is static, has better performance and boxes stick to the ground.
    // nh->param<bool>("Static", positional_tracking_parameters.set_as_static, true); // Default true
    // returned_state = ZED.enablePositionalTracking(positional_tracking_parameters);
    // if (returned_state != sl::ERROR_CODE::SUCCESS) {
    // ROS_WARN("Cannot enable positional tracking.");
    // Shutdown();
    // }

    // Enable the objects detection module
    // sl::ObjectDetectionParameters obj_det_params;
    // obj_det_params.enable_tracking = true;     // Track people across images flow
    // obj_det_params.enable_body_fitting = true; // Smooth skeletons moves
    // int Model;
    // nh->param<int>("Model", Model, 3); // Default Accurate
    // obj_det_params.detection_model = static_cast<sl::DETECTION_MODEL>(Model);
    // returned_state = ZED.enableObjectDetection(obj_det_params);
    // if (returned_state != sl::ERROR_CODE::SUCCESS) {
    // ROS_WARN("Cannot enable object detection.");
    // Shutdown();
    // }

    // Configure object detection runtime parameters
    // nh->param<float>("Threshold", PublisherParam.detection_confidence_threshold, 70); // Default 70

    // Load the enabled parameter to bypass the service activation
    // nh->param<bool>("Enabled", Enabled, true); // Default true
    // Load the floor parameter
    // nh->param<bool>("Floor", NeedsFloor, true); // Default true
    // Load the image lens parameter
    int LensMode;
    nh->param<int>("LensMode", LensMode, 0); // Default 0
    Lens = static_cast<sl::VIEW>(LensMode);
    // Load the frame parameters
    nh->param<std::string>("LensFrame", LensFrame, "zed2_camera_center"); // Default left lens
    // nh->param<std::string>("SkeletonFrame", SkeletonFrame, "zed2_camera_center");
}

Publisher::~Publisher()
{
    Shutdown();
}

bool Publisher::Publish()
{
    // Grab images
    if (ZED.grab() == sl::ERROR_CODE::SUCCESS) {

        // Once the camera has started, get the floor plane to stick the bounding box to the floor plane
        // if (NeedsFloor) {
        //     if (ZED.findFloorPlane(Floor, FloorTransform) == sl::ERROR_CODE::SUCCESS) {
        //         NeedsFloor = false;
        //     }
        // }

        // Retrieve and publish the image
        ZED.retrieveImage(Image, Lens, sl::MEM::CPU);
        sensor_msgs::ImagePtr ImgMsg = boost::make_shared<sensor_msgs::Image>();
        ToROSImage(ImgMsg, &Image, LensFrame, ros::Time::now());
        PubImage.publish(ImgMsg);
        /*
        // Retrieve detected skeletons
        ZED.retrieveObjects(Bodies, PublisherParam);

        // Create the SkeletonArray ROS message
        zed_human_tracking::SkeletonArray Skeletons;
        Skeletons.header.frame_id = SkeletonFrame;

        // For each detected skeleton
        for (auto i = Bodies.object_list.rbegin(); i != Bodies.object_list.rend(); ++i) {
            // Create a Skeleton object
            zed_human_tracking::Skeleton Skeleton;
            sl::ObjectData &obj = (*i);
            // For each joint
            for (int i = 0; i < obj.keypoint.size(); i++) {
                // Create a new 3D point from the joint's coordinates
                geometry_msgs::Point32 p;
                p.x = obj.keypoint[i][0];
                p.z = obj.keypoint[i][1];
                p.y = -obj.keypoint[i][2];
                // Add the point to the skeleton and marker
                Skeleton.joints.push_back(p);
            }
            // Add the skeleton to the skeletons list
            Skeletons.skeletons.push_back(Skeleton);
        }

        // Publish the skeletons
        PubSkeletons.publish(Skeletons);
        */
        return true;
    }
    return false;
}

void Publisher::Shutdown()
{
    // Release objects
    Image.free();
    // Floor.clear();
    // Bodies.object_list.clear();
    // Disable modules
    // ZED.disableObjectDetection();
    // ZED.disablePositionalTracking();
    ZED.close();
    ROS_WARN("Shutting down");
    ros::shutdown();
}

void Publisher::ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t)
{
    if (!MsgPtr) {
        return;
    }
    MsgPtr->header.stamp = t;
    MsgPtr->header.frame_id = Frame;
    MsgPtr->height = Image->getHeight();
    MsgPtr->width = Image->getWidth();
    int num = 1; // for endianness detection
    MsgPtr->is_bigendian = !(*(char *)&num == 1);
    MsgPtr->step = Image->getStepBytes(sl::MEM::CPU);
    size_t Size = MsgPtr->step * MsgPtr->height;
    MsgPtr->data.resize(Size);
    sl::MAT_TYPE DataType = Image->getDataType();
    switch (DataType) {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float1>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float2>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float3>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float4>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar1>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar2>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar3>(sl::MEM::CPU), Size);
        break;
    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        MsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
        memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar4>(sl::MEM::CPU), Size);
    }
}