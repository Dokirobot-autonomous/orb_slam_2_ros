#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "Stereo");
    ros::start();

    if (argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    //// debug mode
    bool debug = false;
    if (debug) {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    // initialize
    StereoNode node(ORB_SLAM2::System::STEREO, node_handle, image_transport);

    node.Init();

    ros::spin();

    return 0;
}


StereoNode::StereoNode(const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle,
                       image_transport::ImageTransport &image_transport) : Node(sensor, node_handle, image_transport) {
    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "image_left/image_color_rect", 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "image_right/image_color_rect", 1);
    camera_info_topic_ = "image_left/camera_info";

    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&StereoNode::ImageCallback, this, _1, _2));
}


StereoNode::~StereoNode() {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}


void StereoNode::ImageCallback(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight) {

    ROS_DEBUG_STREAM("ImageCallback\n" << msgLeft->header);

    //// initialposeがcallback付されていない場合，falseで返す
//    ROS_WARN("Not initialization. ");

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_frame_time_ = msgLeft->header.stamp;

    orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());

    Update();
}

/*
void StereoNode::InitialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::cout << "call 2D pose estimate" << std::endl;
    try {
        ros::Time now = ros::Time(0);
        listener.waitForTransform("map", input->header.frame_id, now, ros::Duration(10.0));
        listener.lookupTransform("map", input->header.frame_id, now, transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }

    tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                     input->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);



}
*/
