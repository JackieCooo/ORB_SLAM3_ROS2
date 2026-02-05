#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "System.h"


class Tracker : public rclcpp::Node
{
public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Subscriber = message_filters::Subscriber<sensor_msgs::msg::Image>;

    Tracker() : Node("tracker")
    {
        /* Declear parameters */
        this->declare_parameter("rgb_topic", "/camera/rgb/image_raw");
        this->declare_parameter("depth_topic", "/camera/depth_registered/image_raw");
        this->declare_parameter("voc_file", "ORBvoc.txt");
        this->declare_parameter("setting_file", "RealSense_D435i.yaml");

        /* Create SLAM object */
        RCLCPP_INFO(this->get_logger(), "Using vocabulary %s", this->get_parameter("voc_file").as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "Using setting %s", this->get_parameter("setting_file").as_string().c_str());
        _slam = std::make_unique<ORB_SLAM3::System>(
            this->get_parameter("voc_file").as_string(),
            this->get_parameter("setting_file").as_string(),
            ORB_SLAM3::System::RGBD, false
        );

        /* Subscribe topic */
        _rgbSub = std::make_shared<Subscriber>(
            this,
            this->get_parameter("rgb_topic").as_string(),
            rclcpp::QoS(10).get_rmw_qos_profile()
        );
        RCLCPP_INFO(this->get_logger(), "Subscribe RGB topic %s", this->get_parameter("rgb_topic").as_string().c_str());

        _depthSub = std::make_shared<Subscriber>(
            this,
            this->get_parameter("depth_topic").as_string(),
            rclcpp::QoS(10).get_rmw_qos_profile()
        );
        RCLCPP_INFO(this->get_logger(), "Subscribe depth topic %s", this->get_parameter("depth_topic").as_string().c_str());

        /* Register data sync callback */
        _sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *_rgbSub, *_depthSub);
        _sync->registerCallback(
            std::bind(&Tracker::_DataSyncedCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    ~Tracker()
    {
        _slam->Shutdown();
        /* Save camera trajectory */
        _slam->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

private:
    std::unique_ptr<ORB_SLAM3::System> _slam;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> _rgbSub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> _depthSub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> _sync;

    void _DataSyncedCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth
    )
    {
        /* Convert to cv::Mat */
        auto rgbMat = cv_bridge::toCvShare(rgb);
        auto depthMat = cv_bridge::toCvShare(depth);

        /* Track */
        _slam->TrackRGBD(rgbMat->image, depthMat->image, rgbMat->header.stamp.sec);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Tracker>());

    rclcpp::shutdown();

    return 0;
}
