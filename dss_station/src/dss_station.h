#include <rclcpp/rclcpp.hpp>
#include "dss_common/msg/video.hpp" // IWYU pragma: keep
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class H264Decoder;

namespace ORB_SLAM3 {
    class System;
}

class DSS_Station : public rclcpp::Node
{
public:
    DSS_Station();
    ~DSS_Station();

private:
    void init_slam();
    void sub_video(const dss_common::msg::Video::SharedPtr msg);

    // Shutdown service callback
    void handle_shutdown(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::unique_ptr<H264Decoder> decoder_;
    rclcpp::Subscription<dss_common::msg::Video>::SharedPtr video_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;

    rclcpp::TimerBase::SharedPtr timer_init_;

    // ORB-SLAM3
    int slam_mode_ = 2; // 1=monocular, 2=stereo, 3=monocular_imu, 4=stereo_imu
    std::unique_ptr<ORB_SLAM3::System> mpSLAM_;
    std::string vocab_file_;
    std::string config_file_;
};
