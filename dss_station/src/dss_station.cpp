#include "dss_station.h"

#include <opencv2/opencv.hpp>
#include "h264_dec.h"

#include <System.h>   // ORB-SLAM3 header file
#include <sophus/se3.hpp>    // Sophus SE3
#include <string>
#include <yolov8_trt.h>

#include <chrono>
#include <iostream>
#include <cstdlib>
#include "fmt_config.h"
#include "LoopTime.h"

DSS_Station::DSS_Station()
    : Node("dss_station")
{
    slam_mode_ = 2; // 1=monocular, 2=stereo, 3=monocular_imu, 4=stereo_imu
    decoder_ = std::make_unique<H264Decoder>(1280, 480);

    rclcpp::QoS qos = rclcpp::QoS{rclcpp::KeepAll()}.reliable().durability_volatile();  // tcp-like
    video_sub_ = this->create_subscription<dss_common::msg::Video>(
        "dss_vehicle/h264_stream", qos, std::bind(&DSS_Station::sub_video, this, _1));

    RCLCPP_INFO(this->get_logger(), "DSS_Station node created, initializing ORB-SLAM3...");

    init_slam();

    // Create save and shutdown SLAM service
    shutdown_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/save_and_shutdown_slam",
        std::bind(&DSS_Station::handle_shutdown, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Save and shutdown SLAM service available at: ~/save_and_shutdown_slam");
    RCLCPP_INFO(this->get_logger(), "Call: ros2 service call /dss_station/save_and_shutdown_slam std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(), "DSS_Station fully ready");
}

DSS_Station::~DSS_Station()
{
    RCLCPP_INFO(this->get_logger(), "DSS_Station destructor called");
}

void DSS_Station::handle_shutdown(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // unused

    RCLCPP_INFO(this->get_logger(), "Save and shutdown SLAM service called...");

    if (!mpSLAM_) {
        response->success = false;
        response->message = "ORB-SLAM3 not initialized";
        return;
    }

    try {
        // Shutdown ORB-SLAM3 (automatically saves Atlas)
        RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
        mpSLAM_->Shutdown();

        // Save trajectory files
        RCLCPP_INFO(this->get_logger(), "Saving trajectory data...");  // [May hang if no data]
        mpSLAM_->SaveTrajectoryTUM("CameraTrajectory.txt");
        mpSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        response->success = true;
        response->message = "ORB-SLAM3 shutdown and data saved successfully";

        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 shutdown complete. Data saved.");
        RCLCPP_INFO(this->get_logger(), "Node is still running. Call service again to restart SLAM if needed.");

        // Release SLAM object
        mpSLAM_.reset();

    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to shutdown SLAM: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Failed to shutdown SLAM: %s", e.what());
    }
}

void DSS_Station::init_slam() {
    if (timer_init_) timer_init_->cancel();

    // Declare ROS parameters (launch file can override these defaults)
    const char* home = std::getenv("HOME");
    this->declare_parameter<std::string>("vocab_file", std::string(home) + "/Documents/Projects/ORB_SLAM_COMMUNITY/Vocabulary/ORBvoc.bin");
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<bool>("use_viewer", true);

    vocab_file_ = this->get_parameter("vocab_file").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    string onnx_model_path = std::string(home) + "/Documents/Projects/ORB_SLAM_COMMUNITY/yolov8_trt/models/yolov8n-seg.onnx";

    RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3...");
    RCLCPP_INFO(this->get_logger(), "Vocab file: %s", vocab_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());

    try {
        auto start = std::chrono::steady_clock::now();
        if (slam_mode_ == 1) {
            mpSLAM_ = std::make_unique<ORB_SLAM3::System>(vocab_file_, config_file_, ORB_SLAM3::System::MONOCULAR, true);
        } else {
            mpSLAM_ = std::make_unique<ORB_SLAM3::System>(vocab_file_, config_file_, ORB_SLAM3::System::STEREO, true);
        }

        // Create and set YOLOv8 detector for person detection and removal
        YOLOv8TRT* yoloDetector = new YOLOv8TRT(
            true,  // isSegmentation - use segmentation model
            onnx_model_path
        );
        mpSLAM_->setYoloDetector(yoloDetector);

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "ORB_SLAM3::System object created (took %ld ms)", duration);
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
        rclcpp::shutdown();
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: Unknown exception");
        rclcpp::shutdown();
    }
}

void DSS_Station::sub_video(const dss_common::msg::Video::SharedPtr msg)
{
    static LoopTime lt;
    std::cout<< fmt::format("fps={:.1f}\n", 1/lt.update())<< std::flush;

    AVFrame* frame = decoder_->decode_bytes2cpu(msg->frame);
    auto f_bgr = decoder_->yuv2rgb(frame, AV_PIX_FMT_RGB24);
    if (!f_bgr)
        return;
    cv::Mat frame_cv(f_bgr->height, f_bgr->width, CV_8UC3, f_bgr->data[0], f_bgr->linesize[0]);  // RGB24/BGR24 format matches CV_8UC3
    if (frame_cv.empty())
        return;

    if (!mpSLAM_) return;

    // Calculate left and right image width and height
    int w = frame_cv.cols / 2;
    int h = frame_cv.rows;

    // Create left and right ROI on original image with zero-copy reference
    cv::Mat left  = frame_cv(cv::Rect(0, 0, w, h));
    cv::Mat right = frame_cv(cv::Rect(w, 0, w, h));

    // Calculate timestamp (seconds) - stamp_shot is in nanoseconds
    double timestamp = msg->stamp_shot * 1e-9;

    // Input stereo images to SLAM for tracking
    try {
        if (slam_mode_ == 1) {
            Sophus::SE3f Tcw = mpSLAM_->TrackMonocular(left, timestamp);
        } else {
            Sophus::SE3f Tcw = mpSLAM_->TrackStereo(left, right, timestamp);
        }
        // Optional: check tracking status
        // int state = mpSLAM_->GetTrackingState();
        // Status: -1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=RECENTLY_LOST, 4=LOST, 5=OK_KLT
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SLAM tracking error: %s", e.what());
    }
}