#include <rclcpp/rclcpp.hpp>
#include "dss_station.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DSS_Station>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}