#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 mono-inertial path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = false;  // ros2 bag 재생 검증은 헤드리스로 실행 (X11 불필요)
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonocularInertialNode>(&SLAM);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
