#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PathMsg = nav_msgs::msg::Path;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System *pSLAM);
    ~MonocularInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;
    rclcpp::Publisher<PathMsg>::SharedPtr pubPath_;
    PathMsg pathMsg_;
    // 직전 루프에서 관측한 트래킹 상태 - NOT_INITIALIZED(1) -> OK(2) 전이를 감지해 "새 맵(=새
    // 좌표계 원점)이 막 초기화됨"을 판별하는 데 쓴다(#15). 초기값은 상태 전이가 아직 없었음을
    // 뜻하는 SYSTEM_NOT_READY로 둔다.
    int lastTrackingState_ = ORB_SLAM3::Tracking::SYSTEM_NOT_READY;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;
    std::atomic<bool> stopSync_{false};

    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutexImu_;

    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;
};

#endif
