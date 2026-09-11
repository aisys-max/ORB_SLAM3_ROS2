#include "monocular-inertial-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

// 프로젝트 표준 센서 토픽 계약(docs/ros2-topic-contract.md)의 /camera/image_raw, /imu를 따른다.
// CameraInfo는 구독하지 않는다 — 기존 wrapper와 동일하게 캘리브레이션은 ORB-SLAM3 설정 YAML로 받는다.
MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(pSLAM)
{
    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("camera/image_raw", 100, std::bind(&MonocularInertialNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);
}

MonocularInertialNode::~MonocularInertialNode()
{
    // stopSync_ 없이는 SyncWithImu()의 while(1)이 절대 끝나지 않아 join()이 영원히 멈춘다
    // (bag 재생이 끝나고 SIGINT로 종료할 때 실제로 이 문제로 프로세스가 멈췄다).
    stopSync_ = true;
    syncThread_->join();
    delete syncThread_;

    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutexImu_.lock();
    imuBuf_.push(msg);
    bufMutexImu_.unlock();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();
    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);
    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::SyncWithImu()
{
    while (!stopSync_)
    {
        cv::Mat im;
        double tIm = 0;

        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            bufMutexImg_.lock();
            tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);
            bufMutexImg_.unlock();

            // 아직 이 프레임 시각 이후의 IMU 샘플이 없으면(=IMU가 이미지보다 뒤처짐) 다음 IMU가 도착할 때까지 기다린다.
            bufMutexImu_.lock();
            bool imuReady = !imuBuf_.empty() && tIm <= Utility::StampToSec(imuBuf_.back()->header.stamp);
            bufMutexImu_.unlock();
            if (!imuReady)
                continue;

            bufMutexImg_.lock();
            im = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutexImu_.lock();
            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
            {
                double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
            bufMutexImu_.unlock();

            SLAM_->TrackMonocular(im, tIm, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
