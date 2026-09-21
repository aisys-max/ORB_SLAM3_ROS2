#include "monocular-inertial-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

// 프로젝트 표준 센서 토픽 계약(docs/ros2-topic-contract.md)의 /camera/image_raw, /imu를 따른다.
// CameraInfo는 구독하지 않는다 — 기존 wrapper와 동일하게 캘리브레이션은 ORB-SLAM3 설정 YAML로 받는다.
MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System *pSLAM) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(pSLAM)
{
    // 정수 depth만 넘기면 rclcpp가 기본(reliable) QoS로 만든다 - 브리지 노드(aisys-max/slam-tx2#6)는
    // docs/ros2-topic-contract.md대로 SensorDataQoS(best-effort)로 publish하는데, reliable 구독자는
    // best-effort 발행자와 아예 매칭이 안 된다 (DDS QoS 호환성 규칙). 그래서 라이브 캡처에서 이
    // 노드가 이미지를 하나도 못 받았다 - #3 EuRoC 검증 때는 ros2 bag play가 QoS 미지정 토픽을
    // reliable로 재생해 우연히 맞아떨어져 이 버그가 안 드러났다. best_effort()가 라이브 기본값이고,
    // depth는 기존 값(IMU 1000, 이미지 100) 그대로 유지한다.
    //
    // (slam-tx2#20) best-effort는 얕은 큐와 맞물려 재생 타이밍에 따라 매번 다른 메시지를
    // 조용히 드랍한다 - 확인 결과 같은 bag을 재생해도 실제 도착하는 프레임/IMU 개수가 재생마다
    // 달랐다(예: 이미지 1184개 중 1182~1183개, IMU 16525개 중 16007~16062개). 이러면 bag 기반
    // 회귀 테스트가 매번 다른 입력을 넣는 셈이라 비교가 무의미해진다. `reliable_sensor_qos`
    // 파라미터(기본 false, 라이브 캡처는 그대로 best-effort)를 true로 주면 reliable + 넉넉한
    // depth로 구독해 재생 테스트에서 유실 없는 결정적 입력을 보장한다(같은 bag을 2번 재생해
    // 이미지 1184/1184, IMU 16522/16522로 완전히 동일하게 도착함을 확인).
    //   ros2 run orbslam3 mono-inertial <vocab> <settings> --ros-args -p reliable_sensor_qos:=true
    bool reliableSensorQos = this->declare_parameter<bool>("reliable_sensor_qos", false);
    rclcpp::QoS imuQos(1000);
    rclcpp::QoS imgQos(100);
    if (reliableSensorQos)
    {
        imuQos = rclcpp::QoS(5000).reliable();
        imgQos = rclcpp::QoS(200).reliable();
        RCLCPP_WARN(this->get_logger(), "reliable_sensor_qos=true: 센서 구독을 reliable로 전환함 - "
                    "bag 재생 기반 결정적 테스트 전용, 라이브 캡처(best-effort 발행)에는 쓰지 말 것");
    }
    else
    {
        imuQos = imuQos.best_effort();
        imgQos = imgQos.best_effort();
    }
    subImu_ = this->create_subscription<ImuMsg>("imu", imuQos, std::bind(&MonocularInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("camera/image_raw", imgQos, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    // docs/ros2-topic-contract.md: /orb_slam3/trajectory, reliable QoS(10) - 유실되면 안 되는 결과물이라
    // 센서 토픽(image_raw/imu)의 SensorDataQoS와 다르게 reliable을 쓴다.
    pubPath_ = this->create_publisher<PathMsg>("orb_slam3/trajectory", rclcpp::QoS(10));
    // "map"은 SLAM이 초기화 시 잡는 세계 좌표계다 - camera_link(카메라에 고정된, 계속 움직이는
    // 프레임)로는 궤적처럼 시간에 걸친 고정 기준점이 필요한 걸 표현할 수 없다.
    pathMsg_.header.frame_id = "map";

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
    // 처리할 게 없을 때(버퍼가 비었거나 IMU가 아직 이미지 시각을 못 따라잡았을 때)마다 매번 아래
    // sleep_for로 양보한다 - 원래 이 경로들엔 sleep이 전혀 없어서 스레드가 CPU 한 코어를 100%
    // 계속 태우는 busy-wait이었다. TX2는 코어가 4개뿐이라, 이 busy-wait이 실제 메시지를
    // 콜백으로 전달하는 rclcpp executor 스레드와 CPU를 놓고 경쟁해 라이브 캡처(#7)에서
    // 이미지/IMU 콜백 자체가 지연되고 트래킹이 아예 시작을 못 하는 원인이 됐다 (5분 넘게 100%
    // CPU를 쓰면서도 로그에 진전이 전혀 없었음 - EuRoC bag 재생(#3)은 느슨한 재생 속도라 이
    // 버그가 있어도 안 드러났을 뿐이다).
    const auto kIdleSleep = std::chrono::milliseconds(1);

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
            {
                std::this_thread::sleep_for(kIdleSleep);
                continue;
            }

            bufMutexImg_.lock();
            ImageMsg::SharedPtr imgMsg = imgBuf_.front();
            im = GetImage(imgMsg);
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

            Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);

            int trackingState = SLAM_->GetTrackingState();

            // NOT_INITIALIZED(1) -> OK(2) 전이는 새 맵(Atlas가 만든 새 좌표계 원점)의 초기화가 막
            // 끝난 시점이다 - 최초 초기화든 트래킹을 완전히 잃은 뒤 재초기화든 동일하게 여기서
            // 잡힌다. 이 시점에 쌓아온 pathMsg_를 비우지 않으면, 새 원점 기준 포즈가 이전 원점
            // 기준 포즈 뒤에 그대로 이어붙어 RViz Path가 서로 다른 좌표계의 점을 하나의 연속된
            // 선으로 그린다 (#15 - "실제 움직임과 다른 모양으로 궤적이 그려짐"의 근본 원인).
            // RECENTLY_LOST(3)에서 OK로 돌아오는 relocalization은 같은 맵/원점을 재사용하므로
            // 여기 해당하지 않는다 - 끊지 않고 이어 그리는 게 맞다.
            if (lastTrackingState_ == ORB_SLAM3::Tracking::NOT_INITIALIZED &&
                trackingState == ORB_SLAM3::Tracking::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Map (re)initialized - clearing published trajectory to avoid stitching poses from a different origin (#15)");
                pathMsg_.poses.clear();
            }
            lastTrackingState_ = trackingState;

            // OK(2) - 트래킹이 안 됐거나(초기화 전/유실) 아직 신뢰할 수 없는 포즈까지 궤적에
            // 넣으면 RViz에 원점 근처로 튀는 지점이 섞인다.
            if (trackingState == ORB_SLAM3::Tracking::OK)
            {
                Sophus::SE3f Twc = Tcw.inverse();
                Eigen::Vector3f twc = Twc.translation();
                Eigen::Quaternionf q = Twc.unit_quaternion();

                geometry_msgs::msg::PoseStamped poseMsg;
                poseMsg.header.stamp = imgMsg->header.stamp;
                poseMsg.header.frame_id = "map";
                poseMsg.pose.position.x = twc.x();
                poseMsg.pose.position.y = twc.y();
                poseMsg.pose.position.z = twc.z();
                poseMsg.pose.orientation.x = q.x();
                poseMsg.pose.orientation.y = q.y();
                poseMsg.pose.orientation.z = q.z();
                poseMsg.pose.orientation.w = q.w();

                pathMsg_.header.stamp = poseMsg.header.stamp;
                pathMsg_.poses.push_back(poseMsg);
                pubPath_->publish(pathMsg_);
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        else
        {
            std::this_thread::sleep_for(kIdleSleep);
        }
    }
}
