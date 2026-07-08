#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "interface.h"

namespace
{
AstrallSubscribeFreq freqFromHz(const int hz)
{
    switch (hz)
    {
    case 0: return ASTRALL_SUB_FREQ_CLOSE;
    case 1: return ASTRALL_SUB_FREQ_1HZ;
    case 25: return ASTRALL_SUB_FREQ_25HZ;
    case 50: return ASTRALL_SUB_FREQ_50HZ;
    case 125: return ASTRALL_SUB_FREQ_125HZ;
    case 250: return ASTRALL_SUB_FREQ_250HZ;
    default:
        ROS_WARN("Unsupported subscription frequency %d Hz; using 25 Hz", hz);
        return ASTRALL_SUB_FREQ_25HZ;
    }
}

float clampFloat(const float value, const double limit)
{
    const float lim = static_cast<float>(std::abs(limit));
    return std::max(-lim, std::min(value, lim));
}

sensor_msgs::Imu imuToMsg(const AstrallImuData &data,
                          const std::string &frame_id,
                          const ros::Time &stamp,
                          const bool orientation_from_rpy,
                          const bool rpy_in_degrees)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    if (orientation_from_rpy)
    {
        // Measurements on the target robot show that the SDK Euler fields are
        // degrees (their values agree with the gravity vector only after a
        // degree-to-radian conversion). The native quaternion ordering remains
        // undocumented, so keep Euler conversion explicit and configurable.
        constexpr double kDegreesToRadians = 0.017453292519943295;
        const double scale = rpy_in_degrees ? kDegreesToRadians : 1.0;
        const double roll = static_cast<double>(data.roll) * scale;
        const double pitch = static_cast<double>(data.pitch) * scale;
        const double yaw = static_cast<double>(data.yaw) * scale;
        const double cr = std::cos(roll * 0.5);
        const double sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5);
        const double sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5);
        const double sy = std::sin(yaw * 0.5);
        msg.orientation.w = cr * cp * cy + sr * sp * sy;
        msg.orientation.x = sr * cp * cy - cr * sp * sy;
        msg.orientation.y = cr * sp * cy + sr * cp * sy;
        msg.orientation.z = cr * cp * sy - sr * sp * cy;
    }
    else
    {
        msg.orientation.w = data.quaternion[0];
        msg.orientation.x = data.quaternion[1];
        msg.orientation.y = data.quaternion[2];
        msg.orientation.z = data.quaternion[3];
    }
    msg.angular_velocity.x = data.gyroscope[0];
    msg.angular_velocity.y = data.gyroscope[1];
    msg.angular_velocity.z = data.gyroscope[2];
    msg.linear_acceleration.x = data.accelerometer[0];
    msg.linear_acceleration.y = data.accelerometer[1];
    msg.linear_acceleration.z = data.accelerometer[2];
    return msg;
}
}  // namespace

class AstrallDriverNode
{
public:
    AstrallDriverNode()
        : nh_(), private_nh_("~")
    {
        readParameters();
        initPublishers();
        initServices();
        initSdk();
        initSubscriptions();
        initTimers();
    }

    ~AstrallDriverNode()
    {
        if (sdk_initialized_)
        {
            AstrallMove(0.0f, 0.0f, 0.0f);
            if (camera_enabled_)
            {
                const auto discard = [](void *, uint16_t) {};
                const uint16_t result = AstrallSubscriptionData(
                    ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
                    ASTRALL_SUB_FREQ_CLOSE, discard);
                if (result != ASTRALL_RES_SUCCESSED)
                    ROS_WARN("Disable camera RGB stream failed: 0x%04X",
                             static_cast<unsigned int>(result));
            }
            AstrallSdkDeinit();
        }
    }

private:
    void readParameters()
    {
        private_nh_.param("heartbeat_period_ms", heartbeat_period_ms_, 100);
        private_nh_.param("status_period_ms", status_period_ms_, 500);
        private_nh_.param("auto_enable_sdk_auth", auto_enable_sdk_auth_, true);
        private_nh_.param("auto_enter_move_mode", auto_enter_move_mode_, false);
        private_nh_.param("cmd_vel_timeout_ms", cmd_vel_timeout_ms_, 500);
        private_nh_.param("full_down_timeout_sec", full_down_timeout_sec_, 15.0);
        private_nh_.param("max_vx", max_vx_, 1.0);
        private_nh_.param("max_vy", max_vy_, 1.0);
        private_nh_.param("max_vyaw", max_vyaw_, 1.0);
        private_nh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
        private_nh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_raw");
        private_nh_.param("camera_enabled", camera_enabled_, true);
        private_nh_.param("camera_sdk_callback_enabled", camera_sdk_callback_enabled_, false);
        private_nh_.param("camera_stream_start_delay_sec",
                          camera_stream_start_delay_sec_, 1.0);
        private_nh_.param("camera_stream_retry_interval_sec",
                          camera_stream_retry_interval_sec_, 2.0);
        private_nh_.param("camera_stream_max_retries",
                          camera_stream_max_retries_, 15);
        private_nh_.param("camera_width", camera_width_, 640);
        private_nh_.param("camera_height", camera_height_, 480);
        private_nh_.param<std::string>("camera_encoding", camera_encoding_, "rgb8");
        private_nh_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");
        private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
        private_nh_.param("imu_orientation_from_rpy", imu_orientation_from_rpy_, true);
        private_nh_.param("use_sdk_imu_timestamp", use_sdk_imu_timestamp_, true);
        private_nh_.param("imu_rpy_in_degrees", imu_rpy_in_degrees_, true);
        private_nh_.param("camera_freq", camera_freq_, 25);
        private_nh_.param("imu_freq", imu_freq_, 250);
        private_nh_.param("sport_freq", sport_freq_, 250);
    }

    void initPublishers()
    {
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
        if (camera_sdk_callback_enabled_)
            camera_pub_ = nh_.advertise<sensor_msgs::Image>(camera_topic_, 2);
        sport_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/astrall/sport_data", 10);
        sdk_status_pub_ = nh_.advertise<std_msgs::UInt16>("/astrall/sdk_status", 10);
        system_status_pub_ = nh_.advertise<std_msgs::UInt32>("/astrall/system_status", 10);
        error_code_pub_ = nh_.advertise<std_msgs::UInt32>("/astrall/error_code", 10);
        warn_code_pub_ = nh_.advertise<std_msgs::UInt32>("/astrall/warn_code", 10);
        sport_status_pub_ = nh_.advertise<std_msgs::UInt16>("/astrall/sport_status", 10);
        battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/battery_state", 10);
    }

    void initServices()
    {
        sdk_auth_srv_ = nh_.advertiseService("/astrall/set_sdk_auth",
                                             &AstrallDriverNode::setSdkAuth, this);
        move_mode_srv_ = nh_.advertiseService("/astrall/set_move_mode",
                                              &AstrallDriverNode::setMoveMode, this);
        stand_srv_ = nh_.advertiseService("/astrall/stand",
                                          &AstrallDriverNode::stand, this);
        down_srv_ = nh_.advertiseService("/astrall/down",
                                         &AstrallDriverNode::down, this);
        full_down_srv_ = nh_.advertiseService("/astrall/full_down",
                                              &AstrallDriverNode::fullDown, this);
        light_srv_ = nh_.advertiseService("/astrall/set_light",
                                          &AstrallDriverNode::setLight, this);
    }

    void initSdk()
    {
        AstrallConfig cfg;
        cfg.heartbeatCb = [](void *, uint16_t) {};
        cfg.sdkStatusCb = [this](void *data, uint16_t len) {
            if (data == nullptr || len != sizeof(AstrallSdkStatus)) return;
            const auto *status = static_cast<const AstrallSdkStatus *>(data);
            std_msgs::UInt16 msg;
            msg.data = static_cast<uint16_t>((status->link ? 1U : 0U) |
                                             (status->ctrlAuthority ? 2U : 0U));
            sdk_status_pub_.publish(msg);
        };

        const uint16_t result = AstrallSdkInit(cfg);
        if (result != ASTRALL_RES_SUCCESSED)
        {
            throw std::runtime_error(resultMessage("AstrallSdkInit", result));
        }
        sdk_initialized_ = true;

        if (auto_enable_sdk_auth_)
            logSdkResult("AstrallAuthControl", AstrallAuthControl(ASTRALL_AUTH_SDK));
        if (auto_enter_move_mode_)
            logSdkResult("AstrallSportModeControl",
                         AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE));

        logSdkResult("Subscribe IMU",
                     AstrallSubscriptionData(
                         ASTRALL_SUB_TOPIC_ID_IMU, freqFromHz(imu_freq_),
                         std::bind(&AstrallDriverNode::handleImu, this,
                                   std::placeholders::_1, std::placeholders::_2)));
        logSdkResult("Subscribe sport",
                     AstrallSubscriptionData(
                         ASTRALL_SUB_TOPIC_ID_SPORT, freqFromHz(sport_freq_),
                         std::bind(&AstrallDriverNode::handleSport, this,
                                   std::placeholders::_1, std::placeholders::_2)));
        if (camera_enabled_)
            initCameraStream();
    }

    void initSubscriptions()
    {
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &AstrallDriverNode::handleCmdVel, this);
    }

    void initTimers()
    {
        heartbeat_timer_ = nh_.createTimer(
            ros::Duration(heartbeat_period_ms_ / 1000.0),
            &AstrallDriverNode::heartbeatTimer, this);
        status_timer_ = nh_.createTimer(
            ros::Duration(status_period_ms_ / 1000.0),
            &AstrallDriverNode::statusTimer, this);
    }

    void handleCmdVel(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (full_down_pending_)
        {
            const bool nonzero_command = std::abs(msg->linear.x) > 1e-6 ||
                                         std::abs(msg->linear.y) > 1e-6 ||
                                         std::abs(msg->angular.z) > 1e-6;
            if (nonzero_command)
                ROS_WARN_THROTTLE(1.0,
                                  "Ignoring cmd_vel while full-down sequence is active");
            return;
        }
        if (AstrallGetSportStatus() != ASTRALL_SPORT_STATUS_MOVE)
        {
            const bool nonzero_command = std::abs(msg->linear.x) > 1e-6 ||
                                         std::abs(msg->linear.y) > 1e-6 ||
                                         std::abs(msg->angular.z) > 1e-6;
            if (nonzero_command)
                ROS_WARN_THROTTLE(1.0,
                                  "Ignoring cmd_vel because robot is not in MOVE mode");
            stop_motion_ = true;
            return;
        }

        last_cmd_time_ = ros::Time::now();
        stop_motion_ = false;
        const uint16_t result = AstrallMove(
            clampFloat(static_cast<float>(msg->linear.x), max_vx_),
            clampFloat(static_cast<float>(msg->linear.y), max_vy_),
            clampFloat(static_cast<float>(msg->angular.z), max_vyaw_));
        if (result != ASTRALL_RES_SUCCESSED)
            ROS_WARN_THROTTLE(1.0, "AstrallMove failed: 0x%04X",
                              static_cast<unsigned int>(result));
    }

    void heartbeatTimer(const ros::TimerEvent &)
    {
        const uint16_t result = AstrallHeartbeat();
        if (result != ASTRALL_RES_SUCCESSED)
            ROS_WARN_THROTTLE(1.0, "AstrallHeartbeat failed: 0x%04X",
                              static_cast<unsigned int>(result));
    }

    void statusTimer(const ros::TimerEvent &)
    {
        advanceFullDown();
        retryCameraStream();
        publishStatus();
        enforceCmdVelTimeout();
    }

    void handleImu(void *data, uint16_t len)
    {
        if (data == nullptr || len != sizeof(AstrallImuData)) return;
        const auto *imu = static_cast<const AstrallImuData *>(data);
        if (use_sdk_imu_timestamp_ && imu_timestamp_last_ != 0 &&
            imu->timestamp == imu_timestamp_last_)
        {
            ROS_WARN_THROTTLE(1.0, "Dropping duplicate Astrall IMU timestamp");
            return;
        }
        imu_pub_.publish(imuToMsg(*imu, imu_frame_id_, imuStamp(imu->timestamp),
                                  imu_orientation_from_rpy_, imu_rpy_in_degrees_));
    }

    ros::Time imuStamp(const int64_t sdk_timestamp)
    {
        const ros::Time now = ros::Time::now();
        if (!use_sdk_imu_timestamp_ || sdk_timestamp <= 0)
            return now;

        if (imu_timestamp_base_ == 0 || sdk_timestamp < imu_timestamp_last_)
        {
            imu_timestamp_base_ = sdk_timestamp;
            imu_timestamp_last_ = sdk_timestamp;
            imu_ros_time_base_ = now;
            imu_timestamp_scale_ = 0.0;
            return now;
        }

        const int64_t raw_delta = sdk_timestamp - imu_timestamp_last_;
        if (imu_timestamp_scale_ == 0.0 && raw_delta > 0)
        {
            // Select the unit that puts one sample closest to the configured
            // IMU period. This handles integer ms/us/ns vendor timestamps.
            const double expected_dt = 1.0 / std::max(1, imu_freq_);
            const double candidates[] = {1e-3, 1e-6, 1e-9};
            double best_error = std::numeric_limits<double>::infinity();
            for (const double candidate : candidates)
            {
                const double dt = static_cast<double>(raw_delta) * candidate;
                const double error = std::abs(std::log(dt / expected_dt));
                if (error < best_error)
                {
                    best_error = error;
                    imu_timestamp_scale_ = candidate;
                }
            }
            ROS_INFO("Astrall IMU timestamp unit detected: scale %.0e s/tick",
                     imu_timestamp_scale_);
        }

        if (raw_delta > 0)
            imu_timestamp_last_ = sdk_timestamp;

        if (imu_timestamp_scale_ == 0.0)
            return now;

        const double elapsed = static_cast<double>(sdk_timestamp - imu_timestamp_base_) *
                               imu_timestamp_scale_;
        const ros::Time stamp = imu_ros_time_base_ + ros::Duration(elapsed);

        // Re-anchor if the sensor clock jumps or drifts far from ROS wall time.
        if (std::abs((stamp - now).toSec()) > 1.0)
        {
            ROS_WARN("Astrall IMU timestamp jumped; re-anchoring to ROS time");
            imu_timestamp_base_ = sdk_timestamp;
            imu_timestamp_last_ = sdk_timestamp;
            imu_ros_time_base_ = now;
            return now;
        }
        return stamp;
    }

    void handleSport(void *data, uint16_t len)
    {
        if (data == nullptr || len != sizeof(AstrallSportData)) return;
        const auto *sport = static_cast<const AstrallSportData *>(data);
        std_msgs::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(sport->timestamp), sport->wheelSpeed[0],
            sport->wheelSpeed[1], sport->wheelSpeed[2], sport->wheelSpeed[3]};
        sport_pub_.publish(msg);
    }

    void initCameraStream()
    {
        camera_callback_ = std::bind(&AstrallDriverNode::handleCamera, this,
                                     std::placeholders::_1, std::placeholders::_2);
        camera_stream_init_time_ = ros::WallTime::now();

        // SDK 1.0.5 requires this task registration before the RGB subscription.
        // Register it once. Stream reset/start is deferred to the ROS timer so
        // the 10 Hz heartbeat is already running and control is not revoked.
        const uint16_t register_result = AstrallRegisterLoadTask(camera_callback_);
        camera_load_task_registered_ = register_result == ASTRALL_RES_SUCCESSED;
        logSdkResult("Register camera load task", register_result);
    }

    void requestCameraStream()
    {
        camera_last_attempt_ = ros::WallTime::now();
        ++camera_stream_attempts_;
        const uint16_t result = AstrallSubscriptionData(
            ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
            freqFromHz(camera_freq_), camera_callback_);
        camera_stream_active_ = result == ASTRALL_RES_SUCCESSED;

        if (camera_stream_active_)
        {
            ROS_INFO("Enable camera RGB stream succeeded on attempt %d: 0x%04X",
                     camera_stream_attempts_, static_cast<unsigned int>(result));
            return;
        }

        ROS_WARN("Enable camera RGB stream failed on attempt %d: 0x%04X",
                 camera_stream_attempts_, static_cast<unsigned int>(result));
        AstrallSystemStatus status;
        if (AstrallGetSystemStatus(status) == ASTRALL_RES_SUCCESSED)
        {
            const uint32_t error = static_cast<uint32_t>(status.errorCode);
            const uint32_t warning = static_cast<uint32_t>(status.warnCode);
            ROS_WARN("Camera subscription diagnostic: system=%u error=0x%08X warning=0x%08X",
                     static_cast<unsigned int>(status.sysStatus), error, warning);
            if ((error & static_cast<uint32_t>(ASTRALL_ERR_LOSE_CAMERA)) != 0U)
                ROS_ERROR("Astrall reports that no camera device was detected");
        }
    }

    void retryCameraStream()
    {
        if (!camera_enabled_ || camera_stream_active_ ||
            !camera_load_task_registered_)
            return;

        const ros::WallTime now = ros::WallTime::now();
        if (!camera_stream_reset_done_)
        {
            if ((now - camera_stream_init_time_).toSec() <
                std::max(0.0, camera_stream_start_delay_sec_))
                return;

            const uint16_t stop_result = AstrallSubscriptionData(
                ASTRALL_SUB_TOPIC_ID_CAMERA_RGB,
                ASTRALL_SUB_FREQ_CLOSE, camera_callback_);
            ROS_INFO("Reset camera RGB stream result: 0x%04X",
                     static_cast<unsigned int>(stop_result));
            camera_stream_reset_done_ = true;
            camera_last_attempt_ = now;
            return;
        }

        if (camera_stream_max_retries_ > 0 &&
            camera_stream_attempts_ >= camera_stream_max_retries_)
            return;
        const double wait_seconds = camera_stream_attempts_ == 0
            ? 0.3 : camera_stream_retry_interval_sec_;
        if (wait_seconds < 0.0 ||
            (now - camera_last_attempt_).toSec() < wait_seconds)
            return;
        requestCameraStream();
    }

    void handleCamera(void *data, uint16_t len)
    {
        if (!camera_sdk_callback_enabled_ || data == nullptr || len == 0) return;
        const size_t bytes_per_pixel = cameraEncodingBytesPerPixel();
        const size_t expected_len = static_cast<size_t>(camera_width_) *
                                    static_cast<size_t>(camera_height_) * bytes_per_pixel;
        if (expected_len != len)
        {
            ROS_WARN_THROTTLE(2.0,
                              "Camera payload len=%u does not match expected image bytes=%zu",
                              static_cast<unsigned int>(len), expected_len);
            return;
        }

        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = camera_frame_id_;
        msg.height = static_cast<uint32_t>(camera_height_);
        msg.width = static_cast<uint32_t>(camera_width_);
        msg.encoding = camera_encoding_;
        msg.is_bigendian = false;
        msg.step = static_cast<uint32_t>(camera_width_ * bytes_per_pixel);
        const auto *begin = static_cast<const uint8_t *>(data);
        msg.data.assign(begin, begin + len);
        camera_pub_.publish(msg);
    }

    void publishStatus()
    {
        AstrallSystemStatus status;
        uint16_t result = AstrallGetSystemStatus(status);
        if (result == ASTRALL_RES_SUCCESSED)
        {
            std_msgs::UInt32 system_msg;
            system_msg.data = static_cast<uint32_t>(status.sysStatus);
            system_status_pub_.publish(system_msg);
            std_msgs::UInt32 error_msg;
            error_msg.data = static_cast<uint32_t>(status.errorCode);
            error_code_pub_.publish(error_msg);
            std_msgs::UInt32 warn_msg;
            warn_msg.data = static_cast<uint32_t>(status.warnCode);
            warn_code_pub_.publish(warn_msg);
        }

        std_msgs::UInt16 sport_msg;
        sport_msg.data = AstrallGetSportStatus();
        sport_status_pub_.publish(sport_msg);

        AstrallPowerStatus power;
        result = AstrallGetPowerStatus(power);
        if (result == ASTRALL_RES_SUCCESSED)
        {
            sensor_msgs::BatteryState msg;
            msg.header.stamp = ros::Time::now();
            msg.percentage = power.soc / 100.0f;
            msg.temperature = power.temp;
            msg.voltage = power.voltage;
            msg.power_supply_status = power.charged
                ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            battery_pub_.publish(msg);
        }
    }

    void enforceCmdVelTimeout()
    {
        if (stop_motion_) return;
        if ((ros::Time::now() - last_cmd_time_).toSec() * 1000.0 > cmd_vel_timeout_ms_)
        {
            AstrallMove(0.0f, 0.0f, 0.0f);
            stop_motion_ = true;
        }
    }

    bool setSdkAuth(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (!req.data)
            cancelFullDown("control authority released to joystick");
        const uint16_t result = AstrallAuthControl(
            req.data ? ASTRALL_AUTH_SDK : ASTRALL_AUTH_JOYSTICK);
        res.success = result == ASTRALL_RES_SUCCESSED;
        res.message = resultMessage("set_sdk_auth", result);
        return true;
    }

    bool setMoveMode(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        cancelFullDown("set_move_mode requested");
        if (!req.data)
        {
            // Stop commanded motion before entering DAMPING (soft emergency stop).
            if (AstrallGetSportStatus() == ASTRALL_SPORT_STATUS_MOVE)
                AstrallMove(0.0f, 0.0f, 0.0f);
            stop_motion_ = true;
        }
        const AstrallSportModeCmd mode = req.data
            ? ASTRALL_SPORT_MODE_CMD_MOVE : ASTRALL_SPORT_MODE_CMD_DAMPING;
        const uint16_t result = AstrallSportModeControl(mode);
        res.success = result == ASTRALL_RES_SUCCESSED;
        res.message = resultMessage("set_move_mode", result);
        return true;
    }

    bool stand(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        cancelFullDown("stand requested");
        // Always clear the velocity command before changing the support state.
        AstrallMove(0.0f, 0.0f, 0.0f);
        stop_motion_ = true;
        const uint16_t result =
            AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND);
        res.success = result == ASTRALL_RES_SUCCESSED;
        res.message = resultMessage("stand", result);
        return true;
    }

    bool down(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        cancelFullDown("locked down requested");
        // Never request FIXEDDOWN while a stale non-zero command is active.
        AstrallMove(0.0f, 0.0f, 0.0f);
        stop_motion_ = true;
        const uint16_t result =
            AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDDOWN);
        res.success = result == ASTRALL_RES_SUCCESSED;
        res.message = resultMessage("down", result);
        return true;
    }

    bool fullDown(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        if (full_down_timeout_sec_ <= 0.0)
        {
            res.success = false;
            res.message = "full_down_timeout_sec must be greater than zero";
            return true;
        }

        // DAMPING removes active posture support. Always reach the mechanically
        // safe FIXEDDOWN posture first instead of damping from standing/MOVE.
        AstrallMove(0.0f, 0.0f, 0.0f);
        stop_motion_ = true;

        const uint16_t status = AstrallGetSportStatus();
        if (status == ASTRALL_SPORT_STATUS_DAMPING)
        {
            full_down_pending_ = false;
            full_down_damping_requested_ = false;
            res.success = true;
            res.message = "robot is already fully down in DAMPING";
            return true;
        }

        full_down_pending_ = true;
        full_down_damping_requested_ = false;
        full_down_started_ = ros::WallTime::now();

        uint16_t result = ASTRALL_RES_SUCCESSED;
        if (status != ASTRALL_SPORT_STATUS_FIXEDDOWN &&
            status != ASTRALL_SPORT_STATUS_FIXEDDOWNING)
        {
            result = AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_FIXEDDOWN);
        }

        if (result != ASTRALL_RES_SUCCESSED)
        {
            full_down_pending_ = false;
            res.success = false;
            res.message = resultMessage("full_down/fixed_down", result);
            return true;
        }

        res.success = true;
        res.message = "full-down sequence started: FIXEDDOWN, then DAMPING";
        return true;
    }

    void advanceFullDown()
    {
        if (!full_down_pending_) return;

        if ((ros::WallTime::now() - full_down_started_).toSec() >
            full_down_timeout_sec_)
        {
            ROS_ERROR("Full-down sequence timed out after %.1f seconds; current status 0x%04X",
                      full_down_timeout_sec_,
                      static_cast<unsigned int>(AstrallGetSportStatus()));
            full_down_pending_ = false;
            full_down_damping_requested_ = false;
            return;
        }

        const uint16_t status = AstrallGetSportStatus();
        if (status == ASTRALL_SPORT_STATUS_DAMPING)
        {
            ROS_INFO("Full-down sequence complete: robot is in DAMPING");
            full_down_pending_ = false;
            full_down_damping_requested_ = false;
            return;
        }

        if (status == ASTRALL_SPORT_STATUS_FIXEDDOWN &&
            !full_down_damping_requested_)
        {
            const uint16_t result =
                AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_DAMPING);
            if (result != ASTRALL_RES_SUCCESSED)
            {
                ROS_ERROR("%s", resultMessage("full_down/damping", result).c_str());
                full_down_pending_ = false;
                return;
            }
            full_down_damping_requested_ = true;
            ROS_INFO("Full-down sequence: FIXEDDOWN reached, requesting DAMPING");
        }
    }

    void cancelFullDown(const char *reason)
    {
        if (!full_down_pending_) return;
        full_down_pending_ = false;
        full_down_damping_requested_ = false;
        ROS_WARN("Full-down sequence cancelled: %s", reason);
    }

    bool setLight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        const AstrallLightCmd command = req.data
            ? ASTRALL_CMD_LIGHT_OPEN : ASTRALL_CMD_LIGHT_CLOSE;
        const uint16_t result = AstrallLightControl(command);
        res.success = result == ASTRALL_RES_SUCCESSED;
        res.message = resultMessage("set_light", result);
        return true;
    }

    size_t cameraEncodingBytesPerPixel() const
    {
        if (camera_encoding_ == "rgb8" || camera_encoding_ == "bgr8") return 3;
        if (camera_encoding_ == "rgba8" || camera_encoding_ == "bgra8") return 4;
        if (camera_encoding_ == "mono8") return 1;
        return 3;
    }

    std::string resultMessage(const std::string &operation, const uint16_t result) const
    {
        char buffer[96];
        std::snprintf(buffer, sizeof(buffer), "%s result: 0x%04X",
                      operation.c_str(), static_cast<unsigned int>(result));
        return std::string(buffer);
    }

    void logSdkResult(const std::string &operation, const uint16_t result)
    {
        if (result == ASTRALL_RES_SUCCESSED)
            ROS_INFO("%s succeeded: 0x%04X", operation.c_str(),
                     static_cast<unsigned int>(result));
        else
            ROS_WARN("%s failed: 0x%04X", operation.c_str(),
                     static_cast<unsigned int>(result));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    int heartbeat_period_ms_{100};
    int status_period_ms_{500};
    bool auto_enable_sdk_auth_{true};
    bool auto_enter_move_mode_{false};
    int cmd_vel_timeout_ms_{500};
    double full_down_timeout_sec_{15.0};
    double max_vx_{1.0};
    double max_vy_{1.0};
    double max_vyaw_{1.0};
    std::string imu_topic_{"/imu/data"};
    std::string camera_topic_{"/camera/image_raw"};
    bool camera_enabled_{true};
    bool camera_sdk_callback_enabled_{false};
    double camera_stream_start_delay_sec_{1.0};
    double camera_stream_retry_interval_sec_{2.0};
    int camera_stream_max_retries_{15};
    int camera_stream_attempts_{0};
    bool camera_load_task_registered_{false};
    bool camera_stream_active_{false};
    bool camera_stream_reset_done_{false};
    ros::WallTime camera_stream_init_time_;
    ros::WallTime camera_last_attempt_;
    std::function<void(void *, uint16_t)> camera_callback_;
    int camera_width_{640};
    int camera_height_{480};
    std::string camera_encoding_{"rgb8"};
    std::string camera_frame_id_{"camera_link"};
    std::string imu_frame_id_{"imu_link"};
    bool imu_orientation_from_rpy_{true};
    bool imu_rpy_in_degrees_{true};
    bool use_sdk_imu_timestamp_{true};
    int64_t imu_timestamp_base_{0};
    int64_t imu_timestamp_last_{0};
    double imu_timestamp_scale_{0.0};
    ros::Time imu_ros_time_base_;
    int camera_freq_{25};
    int imu_freq_{250};
    int sport_freq_{250};
    bool sdk_initialized_{false};
    bool stop_motion_{true};
    bool full_down_pending_{false};
    bool full_down_damping_requested_{false};
    ros::WallTime full_down_started_;
    ros::Time last_cmd_time_;

    ros::Publisher imu_pub_;
    ros::Publisher camera_pub_;
    ros::Publisher sport_pub_;
    ros::Publisher sdk_status_pub_;
    ros::Publisher system_status_pub_;
    ros::Publisher error_code_pub_;
    ros::Publisher warn_code_pub_;
    ros::Publisher sport_status_pub_;
    ros::Publisher battery_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::ServiceServer sdk_auth_srv_;
    ros::ServiceServer move_mode_srv_;
    ros::ServiceServer stand_srv_;
    ros::ServiceServer down_srv_;
    ros::ServiceServer full_down_srv_;
    ros::ServiceServer light_srv_;
    ros::Timer heartbeat_timer_;
    ros::Timer status_timer_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astrall_driver_node");
    try
    {
        AstrallDriverNode node;
        ros::spin();
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Fatal error: %s", error.what());
        return 1;
    }
    return 0;
}
