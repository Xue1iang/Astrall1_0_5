#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace
{
constexpr uint16_t kSportStatusFixedStand = 0xB102;
constexpr uint16_t kSportStatusMove = 0xB104;
std::atomic<uint16_t> sport_status{0};

void sportStatusCallback(const std_msgs::UInt16::ConstPtr &msg)
{
    sport_status.store(msg->data);
}

class TerminalMode
{
public:
    TerminalMode()
    {
        if (!isatty(STDIN_FILENO))
            throw std::runtime_error("stdin is not a terminal; run this node in its own terminal");
        if (tcgetattr(STDIN_FILENO, &original_) != 0)
            throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
        active_ = true;
    }

    ~TerminalMode()
    {
        if (active_)
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }

    TerminalMode(const TerminalMode &) = delete;
    TerminalMode &operator=(const TerminalMode &) = delete;

private:
    termios original_{};
    bool active_{false};
};

bool readKey(char &key, const double timeout_seconds)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    const double bounded_timeout = std::max(0.0, timeout_seconds);
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(bounded_timeout);
    timeout.tv_usec = static_cast<suseconds_t>(
        (bounded_timeout - std::floor(bounded_timeout)) * 1e6);

    const int result = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (result < 0)
    {
        if (errno == EINTR) return false;
        throw std::runtime_error(std::string("select failed: ") + std::strerror(errno));
    }
    return result > 0 && read(STDIN_FILENO, &key, 1) == 1;
}

void printHelp(const double linear_speed, const double angular_speed)
{
    std::printf(
        "\nAstrall keyboard teleop (publishes /cmd_vel only)\n"
        "-----------------------------------------------\n"
        "w/s : forward/backward    a/d : left/right\n"
        "q/e : turn left/right     space or x : stop\n"
        "2   : stand               3   : lie down\n"
        "4   : enter MOVE mode\n"
        "Ctrl-C: exit\n"
        "linear speed: %.2f m/s, angular speed: %.2f rad/s\n\n",
        linear_speed, angular_speed);
    std::fflush(stdout);
}
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astrall_keyboard_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double linear_speed = 0.25;
    double angular_speed = 0.5;
    double publish_rate = 20.0;
    double key_timeout = 0.6;
    private_nh.param("linear_speed", linear_speed, linear_speed);
    private_nh.param("angular_speed", angular_speed, angular_speed);
    private_nh.param("publish_rate", publish_rate, publish_rate);
    private_nh.param("key_timeout", key_timeout, key_timeout);

    if (publish_rate <= 0.0 || key_timeout <= 0.0)
    {
        ROS_FATAL("publish_rate and key_timeout must be greater than zero");
        return 1;
    }

    const ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::ServiceClient stand_client =
        nh.serviceClient<std_srvs::Trigger>("/astrall/stand");
    ros::ServiceClient down_client =
        nh.serviceClient<std_srvs::Trigger>("/astrall/down");
    ros::ServiceClient move_mode_client =
        nh.serviceClient<std_srvs::SetBool>("/astrall/set_move_mode");
    ros::Subscriber sport_status_sub =
        nh.subscribe<std_msgs::UInt16>("/astrall/sport_status", 10, sportStatusCallback);

    try
    {
        TerminalMode terminal_mode;
        printHelp(linear_speed, angular_speed);

        geometry_msgs::Twist command;
        ros::WallTime last_motion_key;
        bool motion_active = false;
        const double poll_period = 1.0 / publish_rate;

        while (ros::ok())
        {
            char key = '\0';
            if (readKey(key, poll_period))
            {
                geometry_msgs::Twist next_command;
                bool motion_key = true;
                switch (key)
                {
                case 'w': next_command.linear.x = linear_speed; break;
                case 's': next_command.linear.x = -linear_speed; break;
                case 'a': next_command.linear.y = linear_speed; break;
                case 'd': next_command.linear.y = -linear_speed; break;
                case 'q': next_command.angular.z = angular_speed; break;
                case 'e': next_command.angular.z = -angular_speed; break;
                case '2':
                {
                    motion_key = false;
                    motion_active = false;
                    command = geometry_msgs::Twist();
                    cmd_vel_pub.publish(command);
                    std_srvs::Trigger service;
                    if (!stand_client.call(service) || !service.response.success)
                        ROS_ERROR("Stand request failed: %s", service.response.message.c_str());
                    else
                        ROS_INFO("%s", service.response.message.c_str());
                    break;
                }
                case '3':
                {
                    motion_key = false;
                    motion_active = false;
                    command = geometry_msgs::Twist();
                    cmd_vel_pub.publish(command);
                    std_srvs::Trigger service;
                    if (!down_client.call(service) || !service.response.success)
                        ROS_ERROR("Down request failed: %s", service.response.message.c_str());
                    else
                        ROS_INFO("%s", service.response.message.c_str());
                    break;
                }
                case '4':
                {
                    motion_key = false;
                    if (sport_status.load() != kSportStatusFixedStand)
                    {
                        ROS_WARN("Refusing MOVE: wait for sport status 0xB102 (current 0x%04X)",
                                 static_cast<unsigned int>(sport_status.load()));
                        break;
                    }
                    motion_active = false;
                    command = geometry_msgs::Twist();
                    cmd_vel_pub.publish(command);
                    std_srvs::SetBool service;
                    service.request.data = true;
                    if (!move_mode_client.call(service) || !service.response.success)
                        ROS_ERROR("MOVE request failed: %s", service.response.message.c_str());
                    else
                        ROS_INFO("%s", service.response.message.c_str());
                    break;
                }
                case 'x':
                case ' ':
                    motion_key = false;
                    motion_active = false;
                    command = geometry_msgs::Twist();
                    cmd_vel_pub.publish(command);
                    break;
                default:
                    motion_key = false;
                    break;
                }

                if (motion_key)
                {
                    if (sport_status.load() != kSportStatusMove)
                    {
                        ROS_WARN_THROTTLE(1.0,
                                          "Motion ignored: press 2, wait for 0xB102, then press 4");
                    }
                    else
                    {
                        command = next_command;
                        last_motion_key = ros::WallTime::now();
                        motion_active = true;
                    }
                }
            }

            if (motion_active &&
                (ros::WallTime::now() - last_motion_key).toSec() > key_timeout)
            {
                command = geometry_msgs::Twist();
                motion_active = false;
                cmd_vel_pub.publish(command);
            }
            if (sport_status.load() != kSportStatusMove)
            {
                command = geometry_msgs::Twist();
                motion_active = false;
            }

            // Keep publishing only while motion is active. AstrallMove is invalid
            // in FIXEDSTAND/FIXEDDOWN, even when the requested velocity is zero.
            if (motion_active)
                cmd_vel_pub.publish(command);
            ros::spinOnce();
        }

        cmd_vel_pub.publish(geometry_msgs::Twist());
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Keyboard teleop failed: %s", error.what());
        return 1;
    }

    return 0;
}
