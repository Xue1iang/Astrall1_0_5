#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class AstrallH264CameraNode
{
public:
    AstrallH264CameraNode()
        : private_nh_("~")
    {
        private_nh_.param("udp_port", udp_port_, 6000);
        private_nh_.param("width", width_, 640);
        private_nh_.param("height", height_, 480);
        private_nh_.param("output_fps", output_fps_, 25);
        private_nh_.param<std::string>("frame_id", frame_id_, "camera_link");
        private_nh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");

        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
    }

    ~AstrallH264CameraNode()
    {
        stopFfmpeg();
    }

    bool run()
    {
        if (!startFfmpeg()) return false;

        const size_t frame_size = static_cast<size_t>(width_) *
                                  static_cast<size_t>(height_) * 3U;
        std::vector<uint8_t> frame(frame_size);
        ROS_INFO("Decoding H.264 UDP port %d to %dx%d RGB at up to %d FPS",
                 udp_port_, width_, height_, output_fps_);

        while (ros::ok())
        {
            if (!readFrame(frame))
            {
                if (ros::ok()) ROS_ERROR("FFmpeg camera stream ended");
                return false;
            }

            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id_;
            msg.height = static_cast<uint32_t>(height_);
            msg.width = static_cast<uint32_t>(width_);
            msg.encoding = "rgb8";
            msg.is_bigendian = false;
            msg.step = static_cast<uint32_t>(width_ * 3);
            msg.data = frame;
            image_pub_.publish(msg);
            ros::spinOnce();
        }
        return true;
    }

private:
    bool startFfmpeg()
    {
        int pipe_fds[2];
        if (pipe(pipe_fds) != 0)
        {
            ROS_ERROR("pipe failed: %s", std::strerror(errno));
            return false;
        }

        ffmpeg_pid_ = fork();
        if (ffmpeg_pid_ < 0)
        {
            ROS_ERROR("fork failed: %s", std::strerror(errno));
            close(pipe_fds[0]);
            close(pipe_fds[1]);
            return false;
        }

        if (ffmpeg_pid_ == 0)
        {
            dup2(pipe_fds[1], STDOUT_FILENO);
            close(pipe_fds[0]);
            close(pipe_fds[1]);

            const std::string input = "udp://0.0.0.0:" + std::to_string(udp_port_) +
                "?fifo_size=5000000&overrun_nonfatal=1";
            const std::string filter = "scale=" + std::to_string(width_) + ":" +
                std::to_string(height_) + ":flags=fast_bilinear,fps=" +
                std::to_string(output_fps_);

            execlp("ffmpeg", "ffmpeg",
                   "-nostdin",
                   "-loglevel", "warning",
                   "-fflags", "nobuffer",
                   "-flags", "low_delay",
                   "-probesize", "5000000",
                   "-analyzeduration", "5000000",
                   "-f", "h264",
                   "-i", input.c_str(),
                   "-an",
                   "-vf", filter.c_str(),
                   "-pix_fmt", "rgb24",
                   "-f", "rawvideo",
                   "pipe:1",
                   static_cast<char *>(nullptr));
            _exit(127);
        }

        close(pipe_fds[1]);
        ffmpeg_stdout_ = pipe_fds[0];
        return true;
    }

    bool readFrame(std::vector<uint8_t> &frame)
    {
        size_t offset = 0;
        while (offset < frame.size() && ros::ok())
        {
            const ssize_t count = read(ffmpeg_stdout_, frame.data() + offset,
                                       frame.size() - offset);
            if (count > 0)
            {
                offset += static_cast<size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) continue;
            if (count < 0)
                ROS_ERROR("Reading FFmpeg output failed: %s", std::strerror(errno));
            return false;
        }
        return offset == frame.size();
    }

    void stopFfmpeg()
    {
        if (ffmpeg_stdout_ >= 0)
        {
            close(ffmpeg_stdout_);
            ffmpeg_stdout_ = -1;
        }
        if (ffmpeg_pid_ > 0)
        {
            kill(ffmpeg_pid_, SIGTERM);
            waitpid(ffmpeg_pid_, nullptr, 0);
            ffmpeg_pid_ = -1;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher image_pub_;
    int udp_port_{6000};
    int width_{640};
    int height_{480};
    int output_fps_{25};
    std::string frame_id_{"camera_link"};
    std::string image_topic_{"/camera/image_raw"};
    pid_t ffmpeg_pid_{-1};
    int ffmpeg_stdout_{-1};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astrall_h264_camera_node");
    AstrallH264CameraNode node;
    return node.run() ? 0 : 1;
}
