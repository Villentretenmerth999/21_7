#include <iostream>
#include <fstream>
#include <string>

#include <stdint.h> // uint64_t;
#include <chrono> // time related
#include <ctime> // get_date_time

#include <rclcpp/rclcpp.hpp> // ros2
#include <px4_ros_com/frame_transforms.h> // <Eigen/Eigen> <Eigen/Geometry>

#include <px4_msgs/msg/rc_channels.hpp> // normalized rc *** scaled to -1..1 (throttle: 0..1) ***
#include <px4_msgs/msg/vehicle_status.hpp> // armed status

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

#include <px4_msgs/msg/reference_trajectory.hpp>
#include <px4_msgs/msg/reference_attitude_thrust.hpp>
#include <px4_msgs/msg/reference_virtual_rate.hpp>
#include <px4_msgs/msg/reference_torque.hpp>

#include <px4_msgs/msg/param_translation.hpp>
#include <px4_msgs/msg/param_attitude.hpp>

using namespace std::chrono; // time related
using namespace std::chrono_literals; // this->std::chrono_literals::create_wall_timer
using namespace px4_msgs::msg;
using namespace px4_ros_com::frame_transforms::utils::quaternion; // quaternion
using std::placeholders::_1; // using in std::bind


class LoggerCsv : public rclcpp::Node {
public:
    LoggerCsv();
    ~LoggerCsv();

private:
    // Subscriber callback
    void local_pos_cb(const VehicleLocalPosition::SharedPtr msg);

    // Helper functions
    std::string get_date_time();
    void init_file_header();

private:
    // Date and Time
    std::string date_time_;
    // Directory
    std::string directory_;

    // Files
    std::ofstream pos_fb_;
    std::ofstream vel_fb_;
    std::ofstream acc_fb_;

    // Subscriber
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
};

LoggerCsv::LoggerCsv() : Node("logger_csv")  {
    // Date and Time
    date_time_ = get_date_time();

    // Directory
    this->declare_parameter<std::string>("directory", "~/log/px4");
    directory_ = this->get_parameter("directory").as_string();

    if (directory_.front() == '~') {
        directory_.erase(directory_.begin());
        directory_ = std::string(getenv("HOME")) + directory_;
    }
    
    if (directory_.back() == '/') {
        directory_ = directory_ + date_time_ + std::string("/");
    } else {
        directory_ = directory_ + std::string("/") + date_time_ + std::string("/");
    }

    std::string cmd = std::string("mkdir -p ") + directory_;
    if (!system(cmd.data())) {
        RCLCPP_INFO(this->get_logger(), std::string("Folder created: ") + directory_);
    }

    // Files
    pos_fb_.open(directory_ + std::string("pos_fb.csv"));
    vel_fb_.open(directory_ + std::string("vel_fb.csv"));
    acc_fb_.open(directory_ + std::string("acc_fb.csv"));

    init_file_header();

    // Subscriber
    local_pos_sub_ = this->create_subscription<VehicleLocalPosition>(
        "VehicleLocalPosition_PubSubTopic", 10, std::bind(&LoggerCsv::local_pos_cb, this, _1));
}

LoggerCsv::~LoggerCsv() {
    pos_fb_.close();
    vel_fb_.close();
    acc_fb_.close();
}

std::string LoggerCsv::get_date_time() {
    auto to_string = [](const system_clock::time_point& t)->std::string {
        auto as_time_t = system_clock::to_time_t(t);
        struct tm tm;
        #if defined(WIN32) || defined(_WINDLL)
            localtime_s(&tm, &as_time_t); // win api，thread safe
        #else
            localtime_r(&as_time_t, &tm); // linux api，thread safe
        #endif
        milliseconds ms = duration_cast<milliseconds>(t.time_since_epoch());
        // char buf[128];
        // snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03ld",
        // tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms.count() % 1000);
        char buf[64];
        snprintf(buf, sizeof(buf), "%02d%02d%02d_%02d%02d%02d",
        tm.tm_year + 1900 - 2000, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        return buf;
    };
    system_clock::time_point t = system_clock::now();
    return to_string(t);
}

void LoggerCsv::init_file_header() {
    // if(!outfile.is_open ()) {
    //     std::cout << "Open file failure" << std::endl;
    // }
    pos_fb_ << "timestamp" << "," << "timestamp_sample" << "," << "x" << "," << "y" << "," << "z" << std::endl;
    vel_fb_ << "timestamp" << "," << "timestamp_sample" << "," << "vx" << "," << "vy" << "," << "vz" << std::endl;
    acc_fb_ << "timestamp" << "," << "timestamp_sample" << "," << "ax" << "," << "ay" << "," << "az" << std::endl;
}

void LoggerCsv::local_pos_cb(const VehicleLocalPosition::SharedPtr msg) {
    pos_fb_ << msg->timestamp << "," << msg->timestamp_sample << "," << msg->x << "," << msg->y << "," << msg->z << std::endl;
    vel_fb_ << msg->timestamp << "," << msg->timestamp_sample << "," << msg->vx << "," << msg->vy << "," << msg->vz << std::endl;
    acc_fb_ << msg->timestamp << "," << msg->timestamp_sample << "," << msg->ax << "," << msg->ay << "," << msg->az << std::endl;
}


int main(int argc, char const *argv[]) {
    std::cout << "Starting Logger CSV ..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoggerCsv>());
    rclcpp::shutdown();
    return 0;
}

