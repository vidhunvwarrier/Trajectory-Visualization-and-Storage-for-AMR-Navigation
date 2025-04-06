#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_service/srv/trajectory_saver.hpp" //adding custom service header
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdlib>

#include <filesystem>
namespace fs = std::filesystem;

struct PoseData // a struct for saving position data
{
    rclcpp::Time time; // time of the data
    double x;          // position x
    double y;          // position y
};

class TrajectorySaver : public rclcpp::Node
{
public:
    TrajectorySaver() : Node("trajectory_saver")
    {
        using std::placeholders::_1;
        using std::placeholders::_2;

        trajectory_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TrajectorySaver::pose_callback, this, _1));                                         // subscribe odom topic for position of the robot
        trajectory_service_ = this->create_service<custom_service::srv::TrajectorySaver>("/trajectory/saver", std::bind(&TrajectorySaver::trajectory_saver_srv_callback, this, _1, _2)); // service initialization for file name and duration
    }

    // service callback
    void trajectory_saver_srv_callback(
        const std::shared_ptr<custom_service::srv::TrajectorySaver::Request> request,
        std::shared_ptr<custom_service::srv::TrajectorySaver::Response> response)
    {
        std::string filename = request->filename;

        if (filename.empty() || filename == "none")
        { 
            // if filename is not given use path.yaml as filename
            filename = "path.yaml";
        }

        float duration = request->duration;
        std::vector<std::pair<float, float>> selected_xy;

        rclcpp::Time start_time = this->get_clock()->now();

        for (const auto &p : pos_xy_)
        {
            if ((start_time - p.time).seconds() <= duration) // select the position from the time to back of duration
            {
                selected_xy.emplace_back(p.x, p.y);
            }
        }
        file_saver(filename, selected_xy); // saving the data to yaml file
        response->success = true;
    }

private:
    void file_saver(const std::string &filename, std::vector<std::pair<float, float>> &pos_xy)
    {
        YAML::Emitter data_out_;
        const char *home_dir = std::getenv("HOME");
        fs::path folder_path = fs::path(home_dir) / "logged_path";
        fs::path file_path = folder_path / filename; // taking the file path

        if (!fs::exists(folder_path))
        {
            if (!fs::create_directories(folder_path))
            {
                RCLCPP_INFO(this->get_logger(), "Error in creating folder");
                return;
            }
        }
        data_out_ << YAML::BeginMap;
        data_out_ << YAML::Key << "path" << YAML::Value << YAML::BeginSeq;
        for (const auto &[x, y] : pos_xy) // saving data to the file
        {
            std::stringstream position;
            position << x << "," << y;
            data_out_ << position.str();
        }
        data_out_ << YAML::EndSeq;
        data_out_ << YAML::EndMap;

        std::ofstream fout(file_path);
        fout << data_out_.c_str();
        fout.close();
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) // callback for position data
    {
        auto pose = msg->pose.pose.position;
        auto t = this->get_clock()->now();
        pos_xy_.push_back({t, pose.x, pose.y});
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr trajectory_sub_;
    rclcpp::Service<custom_service::srv::TrajectorySaver>::SharedPtr trajectory_service_;
    std::vector<PoseData> pos_xy_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySaver>());
    rclcpp::shutdown();
    return 0;
}
