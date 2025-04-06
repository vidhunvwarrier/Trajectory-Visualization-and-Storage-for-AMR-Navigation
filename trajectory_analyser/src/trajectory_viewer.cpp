#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace fs = std::filesystem;

class TrajectoryViewer : public rclcpp::Node
{
public:
    TrajectoryViewer() : Node("trajectory_viewer_node")
    {
        this->declare_parameter<std::string>("filename", "");                 // parameter for input filename
        this->declare_parameter<std::string>("marker_topic", "/marker/data"); // parameter for topic to publish marker data

        std::string filename;
        std::string marker_topic;

        this->get_parameter("filename", filename);
        this->get_parameter("marker_topic", marker_topic);
        if (filename.empty()) // check whether the file is empty or not
        {
            RCLCPP_ERROR(this->get_logger(), "No filename speciffied");
            return;
        }

        marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10); // publisher for publishing marker data

        read_file(filename); // read the file

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TrajectoryViewer::publish_data, this)); // timer for publishing marker data
    }

private:
    void read_file(const std::string &filename)
    {
        const char *home_dir = std::getenv("HOME"); // open home directory

        fs::path folder_path = fs::path(home_dir) / "logged_path"; // find the directory where the folder located
        fs::path file_path = folder_path / filename;

        if (!fs::exists(file_path)) // check the file exists or not
        {
            RCLCPP_INFO(this->get_logger(), "File not exists");
            return;
        }

        YAML::Node config = YAML::LoadFile(file_path.string()); // load the yaml file for taking the string

        for (const auto &poses : config["path"])
        {
            std::string pose_str = poses.as<std::string>();
            std::stringstream data(pose_str);
            float x, y;
            char comma;
            data >> x >> comma >> y;
            geometry_msgs::msg::Point pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.0;
            points_.push_back(pt); // converting the data to points
        }
    }
    void publish_data()
    {

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line_strip;

        line_strip.header.frame_id = "odom";
        line_strip.header.stamp = this->get_clock()->now();
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.05;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.points = points_;
        line_strip.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(line_strip);
        marker_publisher->publish(marker_array); // publish the marker data
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
    std::vector<geometry_msgs::msg::Point> points_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryViewer>());
    rclcpp::shutdown();
    return 0;
}
