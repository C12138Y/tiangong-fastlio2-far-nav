#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class GlobalMapBuilder : public rclcpp::Node
{
public:
    GlobalMapBuilder()
        : Node("global_map_builder")
    {
        cloud_topic_ = declare_parameter<std::string>(
            "cloud_topic", "/fastlio2/world_cloud");
        odom_topic_ = declare_parameter<std::string>(
            "odom_topic", "/my_nav/odom");
        map_topic_ = declare_parameter<std::string>(
            "map_topic", "/my_nav/global_map");
        map_frame_ = declare_parameter<std::string>(
            "map_frame", "fastlio_odom");

        resolution_ = declare_parameter<double>("resolution", 0.10);
        min_range_ = declare_parameter<double>("min_range", 0.30);
        max_range_ = declare_parameter<double>("max_range", 15.0);
        min_obstacle_height_ =
            declare_parameter<double>("min_obstacle_height", 0.08);
        max_obstacle_height_ =
            declare_parameter<double>("max_obstacle_height", 1.80);
        map_margin_ = declare_parameter<double>("map_margin", 1.0);

        scan_stride_ = static_cast<int>(std::max<int64_t>(
            1, declare_parameter<int>("scan_stride", 2)));

        point_stride_ = static_cast<int>(std::max<int64_t>(
            1, declare_parameter<int>("point_stride", 2)));

        hit_increment_ = declare_parameter<int>("hit_increment", 4);
        miss_decrement_ = declare_parameter<int>("miss_decrement", 1);
        occupied_threshold_ =
            declare_parameter<int>("occupied_threshold", 2);

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            rclcpp::QoS(1).reliable().transient_local());

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                latest_odom_ = msg;
            });

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &GlobalMapBuilder::cloudCallback,
                this,
                std::placeholders::_1));

        publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GlobalMapBuilder::publishMap, this));

        RCLCPP_INFO(
            get_logger(),
            "Global map builder: cloud=%s odom=%s output=%s",
            cloud_topic_.c_str(),
            odom_topic_.c_str(),
            map_topic_.c_str());
    }

private:
    struct Cell
    {
        int x;
        int y;

        bool operator==(const Cell &other) const
        {
            return x == other.x && y == other.y;
        }
    };

    struct CellHash
    {
        std::size_t operator()(const Cell &cell) const
        {
            const auto ux = static_cast<uint32_t>(cell.x);
            const auto uy = static_cast<uint32_t>(cell.y);
            return (static_cast<std::size_t>(ux) << 32) ^
                   static_cast<std::size_t>(uy);
        }
    };

    Cell worldToCell(double x, double y) const
    {
        return Cell{
            static_cast<int>(std::floor(x / resolution_)),
            static_cast<int>(std::floor(y / resolution_))};
    }

    void updateBounds(const Cell &cell)
    {
        if (!have_bounds_)
        {
            min_x_ = max_x_ = cell.x;
            min_y_ = max_y_ = cell.y;
            have_bounds_ = true;
            return;
        }

        min_x_ = std::min(min_x_, cell.x);
        max_x_ = std::max(max_x_, cell.x);
        min_y_ = std::min(min_y_, cell.y);
        max_y_ = std::max(max_y_, cell.y);
    }

    void updateCell(const Cell &cell, int delta)
    {
        auto &score = cells_[cell];
        score = static_cast<int16_t>(
            std::clamp(
                static_cast<int>(score) + delta,
                -10,
                20));

        updateBounds(cell);
    }

    void traceRay(
        const Cell &start,
        const Cell &end,
        bool endpoint_is_obstacle)
    {
        int x = start.x;
        int y = start.y;

        const int dx = std::abs(end.x - start.x);
        const int sx = start.x < end.x ? 1 : -1;
        const int dy = -std::abs(end.y - start.y);
        const int sy = start.y < end.y ? 1 : -1;

        int error = dx + dy;

        while (!(x == end.x && y == end.y))
        {
            updateCell(Cell{x, y}, -miss_decrement_);

            const int twice_error = 2 * error;

            if (twice_error >= dy)
            {
                error += dy;
                x += sx;
            }

            if (twice_error <= dx)
            {
                error += dx;
                y += sy;
            }
        }

        if (endpoint_is_obstacle)
        {
            updateCell(end, hit_increment_);
        }
        else
        {
            updateCell(end, -miss_decrement_);
        }
    }

    double pointHeightInBase(
        double world_x,
        double world_y,
        double world_z,
        const nav_msgs::msg::Odometry &odom) const
    {
        const auto &position = odom.pose.pose.position;
        const auto &orientation = odom.pose.pose.orientation;

        double qx = orientation.x;
        double qy = orientation.y;
        double qz = orientation.z;
        double qw = orientation.w;

        const double norm =
            std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

        if (norm < 1e-9)
        {
            return world_z - position.z;
        }

        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

        const double dx = world_x - position.x;
        const double dy = world_y - position.y;
        const double dz = world_z - position.z;

        // p_base = R_world_base^T * (p_world - t_world_base)
        return 2.0 * (qx * qz + qw * qy) * dx +
               2.0 * (qy * qz - qw * qx) * dy +
               (1.0 - 2.0 * (qx * qx + qy * qy)) * dz;
    }

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        ++received_scans_;

        if ((received_scans_ - 1) % scan_stride_ != 0)
        {
            return;
        }

        if (!latest_odom_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Waiting for /my_nav/odom");
            return;
        }

        const rclcpp::Time cloud_stamp(msg->header.stamp);
        const rclcpp::Time odom_stamp(latest_odom_->header.stamp);
        const double stamp_difference =
            std::abs((cloud_stamp - odom_stamp).seconds());

        if (stamp_difference > 0.25)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Cloud and odometry differ by %.3f seconds",
                stamp_difference);
            return;
        }

        const auto &robot_position =
            latest_odom_->pose.pose.position;

        const Cell ray_start =
            worldToCell(robot_position.x, robot_position.y);

        sensor_msgs::PointCloud2ConstIterator<float>
            iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float>
            iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float>
            iter_z(*msg, "z");

        std::size_t point_index = 0;
        std::size_t accepted_points = 0;

        for (
            ;
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z, ++point_index)
        {
            if (point_index % point_stride_ != 0)
            {
                continue;
            }

            const double world_x = *iter_x;
            const double world_y = *iter_y;
            const double world_z = *iter_z;

            if (
                !std::isfinite(world_x) ||
                !std::isfinite(world_y) ||
                !std::isfinite(world_z))
            {
                continue;
            }

            const double dx = world_x - robot_position.x;
            const double dy = world_y - robot_position.y;
            const double planar_range = std::hypot(dx, dy);

            if (
                planar_range < min_range_ ||
                planar_range > max_range_)
            {
                continue;
            }

            const double point_height = pointHeightInBase(
                world_x,
                world_y,
                world_z,
                *latest_odom_);

            const bool endpoint_is_obstacle =
                point_height >= min_obstacle_height_ &&
                point_height <= max_obstacle_height_;

            if (!endpoint_is_obstacle)
            {
                continue;
            }

            traceRay(
                ray_start,
                worldToCell(world_x, world_y),
                true);

            ++accepted_points;
        }

        ++processed_scans_;

        if (processed_scans_ % 50 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "Processed %zu scans, last scan accepted %zu points, cells=%zu",
                processed_scans_,
                accepted_points,
                cells_.size());
        }
    }

    void publishMap()
    {
        if (!have_bounds_ || cells_.empty())
        {
            return;
        }

        const int margin_cells =
            static_cast<int>(std::ceil(map_margin_ / resolution_));

        const int output_min_x = min_x_ - margin_cells;
        const int output_max_x = max_x_ + margin_cells;
        const int output_min_y = min_y_ - margin_cells;
        const int output_max_y = max_y_ + margin_cells;

        const int width = output_max_x - output_min_x + 1;
        const int height = output_max_y - output_min_y + 1;

        if (width <= 0 || height <= 0)
        {
            return;
        }

        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = now();
        map.header.frame_id = map_frame_;
        map.info.map_load_time = now();
        map.info.resolution = static_cast<float>(resolution_);
        map.info.width = static_cast<uint32_t>(width);
        map.info.height = static_cast<uint32_t>(height);
        map.info.origin.position.x = output_min_x * resolution_;
        map.info.origin.position.y = output_min_y * resolution_;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.data.assign(
            static_cast<std::size_t>(width) *
                static_cast<std::size_t>(height),
            -1);

        for (const auto &entry : cells_)
        {
            const Cell &cell = entry.first;
            const int score = entry.second;

            const int local_x = cell.x - output_min_x;
            const int local_y = cell.y - output_min_y;

            if (
                local_x < 0 || local_x >= width ||
                local_y < 0 || local_y >= height)
            {
                continue;
            }

            const std::size_t index =
                static_cast<std::size_t>(local_y) *
                    static_cast<std::size_t>(width) +
                static_cast<std::size_t>(local_x);

            map.data[index] =
                score >= occupied_threshold_ ? 100 : 0;
        }

        map_pub_->publish(map);
    }

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string map_frame_;

    double resolution_;
    double min_range_;
    double max_range_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double map_margin_;

    int scan_stride_;
    int point_stride_;
    int hit_increment_;
    int miss_decrement_;
    int occupied_threshold_;

    std::size_t received_scans_{0};
    std::size_t processed_scans_{0};

    bool have_bounds_{false};
    int min_x_{0};
    int max_x_{0};
    int min_y_{0};
    int max_y_{0};

    std::unordered_map<Cell, int16_t, CellHash> cells_;

    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapBuilder>());
    rclcpp::shutdown();
    return 0;
}