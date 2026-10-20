#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class PointCloudAdapter : public rclcpp::Node
{
public:
    PointCloudAdapter() : Node("fastlio2_pointcloud_adapter")
    {
        pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
            "/fastlio2/livox_custom", rclcpp::QoS(10).reliable());

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", rclcpp::QoS(10).reliable(),
            std::bind(&PointCloudAdapter::callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "Converting /livox/lidar -> /fastlio2/livox_custom");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        using livox_ros_driver2::msg::CustomMsg;
        using livox_ros_driver2::msg::CustomPoint;
        using sensor_msgs::PointCloud2ConstIterator;

        const uint64_t header_ns =
            static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
            msg->header.stamp.nanosec;

        CustomMsg out;
        out.header = msg->header;
        out.timebase = header_ns;
        out.lidar_id = 0;
        out.points.reserve(static_cast<size_t>(msg->width) * msg->height);

        PointCloud2ConstIterator<float> x(*msg, "x");
        PointCloud2ConstIterator<float> y(*msg, "y");
        PointCloud2ConstIterator<float> z(*msg, "z");
        PointCloud2ConstIterator<float> intensity(*msg, "intensity");
        PointCloud2ConstIterator<double> timestamp(*msg, "timestamp");
        PointCloud2ConstIterator<uint8_t> line(*msg, "line");
        PointCloud2ConstIterator<uint8_t> tag(*msg, "tag");

        for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++timestamp, ++line, ++tag)
        {
            if (!std::isfinite(*x) || !std::isfinite(*y) ||
                !std::isfinite(*z) || !std::isfinite(*timestamp) ||
                !std::isfinite(*intensity))
            {
                continue;
            }

            // 天工 timestamp 是绝对纳秒；FAST-LIO2 要求相对帧头的纳秒偏移。
            double offset_ns = *timestamp - static_cast<double>(header_ns);
            if (offset_ns < -5000.0 || offset_ns > 200000000.0)
            {
                continue;
            }
            if (offset_ns < 0.0)
            {
                offset_ns = 0.0; // 处理浮点精度导致的极小负偏移
            }

            CustomPoint p;
            p.offset_time = static_cast<uint32_t>(std::llround(offset_ns));
            p.x = *x;
            p.y = *y;
            p.z = *z;
            p.reflectivity = static_cast<uint8_t>(
                std::lround(std::clamp(*intensity, 0.0f, 255.0f)));
            p.line = *line;
            p.tag = *tag;
            out.points.push_back(p);
        }

        out.point_num = static_cast<uint32_t>(out.points.size());
        pub_->publish(out);
    }

    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAdapter>());
    rclcpp::shutdown();
    return 0;
}