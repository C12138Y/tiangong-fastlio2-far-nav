#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>

class FastlioNavAdapter : public rclcpp::Node
{
public:
    FastlioNavAdapter()
        : Node("fastlio_nav_adapter")
    {
        input_odom_topic_ =
            declare_parameter<std::string>("input_odom_topic", "/fastlio2/lio_odom");
        output_odom_topic_ =
            declare_parameter<std::string>("output_odom_topic", "/my_nav/odom");

        odom_frame_ =
            declare_parameter<std::string>("odom_frame", "fastlio_odom");
        imu_frame_ =
            declare_parameter<std::string>("imu_frame", "fastlio_body");
        base_frame_ =
            declare_parameter<std::string>("base_frame", "my_nav_base_link");

        const auto translation = declare_parameter<std::vector<double>>(
            "imu_to_base_translation",
            {0.009, -0.02329, 0.65732});

        const auto rotation = declare_parameter<std::vector<double>>(
            "imu_to_base_rotation",
            {0.0, 1.0, 0.0,
             1.0, 0.0, 0.0,
             0.0, 0.0, -1.0});

        if (translation.size() != 3 || rotation.size() != 9)
        {
            throw std::runtime_error(
                "imu_to_base_translation must have 3 values and "
                "imu_to_base_rotation must have 9 values");
        }

        t_imu_base_.setValue(
            translation[0], translation[1], translation[2]);

        tf2::Matrix3x3 rotation_matrix(
            rotation[0], rotation[1], rotation[2],
            rotation[3], rotation[4], rotation[5],
            rotation[6], rotation[7], rotation[8]);

        rotation_matrix.getRotation(q_imu_base_);
        q_imu_base_.normalize();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            output_odom_topic_, rclcpp::QoS(20).reliable());

        static_tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publish_static_extrinsic();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            rclcpp::QoS(20).reliable(),
            std::bind(
                &FastlioNavAdapter::odom_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "FAST-LIO adapter: %s -> %s, frames %s -> %s -> %s",
            input_odom_topic_.c_str(),
            output_odom_topic_.c_str(),
            odom_frame_.c_str(),
            imu_frame_.c_str(),
            base_frame_.c_str());
    }

private:
    void publish_static_extrinsic()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = imu_frame_;
        transform.child_frame_id = base_frame_;

        transform.transform.translation.x = t_imu_base_.x();
        transform.transform.translation.y = t_imu_base_.y();
        transform.transform.translation.z = t_imu_base_.z();

        transform.transform.rotation.x = q_imu_base_.x();
        transform.transform.rotation.y = q_imu_base_.y();
        transform.transform.rotation.z = q_imu_base_.z();
        transform.transform.rotation.w = q_imu_base_.w();

        static_tf_broadcaster_->sendTransform(transform);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q_world_imu(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        q_world_imu.normalize();

        const tf2::Vector3 p_world_imu(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        const tf2::Transform world_imu(q_world_imu, p_world_imu);
        const tf2::Transform imu_base(q_imu_base_, t_imu_base_);
        const tf2::Transform world_base = world_imu * imu_base;

        const tf2::Vector3 p_world_base = world_base.getOrigin();
        tf2::Quaternion q_world_base = world_base.getRotation();
        q_world_base.normalize();

        nav_msgs::msg::Odometry output;
        output.header.stamp = msg->header.stamp;
        output.header.frame_id = odom_frame_;
        output.child_frame_id = base_frame_;

        output.pose.pose.position.x = p_world_base.x();
        output.pose.pose.position.y = p_world_base.y();
        output.pose.pose.position.z = p_world_base.z();

        output.pose.pose.orientation.x = q_world_base.x();
        output.pose.pose.orientation.y = q_world_base.y();
        output.pose.pose.orientation.z = q_world_base.z();
        output.pose.pose.orientation.w = q_world_base.w();

        const bool input_has_pose_covariance = std::any_of(
            msg->pose.covariance.begin(),
            msg->pose.covariance.end(),
            [](double value)
            { return value != 0.0; });

        if (input_has_pose_covariance)
        {
            output.pose.covariance = msg->pose.covariance;
        }
        else
        {
            output.pose.covariance.fill(0.0);
            output.pose.covariance[0] = 0.0004;
            output.pose.covariance[7] = 0.0004;
            output.pose.covariance[14] = 0.0016;
            output.pose.covariance[21] = 0.0004;
            output.pose.covariance[28] = 0.0004;
            output.pose.covariance[35] = 0.0012;
        }

        const rclcpp::Time current_stamp(msg->header.stamp, RCL_ROS_TIME);

        if (have_previous_pose_)
        {
            const double dt = (current_stamp - previous_stamp_).seconds();

            if (dt > 0.001 && dt < 1.0)
            {
                const tf2::Vector3 velocity_world =
                    (p_world_base - previous_position_) / dt;

                const tf2::Vector3 velocity_base =
                    tf2::quatRotate(q_world_base.inverse(), velocity_world);

                output.twist.twist.linear.x = velocity_base.x();
                output.twist.twist.linear.y = velocity_base.y();
                output.twist.twist.linear.z = velocity_base.z();

                double previous_roll;
                double previous_pitch;
                double previous_yaw;
                tf2::Matrix3x3(previous_orientation_).getRPY(previous_roll, previous_pitch, previous_yaw);

                double current_roll;
                double current_pitch;
                double current_yaw;
                tf2::Matrix3x3(q_world_base).getRPY(current_roll, current_pitch, current_yaw);

                constexpr double two_pi = 6.28318530717958647692;
                const double yaw_delta =
                    std::remainder(current_yaw - previous_yaw, two_pi);

                output.twist.twist.angular.z = yaw_delta / dt;
            }
        }

        output.twist.covariance.fill(0.0);
        output.twist.covariance[0] = 0.0025;
        output.twist.covariance[7] = 0.0025;
        output.twist.covariance[14] = 0.01;
        output.twist.covariance[21] = 0.01;
        output.twist.covariance[28] = 0.01;
        output.twist.covariance[35] = 0.0025;

        previous_position_ = p_world_base;
        previous_orientation_ = q_world_base;
        previous_stamp_ = current_stamp;
        have_previous_pose_ = true;

        odom_pub_->publish(output);
    }

    std::string input_odom_topic_;
    std::string output_odom_topic_;
    std::string odom_frame_;
    std::string imu_frame_;
    std::string base_frame_;

    tf2::Vector3 t_imu_base_;
    tf2::Quaternion q_imu_base_;

    bool have_previous_pose_{false};
    tf2::Vector3 previous_position_;
    tf2::Quaternion previous_orientation_;
    rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
        static_tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastlioNavAdapter>());
    rclcpp::shutdown();
    return 0;
}