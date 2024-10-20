#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <robot_localization/srv/set_pose.hpp>

class PoseInitializer : public rclcpp::Node
{
public:
    PoseInitializer() : Node("pose_initializer")
    {
        // Create a service client for SetPose
        client_ = this->create_client<robot_localization::srv::SetPose>("/set_pose");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /set_pose service...");
        }

        // Subscriber to 'initialpose'
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&PoseInitializer::callback, this, std::placeholders::_1));
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose = *msg;
        RCLCPP_INFO(this->get_logger(), "\n \n Position: [x: %f, y: %f, z: %f]",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);

        // Call the service asynchronously
        auto result_future = client_->async_send_request(request);
        //result_future.wait(); // Optionally block until the result is received
        if (result_future.valid()) {
            RCLCPP_INFO(this->get_logger(), "Successfully called set_pose service");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
