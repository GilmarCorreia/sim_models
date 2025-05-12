#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rclcpp/qos.hpp>

class ClockWaiter : public rclcpp::Node
{
public:
    ClockWaiter()
        : Node("clock_waiter"), flag_(true)
    {
        // Define QoS profile: best effort and volatile durability
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                               .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                               .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                               .keep_last(1); // Queue depth of 1

        // Create the subscription to the /clock topic
        subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", qos_profile,
            std::bind(&ClockWaiter::clock_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for /clock topic...");
    }

private:
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        float total_time = (float) msg->clock.sec + ((float) msg->clock.nanosec / 1e9);

        // Stop spinning after receiving the first message
        if(total_time > 0.0){
            RCLCPP_INFO(this->get_logger(), "/clock topic detected! Initializing the subsequent nodes...");
            flag_ = false;
        }
    }

public:
    bool flag_;

private:
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClockWaiter>();

    // Use a loop to spin once until a message is received
    while (rclcpp::ok() && node->flag_)
    {
        rclcpp::spin_some(node);
    }

    //node->destroy_node();
    rclcpp::shutdown();
    return 0;
}
