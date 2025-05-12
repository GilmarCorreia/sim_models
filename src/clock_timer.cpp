#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rclcpp/qos.hpp>

class ClockWaiter : public rclcpp::Node
{
public:
    ClockWaiter()
        : Node("clock_waiter"), flag_(true)
    {
        // Define parameters
        this->declare_parameter("sim_time", 0);

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

        // Getting sim_time parameter
        this->sim_time = this->get_parameter("sim_time").as_int();

        RCLCPP_INFO(this->get_logger(), ("Waiting simulation for " + std::to_string(this->sim_time) + " seconds").c_str());
    }

private:
    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        int32_t total_time = msg->clock.sec;

        if (total_time > this->sim_time) {
            // Stop spinning after receiving the first message
            flag_ = false;
        }        
    }

public:
    bool flag_;
    int32_t sim_time;

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
