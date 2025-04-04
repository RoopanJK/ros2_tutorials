#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>

#include <chrono>
#include <memory>

#include <std_msgs/msg/string.hpp>

class MinimalSubscriberWithTopicStatistics : public rclcpp::Node
{
    public:
        MinimalSubscriberWithTopicStatistics() : Node("minimal_subscriber_with_topic_statistics")
        {
            auto options = rclcpp::SubscriptionOptions();
            options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

            options.topic_stats_options.publish_period = std::chrono::seconds(10);

            auto callback = [this](std_msgs::msg::String::SharedPtr msg)
            {
                this->topic_callback(msg);
            };

            subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, callback, options);
        }
    
    private:
        void topic_callback(const std_msgs::msg::String::ConstSharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
    rclcpp::shutdown();
    return 0;
}