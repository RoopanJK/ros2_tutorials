#include<memory>

#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{
    public:
        SampleNodeWithParameters() : Node("node_with_parameters")
        {
            this->declare_parameter("an_int_param", 0);

            param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

            auto cb = [this](const rclcpp::Parameter &p)
            {
                RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
                p.get_name().c_str(), 
                p.get_type_name().c_str(),
                p.as_int());
            };

            auto cb2 = [this](const rclcpp::Parameter &p)
            {
                RCLCPP_INFO(this->get_logger(), "cb2: Received an update to parameter \"%s\" of type %s: \"%.02lf\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                p.as_double());
            };

            auto remote_node_name = std::string("parameter_blackboard");
            auto remote_param_name = std::string("a_double_param");

            cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
            cb_handle2_ = param_subscriber_->add_parameter_callback("a_double_param", cb2);
        }

    private:
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
    rclcpp::shutdown();

    return 0;
}