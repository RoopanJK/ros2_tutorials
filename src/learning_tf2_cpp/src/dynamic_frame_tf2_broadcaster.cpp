#include <chrono>
#include <functional>
#include <memory>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
    public:
        DynamicFrameBroadcaster() : Node("dynamice_frame_tf2_broadcaster")
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_ = this->create_wall_timer(100ms, std::bind(&DynamicFrameBroadcaster::broadcaster_timer_callback, this));
        }
    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

        void broadcaster_timer_callback()
        {
            rclcpp::Time now = this->get_clock()->now();
            double x = now.seconds() * PI;

            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.frame_id = "turtle1";
            t.child_frame_id = "carrot1";
            t.transform.translation.x = 10 * sin(x);
            t.transform.translation.y = 10 * cos(x);
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t);
        }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
}