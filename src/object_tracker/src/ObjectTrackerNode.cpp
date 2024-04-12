#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "dl_perception_interfaces/msg/object_instance_array.hpp"
#include "dd2419_interfaces/msg/object_list.hpp"
#include "dd2419_interfaces/msg/object_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class ObjectTrackerNode : public rclcpp::Node
{
    public:
    ObjectTrackerNode() : Node("object_tracker_node")
    {
        heart_beat_pub = this->create_publisher<std_msgs::msg::String>("/object_tracker_heartbeat", 10);
        object_poses_pub = this->create_publisher<dd2419_interfaces::msg::ObjectPoses>("/object_list_real", 10);

        object_instance_sub = this->create_subscription<dl_perception_interfaces::msg::ObjectInstanceArray>(\
                "/detection/object_instances", 10, std::bind(&ObjectTrackerNode::object_instance_cb, this, _1));

        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ObjectTrackerNode::timer_callback, this));
    }

    private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Object tracker heart beat: " + std::to_string(counter++);
      heart_beat_pub->publish(message);
    }

    void object_instance_cb(const dl_perception_interfaces::msg::ObjectInstanceArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received %d object instances", msg->instances.size());

        auto object_poses_msg = dd2419_interfaces::msg::ObjectPoses();

        for (auto instance : msg->instances)
        {
            auto object_pose = geometry_msgs::msg::PoseStamped();
            object_pose.header.stamp = msg->header.stamp;
            object_pose.header.frame_id = "map";
            object_pose.pose.position.x = instance.object_position.x;
            object_pose.pose.position.y = instance.object_position.y;
            object_pose.pose.position.z = 0;

            auto object_name = std::string(instance.category_name) + "_" + std::string(instance.instance_name);

            object_poses_msg.poses.push_back(object_pose);
            object_poses_msg.object_list.object_list.push_back(object_name);
        }

        object_poses_pub->publish(object_poses_msg);
    }
    
    int counter = 0;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heart_beat_pub;
    rclcpp::Publisher<dd2419_interfaces::msg::ObjectPoses>::SharedPtr object_poses_pub;
    rclcpp::Subscription<dl_perception_interfaces::msg::ObjectInstanceArray>::SharedPtr object_instance_sub;
};

int main(int ac, char ** av)
{
    rclcpp::init(ac, av);
    rclcpp::spin(std::make_shared<ObjectTrackerNode>());
    rclcpp::shutdown();
    return 0;
}