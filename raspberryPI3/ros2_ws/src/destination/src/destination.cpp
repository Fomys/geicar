
#include "../include/destination/destination.h"

using namespace std;
using placeholders::_1;
class destination : public rclcpp::Node {


public:
    destination():Node("destination_node") {
        //Publishers
        // sur le topic /goal_pose
        publisher_dest_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        //Subscriptions
        // Consigne de destination
        subscription_dest_cmd = this->create_subscription<interfaces::msg::DestCmd>(
                "dest_cmd", 10, std::bind(&destination::destCmdCallback, this, _1));

        //Inform the log the node has been launched
        RCLCPP_INFO(this->get_logger(), "destination_node READY");
    }

private:

    /************ VARIABLE DECLARATIONS *******************/

    //**ROS TOPIC **
    //Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_dest_;

    //Subscriptions
    rclcpp::Subscription<interfaces::msg::DestCmd>::SharedPtr subscription_dest_cmd;


    /************ FUNCTION DECLARATIONS *******************/

    //** CALLBACK FUNCTIONS TO UPDATE INTERNAL VARIABLES **


    /*
     * Callback to update the destination coming from the web app
     */
    void destCmdCallback(const interfaces::msg::DestCmd &dest_cmd) {
        auto goal_pose = geometry_msgs::msg::PoseStamped();
        rclcpp::Time time = this->get_clock()->now();
        goal_pose.header.stamp = time;
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = dest_cmd.x;
        goal_pose.pose.position.y = dest_cmd.y;
        goal_pose.pose.orientation.z = dest_cmd.z_orien;
        goal_pose.pose.orientation.w = dest_cmd.w_orien;

        //Publication
        publisher_dest_->publish(goal_pose);

        RCLCPP_INFO(this->get_logger(), "La position demandee est x = %f, y = %f , z_orien = %f, w_orien = %f",
                    goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.orientation.z,
                    goal_pose.pose.orientation.w);
    }

};

//Running node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<destination>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
