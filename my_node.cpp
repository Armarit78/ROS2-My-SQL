#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mysql/mysql.h>
#include <iostream>

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        // Initialisation de la connexion à la base de données
        connect_to_database();

        // Abonnement au topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/test", 10, std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));
    }

    ~SubscriberNode()
    {
        if (conn_) {
            mysql_close(conn_);
        }
    }

private:
    void connect_to_database()
    {
        conn_ = mysql_init(NULL);
        if (!mysql_real_connect(conn_, "localhost", "ros2", "ros2", "rosdb", 0, NULL, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to database: %s", mysql_error(conn_));
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to the database.");
        }
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        if (conn_) {
            std::string query = "INSERT INTO messages (content) VALUES('" + msg->data + "')";
            if (mysql_query(conn_, query.c_str())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to insert data: %s", mysql_error(conn_));
            } else {
                RCLCPP_INFO(this->get_logger(), "Data inserted successfully.");
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    MYSQL* conn_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

