#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "notification_manager.hpp"

namespace cleaning_hardware {

// Base class
class HardwareElement {
public:
    explicit HardwareElement(const std::string& element_name);
    virtual ~HardwareElement() = default;

    HardwareElement(const HardwareElement&) = delete;
    HardwareElement& operator=(const HardwareElement&) = delete;

    virtual void initialize() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

protected:
    void sendNotification(const std::string& message, 
                          system_notifications::MessageLevel priority = 
                          system_notifications::MessageLevel::INFO);

    ros::NodeHandle nh_;
    std::string name_;
    
    // Publishers
    ros::Publisher activation_pub_;
    ros::Publisher deactivation_pub_;

    // Subscribers
    ros::Subscriber activation_sub_;
    ros::Subscriber deactivation_sub_;

private:
    // ROS Callbacks to handle incoming activation/deactivation commands
    void activationCallback(const std_msgs::String::ConstPtr& msg);
    void deactivationCallback(const std_msgs::String::ConstPtr& msg);
};

} // namespace cleaning_hardware