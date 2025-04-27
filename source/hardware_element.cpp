#include "hardware_element.hpp"
#include "notification_manager.hpp"

namespace cleaning_hardware {
using system_notifications::NotificationManager;
using system_notifications::MessageLevel;

HardwareElement::HardwareElement(const std::string& element_name)
    : name_(element_name), nh_("~") {
    // Setup publishers
    activation_pub_ = nh_.advertise<std_msgs::String>(name_ + "/activate", 10);
    deactivation_pub_ = nh_.advertise<std_msgs::String>(name_ + "/deactivate", 10);

    // Setup subscribers
    activation_sub_ = nh_.subscribe(name_ + "/activation_command", 10, &HardwareElement::activationCallback, this);
    deactivation_sub_ = nh_.subscribe(name_ + "/deactivation_command", 10, &HardwareElement::deactivationCallback, this);
}

void HardwareElement::sendNotification(const std::string& message, MessageLevel priority) {
    NotificationManager::getInstance().addNotification(name_ + ": " + message, priority);
}

void HardwareElement::activationCallback(const std_msgs::String::ConstPtr& msg) {
    try {
        ROS_INFO_STREAM(name_ << " received activation command: " << msg->data);
        sendNotification("Activation command received: " + msg->data, system_notifications::MessageLevel::INFO);
        start();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM(name_ << " activationCallback exception: " << e.what());
        sendNotification("Exception in activationCallback: " + std::string(e.what()), system_notifications::MessageLevel::ERROR);
    }
}

void HardwareElement::deactivationCallback(const std_msgs::String::ConstPtr& msg) {
    try {
        ROS_INFO_STREAM(name_ << " received deactivation command: " << msg->data);
        sendNotification("Deactivation command received: " + msg->data, system_notifications::MessageLevel::INFO);
        stop();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM(name_ << " deactivationCallback exception: " << e.what());
        sendNotification("Exception in deactivationCallback: " + std::string(e.what()), system_notifications::MessageLevel::ERROR);
    }
}

} // namespace cleaning_hardware