#include "notification_manager.hpp" 

namespace system_notifications {

NotificationManager::NotificationManager()
    : notification_buffer_(MAX_NOTIFICATIONS) {}

NotificationManager& NotificationManager::getInstance() {
    static NotificationManager instance;
    return instance;
}

void NotificationManager::addNotification(const std::string& message, MessageLevel priority) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    Notification notif{oss.str(), message, priority};

    std::lock_guard<std::mutex> lock(mutex_);
    notification_buffer_.push_back(std::move(notif));
}

void NotificationManager::printNotifications() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& notif : notification_buffer_) {
        std::string priority_str;
        switch (notif.priority) {
            case MessageLevel::INFO: 
                priority_str = "[INFO]";
                break;
            case MessageLevel::WARNING: 
                priority_str = "[WARNING]";
                break;
            case MessageLevel::ERROR:
                priority_str = "[ERROR]";
                break;
            default:
                priority_str = "[UNDEFINED]";
        }

        std::cout << "(" << notif.timestamp << "): "
                  << priority_str << " " << notif.message 
                  << std::endl;
    }
}

} // namespace system_notifications
