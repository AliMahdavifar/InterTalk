#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <boost/circular_buffer.hpp>

namespace system_notifications {

enum class MessageLevel {
    INFO,
    WARNING,
    ERROR
};

struct Notification {
    std::string timestamp;
    std::string message;
    
    MessageLevel priority;
};

class NotificationManager {   
public:

    static NotificationManager& getInstance();

    void addNotification(const std::string& message, MessageLevel priority = MessageLevel::INFO);
    void printNotifications();

private:    
    NotificationManager() = default;
    ~NotificationManager() = default;

    NotificationManager(const NotificationManager&) = delete;
    NotificationManager& operator=(const NotificationManager&) = delete;

    boost::circular_buffer<Notification> notification_buffer_;
    std::mutex mutex_;
    
    static constexpr std::size_t MAX_NOTIFICATIONS = 10;
};

} // namespace system_notifications
