#include "cleaning_head.hpp"

namespace cleaning_hardware {

CleaningHead::CleaningHead()
    : HardwareElement("cleaning_head"),
      brush_motor_active_(false),
      water_pump_active_(false) {
    sendNotification("Cleaning Head created.");
}

void CleaningHead::initialize() {
    // Pretend to initialize motors, sensors
    brush_motor_active_ = false;
    water_pump_active_ = false;

    sendNotification("Cleaning Head initialized: motors and pumps ready.");
}

void CleaningHead::start() {
    brush_motor_active_ = true;
    water_pump_active_ = true;

    sendNotification("Cleaning Head started: brushes spinning and water spraying.");
}

void CleaningHead::stop() {
    brush_motor_active_ = false;
    water_pump_active_ = false;

    sendNotification("Cleaning Head stopped: brushes and water pump shut down.");
}

void CleaningHead::checkSystems() {
    if (!brush_motor_active_ || !water_pump_active_) {
        sendNotification("Warning: One or more cleaning subsystems inactive!");
    }
}

} // namespace cleaning_hardware
