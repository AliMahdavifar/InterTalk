#include "disinfection_module.hpp"

namespace cleaning_hardware {

DisinfectionModule::DisinfectionModule()
    : HardwareElement("disinfection_module"),
      sprayer_active_(false) {
    sendNotification("Disinfection Module created.");
}

void DisinfectionModule::initialize() {
    sprayer_active_ = false;

    sendNotification("Disinfection Module initialized: sprayer system ready.");
}

void DisinfectionModule::start() {
    sprayer_active_ = true;

    sendNotification("Disinfection spray activated.");
}

void DisinfectionModule::stop() {
    sprayer_active_ = false;

    sendNotification("Disinfection spray deactivated.");
}

void DisinfectionModule::checkSprayer() {
    if (!sprayer_active_) {
        sendNotification("Warning: Disinfection sprayer inactive during operation!");
    }
}

} // namespace cleaning_hardware
