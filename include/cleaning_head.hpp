#include "hardware_element.hpp"

namespace cleaning_hardware {

class CleaningHead : public HardwareElement {
public:
    CleaningHead();

    void initialize() override;
    void start() override;
    void stop() override;

private:
    // Internal state variables
    bool brush_motor_active_;
    bool water_pump_active_;

    // Helper function
    void checkSystems();
};

} // namespace cleaning_hardware
