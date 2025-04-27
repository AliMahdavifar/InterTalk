#pragma once

#include "hardware_element.hpp"

namespace cleaning_hardware {

class DisinfectionModule : public HardwareElement {
public:
    DisinfectionModule();

    void initialize() override;
    void start() override;
    void stop() override;

private:
    bool sprayer_active_;

    void checkSprayer();
};

} // namespace cleaning_hardware
