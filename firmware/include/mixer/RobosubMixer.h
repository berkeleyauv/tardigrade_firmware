#pragma once
//
// RobosubMixer — 8-thruster mix for the robosub.
//
// The mix matrix is transcribed from tardigrade_ws
// src/tardigrade_esp/config/esp_thruster_map.json, in the SAME thruster order
// as the robosub's kMotorPins ({21,19,27,18,5,14,12,26} = slots 1..8). That
// JSON is the source of truth for the geometry; if a thruster is remounted,
// update it there and mirror the change here.
//
// Each thruster's output is the dot product of the demanded wrench with that
// thruster's contribution row, clamped to the reversible -1..+1 range.

#include "core/types.h"
#include "mixer/IMixer.h"

namespace tardigrade {

class RobosubMixer : public IMixer {
public:
    void mix(const ControlOutput& wrench, MotorOutput& out) override;

    static constexpr uint8_t kThrusters = 8;
};

}  // namespace tardigrade
