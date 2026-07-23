#pragma once
//
// ExternalEstimator — the robosub's IStateEstimator.
//
// Does NO fusion. The Jetson already fused ZED visual odometry with the
// VectorNav IMU (robot_localization EKF); this copies that solution into
// VehicleState. The whole point of the IStateEstimator seam is that Controller,
// Safety and Telemetry cannot tell this apart from the hopcopter's onboard
// Mahony filter — see docs/estimator.md.
//
// It reads a JetsonLink (the transport) and applies a freshness contract:
// update() never blocks, reuses the last pose when nothing new has arrived, and
// reports healthy() == false once the pose has gone stale, which trips Safety's
// sensor-timeout failsafe.

#include "comms/JetsonLink.h"
#include "core/types.h"
#include "estimator/IStateEstimator.h"

namespace tardigrade {

class ExternalEstimator : public IStateEstimator {
public:
    // `link` is borrowed and must outlive this object. The EKF runs at ~30 Hz,
    // so the default freshness window allows for a couple of missed messages
    // before declaring the pose stale.
    explicit ExternalEstimator(const JetsonLink& link,
                               uint32_t freshness_timeout_us = 100000);

    bool begin() override;
    bool update(uint32_t now_us) override;
    const VehicleState& state() const override { return state_; }
    bool healthy() const override;

private:
    void fillEulerFromQuaternion();

    const JetsonLink& link_;
    VehicleState state_;
    uint32_t freshness_timeout_us_;
    uint16_t last_seq_ = 0;
    bool seq_started_ = false;
    bool stale_ = true;
};

}  // namespace tardigrade
