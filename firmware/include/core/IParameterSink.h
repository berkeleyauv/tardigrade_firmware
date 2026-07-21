#pragma once
//
// IParameterSink — runtime-tunable parameters, addressed by a stable u16 id.
//
// Exists so CommandLink can service SetParameter / GetParameters without knowing
// what the parameters mean. A vehicle's controller implements it; the id space
// is defined in control/Parameters.h and mirrored by the ground station.
//
// The enumeration pair (count + at) lets the ground station read every current
// value back on connect, so its sliders start where the firmware actually is
// rather than at some default the two disagree on.

#include <stdint.h>

namespace tardigrade {

class IParameterSink {
public:
    virtual ~IParameterSink() = default;

    // Apply a value. Returns false for an unknown id or an out-of-range value,
    // which becomes a rejected Ack.
    virtual bool setParameter(uint16_t id, float value) = 0;

    // Enumeration for read-back. `index` in [0, parameterCount()).
    virtual uint16_t parameterCount() const = 0;
    virtual bool parameterAt(uint16_t index, uint16_t& id, float& value) const = 0;

    // Persist the current values to non-volatile storage (flash), so they
    // survive a reboot. Default: unsupported. This is the session scratchpad —
    // the checked-in constructor defaults remain the source of truth.
    virtual bool saveParameters() { return false; }

    // Re-apply the compiled defaults AND clear any saved values, so a bad save
    // cannot brick every future boot. Default: unsupported.
    virtual bool resetParameters() { return false; }
};

}  // namespace tardigrade
