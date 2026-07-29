#include <Arduino.h>

#include "FSR.h"

// Constructor
// --------------------------------------------------------------------------------------------
FSR::FSR(String name, int pin, uint8_t threshold, float alpha, float gain)
    : _pin(pin)
    , _sensorName(name)
    , _threshold(threshold)
    , _value(0)
    , _contact(false)
    , _ema(0.0f)
    , _emaAlpha(alpha)
    , _emaInit(false)
    , _gain(gain)
    , _debounce(0)
{}

// Helper Functions
// --------------------------------------------------------------------------------------------
// Read the pin, scale to 0–255, EMA-filter, then apply the per-sensor gain.
void FSR::_readRawData() {
    // 10-bit ADC → 0–255.
    float raw = (float)(analogRead(_pin) >> 2);

    // Seed on the first read so we don't ramp up from 0.
    if (!_emaInit) {
        _ema = raw;
        _emaInit = true;
    }
    else {
        _ema += _emaAlpha * (raw - _ema);   // ema = alpha*new + (1-alpha)*ema
    }

    // Gain last, so the EMA runs over the sensor's full unscaled range and the 255 clamp is
    // applied once at the output rather than saturating every sample on the way in.
    float scaled = _ema * _gain;
    if (scaled > 255.0f) scaled = 255.0f;

    _value = (uint8_t)(scaled + 0.5f);   // round
}

// Hysteresis + debounce.
void FSR::_updateContactState() {
    // Candidate state from the hysteresis band. Inside the deadband nothing changes.
    bool candidate = _contact;
    if      (_value > _threshold + Config::FSR::HYSTERESIS) candidate = true;
    else if (_value < _threshold - Config::FSR::HYSTERESIS) candidate = false;

    // Adopt the candidate only once it has held for DEBOUNCE_SAMPLES in a row. Any sample that
    // agrees with the current state resets the count, so a lone spike or dropout is ignored.
    if (candidate == _contact) {
        _debounce = 0;
    }
    else if (++_debounce >= Config::FSR::DEBOUNCE_SAMPLES) {
        _contact  = candidate;
        _debounce = 0;
    }
}

// Public Methods
// --------------------------------------------------------------------------------------------
void FSR::init() {
    pinMode(_pin, INPUT);
}

// Sample the sensor and write the results into the shared globals.
void FSR::read(volatile uint8_t* value, volatile bool* contact) {
    _readRawData();
    _updateContactState();
    *value   = _value;
    *contact = _contact;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void FSR::printAnalogData() {
    Serial.print(_sensorName);
    Serial.print(": ");
    Serial.print((int)_value);
    Serial.print(" | thr:");
    Serial.print((int)_threshold);
    Serial.print(" | contact:");
    Serial.println(_contact ? 1 : 0);
}
