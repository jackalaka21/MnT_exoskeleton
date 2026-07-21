#include <Arduino.h>

#include "FSR.h"

// Helper Functions
// --------------------------------------------------------------------------------------------
// Read Raw Analog Data, normalise to 0–255 and apply an EMA spike filter
void FSR::_readRawData() {
    float raw = (float)(analogRead(_pin) >> 2);

    // Seed the average on the first read so we don't ramp up from 0.
    if (!_emaInit) {
        _ema = raw;
        _emaInit = true;
    }
    else {
        // ema = alpha * new + (1 - alpha) * ema
        _ema += _emaAlpha * (raw - _ema);
    }

    _value = (uint8_t)(_ema + 0.5f);   // round to nearest
}

// Update Contact State
void FSR::_updateContactState() {
    if (_value > _threshold + Config::FSR::HYSTERESIS) {
        _contact = true;
    }
    else if (_value < _threshold - Config::FSR::HYSTERESIS) {
        _contact = false;
    }
}

// Constructor
// --------------------------------------------------------------------------------------------
FSR::FSR(String name, int pin, float alpha) {
    _sensorName = name;
    _pin     = pin;
    _value   = 0;
    _contact = false;
    _threshold = Config::FSR::THRESHOLD;
    _ema      = 0.0f;
    _emaAlpha = alpha;
    _emaInit  = false;
}

// Public Methods
// --------------------------------------------------------------------------------------------
// FSR setup
void FSR::init(){
    pinMode(_pin, INPUT);
}

// Read latest sensor data and write directly into the VolatileData.cpp
void FSR::read(volatile uint8_t* value, volatile bool* contact) {
    _readRawData();
    _updateContactState();
    *value = _value;
    *contact = _contact;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
// Prints sensor name, raw value, threshold and contact state to Serial Monitor
void FSR::printAnalogData() {
    Serial.print(_sensorName);
    Serial.print(": ");
    Serial.print((int)_value);
    Serial.print(" | thr:");
    Serial.print((int)_threshold);
    Serial.print(" | contact:");
    Serial.println(_contact ? 1 : 0);
}



