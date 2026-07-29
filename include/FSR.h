#pragma once

#include <Arduino.h>

#include "Config.h"
#include "VolatileData.h"

// One foot force sensor: reads the analog pin, filters it, and reports a debounced contact flag.
class FSR {
    private:
        // Internal Variables
        // --------------------------------------------------------------------------------------------
        int     _pin;           // analog pin
        String  _sensorName;    // label for debug logs
        uint8_t _threshold;     // contact threshold (0–255)

        uint8_t _value;         // filtered reading (0–255)
        bool    _contact;       // debounced contact state

        float   _ema;           // EMA accumulator (0–255, pre-gain)
        float   _emaAlpha;      // smoothing factor (0–1); lower = smoother
        bool    _emaInit;       // seeds the EMA with the first sample

        float   _gain;          // sensitivity multiplier applied after the EMA; >1.0 = more sensitive
        uint8_t _debounce;      // samples the pending state has held

        // Helper Functions
        // --------------------------------------------------------------------------------------------
        void _readRawData();          // read the pin, scale to 0–255, EMA-filter, apply gain
        void _updateContactState();   // hysteresis + debounce

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        // threshold : contact threshold (0–255), lower = more sensitive. Pass
        //             Config::FSR::TOE_THRESHOLD for a forefoot sensor.
        // alpha     : EMA smoothing (0–1). Lower kills spikes harder but adds lag.
        // gain      : multiplier on the filtered reading before thresholding (clamped to 255). Use it
        //             to match a physically weaker sensor to the others — see Config::FSR.
        FSR(String name, int pin,
            uint8_t threshold = Config::FSR::THRESHOLD,
            float   alpha     = Config::FSR::EMA_ALPHA,
            float   gain      = Config::FSR::DEFAULT_GAIN);

        // Public Methods
        // --------------------------------------------------------------------------------------------
        void init();

        // Sample the sensor and write the results into the shared globals.
        void read(volatile uint8_t* value, volatile bool* contact);

        // Serial Monitor (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printAnalogData();
};
