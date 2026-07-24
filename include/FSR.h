#pragma once

#include <Arduino.h>

#include "Config.h"
#include "VolatileData.h"

class FSR {
    private:
        // Internal Variables
        // --------------------------------------------------------------------------------------------
        // Sensor configuration
        int    _pin;              // Analog pin
        String _sensorName;       // For clean debug logs
        uint8_t _threshold;        // Contact threshold (0–255)

        // Sensor data
        uint8_t _value;            // EMA-filtered reading (0–255)
        bool    _contact;          // debounced contact state

        // EMA (exponential moving average) spike filter
        float   _ema;              // running average accumulator (0–255)
        float   _emaAlpha;         // smoothing factor (0–1); lower = smoother
        bool    _emaInit;          // seed EMA with first sample

        // Debounce — consecutive samples the pending contact state has held (see Config::FSR)
        uint8_t _debounce;



        // Helper Functions
        // --------------------------------------------------------------------------------------------
        // Read Raw Analog Data, normalise to 0–255 and EMA-filter
        void _readRawData();

        // Update Contact State
        void _updateContactState();

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        // threshold: contact threshold (0–255). LOWER = more sensitive. Defaults to
        //            Config::FSR::THRESHOLD; pass Config::FSR::TOE_THRESHOLD for a forefoot sensor.
        // alpha:     EMA smoothing factor (0–1). Lower filters spikes harder but adds lag.
        //            Defaults to Config::FSR::EMA_ALPHA; override per sensor here.
        FSR(String name, int pin,
            uint8_t threshold = Config::FSR::THRESHOLD,
            float   alpha     = Config::FSR::EMA_ALPHA);

        // Public Methods
        // --------------------------------------------------------------------------------------------
        // FSR setup
        void init();

        // Read latest sensor data and write directly into the VolatileData.cpp
        void read(volatile uint8_t* value, volatile bool* contact);

        // Serial Monitor (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printAnalogData();

};
