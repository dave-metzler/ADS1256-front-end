/*
  afe_ads1256.h -- ADS1256 front end (chip specific layer)
  Registers, init, scan state machine, and the command dispatch table.
*/
#ifndef AFE_ADS1256_H
#define AFE_ADS1256_H

#include <Arduino.h>
#include "afe_common.h"

#define ADS_AINCOM 8
#define ADS_MUX(p, n) ((uint8_t)(((p) << 4) | (n)))

typedef struct {
  uint8_t  pin_sclk, pin_mosi, pin_miso, pin_cs, pin_drdy, pin_rst;
  uint32_t spi_hz;
  uint8_t  mux[AFE_NUM_CHANNELS];   // ADS_MUX(pos, neg) per channel
  uint16_t gain;                    // 1,2,4,8,16,32,64
  float    sps;                     // ADS1256 data rate, e.g. 1000
  uint8_t  buffer;                  // 1 = input buffer on
  uint8_t  sdcs;                    // sensor-detect current 0..3
  float    vref;                    // volts (for STATUS text only)
} AdsConfig;

// Initialise SPI/ADC and calibrate. Returns false on failure.
bool ads_begin(const AdsConfig *cfg);

// Call every loop(). Non-blocking; returns true when a full 3-channel
// scan is ready in raw[] (signed 24-bit counts) with a per-channel
// overrange bitmask in *fault (bit n = channel n at full scale).
bool ads_poll(int32_t raw[AFE_NUM_CHANNELS], uint8_t *fault);

bool ads_streaming();
void ads_send_status();

// Command dispatch: HELP, STATUS, STREAM <0|1>, RATE <sps>, GAIN <n>
void ads_handle_command(char *line);

#endif
