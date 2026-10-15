/*
  ADS1256 (HiLetgo module) -> Raspberry Pi Pico 2 -> USB CDC, framed binary
  protocol v1.0 (see "Binary packet structure"). Core: arduino-pico.

  Files in this sketch folder (folder name must be ads1256_pico2):
    ads1256_pico2.ino   settings + setup()/loop() glue   (this file)
    afe_common.h/.cpp   protocol: framing, CRC, packetizer, commands
    afe_ads1256.h/.cpp  ADS1256 registers, init, scan, command table

  Wiring (SPI0):
    SCLK GP18 | DIN GP19 | DOUT GP16 | CS GP17 | DRDY GP20 | RST GP21
    PDWN/SYNC -> 3V3 (must be high) | GND -> GND | VCC per your board

  Output: raw signed 24-bit counts (not volts). Volts = counts * LSB,
  LSB = 2*Vref / (GAIN * 8388607). Send STATUS to get the LSB in uV.
  Per-channel sample rate = ADS_SPS / 3 (channels are multiplexed).
  Keep ADS_SPS <= ~2000 (SPI command overhead per channel switch).
*/

#include "afe_common.h"
#include "afe_ads1256.h"

// ====================== USER SETTINGS ======================

// Inputs: ADS_MUX(positive, negative). 0..7 = AIN0..AIN7, ADS_AINCOM = AINCOM.
#define CH1_MUX ADS_MUX(0, ADS_AINCOM)
#define CH2_MUX ADS_MUX(1, ADS_AINCOM)
#define CH3_MUX ADS_MUX(2, ADS_AINCOM)

#define ADS_GAIN   1        // 1, 2, 4, 8, 16, 32, 64
#define ADS_SPS    1000     // 30000 15000 7500 3750 2000 1000 500 100 60 50
                            // 30 25 15 10 5 2.5   (total, shared by 3 channels)
#define ADS_BUFFER 0        // 1 = input buffer on (inputs must stay 0..AVDD-2V)
#define ADS_SDCS   0        // sensor-detect current: 0=off 1=0.5uA 2=2uA 3=10uA
#define ADS_VREF   2.5f     // volts, reported by STATUS only
#define ADS_SPI_HZ 1900000  // max = CLKIN/4 (1.92 MHz with 7.68 MHz crystal)

// 1 = every sample carries a fault byte (bit n = channel n at full scale)
#define FAULT_BYTE 1

// Pins
#define PIN_SCLK 18
#define PIN_MOSI 19
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_DRDY 20
#define PIN_RST  21

// (Packet batching: AFE_SAMPLES_PER_PACKET / AFE_FLUSH_TIMEOUT_MS in afe_common.h)

// ===========================================================

static const AdsConfig cfg = {
  PIN_SCLK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_DRDY, PIN_RST,
  ADS_SPI_HZ,
  { CH1_MUX, CH2_MUX, CH3_MUX },
  ADS_GAIN, ADS_SPS, ADS_BUFFER, ADS_SDCS, ADS_VREF
};

void setup() {
  Serial.begin(115200);                        // nominal on USB CDC
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }

  afe_begin(FAULT_BYTE);
  bool ok = ads_begin(&cfg);

  afe_text("#BOOT protocol " AFE_PROTOCOL_VERSION " ads1256 3ch");
  ads_send_status();
  if (!ok) afe_text("#FAULT ADS1256 init failed (check wiring/power, ADS_SPS, ADS_GAIN)");
}

void loop() {
  int32_t raw[AFE_NUM_CHANNELS];
  uint8_t fault;

  afe_poll_commands(ads_handle_command);
  if (ads_poll(raw, &fault)) afe_add_sample(raw, fault);
  afe_service();
}
