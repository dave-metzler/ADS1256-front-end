/*
  ADS1256 (HiLetgo module) -> Raspberry Pi Pico 2 (RP2350) -> USB serial CSV
  Core: Earle Philhower "Raspberry Pi Pico/RP2040/RP2350" (arduino-pico)

  Output: one line per scan:  v1,v2,v3\n

  Wiring (SPI0):
    ADS1256  ->  Pico 2
    SCLK     ->  GP18
    DIN      ->  GP19  (MOSI)
    DOUT     ->  GP16  (MISO)
    CS       ->  GP17
    DRDY     ->  GP20
    RST      ->  GP21  (or tie high via 3V3)
    PDWN/SYNC-> 3V3    (must be HIGH; SYNC is done by command)
    GND      ->  GND
    VCC      ->  per your board (usually 5V for analog; check logic is 3.3V-safe)
*/

#include <SPI.h>

// ====================== USER SETTINGS ======================

// Input selection. Each channel = positive input, negative input.
// Use 0..7 for AIN0..AIN7, or ADS_AINCOM (8) for single-ended vs. AINCOM.
#define ADS_AINCOM 8
#define CH1_P 0
#define CH1_N ADS_AINCOM
#define CH2_P 1
#define CH2_N ADS_AINCOM
#define CH3_P 2
#define CH3_N ADS_AINCOM

// PGA gain: 1, 2, 4, 8, 16, 32 or 64
#define ADS_GAIN 1

// ADC data rate (SPS): 30000 15000 7500 3750 2000 1000 500 100 60 50 30 25 15 10 5
// (2.5 -> use 0x03 directly, see DRATE_BYTE below)
// NOTE: channels are multiplexed, so per-channel rate = ADS_SPS / 3.
// Because of SPI command overhead, keep this <= ~2000 for reliable scanning.
#define ADS_SPS 1000

// Input buffer: 1 = on (higher input impedance, but inputs must stay within
// roughly 0V .. AVDD-2V), 0 = off.
#define ADS_BUFFER 0

// Output format: 1 = volts (float), 0 = raw signed 24-bit counts
#define OUTPUT_VOLTS 1
#define VOLT_DECIMALS 6

// Reference voltage (HiLetgo board has a 2.5V reference; change if modified)
#define ADS_VREF 2.5f

// Sensor detect current sources (0=off, 1=0.5uA, 2=2uA, 3=10uA)
#define ADS_SDCS 0

// SPI clock. Max is CLKIN/4 (7.68 MHz crystal -> 1.92 MHz).
#define ADS_SPI_HZ 1900000

// Pins
#define PIN_SCLK 18
#define PIN_MOSI 19
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_DRDY 20
#define PIN_RST  21

// ====================== DERIVED / REGISTER DEFS ======================

#define CMD_WAKEUP 0x00
#define CMD_RDATA  0x01
#define CMD_SDATAC 0x0F
#define CMD_RREG   0x10
#define CMD_WREG   0x50
#define CMD_SELFCAL 0xF0
#define CMD_SYNC   0xFC
#define CMD_RESET  0xFE

#define REG_STATUS 0x00
#define REG_MUX    0x01
#define REG_ADCON  0x02
#define REG_DRATE  0x03
#define REG_IO     0x04

#if   ADS_SPS == 30000
  #define DRATE_BYTE 0xF0
#elif ADS_SPS == 15000
  #define DRATE_BYTE 0xE0
#elif ADS_SPS == 7500
  #define DRATE_BYTE 0xD0
#elif ADS_SPS == 3750
  #define DRATE_BYTE 0xC0
#elif ADS_SPS == 2000
  #define DRATE_BYTE 0xB0
#elif ADS_SPS == 1000
  #define DRATE_BYTE 0xA1
#elif ADS_SPS == 500
  #define DRATE_BYTE 0x92
#elif ADS_SPS == 100
  #define DRATE_BYTE 0x82
#elif ADS_SPS == 60
  #define DRATE_BYTE 0x72
#elif ADS_SPS == 50
  #define DRATE_BYTE 0x63
#elif ADS_SPS == 30
  #define DRATE_BYTE 0x53
#elif ADS_SPS == 25
  #define DRATE_BYTE 0x43
#elif ADS_SPS == 15
  #define DRATE_BYTE 0x33
#elif ADS_SPS == 10
  #define DRATE_BYTE 0x23
#elif ADS_SPS == 5
  #define DRATE_BYTE 0x13
#else
  #error "ADS_SPS is not a valid ADS1256 data rate"
#endif

#if   ADS_GAIN == 1
  #define PGA_BITS 0
#elif ADS_GAIN == 2
  #define PGA_BITS 1
#elif ADS_GAIN == 4
  #define PGA_BITS 2
#elif ADS_GAIN == 8
  #define PGA_BITS 3
#elif ADS_GAIN == 16
  #define PGA_BITS 4
#elif ADS_GAIN == 32
  #define PGA_BITS 5
#elif ADS_GAIN == 64
  #define PGA_BITS 6
#else
  #error "ADS_GAIN must be 1,2,4,8,16,32 or 64"
#endif

#define NUM_CH 3
static const uint8_t muxByte[NUM_CH] = {
  (CH1_P << 4) | CH1_N,
  (CH2_P << 4) | CH2_N,
  (CH3_P << 4) | CH3_N
};

static const float LSB_VOLTS = (2.0f * ADS_VREF) / (ADS_GAIN * 8388607.0f);

// ====================== LOW-LEVEL ======================

static SPISettings adsSPI(ADS_SPI_HZ, MSBFIRST, SPI_MODE1);  // CPOL=0, CPHA=1

static inline void waitDRDY() {
  while (digitalRead(PIN_DRDY) == HIGH) { }
}

static void adsCommand(uint8_t cmd) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

static void adsWriteReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);            // write 1 register
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
}

static uint8_t adsReadReg(uint8_t reg) {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RREG | reg);
  SPI.transfer(0x00);
  delayMicroseconds(10);         // t6 delay
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return v;
}

static int32_t adsReadData() {
  SPI.beginTransaction(adsSPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(CMD_RDATA);
  delayMicroseconds(10);         // t6 delay (>= 50 CLKIN periods)
  uint32_t b2 = SPI.transfer(0x00);
  uint32_t b1 = SPI.transfer(0x00);
  uint32_t b0 = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  int32_t v = (b2 << 16) | (b1 << 8) | b0;
  if (v & 0x800000) v |= 0xFF000000;   // sign extend 24 -> 32
  return v;
}

// Switch input mux and restart conversion (call after DRDY goes low).
static inline void adsSelectChannel(uint8_t mux) {
  adsWriteReg(REG_MUX, mux);
  adsCommand(CMD_SYNC);
  delayMicroseconds(5);
  adsCommand(CMD_WAKEUP);
}

static void adsInit() {
  pinMode(PIN_CS, OUTPUT);   digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  pinMode(PIN_RST, OUTPUT);

  digitalWrite(PIN_RST, LOW);  delay(10);
  digitalWrite(PIN_RST, HIGH); delay(100);

  adsCommand(CMD_RESET);
  delay(50);
  adsCommand(CMD_SDATAC);

  // STATUS: auto-cal OFF while configuring (avoids ignored writes),
  // buffer per setting (bit1), MSB first
  adsWriteReg(REG_STATUS, (ADS_BUFFER ? 0x02 : 0x00));
  adsWriteReg(REG_ADCON, (ADS_SDCS << 3) | PGA_BITS);   // CLKOUT off
  adsWriteReg(REG_DRATE, DRATE_BYTE);
  adsWriteReg(REG_MUX, muxByte[0]);

  // Now calibrate explicitly with the final gain/rate settings
  adsCommand(CMD_SELFCAL);
  uint32_t t = micros();
  while (digitalRead(PIN_DRDY) == LOW && (micros() - t) < 5000) { }  // DRDY -> high
  waitDRDY();                    // DRDY -> low when calibration is done
  delay(1);
}

// ====================== MAIN ======================

void setup() {
  Serial.begin(115200);          // USB CDC; baud value is ignored
  while (!Serial && millis() < 3000) { }

  SPI.setSCK(PIN_SCLK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  adsInit();

  // Sanity check: DRATE readback
  uint8_t drate = adsReadReg(REG_DRATE);
  if (drate != DRATE_BYTE) {
    Serial.print("# ERROR: DRATE readback 0x");
    Serial.print(drate, HEX);
    Serial.print(" expected 0x");
    Serial.print(DRATE_BYTE, HEX);
    Serial.print(", ADCON 0x");
    Serial.println(adsReadReg(REG_ADCON), HEX);
  }

  // Prime the pipeline: ch0 selected, restart conversion
  adsSelectChannel(muxByte[0]);
}

void loop() {
  int32_t raw[NUM_CH];

  // Each DRDY: queue up the NEXT channel, then read the result of the one
  // that just finished (the data register holds it until the next conversion).
  for (uint8_t i = 0; i < NUM_CH; i++) {
    waitDRDY();
    adsSelectChannel(muxByte[(i + 1) % NUM_CH]);
    raw[i] = adsReadData();
  }

  char line[96];
#if OUTPUT_VOLTS
  int n = snprintf(line, sizeof(line), "%.*f,%.*f,%.*f\n",
                   VOLT_DECIMALS, raw[0] * LSB_VOLTS,
                   VOLT_DECIMALS, raw[1] * LSB_VOLTS,
                   VOLT_DECIMALS, raw[2] * LSB_VOLTS);
#else
  int n = snprintf(line, sizeof(line), "%ld,%ld,%ld\n",
                   (long)raw[0], (long)raw[1], (long)raw[2]);
#endif
  Serial.write((const uint8_t*)line, n);
}
