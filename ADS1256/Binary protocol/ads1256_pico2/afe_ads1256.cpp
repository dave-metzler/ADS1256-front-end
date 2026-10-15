#include "afe_ads1256.h"
#include <SPI.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define CMD_WAKEUP  0x00
#define CMD_RDATA   0x01
#define CMD_SDATAC  0x0F
#define CMD_RREG    0x10
#define CMD_WREG    0x50
#define CMD_SELFCAL 0xF0
#define CMD_SYNC    0xFC
#define CMD_RESET   0xFE

#define REG_STATUS  0x00
#define REG_MUX     0x01
#define REG_ADCON   0x02
#define REG_DRATE   0x03

static const struct { float sps; uint8_t reg; } RATES[] = {
  {30000, 0xF0}, {15000, 0xE0}, {7500, 0xD0}, {3750, 0xC0}, {2000, 0xB0},
  {1000, 0xA1}, {500, 0x92}, {100, 0x82}, {60, 0x72}, {50, 0x63},
  {30, 0x53}, {25, 0x43}, {15, 0x33}, {10, 0x23}, {5, 0x13}, {2.5f, 0x03}
};
#define NUM_RATES (sizeof(RATES) / sizeof(RATES[0]))

static AdsConfig   s_cfg;
static SPISettings s_spi;
static bool        s_streaming = false;
static uint8_t     s_idx = 0;                     // channel being converted
static int32_t     s_raw[AFE_NUM_CHANNELS];
static uint8_t     s_fault = 0;

// ------------------------------ helpers ------------------------------

static int find_rate(float sps) {
  for (unsigned i = 0; i < NUM_RATES; i++)
    if (fabsf(RATES[i].sps - sps) < 0.01f) return (int)i;
  return -1;
}

static int gain_bits(uint16_t g) {
  switch (g) {
    case 1: return 0;  case 2: return 1;  case 4: return 2;  case 8: return 3;
    case 16: return 4; case 32: return 5; case 64: return 6;
  }
  return -1;
}

static bool wait_drdy_low(uint32_t timeout_ms) {
  uint32_t t = millis();
  while (digitalRead(s_cfg.pin_drdy) == HIGH)
    if (millis() - t > timeout_ms) return false;
  return true;
}

// ------------------------------ SPI level ------------------------------

static void command(uint8_t cmd) {
  SPI.beginTransaction(s_spi);
  digitalWrite(s_cfg.pin_cs, LOW);
  SPI.transfer(cmd);
  digitalWrite(s_cfg.pin_cs, HIGH);
  SPI.endTransaction();
}

static void write_reg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(s_spi);
  digitalWrite(s_cfg.pin_cs, LOW);
  SPI.transfer(CMD_WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(val);
  digitalWrite(s_cfg.pin_cs, HIGH);
  SPI.endTransaction();
  delayMicroseconds(5);
}

static uint8_t read_reg(uint8_t reg) {
  SPI.beginTransaction(s_spi);
  digitalWrite(s_cfg.pin_cs, LOW);
  SPI.transfer(CMD_RREG | reg);
  SPI.transfer(0x00);
  delayMicroseconds(10);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(s_cfg.pin_cs, HIGH);
  SPI.endTransaction();
  return v;
}

static int32_t read_data() {
  SPI.beginTransaction(s_spi);
  digitalWrite(s_cfg.pin_cs, LOW);
  SPI.transfer(CMD_RDATA);
  delayMicroseconds(10);
  uint32_t b2 = SPI.transfer(0x00);
  uint32_t b1 = SPI.transfer(0x00);
  uint32_t b0 = SPI.transfer(0x00);
  digitalWrite(s_cfg.pin_cs, HIGH);
  SPI.endTransaction();
  int32_t v = (int32_t)((b2 << 16) | (b1 << 8) | b0);
  if (v & 0x800000) v |= (int32_t)0xFF000000;
  return v;
}

static void select_channel(uint8_t mux) {
  write_reg(REG_MUX, mux);
  command(CMD_SYNC);
  delayMicroseconds(5);
  command(CMD_WAKEUP);
}

// Restart the channel pipeline at channel 0.
static void prime() {
  s_idx = 0;
  s_fault = 0;
  select_channel(s_cfg.mux[0]);
  uint32_t t = micros();                       // let DRDY go high first
  while (digitalRead(s_cfg.pin_drdy) == LOW && (micros() - t) < 2000) { }
}

// Write config registers, self-calibrate, verify, prime. Auto-cal is
// disabled while writing so no register write is ignored mid-calibration.
static bool apply_config() {
  int ri = find_rate(s_cfg.sps);
  int gb = gain_bits(s_cfg.gain);
  if (ri < 0 || gb < 0) return false;

  uint8_t adcon = (uint8_t)((s_cfg.sdcs << 3) | gb);
  uint8_t drate = RATES[ri].reg;

  command(CMD_SDATAC);
  write_reg(REG_STATUS, s_cfg.buffer ? 0x02 : 0x00);
  write_reg(REG_ADCON, adcon);
  write_reg(REG_DRATE, drate);
  write_reg(REG_MUX, s_cfg.mux[0]);

  command(CMD_SELFCAL);
  uint32_t t = micros();
  while (digitalRead(s_cfg.pin_drdy) == LOW && (micros() - t) < 5000) { }
  bool cal_ok = wait_drdy_low(3000);
  delay(1);

  bool ok = cal_ok &&
            read_reg(REG_DRATE) == drate &&
            (read_reg(REG_ADCON) & 0x1F) == (adcon & 0x1F);
  prime();
  return ok;
}

// ------------------------------ public API ------------------------------

bool ads_begin(const AdsConfig *cfg) {
  s_cfg = *cfg;
  s_spi = SPISettings(s_cfg.spi_hz, MSBFIRST, SPI_MODE1);   // CPOL=0, CPHA=1

  SPI.setSCK(s_cfg.pin_sclk);
  SPI.setTX(s_cfg.pin_mosi);
  SPI.setRX(s_cfg.pin_miso);
  SPI.begin();

  pinMode(s_cfg.pin_cs, OUTPUT);
  digitalWrite(s_cfg.pin_cs, HIGH);
  pinMode(s_cfg.pin_drdy, INPUT_PULLUP);
  pinMode(s_cfg.pin_rst, OUTPUT);
  digitalWrite(s_cfg.pin_rst, LOW);  delay(10);
  digitalWrite(s_cfg.pin_rst, HIGH); delay(100);
  command(CMD_RESET);
  delay(50);

  bool ok = apply_config();
  s_streaming = ok;
  return ok;
}

bool ads_poll(int32_t raw[AFE_NUM_CHANNELS], uint8_t *fault) {
  if (!s_streaming) return false;
  if (digitalRead(s_cfg.pin_drdy) == HIGH) return false;

  // Queue the NEXT channel, then fetch the result of the one that just
  // finished (the data register holds it until the next conversion).
  select_channel(s_cfg.mux[(s_idx + 1) % AFE_NUM_CHANNELS]);
  int32_t v = read_data();
  s_raw[s_idx] = v;
  if (v == 0x7FFFFF || v == -0x800000) s_fault |= (uint8_t)(1u << s_idx);

  if (++s_idx < AFE_NUM_CHANNELS) return false;

  s_idx = 0;
  memcpy(raw, s_raw, sizeof(s_raw));
  *fault = s_fault;
  s_fault = 0;
  return true;
}

bool ads_streaming() { return s_streaming; }

void ads_send_status() {
  double lsb_uv = 2.0 * s_cfg.vref / ((double)s_cfg.gain * 8388607.0) * 1e6;
  afe_textf("#STATUS stream=%d rate=%g sps (%g per ch) gain=%u buffer=%u "
            "lsb=%.4f uV vref=%.3f mux=%02X,%02X,%02X",
            s_streaming ? 1 : 0, (double)s_cfg.sps,
            (double)s_cfg.sps / AFE_NUM_CHANNELS, (unsigned)s_cfg.gain,
            (unsigned)s_cfg.buffer, lsb_uv, (double)s_cfg.vref,
            s_cfg.mux[0], s_cfg.mux[1], s_cfg.mux[2]);
}

// ------------------------------ commands ------------------------------

static void reconfigure(const char *what) {
  afe_flush();
  bool was = s_streaming;
  s_streaming = false;
  bool ok = apply_config();
  s_streaming = was && ok;
  if (!ok) afe_textf("#FAULT %s change failed; ADC not responding", what);
  ads_send_status();
}

void ads_handle_command(char *line) {
  char *cmd = strtok(line, " ,=");
  char *arg = strtok(NULL, " ,=");
  if (!cmd) return;

  if (!strcmp(cmd, "HELP")) {
    afe_text("#HELP commands: HELP, STATUS, STREAM <0|1>, RATE <sps>, GAIN <1|2|4|8|16|32|64>");
    afe_text("#HELP RATE: 30000 15000 7500 3750 2000 1000 500 100 60 50 30 25 15 10 5 2.5");
    afe_text("#HELP data: raw signed 24-bit counts; per-sample fault bit n = channel n at full scale");
  } else if (!strcmp(cmd, "STATUS")) {
    ads_send_status();
  } else if (!strcmp(cmd, "STREAM")) {
    if (!arg) { afe_text("#ERR usage: STREAM <0|1>"); return; }
    bool on = !strcmp(arg, "1") || !strcmp(arg, "ON");
    if (on && !s_streaming) {
      afe_reset_stream();
      prime();
      s_streaming = true;
    } else if (!on && s_streaming) {
      afe_flush();
      s_streaming = false;
    }
    ads_send_status();
  } else if (!strcmp(cmd, "RATE")) {
    float r = arg ? (float)atof(arg) : 0.0f;
    if (find_rate(r) < 0) { afe_text("#ERR invalid RATE (see HELP)"); return; }
    s_cfg.sps = r;
    reconfigure("RATE");
  } else if (!strcmp(cmd, "GAIN")) {
    int g = arg ? atoi(arg) : 0;
    if (gain_bits((uint16_t)g) < 0) { afe_text("#ERR invalid GAIN (see HELP)"); return; }
    s_cfg.gain = (uint16_t)g;
    reconfigure("GAIN");
  } else {
    afe_textf("#ERR unknown command '%s' (try HELP)", cmd);
  }
}
