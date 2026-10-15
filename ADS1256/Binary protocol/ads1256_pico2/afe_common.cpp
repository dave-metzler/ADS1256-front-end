#include "afe_common.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define SYNC0          0xA5
#define SYNC1          0x5A
#define TYPE_ECG_DATA  0x01
#define TYPE_TEXT_LOG  0x02
#define MAX_PAYLOAD    512
#define ECG_HDR_LEN    8

static bool     s_fault;
static uint16_t s_seq;            // shared by all frame types
static uint32_t s_counter;        // index of the NEXT sample to be queued
static uint32_t s_first_counter;  // index of first sample in current packet
static uint8_t  s_n;              // samples queued in current packet
static uint32_t s_first_ms;       // when the first queued sample arrived

static uint8_t s_ecg[ECG_HDR_LEN + AFE_SAMPLES_PER_PACKET * (AFE_NUM_CHANNELS * 3 + 1)];
static uint8_t s_tx[9 + MAX_PAYLOAD];

static inline uint8_t group_len() { return AFE_NUM_CHANNELS * 3 + (s_fault ? 1 : 0); }

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0
static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void send_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
  if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
  s_tx[0] = SYNC0;
  s_tx[1] = SYNC1;
  s_tx[2] = type;
  s_tx[3] = (uint8_t)(len & 0xFF);
  s_tx[4] = (uint8_t)(len >> 8);
  s_tx[5] = (uint8_t)(s_seq & 0xFF);
  s_tx[6] = (uint8_t)(s_seq >> 8);
  s_seq++;
  memcpy(s_tx + 7, payload, len);
  uint16_t crc = crc16(s_tx + 2, 5 + len);          // TYPE..PAYLOAD
  s_tx[7 + len] = (uint8_t)(crc & 0xFF);
  s_tx[8 + len] = (uint8_t)(crc >> 8);
  Serial.write(s_tx, 9 + len);
}

void afe_begin(bool fault_byte) {
  s_fault = fault_byte;
  s_seq = 0;
  afe_reset_stream();
}

void afe_reset_stream() {
  s_n = 0;
  s_counter = 0;
}

void afe_flush() {
  if (s_n == 0) return;
  s_ecg[0] = (uint8_t)(s_first_counter);
  s_ecg[1] = (uint8_t)(s_first_counter >> 8);
  s_ecg[2] = (uint8_t)(s_first_counter >> 16);
  s_ecg[3] = (uint8_t)(s_first_counter >> 24);
  s_ecg[4] = AFE_NUM_CHANNELS;
  s_ecg[5] = s_n;
  s_ecg[6] = s_fault ? 0x01 : 0x00;
  s_ecg[7] = 0;
  send_frame(TYPE_ECG_DATA, s_ecg, ECG_HDR_LEN + (uint16_t)s_n * group_len());
  s_n = 0;
}

void afe_add_sample(const int32_t *ch, uint8_t fault) {
  if (s_n == 0) {
    s_first_counter = s_counter;
    s_first_ms = millis();
  }
  uint8_t *p = s_ecg + ECG_HDR_LEN + (uint16_t)s_n * group_len();
  for (int c = 0; c < AFE_NUM_CHANNELS; c++) {
    int32_t v = ch[c];
    *p++ = (uint8_t)(v);
    *p++ = (uint8_t)(v >> 8);
    *p++ = (uint8_t)(v >> 16);
  }
  if (s_fault) *p++ = fault;
  s_n++;
  s_counter++;
  if (s_n >= AFE_SAMPLES_PER_PACKET) afe_flush();
}

void afe_service() {
  if (s_n && (millis() - s_first_ms) >= AFE_FLUSH_TIMEOUT_MS) afe_flush();
}

void afe_text(const char *s) {
  send_frame(TYPE_TEXT_LOG, (const uint8_t *)s, (uint16_t)strlen(s));
}

void afe_textf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  afe_text(buf);
}

void afe_poll_commands(void (*handler)(char *line)) {
  static char line[64];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      len = 0;
      if (line[0]) handler(line);
      continue;
    }
    if (len < sizeof(line) - 1) line[len++] = (char)toupper(c);
  }
}
