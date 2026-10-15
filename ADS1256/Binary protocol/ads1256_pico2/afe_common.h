/*
  afe_common.h -- USB framing layer, protocol v1.0 (chip independent)

  Implements the framed binary protocol from "Binary packet structure":
    SYNC0 SYNC1 TYPE LENGTH(2) SEQ(2) PAYLOAD CRC16(2)   (little-endian)
  - ECG_DATA (0x01): batched, interleaved signed 24-bit samples
  - TEXT_LOG (0x02): ASCII boot/status/fault text
  - CRC-16/CCITT-FALSE over TYPE, LENGTH, SEQ, PAYLOAD
  Host->device commands remain plain ASCII lines (unframed).

  NOTHING else may write to Serial: every device->host byte must be
  inside a frame, or the receiver's parser will see it as garbage.
*/
#ifndef AFE_COMMON_H
#define AFE_COMMON_H

#include <Arduino.h>
#include <stdint.h>

#define AFE_PROTOCOL_VERSION "1.0"

// Channels per sample. The scope back end requires exactly 3.
#define AFE_NUM_CHANNELS 3

// ---- Batching (edit here) ----
#define AFE_SAMPLES_PER_PACKET 8     // 1..16 (8-16 recommended)
#define AFE_FLUSH_TIMEOUT_MS   50    // send a partial packet after this long

#if AFE_SAMPLES_PER_PACKET < 1 || AFE_SAMPLES_PER_PACKET > 16
#error "AFE_SAMPLES_PER_PACKET must be 1..16"
#endif

// fault_byte: if true, every sample group carries a trailing fault
// bitmask byte (FLAGS bit0 = 1). Call once from setup().
void afe_begin(bool fault_byte);

// Drop any partially built packet and restart SAMPLE_COUNTER at 0.
void afe_reset_stream();

// Queue one sample (AFE_NUM_CHANNELS signed 24-bit values, as int32).
// Sends a frame automatically when the packet is full.
void afe_add_sample(const int32_t *ch, uint8_t fault);

// Call every loop(): flushes a partial packet after the timeout.
void afe_service();

// Send any partially built packet now.
void afe_flush();

// TEXT_LOG frames (no trailing newline is sent on the wire).
void afe_text(const char *s);
void afe_textf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Reads ASCII command lines from Serial; calls handler(line) with each
// complete, upper-cased, non-empty line (CR/LF stripped).
void afe_poll_commands(void (*handler)(char *line));

#endif
