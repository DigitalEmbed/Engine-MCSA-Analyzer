#pragma once
#include <stdint.h>
#include <stddef.h>
#include "process.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef F_SPEC_MAX_HZ
#define F_SPEC_MAX_HZ 1000.0f
#endif

#ifndef MCSA_FRAME_MAX
#define MCSA_FRAME_MAX  4096u
#endif

int mcsa_stream_send_channel(uint32_t channel_id, void *huart_void);
uint16_t mcsa_crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
