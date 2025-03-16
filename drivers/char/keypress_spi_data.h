#pragma once

#define NO_OF_KEYS 280

typedef struct __attribute__((packed)) {
	uint8_t _padding_first;
	uint16_t magic;
	uint32_t time;
	uint8_t data[NO_OF_KEYS];
	uint8_t pedal_data[3];
	uint32_t crc;
	uint8_t _padding_last;
} keypress_spi_buffer_t;
