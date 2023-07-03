#pragma once

#define NO_OF_SIMULTANEOUS_KEYS 20

typedef struct __attribute__((packed)) {
	uint16_t key;
	uint8_t data;
} keypress_spi_data_t;

typedef struct __attribute__((packed)) {
	uint16_t magic;
	uint32_t time;
	keypress_spi_data_t data[NO_OF_SIMULTANEOUS_KEYS];
	uint32_t crc;
} keypress_spi_buffer_t;
