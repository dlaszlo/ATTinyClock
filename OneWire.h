#ifndef OneWire_h
#define OneWire_h

#define ONEWIRE_PORT		PORTD
#define ONEWIRE_PIN			PINB
#define ONEWIRE_DDR			DDRB
#define ONEWIRE_PN			PD3

// Perform a 1-Wire reset cycle
uint8_t ow_reset(void);

// Read a bit
uint8_t ow_read_bit(void);

// Write a bit
void ow_write_bit(uint8_t bit);

// Read a byte
uint8_t ow_read(void);

// Write a byte
void ow_write(uint8_t byte);

#endif
