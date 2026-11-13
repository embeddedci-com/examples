/*
 * I2C target mode on the sensor bus (I2C1, PB8 SCL / PB9 SDA), so a test can drive the bus
 * from the pod's I2C master and check what the node received. The node answers at one 7-bit
 * address as a 256-byte register file, like an EEPROM or most sensors: the first byte of a
 * write sets the register pointer, the next bytes are stored from there, a read returns bytes
 * from the pointer; the pointer increments and wraps at 0xFF. Interrupt driven, with clock
 * stretching, so the master can run at 100 kHz or 400 kHz (the F4's I2C is specified to 400 kHz).
 *
 * The env driver must be off while the target mode is on (bench_cmds.c turns it off).
 */
#ifndef I2CTARGET_H
#define I2CTARGET_H

#include <stdint.h>

#define I2CT_MEM_SIZE 256U

typedef struct
{
    uint8_t on;
    uint8_t addr;
    uint8_t ptr;
    uint32_t matches; /* address matches (each START or repeated START to our address) */
    uint32_t rx;      /* bytes received, the pointer byte included */
    uint32_t tx;      /* bytes loaded for the master to read (one more than it took per read) */
    uint32_t errors;  /* bus errors, overruns */
} i2ct_state_t;

/* Start answering at addr (0x08..0x77); the memory reads back mem[i] = i until written. */
int i2ct_start(uint8_t addr);
void i2ct_stop(void);
int i2ct_active(void);
const i2ct_state_t *i2ct_state(void);
const uint8_t *i2ct_mem(void);

#endif
