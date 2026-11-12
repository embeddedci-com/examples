/*
 * Generic transfers on the sensor bus (I2C1, PB8 SCL / PB9 SDA), for the `i2c` console
 * commands. Register level and polled, so every failure says where it happened: the address
 * was not acknowledged, a data byte was not, the bus was busy, a clock stretch or a stuck
 * line ran past the timeout. The env driver (env.c) owns the peripheral's setup; these
 * transfers run between its reads, from the main loop, never at the same time.
 */
#ifndef I2CBUS_H
#define I2CBUS_H

#include <stdint.h>

typedef enum
{
    I2CB_OK = 0,
    I2CB_NACK_ADDR, /* the address (write or read) was not acknowledged */
    I2CB_NACK_DATA, /* a written data byte was not acknowledged; .index says which */
    I2CB_BUS_BUSY,  /* the bus never went idle (a line held low) */
    I2CB_TIMEOUT,   /* a step did not finish in time (clock stretching, a stuck line) */
    I2CB_ARB_LOST,
    I2CB_BUS_ERROR,
} i2cb_status_t;

typedef struct
{
    i2cb_status_t status;
    uint16_t index; /* I2CB_NACK_DATA: the byte (0 = first) the target refused */
    char phase;     /* 'w' or 'r': the phase that failed */
} i2cb_result_t;

#define I2CB_MAX_BYTES 64U
#define I2CB_DEFAULT_TIMEOUT_MS 25U

/* Write nw bytes, then read nr bytes from the 7-bit addr. restart != 0 joins the two with a
 * repeated START (a register read); otherwise each phase is its own START..STOP and delay_ms
 * passes between them (a command sensor: write the command, wait, read). nw = nr = 0 is an
 * address-only write (a probe). */
i2cb_result_t i2cb_xfer(uint8_t addr, const uint8_t *w, uint16_t nw, uint8_t *r, uint16_t nr,
                        int restart, uint32_t delay_ms);

void i2cb_set_timeout_ms(uint32_t ms);
uint32_t i2cb_timeout_ms(void);

/* Free a bus a target holds low: up to 9 SCL pulses by hand, then a STOP, then the peripheral
 * is reset. Returns the SDA level before (*sda_before) and 0 when SDA reads high at the end. */
int i2cb_recover(int *sda_before);

/* The word for a status in console replies: ok, nack_addr, nack_data, bus_busy, ... */
const char *i2cb_status_name(i2cb_status_t s);

#endif
