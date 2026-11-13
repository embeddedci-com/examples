/*
 * Generic bus commands, so a HIL test can talk to any sensor the pod emulates the way a
 * driver would: `i2c` (I2C1, the env sensor's bus), `spi` (SPI2), `uart3` (USART3, with an
 * NMEA mode), `drdy` (a data-ready line on PA8) and `env` (the BMP280/BME280 driver on/off).
 */
#ifndef BENCH_CMDS_H
#define BENCH_CMDS_H

void bench_init(void);
/* Main-loop work: the NMEA printer. */
void bench_service(void);
/* Runs cmd when it is one of these commands; 0 when it is not. */
int bench_command(char *cmd);

/* From main.c: */
int strobe_is_off(void); /* PA8 is an input (strobe off), so drdy may read it */
void node_kick(void);    /* kick the watchdog during a long command */

#endif
