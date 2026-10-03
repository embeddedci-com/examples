/*
 * Minimal MCP2515 CAN controller driver (classic CAN, polled/INT-pin driven).
 *
 * The board code supplies the SPI byte exchange and chip-select; everything
 * else (reset, bit timing, modes, TX/RX buffers, error flags) lives here.
 */
#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>

/* SPI instructions */
#define MCP_RESET       0xC0U
#define MCP_READ        0x03U
#define MCP_WRITE       0x02U
#define MCP_BIT_MODIFY  0x05U
#define MCP_READ_STATUS 0xA0U
#define MCP_READ_RX0    0x90U /* read RXB0SIDH.., clears RX0IF on CS high */
#define MCP_READ_RX1    0x94U
#define MCP_LOAD_TX0    0x40U /* load TXB0SIDH.. */
#define MCP_RTS_TX0     0x81U

/* Registers */
#define MCP_TEC      0x1CU
#define MCP_REC      0x1DU
#define MCP_CANSTAT  0x0EU
#define MCP_CANCTRL  0x0FU
#define MCP_CNF3     0x28U
#define MCP_CNF2     0x29U
#define MCP_CNF1     0x2AU
#define MCP_CANINTE  0x2BU
#define MCP_CANINTF  0x2CU
#define MCP_EFLG     0x2DU
#define MCP_TXB0CTRL 0x30U
#define MCP_RXB0CTRL 0x60U
#define MCP_RXB1CTRL 0x70U

/* CANINTF / CANINTE bits */
#define MCP_INT_RX0  0x01U
#define MCP_INT_RX1  0x02U
#define MCP_INT_TX0  0x04U
#define MCP_INT_ERR  0x20U

/* EFLG bits */
#define MCP_EFLG_RX1OVR 0x80U
#define MCP_EFLG_RX0OVR 0x40U
#define MCP_EFLG_TXBO   0x20U
#define MCP_EFLG_TXEP   0x10U
#define MCP_EFLG_RXEP   0x08U

/* TXBnCTRL bits */
#define MCP_TXB_ABTF  0x40U
#define MCP_TXB_MLOA  0x20U
#define MCP_TXB_TXERR 0x10U
#define MCP_TXB_TXREQ 0x08U

typedef enum
{
    MCP_MODE_NORMAL = 0x00,
    MCP_MODE_SLEEP = 0x20,
    MCP_MODE_LOOPBACK = 0x40,
    MCP_MODE_LISTEN = 0x60,
    MCP_MODE_CONFIG = 0x80,
} mcp_mode_t;

typedef struct
{
    uint32_t id;
    uint8_t ext;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum
{
    MCP_TX_OK = 0,
    MCP_TX_TIMEOUT,   /* still pending at the deadline (no ACK, bus-off); aborted */
    MCP_TX_ERROR,     /* TXERR or MLOA set */
} mcp_tx_result_t;

/* Board hooks, implemented by the application. */
void mcp_board_select(int active);           /* 1 = CS low */
uint8_t mcp_board_xfer(uint8_t out);
uint32_t mcp_board_millis(void);

uint8_t mcp_read_reg(uint8_t addr);
void mcp_write_reg(uint8_t addr, uint8_t val);
void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t val);

/* Reset and check the chip answers (CANSTAT reads config mode). 0 = found. */
int mcp_reset(void);
/* Bit timing + accept-all filters + RX interrupts. Leaves the chip in config mode. */
int mcp_configure(uint32_t osc_hz, uint32_t bitrate);
/* Request a mode and wait for CANSTAT to confirm it. 0 = ok. */
int mcp_set_mode(mcp_mode_t mode);
mcp_mode_t mcp_get_mode(void);

/* Send one frame from TXB0 and wait up to timeout_ms for it to go out. */
mcp_tx_result_t mcp_send(const can_frame_t *f, uint32_t timeout_ms);
/* Pop one received frame (RXB0 first). 1 = got a frame, 0 = none. */
int mcp_receive(can_frame_t *f);

#endif /* MCP2515_H */
