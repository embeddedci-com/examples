#include "mcp2515.h"
#include "mcp2515_timing.h"

#define MCP_RESET_SETTLE_MS 5U
#define MCP_MODE_TIMEOUT_MS 50U

uint8_t mcp_read_reg(uint8_t addr)
{
    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_READ);
    (void)mcp_board_xfer(addr);
    uint8_t v = mcp_board_xfer(0x00U);
    mcp_board_select(0);
    return v;
}

void mcp_write_reg(uint8_t addr, uint8_t val)
{
    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_WRITE);
    (void)mcp_board_xfer(addr);
    (void)mcp_board_xfer(val);
    mcp_board_select(0);
}

void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t val)
{
    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_BIT_MODIFY);
    (void)mcp_board_xfer(addr);
    (void)mcp_board_xfer(mask);
    (void)mcp_board_xfer(val);
    mcp_board_select(0);
}

static void wait_ms(uint32_t ms)
{
    uint32_t start = mcp_board_millis();
    while ((mcp_board_millis() - start) < ms)
    {
    }
}

int mcp_reset(void)
{
    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_RESET);
    mcp_board_select(0);
    wait_ms(MCP_RESET_SETTLE_MS);
    /* After reset the chip sits in configuration mode: CANSTAT.OPMOD = 100. A
     * floating MISO reads 0xFF and a missing chip 0x00, so this is a presence test. */
    return ((mcp_read_reg(MCP_CANSTAT) & 0xE0U) == MCP_MODE_CONFIG) ? 0 : -1;
}

int mcp_configure(uint32_t osc_hz, uint32_t bitrate)
{
    mcp2515_timing_t t;
    if (mcp2515_calc_timing(osc_hz, bitrate, &t) != 0)
    {
        return -1;
    }
    if (mcp_set_mode(MCP_MODE_CONFIG) != 0)
    {
        return -2;
    }
    mcp_write_reg(MCP_CNF1, t.cnf1);
    mcp_write_reg(MCP_CNF2, t.cnf2);
    mcp_write_reg(MCP_CNF3, t.cnf3);
    /* RXM = 11: turn masks/filters off and accept every frame; BUKT: roll a
     * frame over into RXB1 when RXB0 is still full. */
    mcp_write_reg(MCP_RXB0CTRL, 0x60U | 0x04U);
    mcp_write_reg(MCP_RXB1CTRL, 0x60U);
    mcp_write_reg(MCP_CANINTF, 0x00U);
    mcp_write_reg(MCP_EFLG, 0x00U);
    mcp_write_reg(MCP_CANINTE, MCP_INT_RX0 | MCP_INT_RX1 | MCP_INT_ERR);
    return 0;
}

int mcp_set_mode(mcp_mode_t mode)
{
    mcp_bit_modify(MCP_CANCTRL, 0xE0U, (uint8_t)mode);
    uint32_t start = mcp_board_millis();
    while ((mcp_board_millis() - start) < MCP_MODE_TIMEOUT_MS)
    {
        if ((mcp_read_reg(MCP_CANSTAT) & 0xE0U) == (uint8_t)mode)
        {
            return 0;
        }
    }
    return -1;
}

mcp_mode_t mcp_get_mode(void)
{
    return (mcp_mode_t)(mcp_read_reg(MCP_CANSTAT) & 0xE0U);
}

mcp_tx_result_t mcp_send(const can_frame_t *f, uint32_t timeout_ms)
{
    uint8_t hdr[5];
    uint8_t dlc = (f->dlc > 8U) ? 8U : f->dlc;
    if (f->ext)
    {
        uint32_t id = f->id & 0x1FFFFFFFU;
        hdr[0] = (uint8_t)(id >> 21);
        hdr[1] = (uint8_t)((((id >> 18) & 0x07U) << 5) | 0x08U | ((id >> 16) & 0x03U));
        hdr[2] = (uint8_t)(id >> 8);
        hdr[3] = (uint8_t)id;
    }
    else
    {
        uint32_t id = f->id & 0x7FFU;
        hdr[0] = (uint8_t)(id >> 3);
        hdr[1] = (uint8_t)((id & 0x07U) << 5);
        hdr[2] = 0U;
        hdr[3] = 0U;
    }
    hdr[4] = (uint8_t)((f->rtr ? 0x40U : 0x00U) | dlc);

    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_LOAD_TX0);
    for (uint32_t i = 0; i < 5U; i++)
    {
        (void)mcp_board_xfer(hdr[i]);
    }
    for (uint32_t i = 0; i < dlc; i++)
    {
        (void)mcp_board_xfer(f->data[i]);
    }
    mcp_board_select(0);

    mcp_board_select(1);
    (void)mcp_board_xfer(MCP_RTS_TX0);
    mcp_board_select(0);

    uint32_t start = mcp_board_millis();
    for (;;)
    {
        uint8_t ctrl = mcp_read_reg(MCP_TXB0CTRL);
        if ((ctrl & MCP_TXB_TXREQ) == 0U)
        {
            return (ctrl & (MCP_TXB_TXERR | MCP_TXB_MLOA | MCP_TXB_ABTF)) ? MCP_TX_ERROR : MCP_TX_OK;
        }
        if ((mcp_board_millis() - start) >= timeout_ms)
        {
            /* Nobody ACKed (or we are bus-off): the controller would retransmit
             * forever, so abort the request and report it. */
            mcp_bit_modify(MCP_TXB0CTRL, MCP_TXB_TXREQ, 0x00U);
            return MCP_TX_TIMEOUT;
        }
    }
}

static void read_rx_buffer(uint8_t instr, can_frame_t *f)
{
    uint8_t hdr[5];
    mcp_board_select(1);
    (void)mcp_board_xfer(instr);
    for (uint32_t i = 0; i < 5U; i++)
    {
        hdr[i] = mcp_board_xfer(0x00U);
    }
    uint8_t dlc = hdr[4] & 0x0FU;
    if (dlc > 8U)
    {
        dlc = 8U;
    }
    for (uint32_t i = 0; i < dlc; i++)
    {
        f->data[i] = mcp_board_xfer(0x00U);
    }
    mcp_board_select(0); /* raising CS clears the RXnIF flag */

    f->dlc = dlc;
    f->ext = (hdr[1] & 0x08U) ? 1U : 0U;
    if (f->ext)
    {
        f->id = ((uint32_t)hdr[0] << 21) | ((uint32_t)(hdr[1] >> 5) << 18) |
                ((uint32_t)(hdr[1] & 0x03U) << 16) | ((uint32_t)hdr[2] << 8) | hdr[3];
        f->rtr = (hdr[4] & 0x40U) ? 1U : 0U;
    }
    else
    {
        f->id = ((uint32_t)hdr[0] << 3) | (hdr[1] >> 5);
        f->rtr = (hdr[1] & 0x10U) ? 1U : 0U; /* SRR doubles as RTR for standard frames */
    }
}

int mcp_receive(can_frame_t *f)
{
    uint8_t intf = mcp_read_reg(MCP_CANINTF);
    if (intf & MCP_INT_RX0)
    {
        read_rx_buffer(MCP_READ_RX0, f);
        return 1;
    }
    if (intf & MCP_INT_RX1)
    {
        read_rx_buffer(MCP_READ_RX1, f);
        return 1;
    }
    return 0;
}
