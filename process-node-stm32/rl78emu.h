/*
 * rl78emu.h - an emulated RL78 boot ROM speaking RL78 Protocol A (Renesas R01AN0815, "RL78
 * Protocol A Serial Programming Guide"), so a programmer can be tested without an RL78.
 *
 * The node enters it when its console TX (PA9, the bench's LA3) is held low through reset, as
 * an RL78's TOOL0 is: the line then carries the single-wire programming UART instead of the
 * console. The emulated part is an R5F100LE (RL78/G13): 64 KB code flash at 000000H, 4 KB data
 * flash at 0F1000H, 1 KB blocks, held in RAM (power off erases it, as a fresh part).
 *
 * This file is the protocol only (no HAL): bytes in through rl78emu_rx(), frames out through the
 * port's send callback. main.c's port drives USART1; tests/test_rl78emu.c drives it on the host.
 */
#ifndef RL78EMU_H
#define RL78EMU_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define RL78_SOH 0x01u
#define RL78_STX 0x02u
#define RL78_ETX 0x03u
#define RL78_ETB 0x17u

/* Status codes (Table 1-4). */
#define RL78_ST_CMD_ERR    0x04u
#define RL78_ST_PARAM_ERR  0x05u
#define RL78_ST_ACK        0x06u
#define RL78_ST_SUM_ERR    0x07u
#define RL78_ST_VERIFY_ERR 0x0Fu
#define RL78_ST_PROTECT    0x10u
#define RL78_ST_NACK       0x15u
#define RL78_ST_ERASE_ERR  0x1Au
#define RL78_ST_BLANK_ERR  0x1Bu   /* not blank, or internal verify failed */
#define RL78_ST_WRITE_ERR  0x1Cu

/* Commands. */
#define RL78_CMD_RESET      0x00u
#define RL78_CMD_VERIFY     0x13u
#define RL78_CMD_ERASE      0x22u
#define RL78_CMD_BLANK      0x32u
#define RL78_CMD_PROGRAM    0x40u
#define RL78_CMD_BAUD       0x9Au
#define RL78_CMD_SEC_SET    0xA0u
#define RL78_CMD_SEC_GET    0xA1u
#define RL78_CMD_SEC_RELEASE 0xA2u
#define RL78_CMD_CHECKSUM   0xB0u
#define RL78_CMD_SIGNATURE  0xC0u

/* The emulated part (R5F100LE). */
#define RL78_BLOCK         1024u
#define RL78_CODE_START    0x000000u
#define RL78_CODE_END      0x00FFFFu
#define RL78_DATA_START    0x0F1000u
#define RL78_DATA_END      0x0F1FFFu
#define RL78_CODE_SIZE     (RL78_CODE_END - RL78_CODE_START + 1u)
#define RL78_DATA_SIZE     (RL78_DATA_END - RL78_DATA_START + 1u)
#define RL78_LAST_BLOCK    (RL78_CODE_SIZE / RL78_BLOCK - 1u)   /* code flash blocks 0..63 */
#define RL78_BOOT_BLOCKS   0x03u                                /* boot cluster end block (4 KB) */
#define RL78_FREQ_MHZ      32u                                  /* Baud Rate Set reply D01 */

/* Security flags (Security Set/Get FLG). Bits 7, 6, 5, 3 are fixed to 1. */
#define RL78_FLG_FIXED     0xE8u
#define RL78_FLG_PROG_EN   0x10u
#define RL78_FLG_ERASE_EN  0x04u
#define RL78_FLG_BOOT_EN   0x02u

typedef struct {
    void (*send)(void *ctx, const uint8_t *buf, size_t len);   /* one whole frame */
    void (*set_baud)(void *ctx, uint32_t baud);                 /* after the reply went out */
    void *ctx;
} rl78emu_port_t;

typedef enum { RL78_WAIT_CMD, RL78_WAIT_PROG, RL78_WAIT_VERIFY, RL78_WAIT_SEC } rl78_phase_t;

typedef struct {
    rl78emu_port_t port;
    /* frame parser */
    uint8_t  pstate, hdr, len_byte, sum;
    uint16_t want, got;
    uint8_t  buf[256];
    /* session */
    bool     baud_set;
    uint32_t baud;
    rl78_phase_t phase;
    uint32_t cur, end;          /* PROG / VERIFY cursor and last address */
    bool     mismatch;          /* VERIFY: a byte differed */
    bool     write_failed;      /* PROG: a frame was refused */
    /* the part */
    uint8_t  sec_flg, sec_bot;
    uint16_t sec_ss, sec_se;
    /* counters, for the console and the tests */
    uint32_t frames, errors;
    /* last, word-aligned: rl78emu_init fills them 32 bits at a time */
    uint8_t  code[RL78_CODE_SIZE] __attribute__((aligned(4)));
    uint8_t  data[RL78_DATA_SIZE] __attribute__((aligned(4)));
} rl78emu_t;

/* Fresh part: flash erased (FFH), security all enabled, 115200 until Baud Rate Set. ~3 ms at
   16 MHz (word-wise): the port runs it while the programmer still holds TOOL0 low. */
void rl78emu_init(rl78emu_t *e, const rl78emu_port_t *port);
/* One received byte. Frames are answered from here. */
void rl78emu_rx(rl78emu_t *e, uint8_t byte);
/* The line went idle mid-frame (the port's timeout): drop the partial frame. */
void rl78emu_idle(rl78emu_t *e);

/* MCU port (rl78emu_port.c). rl78emu_strap(): PA9 (TOOL0) is held low right after reset.
   rl78emu_main(): run the emulator on PA9 until the next reset; never returns. */
bool rl78emu_strap(void);
void rl78emu_main(void);

/* Helpers shared with the host tests. */
uint8_t  rl78_sum(const uint8_t *p, size_t n);
uint8_t *rl78emu_flash(rl78emu_t *e, uint32_t addr);   /* NULL outside flash */

#endif
