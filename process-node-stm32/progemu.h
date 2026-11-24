/*
 * progemu.h - chip emulators for the BenchPod's pin programmers (`prog_flash`): the NUCLEO-F446RE
 * answers as an AVR (UPDI, TPI, XMEGA PDI), a PIC (ICSP, both command sets) or a Silicon Labs C2
 * part, so the pod's programming protocols run against something that is not their own host test.
 *
 * The chip models (progemu_<proto>.c) are the bit-level models of benchpod-firmware's
 * stm32h563/test/test_prog_<proto>.c, ported to this board. They are plain C: no HAL, no
 * interrupts. They take their time from pe_now, a cycle stamp the caller sets before each call, so
 * the same code runs in the interrupt handlers of progemu.c and in a host harness
 * (tests/test_progemu_host.c).
 *
 * Wires (README): two-wire protocols read the clock on PB14 (EXTI, both edges) and drive data on
 * PB12; PIC MCLR and TPI RESET come in on PB0; UPDI is USART3 on PC10 (half duplex).
 */
#ifndef PROGEMU_H
#define PROGEMU_H

#include <stdint.h>

/* The cycle counter at the event being handled, and the cycles in one microsecond. */
extern uint32_t pe_now;
extern uint32_t pe_cyc_us;
/* Diagnostic knob (`prog_emu pic skew <n>`): the PIC read frame is emitted n clocks early (n > 0) or
   late. 0 is the spec. */
extern int pe_skew;

/* RAM the running chip model keeps its state in. Only one runs at a time, so they share it. */
#define PE_POOL_BYTES 24576U
extern uint32_t pe_pool[PE_POOL_BYTES / 4U];

/* Deferred memory fills. A model that must clear a large region from the edge handler (an erase)
   calls these; progemu.c queues the work and does it in slices while the polling loop idles between
   edges, so a 20 KB erase does not make the handler late. The host harness fills at once. A model
   counts the chip as busy while pe_fill_pending() is nonzero. */
void pe_fill8(uint8_t *p, uint8_t v, uint32_t n);
void pe_fill16(uint16_t *p, uint16_t v, uint32_t n);
int  pe_fill_pending(void);

/* One emulated memory: its size and a reader (dump, CRC32). */
typedef struct {
    const char *name;
    uint32_t    size;
    void      (*read)(uint32_t addr, uint8_t *buf, uint32_t n);
} pe_mem_t;

/* Counters every model keeps, read by `prog_emu stat`. */
typedef struct {
    uint32_t frames;     /* commands or frames the chip took */
    uint32_t errors;     /* things a real chip would not accept: bad frames, early commands ... */
    uint32_t timing;     /* waits the protocol must observe that were too short (a subset of errors) */
} pe_counts_t;

typedef struct {
    const char *name;        /* console name: "pic", "pic8", ... */
    const char *what;        /* the chip in one line */
    uint8_t     uart;        /* 1: one wire over USART3 (UPDI); 0: clock + data */
    uint8_t     reset;       /* 1: PB0 is the chip's reset (PIC MCLR, TPI RESET) */
    void      (*init)(void); /* a fresh chip: memories erased, unlocked */
    void      (*lock)(void); /* lock it (code protection, lock bits) */
    void      (*stat)(void); /* the protocol's own counters, printed after the common ones */
    const pe_counts_t *(*counts)(void);
    const pe_mem_t *mem;
    unsigned    nmem;
    /* two-wire: the clock changed to `clk`; `pod` is the level the pod drives data to, -1 when it
       does not drive (the chip does, or nobody: a pull-up holds 1) */
    void      (*edge)(int clk, int pod);
    int       (*drive)(void);          /* what the chip drives on data now: 0, 1, or -1 (released) */
    /* two-wire, optional: what the chip will drive once the clock goes to `clk`, assuming the pod does
       nothing else (the pin is switched to it before the model runs, to meet the chip's data valid time) */
    int       (*peek)(int clk);
    void      (*nrst)(int low);        /* reset pin changed */
    const uint8_t *data_want;          /* optional: nonzero while the data-line interrupt is needed */
    void      (*data)(int level);      /* optional: the pod changed data while not driven by the chip (EXTI on PB12) */
    /* UPDI: the host side calls these */
    void      (*rx)(uint8_t b);        /* a byte after SYNC */
    void      (*sync)(void);           /* a SYNC was measured */
    void      (*brk)(void);            /* BREAK */
    void      (*pulse)(void);          /* a low pulse (the enable pulse) ended */
    int       (*sync_gate)(void);      /* a frame starts: 1 when the UPDI takes it (enabled, SYNC in its window) */
    int       (*want_sync)(void);      /* 1: the next frame starts with SYNC */
} pe_chip_t;

/* The chips (each progemu_<proto>.c). */
extern const pe_chip_t pe_updi, pe_updi2, pe_updimega, pe_pic, pe_pic8, pe_pic18, pe_pic18q10, pe_c2, pe_c2early, pe_tpi, pe_tpi20, pe_tpi40, pe_tpi104, pe_pdi;

/* UPDI model: queue bytes the chip sends (progemu.c sends them over USART3, the host harness
   puts them in the pod's receive queue). */
void pe_updi_tx(const uint8_t *b, unsigned n);
/* The next reply byte is not sent before this cycle count (an ACK held until a write is done). */
void pe_updi_hold(uint32_t until);
/* us after a BREAK's low ends until its frame is over (2 stop bits at 300 baud); the host harness sets it. */
extern uint32_t pe_break_frame_us;

/* USART3 interrupt hook (uart3.c calls it first): 1 when the UPDI emulator took the interrupt. */
int  progemu_usart3_irq(void);

/* Console (progemu.c): `prog_emu ...`; 1 when the command was one of ours. */
int  progemu_command(char *cmd);
void progemu_help(void);
/* 1 while an emulator runs (the USART3 and PB12/PB14/PB0 users must stay off). */
int  progemu_active(void);

#endif
