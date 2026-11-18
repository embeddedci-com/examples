/*
 * rl78emu_port.c - runs the RL78 boot ROM emulator (rl78emu.c) on the node's console pins.
 *
 * Entry, as on an RL78: the programmer holds TOOL0 low while it releases reset. Here TOOL0 is
 * the console TX, PA9 (the bench's LA3). When main() finds PA9 low right after reset, it calls
 * rl78emu_main() instead of starting the node: the emulator waits for the line to be released,
 * then runs USART1 in single-wire half-duplex mode on PA9 (open drain, the programmer's pull-up
 * holds it high), 115200 8N1 until Baud Rate Set, and answers Protocol A frames until the next
 * reset. The watchdog is not started, so a long idle programming session is fine.
 */
#include "rl78emu.h"

#include "dwt.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define PA9 9U
#define IDLE_DROP_MS 50U     /* a frame that stalls this long is dropped */

/* 68 KB with the emulated flash: kept out of .bss, whose zeroing at startup took ~5 ms at 16 MHz,
   longer than a programmer holds TOOL0 low after reset (rl78emu_init fills it anyway) */
__attribute__((section(".noinit"))) static rl78emu_t s_emu;
static uint32_t s_bit_ns = 8681U;

bool rl78emu_strap(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;
    GPIOA->MODER &= ~(3U << (PA9 * 2U));     /* input */
    /* The internal pull-up while sampling: an undriven LA3 floats (the pod's pull-up is on only
       during a UART session), and a floating pin must not start the emulator. A programmer
       holding TOOL0 low overrides it easily. */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U << (PA9 * 2U))) | (1U << (PA9 * 2U));
    for (volatile int i = 0; i < 200; i++)   /* ~50 us to settle */
    {
    }
    bool low = (GPIOA->IDR & (1U << PA9)) == 0U;
    GPIOA->PUPDR &= ~(3U << (PA9 * 2U));
    return low;
}

static uint32_t brr_for(uint32_t baud)
{
    /* 16 MHz, oversampling by 16: 115200 -> 8.6875 (0x8B), 250k/500k/1M divide exactly */
    return (SystemCoreClock + baud / 2U) / baud;
}

static void port_baud(void *ctx, uint32_t baud)
{
    (void)ctx;
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR = brr_for(baud);
    USART1->CR1 |= USART_CR1_UE;
    s_bit_ns = 1000000000U / baud;
}

static void port_send(void *ctx, const uint8_t *buf, size_t len)
{
    (void)ctx;
    /* The programmer sends 2 stop bits and, on a shared wire, cannot hear a reply that starts
       before its own frame has ended. RXNE comes in the middle of the first stop bit, so wait
       out the rest of both, plus a short turnaround, as the RL78 itself takes tens of us. */
    dwt_delay_us((s_bit_ns * 2U) / 1000U + 20U);
    /* our own bytes come back on RX in half-duplex mode: keep the receiver off while sending */
    USART1->CR1 &= ~USART_CR1_RE;
    for (size_t i = 0; i < len; i++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
        }
        USART1->DR = buf[i];
    }
    while ((USART1->SR & USART_SR_TC) == 0U)
    {
    }
    (void)USART1->SR;
    (void)USART1->DR;
    USART1->CR1 |= USART_CR1_RE;
}

void rl78emu_main(void)
{
    dwt_init();
    /* A fresh part, set up while the programmer still holds TOOL0 low: ~5.6 ms at 16 MHz, so the
       programmer must hold it at least that long after the reset (the real part needs 723 us +
       tHD; tests/rl78_programmer.py holds 10 ms). */
    const rl78emu_port_t port = {port_send, port_baud, NULL};
    rl78emu_init(&s_emu, &port);
    /* the programmer releases TOOL0 after the reset: wait for it */
    while ((GPIOA->IDR & (1U << PA9)) == 0U)
    {
    }

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFU << ((PA9 - 8U) * 4U))) | (7U << ((PA9 - 8U) * 4U));
    GPIOA->OTYPER |= (1U << PA9);                                   /* open drain */
    /* and the internal pull-up: the programmer's pull-up may drop out while it reopens its UART
       at a new baud rate, and a floating TOOL0 decodes as noise */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U << (PA9 * 2U))) | (1U << (PA9 * 2U));
    GPIOA->OSPEEDR |= (3U << (PA9 * 2U));
    GPIOA->MODER = (GPIOA->MODER & ~(3U << (PA9 * 2U))) | (2U << (PA9 * 2U));

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;                         /* 1 stop bit out; 2-stop frames are read fine */
    USART1->CR3 = USART_CR3_HDSEL;            /* single wire: TX and RX on PA9 */
    USART1->BRR = brr_for(115200U);
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    uint32_t last = HAL_GetTick();
    for (;;)
    {
        uint32_t sr = USART1->SR;
        if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
        {
            (void)USART1->DR;                 /* clears the error; the frame's checksum fails */
            continue;
        }
        if ((sr & USART_SR_RXNE) != 0U)
        {
            rl78emu_rx(&s_emu, (uint8_t)USART1->DR);
            last = HAL_GetTick();
        }
        else if (HAL_GetTick() - last > IDLE_DROP_MS)
        {
            rl78emu_idle(&s_emu);
            last = HAL_GetTick();
        }
    }
}
