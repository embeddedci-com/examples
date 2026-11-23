/* NUCLEO-H753ZI board support, bare metal on CMSIS.
 *
 * Clocks: the CPU stays on HSI (64 MHz after reset). USB takes HSI48, trimmed by the CRS from
 * the host's start-of-frame packets, so no crystal is needed. Console: USART3 on PD8/PD9, the
 * ST-LINK virtual COM port, 115200 8N1. USB: OTG_FS (USB2_OTG_FS) on PA11/PA12 = CN13, VBUS
 * sense on PA9. */
#include "board.h"

#include <stdarg.h>
#include <stdio.h>

#include "stm32h7xx.h"
#include "tusb.h"

#define HCLK_HZ 64000000u
#define ROM_BOOTLOADER 0x1FF09800u /* AN2606: STM32H74x/75x system memory bootloader */

static volatile uint32_t s_ms;
void SysTick_Handler(void) { s_ms++; }
uint32_t millis(void) { return s_ms; }
uint32_t tusb_time_millis_api(void) { return s_ms; }
void delay_ms(uint32_t ms)
{
    uint32_t t0 = s_ms;
    while (s_ms - t0 < ms) {
    }
}

static void gpio_mode(GPIO_TypeDef *g, unsigned pin, unsigned mode, unsigned af, unsigned speed)
{
    g->MODER = (g->MODER & ~(3u << (pin * 2))) | (mode << (pin * 2));
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (pin * 2))) | (speed << (pin * 2));
    g->PUPDR &= ~(3u << (pin * 2));
    volatile uint32_t *afr = &g->AFR[pin >> 3];
    unsigned sh = (pin & 7u) * 4u;
    *afr = (*afr & ~(0xFu << sh)) | (af << sh);
}
static void pin_set(GPIO_TypeDef *g, unsigned pin, bool on) { g->BSRR = on ? (1u << pin) : (1u << (pin + 16)); }

void led_green(bool on) { pin_set(GPIOB, 0, on); }
void led_yellow(bool on) { pin_set(GPIOE, 1, on); }
void led_red(bool on) { pin_set(GPIOB, 14, on); }
bool button_pressed(void) { return (GPIOC->IDR & (1u << 13)) != 0; }
bool vbus_present(void) { return (GPIOA->IDR & (1u << 9)) != 0; }

/* ---- console */
static volatile uint8_t s_rx[256];
static volatile uint32_t s_rx_head, s_rx_tail;

void USART3_IRQHandler(void)
{
    uint32_t isr = USART3->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    while (USART3->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t c = (uint8_t)(USART3->RDR & 0xFF);
        uint32_t next = (s_rx_head + 1u) % sizeof(s_rx);
        if (next != s_rx_tail) {
            s_rx[s_rx_head] = c;
            s_rx_head = next;
        }
    }
}

int con_getc(void)
{
    if (s_rx_tail == s_rx_head) {
        return -1;
    }
    int c = s_rx[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1u) % sizeof(s_rx);
    return c;
}

static void uart_putc(char c)
{
    while (!(USART3->ISR & USART_ISR_TXE_TXFNF)) {
    }
    USART3->TDR = (uint8_t)c;
}

void con_write(const char *s)
{
    for (; *s; s++) {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s);
    }
}

void con_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    con_write(buf);
}

/* ---- init */
void board_init(void)
{
    /* The Nucleo runs the core from the LDO. */
    PWR->CR3 = (PWR->CR3 & ~(PWR_CR3_SCUEN | PWR_CR3_BYPASS)) | PWR_CR3_LDOEN;
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY)) {
    }

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN |
                    RCC_AHB4ENR_GPIOEEN;
    (void)RCC->AHB4ENR;

    SysTick->LOAD = HCLK_HZ / 1000u - 1u;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    gpio_mode(GPIOB, 0, 1, 0, 0);
    gpio_mode(GPIOE, 1, 1, 0, 0);
    gpio_mode(GPIOB, 14, 1, 0, 0);
    gpio_mode(GPIOC, 13, 0, 0, 0); /* B1, active high, external pull-down */
    gpio_mode(GPIOA, 9, 0, 0, 0);  /* VBUS sense */

    RCC->APB1LENR |= RCC_APB1LENR_USART3EN;
    (void)RCC->APB1LENR;
    gpio_mode(GPIOD, 8, 2, 7, 1);
    gpio_mode(GPIOD, 9, 2, 7, 1);
    USART3->BRR = HCLK_HZ / 115200u; /* kernel clock = PCLK1 = 64 MHz */
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
    NVIC_SetPriority(USART3_IRQn, 2);
    NVIC_EnableIRQ(USART3_IRQn);
}

void crs_state(unsigned *trim, bool *synced)
{
    *trim = (CRS->CR & CRS_CR_TRIM) >> CRS_CR_TRIM_Pos;
    *synced = (CRS->ISR & CRS_ISR_SYNCOKF) != 0;
    CRS->ICR = CRS_ICR_SYNCOKC;
}

void OTG_FS_IRQHandler(void) { tusb_int_handler(0, true); }

void usb_hw_init(void)
{
    /* HSI48 for the USB kernel clock, trimmed by the CRS from the host's SOF (USB2 = OTG_FS). */
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY)) {
    }
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL) | (3u << RCC_D2CCIP2R_USBSEL_Pos);
    RCC->APB1HENR |= RCC_APB1HENR_CRSEN;
    (void)RCC->APB1HENR;
    CRS->CFGR = (CRS->CFGR & ~CRS_CFGR_SYNCSRC) | (3u << CRS_CFGR_SYNCSRC_Pos);
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    /* The USB transceiver's 3.3 V regulator detector. */
    PWR->CR3 |= PWR_CR3_USB33DEN;
    while (!(PWR->CR3 & PWR_CR3_USB33RDY)) {
    }

    gpio_mode(GPIOA, 11, 2, 10, 3);
    gpio_mode(GPIOA, 12, 2, 10, 3);
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    NVIC_SetPriority(OTG_FS_IRQn, 1);
}

void usb_hw_deinit(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    RCC->AHB1RSTR |= RCC_AHB1RSTR_USB2OTGFSRST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB2OTGFSRST;
    RCC->AHB1ENR &= ~RCC_AHB1ENR_USB2OTGFSEN;
    gpio_mode(GPIOA, 11, 0, 0, 0);
    gpio_mode(GPIOA, 12, 0, 0, 0);
}

void board_reset(void)
{
    delay_ms(20); /* let the console drain */
    NVIC_SystemReset();
    for (;;) {
    }
}

/* Start ST's ROM bootloader (USB DFU / DfuSe on this port) as if BOOT0 had been high at reset. */
void board_enter_rom_dfu(void)
{
    delay_ms(20);
    usb_hw_deinit();
    __disable_irq();
    SysTick->CTRL = 0;
    USART3->CR1 = 0;
    RCC->APB1LRSTR |= RCC_APB1LRSTR_USART3RST;
    RCC->APB1LRSTR &= ~RCC_APB1LRSTR_USART3RST;
    for (unsigned i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SCB->VTOR = ROM_BOOTLOADER;
    uint32_t sp = *(volatile uint32_t *)ROM_BOOTLOADER;
    uint32_t pc = *(volatile uint32_t *)(ROM_BOOTLOADER + 4u);
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))pc)();
    for (;;) {
    }
}
