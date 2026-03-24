/*
 * STM32 selftest with UART polling + line-based commands
 * GPIO: PC10
 *
 * Commands:
 *   help
 *   ping
 *   pc10 on
 *   pc10 off
 *   pc10 toggle
 *   status
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* =========================
 * Globals
 * ========================= */

#define CMD_BUF_SIZE 64

static volatile char cmd_buffer[CMD_BUF_SIZE];
static volatile uint8_t cmd_index = 0;
static volatile uint8_t cmd_ready = 0;

static uint8_t rx_byte;
static volatile uint32_t rx_byte_count = 0;
static volatile uint32_t command_count = 0;

/* ========================= */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void USART1_Init(void);
static int usart1_read_byte_nonblocking(uint8_t *byte);
static void uart_process_rx_byte(uint8_t byte);
static void print_prompt(void);
static void process_command(char *cmd);
static void print_help(void);
static void print_status(void);

/* =========================
 * printf -> UART
 * ========================= */

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
        }
        USART1->DR = (uint8_t)ptr[i];
    }
    return len;
}

/* =========================
 * Main
 * ========================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    USART1_Init();

    HAL_Delay(50);

    printf("\r\nSELFTEST: stm32 interactive GPIO test\r\n");
    printf("SELFTEST: build=%s %s\r\n", __DATE__, __TIME__);
    printf("SELFTEST: cpu=%lu Hz\r\n", HAL_RCC_GetHCLKFreq());

    printf("APP_OK\r\n");
    printf("type a command and press Enter (e.g. help)\r\n");
    print_prompt();

    while (1)
    {
        while (usart1_read_byte_nonblocking(&rx_byte))
        {
            uart_process_rx_byte(rx_byte);
        }

        if (cmd_ready)
        {
            cmd_ready = 0;
            process_command((char *)cmd_buffer);
            memset((void *)cmd_buffer, 0, CMD_BUF_SIZE);
            cmd_index = 0;
            print_prompt();
        }

        __WFI();  /* wakes on SysTick ~1ms */
    }
}

/* =========================
 * UART RX polling
 * ========================= */

static void print_prompt(void)
{
    printf("> ");
}

static int usart1_read_byte_nonblocking(uint8_t *byte)
{
    if ((USART1->SR & USART_SR_RXNE) != 0U)
    {
        *byte = (uint8_t)(USART1->DR & 0xFFU);
        return 1;
    }
    return 0;
}

static void uart_process_rx_byte(uint8_t byte)
{
    rx_byte_count++;

    if (byte == '\r' || byte == '\n')
    {
        if (cmd_index > 0)
        {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
        }
    }
    else if (cmd_index < CMD_BUF_SIZE - 1)
    {
        cmd_buffer[cmd_index++] = (char)byte;
    }
}

/* =========================
 * Command handling
 * ========================= */

static void process_command(char *cmd)
{
    command_count++;

    if (strcmp(cmd, "help") == 0)
    {
        print_help();
    }
    else if (strcmp(cmd, "ping") == 0)
    {
        printf("pong\r\n");
    }
    else if (strcmp(cmd, "pc10 on") == 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
        printf("PC10=ON\r\n");
    }
    else if (strcmp(cmd, "pc10 off") == 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        printf("PC10=OFF\r\n");
    }
    else if (strcmp(cmd, "pc10 toggle") == 0)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
        printf("PC10=TOGGLED\r\n");
    }
    else if (strcmp(cmd, "status") == 0)
    {
        print_status();
    }
    else
    {
        printf("unknown: %s\r\n", cmd);
    }
}

static void print_help(void)
{
    printf("commands:\r\n");
    printf("  help\r\n");
    printf("  ping\r\n");
    printf("  pc10 on\r\n");
    printf("  pc10 off\r\n");
    printf("  pc10 toggle\r\n");
    printf("  status\r\n");
}

static void print_status(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);

    printf("status:\r\n");
    printf("  pc10=%s\r\n", (state == GPIO_PIN_SET) ? "ON" : "OFF");
    printf("  rx_byte_count=%lu\r\n", (unsigned long)rx_byte_count);
    printf("  command_count=%lu\r\n", (unsigned long)command_count);
    printf("  tick=%lu ms\r\n", (unsigned long)HAL_GetTick());
}

/* =========================
 * GPIO (PC10)
 * ========================= */

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
}

/* =========================
 * UART (USART1 PA9/PA10) - register-level init
 * ========================= */

static void USART1_Init(void)
{
    /* 1) Enable clocks for GPIOA and USART1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    /* 2) PA9 (TX), PA10 (RX) -> Alternate Function mode */
    GPIOA->MODER &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->MODER |= (2U << (9U * 2U)) | (2U << (10U * 2U));

    /* PA9 push-pull, high speed */
    GPIOA->OTYPER &= ~(1U << 9U);
    GPIOA->OSPEEDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->OSPEEDR |= (2U << (9U * 2U)) | (2U << (10U * 2U));

    /* PA10 pull-up (RX idle high) */
    GPIOA->PUPDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->PUPDR |= (1U << (10U * 2U));

    /* AF7 for USART1 on PA9/PA10 */
    GPIOA->AFR[1] &= ~((0xFU << ((9U - 8U) * 4U)) | (0xFU << ((10U - 8U) * 4U)));
    GPIOA->AFR[1] |= (7U << ((9U - 8U) * 4U)) | (7U << ((10U - 8U) * 4U));

    /* 3) USART1 config: 115200 @ 16 MHz, 8N1, polling */
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    /* BRR = 16,000,000 / 115,200 = 138.888... -> 0x008B (mantissa 8, fraction 11) */
    USART1->BRR = 0x008BU;

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

/* =========================
 * Clock
 * ========================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}
