/*
 * STM32 selftest with UART wakeup + line-based commands
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

UART_HandleTypeDef huart2;

#define CMD_BUF_SIZE 64

static volatile char cmd_buffer[CMD_BUF_SIZE];
static volatile uint8_t cmd_index = 0;
static volatile uint8_t cmd_ready = 0;

static uint8_t rx_byte;
static volatile uint32_t wake_count = 0;
static volatile uint32_t command_count = 0;

/* ========================= */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

static void uart_start_rx_interrupt(void);
static void uart_arm_rx(void);
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
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
    MX_USART2_UART_Init();

    HAL_Delay(50);

    printf("\r\nSELFTEST: stm32 interactive GPIO test\r\n");
    printf("SELFTEST: build=%s %s\r\n", __DATE__, __TIME__);
    printf("SELFTEST: cpu=%lu Hz\r\n", HAL_RCC_GetHCLKFreq());

    uart_start_rx_interrupt();

    printf("APP_OK\r\n");
    printf("type a command and press Enter (e.g. help)\r\n");
    print_prompt();

    while (1)
    {
        if (cmd_ready)
        {
            cmd_ready = 0;
            process_command((char *)cmd_buffer);
            memset((void *)cmd_buffer, 0, CMD_BUF_SIZE);
            cmd_index = 0;
            print_prompt();
        }

        __WFI();  /* wakes on SysTick ~1ms and on USART2 RX */
    }
}

/* =========================
 * UART RX interrupt
 * ========================= */

static void uart_arm_rx(void)
{
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

static void uart_start_rx_interrupt(void)
{
    uart_arm_rx();
}

static void print_prompt(void)
{
    printf("> ");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        wake_count++;

        if (rx_byte == '\r' || rx_byte == '\n')
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                cmd_ready = 1;
            }
        }
        else if (cmd_index < CMD_BUF_SIZE - 1)
        {
            cmd_buffer[cmd_index++] = rx_byte;
        }

        uart_arm_rx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    /* Overrun/noise can leave UART in error state and stop RX until cleared. */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    uart_arm_rx();
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
    printf("  wake_count=%lu\r\n", (unsigned long)wake_count);
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
 * UART (USART2 PA2/PA3)
 * ========================= */

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
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
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
