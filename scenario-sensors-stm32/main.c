/*
 * STM32 scenario app:
 * - USART1 interactive menu (line-buffering off for interactive echo)
 * - rotor outputs on PC10/PC11
 * - BMP280 on I2C1 (PB8/PB9), auto-detect 0x76/0x77
 * - VL53L0X on same bus (default addr 0x29) for bus validation
 */

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CMD_BUF_SIZE 64
#define BMP280_CHIP_ID 0x58U
#define BMP280_REG_CHIP_ID 0xD0U
#define BMP280_REG_RESET 0xE0U
#define BMP280_REG_STATUS 0xF3U
#define BMP280_REG_CTRL_MEAS 0xF4U
#define BMP280_REG_CONFIG 0xF5U
#define BMP280_REG_PRESS_MSB 0xF7U
#define BMP280_REG_CALIB_START 0x88U

#define BMP280_I2C_ADDR0 0x76U
#define BMP280_I2C_ADDR1 0x77U
#define BMP280_TIMEOUT_MS 100U
#define BMP280_SEA_LEVEL_PA 101325.0f

#define VL53L0X_I2C_ADDR_7B 0x29U
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0U
#define VL53L0X_MODEL_ID_EXPECTED 0xEEU

#define I2C_SCAN_ADDR_FIRST 0x08U
#define I2C_SCAN_ADDR_LAST 0x77U
#define I2C_ISREADY_TRIALS 3U
#define I2C_ISREADY_TIMEOUT_MS 10U

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;
} bmp280_calibration_t;

typedef struct
{
    I2C_HandleTypeDef hi2c1;
    uint8_t i2c_addr_7b;
    uint8_t detected;
    uint8_t initialized;
    uint32_t read_count;
    uint32_t read_fail_count;
    float temperature_c;
    float pressure_pa;
    float altitude_m;
} sensor_state_t;

typedef struct
{
    uint8_t detected;
    uint8_t last_model_id;
} vl53l0x_state_t;

static volatile char cmd_buffer[CMD_BUF_SIZE];
static volatile uint8_t cmd_index = 0;
static volatile uint8_t cmd_ready = 0;

static uint8_t rx_byte;
static volatile uint32_t rx_byte_count = 0;
static volatile uint32_t command_count = 0;
static uint8_t rx_last_was_cr = 0U;

static sensor_state_t g_sensor;
static bmp280_calibration_t g_calib;
static vl53l0x_state_t g_vl53;
static uint8_t g_i2c1_hw_inited = 0U;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void USART1_Init(void);
static void I2C1_Init(void);
static int i2c1_ensure_init(void);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);
static int usart1_read_byte_nonblocking(uint8_t *byte);
static void uart_process_rx_byte(uint8_t byte);
static void print_prompt(void);
static void process_command(char *cmd);
static void print_help(void);
static void print_status(void);
static void set_rotors(GPIO_PinState state);
static void monitor_i2c_until_key(void);
static void sensor_init_until_key(void);
static int bmp280_probe_and_init(uint8_t verbose);
static int bmp280_read_measurement(float *temperature_c, float *pressure_pa, float *altitude_m);
static int bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len, uint8_t verbose);
static int bmp280_write_reg(uint8_t reg, uint8_t value, uint8_t verbose);
static void i2c_dump_hal_context(const char *tag, HAL_StatusTypeDef st);
static void i2c_dump_hal_error_bits(uint32_t err);
static void i2c_bus_scan(void);
static int vl53l0x_read_model_id(uint8_t verbose);
static int sensor_i2c_deinit(void);
static const char *hal_status_to_str(HAL_StatusTypeDef status);
static int32_t bmp280_compensate_temp(int32_t adc_T);
static uint32_t bmp280_compensate_press(int32_t adc_P);
static float pressure_to_altitude_m(float pressure_pa);
static void fault_led_prepare(void);
static void fault_led_delay(volatile uint32_t cycles);
static void fault_led_blink_loop(uint32_t pulses) __attribute__((noreturn));

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

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    USART1_Init();
    (void)setvbuf(stdout, NULL, _IONBF, 0);

    memset(&g_sensor, 0, sizeof(g_sensor));
    memset(&g_vl53, 0, sizeof(g_vl53));
    I2C1_Init();
    (void)bmp280_probe_and_init(1U);
    printf("\r\nVL53L0X model ID read (boot):\r\n");
    (void)vl53l0x_read_model_id(1U);

    HAL_Delay(50);
    printf("\r\nSCENARIO: stm32 bmp280 + vl53l0x + rotor control\r\n");
    printf("SCENARIO: build=%s %s\r\n", __DATE__, __TIME__);
    printf("SCENARIO: cpu=%lu Hz\r\n", HAL_RCC_GetHCLKFreq());
    printf("SCENARIO: uart=USART1(115200), i2c=I2C1 PB8/PB9\r\n");
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

        __WFI();
    }
}

static void print_prompt(void)
{
    printf("> ");
    (void)fflush(stdout);
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

    if (byte == '\r')
    {
        rx_last_was_cr = 1U;
        printf("\r\n");
        if (cmd_index > 0U)
        {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
        }
        else
        {
            print_prompt();
        }
    }
    else if (byte == '\n')
    {
        if (rx_last_was_cr)
        {
            rx_last_was_cr = 0U;
            return;
        }
        printf("\r\n");
        if (cmd_index > 0U)
        {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
        }
        else
        {
            print_prompt();
        }
    }
    else if (byte == '\b' || byte == 0x7FU)
    {
        if (cmd_index > 0U)
        {
            cmd_index--;
            cmd_buffer[cmd_index] = '\0';
            printf("\b \b");
            (void)fflush(stdout);
        }
    }
    else if (byte >= 32U && byte <= 126U)
    {
        rx_last_was_cr = 0U;
        if (cmd_index < (CMD_BUF_SIZE - 1U))
        {
            cmd_buffer[cmd_index++] = (char)byte;
            printf("%c", (char)byte);
            (void)fflush(stdout);
        }
    }
}

static void process_command(char *cmd)
{
    command_count++;

    if (strcmp(cmd, "help") == 0)
    {
        print_help();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        print_status();
    }
    else if (strcmp(cmd, "rotors on") == 0 || strcmp(cmd, "pc10pc11 on") == 0)
    {
        set_rotors(GPIO_PIN_SET);
        printf("ROTORS=ON (PC10=ON, PC11=ON)\r\n");
    }
    else if (strcmp(cmd, "rotors off") == 0 || strcmp(cmd, "pc10pc11 off") == 0)
    {
        set_rotors(GPIO_PIN_RESET);
        printf("ROTORS=OFF (PC10=OFF, PC11=OFF)\r\n");
    }
    else if (strcmp(cmd, "monitor i2c") == 0)
    {
        monitor_i2c_until_key();
    }
    else if (strcmp(cmd, "sensor init") == 0)
    {
        sensor_init_until_key();
    }
    else if (strcmp(cmd, "i2c scan") == 0)
    {
        if (i2c1_ensure_init() == 0)
        {
            i2c_bus_scan();
        }
    }
    else if (strcmp(cmd, "vl53 test") == 0 || strcmp(cmd, "vl53l0x test") == 0)
    {
        if (i2c1_ensure_init() == 0)
        {
            (void)vl53l0x_read_model_id(1U);
        }
    }
    else if (strcmp(cmd, "sensor deinit") == 0)
    {
        (void)sensor_i2c_deinit();
        printf("I2C1 deinitialized; use \"sensor init\", \"i2c scan\", or \"vl53 test\" to run I2C1_Init() again.\r\n");
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
    printf("  status\r\n");
    printf("  rotors on\r\n");
    printf("  rotors off\r\n");
    printf("  monitor i2c   (reads BMP280 every 1s, press any key to stop)\r\n");
    printf("  sensor init   (BMP280 + VL53L0X, 500ms retry; I2C scan once at start; any key stops)\r\n");
    printf("  i2c scan      (HAL_I2C_IsDeviceReady 0x08-0x77, highlights 0x29)\r\n");
    printf("  vl53 test     (read VL53L0X model ID reg 0xC0; alias: vl53l0x test)\r\n");
    printf("  sensor deinit (HAL_I2C_DeInit + clear sensor state; re-init via sensor init / i2c scan / vl53 test)\r\n");
}

static void print_status(void)
{
    GPIO_PinState pc10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
    GPIO_PinState pc11 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
    int sensor_ok = (bmp280_read_measurement(&g_sensor.temperature_c, &g_sensor.pressure_pa, &g_sensor.altitude_m) == 0);
    if (sensor_ok)
    {
        g_sensor.read_count++;
    }
    else
    {
        g_sensor.read_fail_count++;
    }

    printf("status:\r\n");
    printf("  pc10=%s\r\n", (pc10 == GPIO_PIN_SET) ? "ON" : "OFF");
    printf("  pc11=%s\r\n", (pc11 == GPIO_PIN_SET) ? "ON" : "OFF");
    printf("  bmp280_detected=%s\r\n", g_sensor.detected ? "yes" : "no");
    if (g_sensor.detected)
    {
        printf("  bmp280_addr=0x%02X\r\n", g_sensor.i2c_addr_7b);
    }
    printf("  sensor_initialized=%s\r\n", g_sensor.initialized ? "yes" : "no");
    printf("  vl53l0x_detected=%s last_model_id=0x%02X\r\n",
           g_vl53.detected ? "yes" : "no",
           g_vl53.last_model_id);
    if (sensor_ok)
    {
        printf("  temperature=%.2f C\r\n", g_sensor.temperature_c);
        printf("  pressure=%.2f Pa\r\n", g_sensor.pressure_pa);
        printf("  altitude=%.2f m\r\n", g_sensor.altitude_m);
    }
    else
    {
        printf("  sensor_read=FAILED\r\n");
    }
    printf("  read_count=%lu fail_count=%lu\r\n",
           (unsigned long)g_sensor.read_count,
           (unsigned long)g_sensor.read_fail_count);
    printf("  rx_byte_count=%lu\r\n", (unsigned long)rx_byte_count);
    printf("  command_count=%lu\r\n", (unsigned long)command_count);
    printf("  tick=%lu ms\r\n", (unsigned long)HAL_GetTick());
}

static void set_rotors(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, state);
}

static void monitor_i2c_until_key(void)
{
    uint32_t next_print_ms = HAL_GetTick();
    float last_altitude_m = 0.0f;
    uint8_t have_prev = 0U;

    if (i2c1_ensure_init() != 0)
    {
        printf("monitor aborted: I2C1 init failed\r\n");
        return;
    }
    printf("monitoring BMP280 every 1 second (press any key to stop)\r\n");
    while (1)
    {
        if (usart1_read_byte_nonblocking(&rx_byte))
        {
            printf("\r\nmonitor stopped\r\n");
            break;
        }

        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_print_ms) >= 0)
        {
            if (bmp280_read_measurement(&g_sensor.temperature_c, &g_sensor.pressure_pa, &g_sensor.altitude_m) == 0)
            {
                g_sensor.read_count++;
                const char *trend = "-";
                if (have_prev)
                {
                    if (g_sensor.altitude_m > (last_altitude_m + 0.05f))
                    {
                        trend = "up";
                    }
                    else if (g_sensor.altitude_m < (last_altitude_m - 0.05f))
                    {
                        trend = "down";
                    }
                    else
                    {
                        trend = "flat";
                    }
                }
                printf("tick=%lu ms temp=%.2f C pressure=%.2f Pa altitude=%.2f m trend=%s\r\n",
                       (unsigned long)now,
                       g_sensor.temperature_c,
                       g_sensor.pressure_pa,
                       g_sensor.altitude_m,
                       trend);
                last_altitude_m = g_sensor.altitude_m;
                have_prev = 1U;
            }
            else
            {
                g_sensor.read_fail_count++;
                printf("tick=%lu ms BMP280 read failed\r\n", (unsigned long)now);
            }
            next_print_ms += 1000U;
        }
    }
}

static void sensor_init_until_key(void)
{
    uint32_t next_init_ms = HAL_GetTick();
    uint32_t init_attempt = 0U;
    uint8_t did_scan = 0U;

    if (i2c1_ensure_init() != 0)
    {
        printf("sensor init aborted: I2C1 init failed\r\n");
        return;
    }
    printf("retrying BMP280 + VL53L0X every 500ms (press any key to stop)\r\n");
    while (1)
    {
        if (usart1_read_byte_nonblocking(&rx_byte))
        {
            printf("\r\nsensor init retry stopped\r\n");
            break;
        }

        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_init_ms) >= 0)
        {
            if (did_scan == 0U)
            {
                did_scan = 1U;
                printf("one-shot I2C scan:\r\n");
                i2c_bus_scan();
            }
            init_attempt++;
            printf("sensor init attempt=%lu\r\n", (unsigned long)init_attempt);
            if (bmp280_probe_and_init(1U) == 0)
            {
                printf("BMP280 init OK (addr=0x%02X)\r\n", g_sensor.i2c_addr_7b);
            }
            else
            {
                printf("BMP280 init FAILED\r\n");
            }
            printf("VL53L0X check:\r\n");
            if (vl53l0x_read_model_id(1U) == 0)
            {
                printf("VL53L0X read OK\r\n");
            }
            else
            {
                printf("VL53L0X read FAILED\r\n");
            }
            next_init_ms += 500U;
        }
    }
}

static int bmp280_probe_and_init(uint8_t verbose)
{
    uint8_t addresses[2] = {BMP280_I2C_ADDR0, BMP280_I2C_ADDR1};
    uint8_t chip_id = 0U;

    if (verbose)
    {
        printf("BMP280 init start\r\n");
    }

    g_sensor.detected = 0U;
    g_sensor.initialized = 0U;
    for (uint32_t i = 0; i < 2U; i++)
    {
        g_sensor.i2c_addr_7b = addresses[i];
        if (verbose)
        {
            printf("BMP280 init: probe addr=0x%02X\r\n", g_sensor.i2c_addr_7b);
        }

        if (bmp280_read_regs(BMP280_REG_CHIP_ID, &chip_id, 1U, verbose) == 0 && chip_id == BMP280_CHIP_ID)
        {
            g_sensor.detected = 1U;
            if (verbose)
            {
                printf("BMP280 init: chip id match=0x%02X at addr=0x%02X\r\n", chip_id, g_sensor.i2c_addr_7b);
            }
            break;
        }
        else if (verbose)
        {
            printf("BMP280 init: probe miss at addr=0x%02X (chip_id=0x%02X)\r\n", g_sensor.i2c_addr_7b, chip_id);
        }
    }
    if (!g_sensor.detected)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: no device responded with chip id 0x%02X\r\n", BMP280_CHIP_ID);
        }
        return -1;
    }

    if (bmp280_write_reg(BMP280_REG_RESET, 0xB6U, verbose) != 0)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: reset write failed\r\n");
        }
        return -1;
    }
    HAL_Delay(5);

    uint8_t status = 0x01U;
    for (uint32_t tries = 0; tries < 20U && (status & 0x01U) != 0U; tries++)
    {
        if (bmp280_read_regs(BMP280_REG_STATUS, &status, 1U, verbose) != 0)
        {
            if (verbose)
            {
                printf("BMP280 init FAILED: status read failed (try=%lu)\r\n", (unsigned long)(tries + 1U));
            }
            return -1;
        }
        if (verbose)
        {
            printf("BMP280 init: status=0x%02X (try=%lu)\r\n", status, (unsigned long)(tries + 1U));
        }
        HAL_Delay(2);
    }
    if ((status & 0x01U) != 0U)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: status busy timeout\r\n");
        }
        return -1;
    }

    uint8_t calib[24];
    if (bmp280_read_regs(BMP280_REG_CALIB_START, calib, sizeof(calib), verbose) != 0)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: calibration read failed\r\n");
        }
        return -1;
    }
    g_calib.dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
    g_calib.dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    g_calib.dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
    g_calib.dig_P1 = (uint16_t)((calib[7] << 8) | calib[6]);
    g_calib.dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    g_calib.dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    g_calib.dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    g_calib.dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    g_calib.dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    g_calib.dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    g_calib.dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    g_calib.dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);

    if (bmp280_write_reg(BMP280_REG_CONFIG, 0x00U, verbose) != 0)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: config write failed\r\n");
        }
        return -1;
    }
    if (bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0x27U, verbose) != 0)
    {
        if (verbose)
        {
            printf("BMP280 init FAILED: ctrl_meas write failed\r\n");
        }
        return -1;
    }

    g_sensor.initialized = 1U;
    if (verbose)
    {
        printf("BMP280 init done: addr=0x%02X\r\n", g_sensor.i2c_addr_7b);
    }
    return 0;
}

static int bmp280_read_measurement(float *temperature_c, float *pressure_pa, float *altitude_m)
{
    if (!g_sensor.detected || !g_sensor.initialized)
    {
        return -1;
    }

    uint8_t raw[6];
    if (bmp280_read_regs(BMP280_REG_PRESS_MSB, raw, sizeof(raw), 0U) != 0)
    {
        return -1;
    }

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));

    int32_t temp_x100 = bmp280_compensate_temp(adc_T);
    uint32_t press_q24_8 = bmp280_compensate_press(adc_P);
    if (press_q24_8 == 0U)
    {
        return -1;
    }

    *temperature_c = ((float)temp_x100) / 100.0f;
    *pressure_pa = ((float)press_q24_8) / 256.0f;
    *altitude_m = pressure_to_altitude_m(*pressure_pa);
    return 0;
}

static int bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len, uint8_t verbose)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&g_sensor.hi2c1,
                                                (uint16_t)(g_sensor.i2c_addr_7b << 1),
                                                reg,
                                                I2C_MEMADD_SIZE_8BIT,
                                                buf,
                                                len,
                                                BMP280_TIMEOUT_MS);
    if (verbose)
    {
        printf("BMP280 I2C READ addr=0x%02X reg=0x%02X len=%u -> %s",
               g_sensor.i2c_addr_7b,
               reg,
               (unsigned int)len,
               hal_status_to_str(status));
        if (status != HAL_OK)
        {
            printf(" err=0x%08lX", (unsigned long)HAL_I2C_GetError(&g_sensor.hi2c1));
        }
        printf("\r\n");
    }
    if (status != HAL_OK)
    {
        return -1;
    }
    return 0;
}

static int bmp280_write_reg(uint8_t reg, uint8_t value, uint8_t verbose)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&g_sensor.hi2c1,
                                                 (uint16_t)(g_sensor.i2c_addr_7b << 1),
                                                 reg,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 &value,
                                                 1U,
                                                 BMP280_TIMEOUT_MS);
    if (verbose)
    {
        printf("BMP280 I2C WRITE addr=0x%02X reg=0x%02X val=0x%02X -> %s",
               g_sensor.i2c_addr_7b,
               reg,
               value,
               hal_status_to_str(status));
        if (status != HAL_OK)
        {
            printf(" err=0x%08lX", (unsigned long)HAL_I2C_GetError(&g_sensor.hi2c1));
        }
        printf("\r\n");
    }
    if (status != HAL_OK)
    {
        return -1;
    }
    return 0;
}

static const char *hal_status_to_str(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return "OK";
    case HAL_ERROR:
        return "ERROR";
    case HAL_BUSY:
        return "BUSY";
    case HAL_TIMEOUT:
        return "TIMEOUT";
    default:
        return "UNKNOWN";
    }
}

static void i2c_dump_hal_error_bits(uint32_t err)
{
    if (err == 0U)
    {
        return;
    }
    printf(" bits=");
    if ((err & HAL_I2C_ERROR_BERR) != 0U)
    {
        printf("BERR ");
    }
    if ((err & HAL_I2C_ERROR_ARLO) != 0U)
    {
        printf("ARLO ");
    }
    if ((err & HAL_I2C_ERROR_AF) != 0U)
    {
        printf("AF ");
    }
    if ((err & HAL_I2C_ERROR_OVR) != 0U)
    {
        printf("OVR ");
    }
    if ((err & HAL_I2C_ERROR_DMA) != 0U)
    {
        printf("DMA ");
    }
    if ((err & HAL_I2C_ERROR_TIMEOUT) != 0U)
    {
        printf("TIMEOUT ");
    }
}

static void i2c_dump_hal_context(const char *tag, HAL_StatusTypeDef st)
{
    uint32_t err = HAL_I2C_GetError(&g_sensor.hi2c1);
    printf("%s: HAL=%s(%d) err=0x%08lX state=%u prev=%u errCode=0x%08lX",
           tag,
           hal_status_to_str(st),
           (int)st,
           (unsigned long)err,
           (unsigned int)g_sensor.hi2c1.State,
           (unsigned int)g_sensor.hi2c1.PreviousState,
           (unsigned long)g_sensor.hi2c1.ErrorCode);
    i2c_dump_hal_error_bits(err);
    printf("\r\n");
}

static int i2c1_ensure_init(void)
{
    if (g_i2c1_hw_inited != 0U)
    {
        return 0;
    }

    printf("I2C1 init requested\r\n");
    I2C1_Init();
    if (g_i2c1_hw_inited == 0U)
    {
        printf("I2C1 init failed; scan/read operations are blocked\r\n");
        return -1;
    }
    return 0;
}

static void i2c_bus_scan(void)
{
    printf("I2C scan 0x%02X-0x%02X IsDeviceReady(trials=%u to=%ums)\r\n",
           I2C_SCAN_ADDR_FIRST,
           I2C_SCAN_ADDR_LAST,
           (unsigned int)I2C_ISREADY_TRIALS,
           (unsigned int)I2C_ISREADY_TIMEOUT_MS);
    uint32_t found = 0U;
    for (uint32_t a = I2C_SCAN_ADDR_FIRST; a <= I2C_SCAN_ADDR_LAST; a++)
    {
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&g_sensor.hi2c1,
                                                     (uint16_t)(a << 1),
                                                     I2C_ISREADY_TRIALS,
                                                     I2C_ISREADY_TIMEOUT_MS);
        if (st == HAL_OK)
        {
            printf("  FOUND 0x%02lX", (unsigned long)a);
            if (a == (uint32_t)VL53L0X_I2C_ADDR_7B)
            {
                printf(" (VL53L0X default addr)");
            }
            printf("\r\n");
            found++;
        }
        else
        {
            printf("  0x%02lX: %s", (unsigned long)a, hal_status_to_str(st));
            printf(" err=0x%08lX state=%u", (unsigned long)HAL_I2C_GetError(&g_sensor.hi2c1), (unsigned int)g_sensor.hi2c1.State);
            i2c_dump_hal_error_bits(HAL_I2C_GetError(&g_sensor.hi2c1));
            printf("\r\n");
        }
    }
    printf("scan done: found=%lu device(s)\r\n", (unsigned long)found);
}

static int vl53l0x_read_model_id(uint8_t verbose)
{
    uint8_t val = 0U;
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&g_sensor.hi2c1,
                                            (uint16_t)(VL53L0X_I2C_ADDR_7B << 1),
                                            VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                                            I2C_MEMADD_SIZE_8BIT,
                                            &val,
                                            1U,
                                            BMP280_TIMEOUT_MS);
    g_vl53.last_model_id = val;
    if (verbose)
    {
        printf("VL53L0X reg 0x%02X read: %s data=0x%02X (expect 0x%02X)\r\n",
               VL53L0X_REG_IDENTIFICATION_MODEL_ID,
               hal_status_to_str(st),
               val,
               VL53L0X_MODEL_ID_EXPECTED);
        i2c_dump_hal_context("VL53L0X_MemRead", st);
    }
    if (st != HAL_OK)
    {
        g_vl53.detected = 0U;
        return -1;
    }
    g_vl53.detected = (val == VL53L0X_MODEL_ID_EXPECTED) ? 1U : 0U;
    if (val == VL53L0X_MODEL_ID_EXPECTED)
    {
        if (verbose)
        {
            printf("VL53L0X model ID OK\r\n");
        }
        return 0;
    }
    if (verbose)
    {
        printf("VL53L0X model ID mismatch\r\n");
    }
    return -1;
}

static int sensor_i2c_deinit(void)
{
    HAL_StatusTypeDef st = HAL_I2C_DeInit(&g_sensor.hi2c1);
    i2c_dump_hal_context("HAL_I2C_DeInit", st);
    g_i2c1_hw_inited = 0U;
    memset(&g_calib, 0, sizeof(g_calib));
    g_sensor.detected = 0U;
    g_sensor.initialized = 0U;
    g_sensor.i2c_addr_7b = 0U;
    g_vl53.detected = 0U;
    g_vl53.last_model_id = 0U;
    return (st == HAL_OK) ? 0 : -1;
}

static int32_t bmp280_compensate_temp(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)g_calib.dig_T1 << 1))) * ((int32_t)g_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)g_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)g_calib.dig_T1))) >> 12) *
                    ((int32_t)g_calib.dig_T3)) >>
                   14;
    g_calib.t_fine = var1 + var2;
    return (g_calib.t_fine * 5 + 128) >> 8;
}

static uint32_t bmp280_compensate_press(int32_t adc_P)
{
    int64_t var1 = ((int64_t)g_calib.t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)g_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_calib.dig_P3) >> 8) + ((var1 * (int64_t)g_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)g_calib.dig_P1)) >> 33;
    if (var1 == 0)
    {
        return 0;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)g_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)g_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_calib.dig_P7) << 4);
    return (uint32_t)p;
}

static float pressure_to_altitude_m(float pressure_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / BMP280_SEA_LEVEL_PA, 0.1903f));
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
}

static void I2C1_Init(void)
{
    g_sensor.hi2c1.Instance = I2C1;
    g_sensor.hi2c1.Init.ClockSpeed = 100000U;
    g_sensor.hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_sensor.hi2c1.Init.OwnAddress1 = 0U;
    g_sensor.hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_sensor.hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_sensor.hi2c1.Init.OwnAddress2 = 0U;
    g_sensor.hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_sensor.hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_StatusTypeDef st = HAL_I2C_Init(&g_sensor.hi2c1);
    i2c_dump_hal_context("HAL_I2C_Init", st);
    if (st == HAL_OK)
    {
        g_i2c1_hw_inited = 1U;
        printf("I2C1 init OK\r\n");
    }
    else
    {
        g_i2c1_hw_inited = 0U;
        printf("I2C1 init FAILED\r\n");
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    }
}

static void USART1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    GPIOA->MODER &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->MODER |= (2U << (9U * 2U)) | (2U << (10U * 2U));
    GPIOA->OTYPER &= ~(1U << 9U);
    GPIOA->OSPEEDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->OSPEEDR |= (2U << (9U * 2U)) | (2U << (10U * 2U));
    GPIOA->PUPDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->PUPDR |= (1U << (10U * 2U));
    GPIOA->AFR[1] &= ~((0xFU << ((9U - 8U) * 4U)) | (0xFU << ((10U - 8U) * 4U)));
    GPIOA->AFR[1] |= (7U << ((9U - 8U) * 4U)) | (7U << ((10U - 8U) * 4U));

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 0x008BU;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

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

static void fault_led_prepare(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->MODER &= ~(3U << (10U * 2U));
    GPIOC->MODER |= (1U << (10U * 2U));
    GPIOC->OTYPER &= ~(1U << 10U);
    GPIOC->PUPDR &= ~(3U << (10U * 2U));
}

static void fault_led_delay(volatile uint32_t cycles)
{
    while (cycles-- != 0U)
    {
        __NOP();
    }
}

static void fault_led_blink_loop(uint32_t pulses)
{
    __disable_irq();
    fault_led_prepare();

    while (1)
    {
        for (uint32_t i = 0; i < pulses; i++)
        {
            GPIOC->BSRR = GPIO_BSRR_BS10;
            fault_led_delay(600000U);
            GPIOC->BSRR = GPIO_BSRR_BR10;
            fault_led_delay(250000U);
        }
        fault_led_delay(1200000U);
    }
}

void HardFault_Handler(void)
{
    fault_led_blink_loop(1U);
}

void MemManage_Handler(void)
{
    fault_led_blink_loop(2U);
}

void BusFault_Handler(void)
{
    fault_led_blink_loop(3U);
}

void UsageFault_Handler(void)
{
    fault_led_blink_loop(4U);
}
