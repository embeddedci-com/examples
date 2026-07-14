/*
 * STM32 scenario app:
 * - USART1 interactive menu (line-buffering off for interactive echo)
 * - rotor / stepper outputs on PC10/PC11
 *     * "rotors on/off" energises both pins (legacy rotor behaviour)
 *     * "rotors step <n> [fwd|rev]" drives them as a step/direction stepper
 *       interface (PC10 = STEP pulse, PC11 = DIR), as for an A4988/DRV8825
 *     * "rotors step <n> [fwd|rev] verify [seconds]" additionally samples the
 *       motor coil-current feedback on PA0 (ADC1_IN0 — the shunt/INA sense
 *       amplifier) while stepping, printing the peak-to-peak swing every
 *       `seconds` (default 1s) and stopping with STEP FAULT if the current
 *       goes flat (motor stalled / open coil / driver dead)
 * - stepper health monitor: "monitor pwm [seconds]" measures a PWM feedback
 *   signal on PA0 (TIM2_CH1) each cycle and flags "failure detected" if the
 *   PWM stops (signal lost / motor stalled)
 * - BMP280 on I2C1 (PB8/PB9), auto-detect 0x76/0x77
 * - VL53L0X on same bus (default addr 0x29) for bus validation
 */

#include "stm32f4xx_hal.h"
#include "VL53L0X.h"
#include "i2c.h"
#include "motor_health.h"
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
#define VL53L0X_MODEL_ID_EXPECTED 0xEEU

#define I2C_SCAN_ADDR_FIRST 0x08U
#define I2C_SCAN_ADDR_LAST 0x77U
#define I2C_ISREADY_TRIALS 3U
#define I2C_ISREADY_TIMEOUT_MS 10U

/* Boot attach window (ms): main() holds this long before touching any peripheral
 * so a host flasher that just reset us (NRST, an eFuse power-cycle, or the UART
 * "reset" command below) can connect SWD and halt during a known-quiet window.
 * This app never repurposes PA13/PA14, so SWD stays available regardless, but the
 * window + software reset make attach/flash deterministic on rigs with no NRST. */
#define FLASH_ATTACH_WINDOW_MS 200U

/* Stepper (step/direction) on the former rotor pins.  PC10 = STEP (one motor
 * step per rising edge), PC11 = DIR (level selects rotation direction). This is
 * the signalling a common stepper driver module (A4988/DRV8825) expects. */
#define STEP_PORT GPIOC
#define STEP_PIN GPIO_PIN_10
#define DIR_PIN GPIO_PIN_11
#define STEP_HALF_PERIOD_MS 1U /* 1ms high + 1ms low -> ~500 steps/s */
#define STEP_MAX_COUNT 100000U

/* PWM feedback monitor.  We measure the PWM on PA0 (TIM2_CH1) using the timer's
 * PWM-input mode: CH1 (rising) captures the period, CH2 (falling) the high time. */
#define PWM_MON_DEFAULT_SEC 5U
#define PWM_MON_MIN_SEC 1U
#define PWM_MON_MAX_SEC 3600U
#define PWM_MEAS_WINDOW_MS 400U /* no edge within this window = signal lost */
#define PWM_TIM_HZ 1000000U     /* timer timebase: 1 MHz -> 1 tick = 1 us */
/* "rotors step <n> ... verify [sec]" samples the coil-current feedback on PA0 to
 * check the motor is actually turning while we drive STEP. It prints once per
 * second by default (vs the standalone monitor's 5s) and clamps to the same
 * PWM_MON_MIN/MAX_SEC bounds. */
#define PWM_VERIFY_DEFAULT_SEC 1U

/* Motor-health check via the coil-current sense on PA0 (ADC1_IN0). PA0 carries the
 * shunt-resistor voltage through an INA sense amplifier, so a *running* motor makes
 * the reading swing as each step energises/de-energises the coil, while a stalled
 * or open coil (or a dead driver) leaves it sitting flat at a DC bias. The
 * peak-to-peak decision (ADC_VREF_MV / MOTOR_SWING_MIN_MV / motor_swing_is_fault)
 * lives in motor_health.h so the host unit test can exercise it against real
 * captured data. 12-bit ADC against the 3.3 V VREF+ on the NUCLEO-F446RE. */

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
    uint8_t initialized;
    uint8_t last_model_id;
    uint16_t last_range_mm;
    uint32_t read_count;
    uint32_t read_fail_count;
} vl53l0x_state_t;

static volatile char cmd_buffer[CMD_BUF_SIZE];
static volatile uint8_t cmd_index = 0;
static volatile uint8_t cmd_ready = 0;

static uint8_t rx_byte;
static volatile uint32_t rx_byte_count = 0;
static volatile uint32_t command_count = 0;
static uint8_t rx_last_was_cr = 0U;

/* Interrupt-driven RX ring buffer.  USART1 has a single RDR (no RX FIFO) and the
 * main loop sleeps in __WFI between SysTicks, so polling RDR drops bytes on a
 * back-to-back burst — a host command sent at line rate (~87 us/byte @115200)
 * overruns RDR and the loop sees only the first byte or two.  An RX interrupt
 * drains RDR the moment each byte lands into this ring; readers consume it. */
#define RX_RING_SIZE 256u
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_ring_head = 0;   /* written by the ISR  */
static volatile uint16_t rx_ring_tail = 0;   /* read by the main loop */

static sensor_state_t g_sensor;
static bmp280_calibration_t g_calib;
static vl53l0x_state_t g_vl53;
static struct VL53L0X g_vl53_dev;
static uint8_t g_i2c1_hw_inited = 0U;
static TIM_HandleTypeDef g_pwm_tim;
static ADC_HandleTypeDef g_adc;

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
static uint32_t parse_u32_default(const char *s, uint32_t defval);
static uint32_t rotor_step(uint32_t steps, int reverse, int verify, uint32_t verify_ms, int *motor_fault);
static void handle_rotor_step(const char *args);
static void pwm_input_init(void);
static void pwm_input_deinit(void);
static int pwm_measure(uint32_t *freq_hz, uint32_t *duty_pct, uint32_t *period_us);
static void adc_input_init(void);
static void adc_input_deinit(void);
static uint16_t adc_sample(void);
static void monitor_pwm_until_key(uint32_t period_sec);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
static void monitor_i2c_until_key(void);
static void sensor_init_until_key(void);
static int bmp280_probe_and_init(uint8_t verbose);
static int bmp280_read_measurement(float *temperature_c, float *pressure_pa, float *altitude_m);
static int bmp280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len, uint8_t verbose);
static int bmp280_write_reg(uint8_t reg, uint8_t value, uint8_t verbose);
static void i2c_dump_hal_context(const char *tag, HAL_StatusTypeDef st);
static void i2c_dump_hal_error_bits(uint32_t err);
static void i2c_recover(void);
static void i2c_bus_scan(void);
static int vl53l0x_init(uint8_t verbose);
static int vl53l0x_read_model_id(uint8_t verbose);
static int vl53l0x_read_range_mm(uint16_t *range_mm, uint8_t verbose);
static int sensor_i2c_deinit(void);
static const char *hal_status_to_str(HAL_StatusTypeDef status);
static int32_t bmp280_compensate_temp(int32_t adc_T);
static uint32_t bmp280_compensate_press(int32_t adc_P);
static float pressure_to_altitude_m(float pressure_pa);
static int32_t round_float_to_int(float value);
static int32_t round_float_to_centi(float value);
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

    /* Flash-friendliness: keep the debug port (SWD) fully alive even while the
     * app sleeps in __WFI(), so a host can always attach/halt/flash without a
     * hardware reset line. */
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();

    /* Give a flasher a clean, quiet window to connect SWD and halt right after a
     * reset, before we start driving GPIO / I2C / UART. See FLASH_ATTACH_WINDOW_MS. */
    HAL_Delay(FLASH_ATTACH_WINDOW_MS);

    MX_GPIO_Init();
    USART1_Init();
    (void)setvbuf(stdout, NULL, _IONBF, 0);

    memset(&g_sensor, 0, sizeof(g_sensor));
    memset(&g_vl53, 0, sizeof(g_vl53));
    memset(&g_vl53_dev, 0, sizeof(g_vl53_dev));
    g_vl53_dev.io_2v8 = true;
    g_vl53_dev.address = VL53L0X_I2C_ADDR_7B;
    g_vl53_dev.io_timeout = BMP280_TIMEOUT_MS;
    g_vl53_dev.did_timeout = false;
    I2C1_Init();
    (void)bmp280_probe_and_init(1U);
    printf("\r\nVL53L0X init (boot):\r\n");
    (void)vl53l0x_init(1U);

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

/* RX interrupt: drain RDR (and clear a possible overrun) into the ring the moment
 * bytes arrive, so a line-rate burst is never lost while the main loop is asleep.
 * Reading DR clears RXNE; reading SR then DR clears ORE. */
void USART1_IRQHandler(void)
{
    while ((USART1->SR & (USART_SR_RXNE | USART_SR_ORE)) != 0U)
    {
        uint8_t byte = (uint8_t)(USART1->DR & 0xFFU);
        uint16_t next = (uint16_t)((rx_ring_head + 1U) % RX_RING_SIZE);
        if (next != rx_ring_tail)      /* drop on full rather than clobber unread */
        {
            rx_ring[rx_ring_head] = byte;
            rx_ring_head = next;
        }
    }
}

static int usart1_read_byte_nonblocking(uint8_t *byte)
{
    if (rx_ring_tail == rx_ring_head)
    {
        return 0;                      /* ring empty */
    }
    *byte = rx_ring[rx_ring_tail];
    rx_ring_tail = (uint16_t)((rx_ring_tail + 1U) % RX_RING_SIZE);
    return 1;
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
    else if (strncmp(cmd, "rotors step", 11) == 0 && (cmd[11] == '\0' || cmd[11] == ' '))
    {
        handle_rotor_step(cmd + 11);
    }
    else if (strcmp(cmd, "monitor i2c") == 0)
    {
        monitor_i2c_until_key();
    }
    else if (strncmp(cmd, "monitor pwm", 11) == 0 && (cmd[11] == '\0' || cmd[11] == ' '))
    {
        uint32_t sec = parse_u32_default(cmd + 11, PWM_MON_DEFAULT_SEC);
        if (sec < PWM_MON_MIN_SEC)
        {
            sec = PWM_MON_MIN_SEC;
        }
        if (sec > PWM_MON_MAX_SEC)
        {
            sec = PWM_MON_MAX_SEC;
        }
        monitor_pwm_until_key(sec);
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
    else if (strcmp(cmd, "reset") == 0 || strcmp(cmd, "reboot") == 0)
    {
        /* Software reset over UART: lets a host reset the DUT without an NRST wire,
         * then attach SWD during the boot window (see FLASH_ATTACH_WINDOW_MS). */
        printf("RESET: rebooting via NVIC_SystemReset()\r\n");
        (void)fflush(stdout);
        HAL_Delay(20); /* let the line drain before the core resets */
        NVIC_SystemReset();
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
    printf("  rotors step <n> [fwd|rev] [verify [seconds]]  (PC10=STEP pulses, PC11=DIR; ~500 steps/s;\r\n");
    printf("                          verify samples coil current on PA0 (ADC1_IN0) while stepping, printing\r\n");
    printf("                          the peak-to-peak swing every `seconds` (default 1s) and stopping with\r\n");
    printf("                          STEP FAULT if the current goes flat (motor stalled); any key aborts)\r\n");
    printf("  monitor i2c   (reads BMP280 every 1s, press any key to stop)\r\n");
    printf("  monitor pwm [seconds]  (measure PWM on PA0/TIM2_CH1 each cycle; default 5s;\r\n");
    printf("                          prints 'failure detected' if the PWM is lost; any key stops)\r\n");
    printf("  sensor init   (BMP280 + VL53L0X, 500ms retry; I2C scan once at start; any key stops)\r\n");
    printf("  i2c scan      (HAL_I2C_IsDeviceReady 0x08-0x77, highlights 0x29)\r\n");
    printf("  vl53 test     (read VL53L0X model ID reg 0xC0; alias: vl53l0x test)\r\n");
    printf("  sensor deinit (HAL_I2C_DeInit + clear sensor state; re-init via sensor init / i2c scan / vl53 test)\r\n");
    printf("  reset         (NVIC_SystemReset; software reboot so a host can reset+attach without NRST; alias: reboot)\r\n");
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
    printf("  pc10(STEP)=%s\r\n", (pc10 == GPIO_PIN_SET) ? "HIGH" : "LOW");
    printf("  pc11(DIR)=%s\r\n", (pc11 == GPIO_PIN_SET) ? "rev" : "fwd");
    printf("  bmp280_detected=%s\r\n", g_sensor.detected ? "yes" : "no");
    if (g_sensor.detected)
    {
        printf("  bmp280_addr=0x%02X\r\n", g_sensor.i2c_addr_7b);
    }
    printf("  sensor_initialized=%s\r\n", g_sensor.initialized ? "yes" : "no");
    printf("  vl53l0x_detected=%s last_model_id=0x%02X\r\n",
           g_vl53.detected ? "yes" : "no",
           g_vl53.last_model_id);
    printf("  vl53l0x_initialized=%s last_range_mm=%u read_count=%lu fail_count=%lu\r\n",
           g_vl53.initialized ? "yes" : "no",
           (unsigned int)g_vl53.last_range_mm,
           (unsigned long)g_vl53.read_count,
           (unsigned long)g_vl53.read_fail_count);
    if (sensor_ok)
    {
        int32_t temp_centi = round_float_to_centi(g_sensor.temperature_c);
        int32_t altitude_centi = round_float_to_centi(g_sensor.altitude_m);
        int32_t pressure_pa_i = round_float_to_int(g_sensor.pressure_pa);
        uint32_t temp_abs_centi = (temp_centi < 0) ? (uint32_t)(-temp_centi) : (uint32_t)temp_centi;
        uint32_t altitude_abs_centi = (altitude_centi < 0) ? (uint32_t)(-altitude_centi) : (uint32_t)altitude_centi;

        printf("  temperature=%s%lu.%02lu C\r\n",
               (temp_centi < 0) ? "-" : "",
               (unsigned long)(temp_abs_centi / 100U),
               (unsigned long)(temp_abs_centi % 100U));
        printf("  pressure=%ld Pa\r\n", (long)pressure_pa_i);
        printf("  altitude=%s%lu.%02lu m\r\n",
               (altitude_centi < 0) ? "-" : "",
               (unsigned long)(altitude_abs_centi / 100U),
               (unsigned long)(altitude_abs_centi % 100U));
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

/* Parse the first unsigned decimal integer in s; return defval if none present. */
static uint32_t parse_u32_default(const char *s, uint32_t defval)
{
    while (*s == ' ')
    {
        s++;
    }
    if (*s < '0' || *s > '9')
    {
        return defval;
    }
    uint32_t v = 0U;
    while (*s >= '0' && *s <= '9')
    {
        v = (v * 10U) + (uint32_t)(*s - '0');
        s++;
    }
    return v;
}

/* Drive PC10 (STEP) with `steps` pulses; PC11 (DIR) selects direction.
 * Returns the number of steps actually issued (a key press aborts early).
 *
 * When `verify` is set the caller has already brought up the ADC on PA0
 * (ADC1_IN0 — the coil-current sense). We take one ADC reading per step and track
 * the min/max over each `verify_ms` window: a running motor makes the coil-current
 * reading swing step-to-step, so a healthy interval shows a wide peak-to-peak
 * swing, while a stalled/open coil sits flat. If an interval's swing collapses
 * below MOTOR_SWING_MIN_MV we stop the move, set *motor_fault, and return early. */
static uint32_t rotor_step(uint32_t steps, int reverse, int verify, uint32_t verify_ms, int *motor_fault)
{
    if (motor_fault != NULL)
    {
        *motor_fault = 0;
    }

    HAL_GPIO_WritePin(STEP_PORT, DIR_PIN, reverse ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(1); /* let DIR settle before the first STEP edge */

    /* First check one interval in, giving the motor time to spin up before we
     * expect any coil-current swing (an immediate check would see a flat reading). */
    uint32_t next_verify_ms = HAL_GetTick() + verify_ms;
    uint32_t cycle = 0U;
    uint32_t done = 0U;
    uint16_t adc_min = 0xFFFFU; /* min/max of the coil-current reading this interval */
    uint16_t adc_max = 0U;
    for (uint32_t i = 0U; i < steps; i++)
    {
        if (usart1_read_byte_nonblocking(&rx_byte))
        {
            break; /* any key aborts a long move */
        }
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(STEP_HALF_PERIOD_MS);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(STEP_HALF_PERIOD_MS);
        done++;

        if (verify)
        {
            uint16_t s = adc_sample(); /* coil-current sense on PA0 for this step */
            if (s < adc_min)
            {
                adc_min = s;
            }
            if (s > adc_max)
            {
                adc_max = s;
            }

            uint32_t now = HAL_GetTick();
            if ((int32_t)(now - next_verify_ms) >= 0)
            {
                cycle++;
                uint32_t min_mv = adc_counts_to_mv(adc_min);
                uint32_t max_mv = adc_counts_to_mv(adc_max);
                uint32_t vpp_mv = motor_swing_vpp_mv(adc_min, adc_max);
                if (!motor_swing_is_fault(adc_min, adc_max))
                {
                    printf("  verify cycle=%lu tick=%lu ms step=%lu/%lu motor ok: coil Vpp=%lu mV (min=%lu mV max=%lu mV)\r\n",
                           (unsigned long)cycle,
                           (unsigned long)now,
                           (unsigned long)done,
                           (unsigned long)steps,
                           (unsigned long)vpp_mv,
                           (unsigned long)min_mv,
                           (unsigned long)max_mv);
                }
                else
                {
                    printf("  verify cycle=%lu tick=%lu ms step=%lu/%lu failure detected: coil current flat on PA0, Vpp=%lu mV < %u mV (motor stalled / open coil)\r\n",
                           (unsigned long)cycle,
                           (unsigned long)now,
                           (unsigned long)done,
                           (unsigned long)steps,
                           (unsigned long)vpp_mv,
                           (unsigned int)MOTOR_SWING_MIN_MV);
                    if (motor_fault != NULL)
                    {
                        *motor_fault = 1;
                    }
                    break; /* stop the move the moment the coil current goes flat */
                }
                adc_min = 0xFFFFU; /* reset the window for the next interval */
                adc_max = 0U;
                next_verify_ms += verify_ms;
            }
        }
    }
    return done;
}

/* "rotors step <n> [fwd|rev] [verify [seconds]]" — args points just past
 * "rotors step". After the count the direction and the verify option may appear
 * in any order; "verify" turns on the PWM feedback check (see rotor_step). */
static void handle_rotor_step(const char *args)
{
    while (*args == ' ')
    {
        args++;
    }
    if (*args < '0' || *args > '9')
    {
        printf("usage: rotors step <n> [fwd|rev] [verify [seconds]]\r\n");
        return;
    }

    uint32_t steps = 0U;
    const char *p = args;
    while (*p >= '0' && *p <= '9')
    {
        steps = (steps * 10U) + (uint32_t)(*p - '0');
        if (steps > STEP_MAX_COUNT)
        {
            steps = STEP_MAX_COUNT;
        }
        p++;
    }

    int reverse = 0;
    int verify = 0;
    uint32_t verify_sec = PWM_VERIFY_DEFAULT_SEC;

    /* Optional trailing tokens: fwd|rev and verify [seconds], in any order. */
    while (1)
    {
        while (*p == ' ')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }
        char c = *p;
        if (c == 'r' || c == 'R')
        {
            reverse = 1;
        }
        else if (c == 'f' || c == 'F')
        {
            reverse = 0;
        }
        else if (c == 'v' || c == 'V')
        {
            verify = 1;
            /* Skip the "verify" word, then read an optional seconds value. */
            while (*p != '\0' && *p != ' ')
            {
                p++;
            }
            while (*p == ' ')
            {
                p++;
            }
            if (*p >= '0' && *p <= '9')
            {
                verify_sec = parse_u32_default(p, PWM_VERIFY_DEFAULT_SEC);
                if (verify_sec < PWM_MON_MIN_SEC)
                {
                    verify_sec = PWM_MON_MIN_SEC;
                }
                if (verify_sec > PWM_MON_MAX_SEC)
                {
                    verify_sec = PWM_MON_MAX_SEC;
                }
                while (*p >= '0' && *p <= '9')
                {
                    p++;
                }
            }
            continue; /* p already advanced past any seconds value */
        }
        else
        {
            printf("unknown option '%s' (use fwd|rev, verify [seconds])\r\n", p);
            return;
        }
        while (*p != '\0' && *p != ' ') /* advance past fwd|rev token */
        {
            p++;
        }
    }

    if (steps == 0U)
    {
        printf("STEP: count=0, nothing to do\r\n");
        return;
    }

    printf("STEP: %lu step(s) dir=%s (PC10=STEP, PC11=DIR=%s) at ~%u steps/s\r\n",
           (unsigned long)steps,
           reverse ? "rev" : "fwd",
           reverse ? "HIGH" : "LOW",
           (unsigned int)(1000U / (2U * STEP_HALF_PERIOD_MS)));

    if (verify)
    {
        printf("STEP verify: watching coil current on PA0 (ADC1_IN0) every %lu s; "
               "failure = peak-to-peak swing < %u mV (motor flat; any key aborts)\r\n",
               (unsigned long)verify_sec,
               (unsigned int)MOTOR_SWING_MIN_MV);
        adc_input_init();
    }

    int motor_fault = 0;
    uint32_t done = rotor_step(steps, reverse, verify, verify_sec * 1000U, &motor_fault);

    if (verify)
    {
        adc_input_deinit();
    }

    if (motor_fault)
    {
        printf("STEP FAULT: coil current went flat after %lu/%lu step(s) %s "
               "(motor not running)\r\n",
               (unsigned long)done, (unsigned long)steps, reverse ? "rev" : "fwd");
    }
    else if (done < steps)
    {
        printf("STEP aborted: %lu/%lu step(s) %s\r\n",
               (unsigned long)done, (unsigned long)steps, reverse ? "rev" : "fwd");
    }
    else
    {
        printf("STEP done: %lu step(s) %s\r\n", (unsigned long)done, reverse ? "rev" : "fwd");
    }
}

/* Bring up TIM2 in PWM-input mode on PA0 (TI1): CH1 (direct, rising) captures the
 * period, CH2 (indirect, falling) the high time, and TI1 rising resets the counter
 * (slave reset mode). Timebase is 1 MHz so a capture value is directly in us. */
static void pwm_input_init(void)
{
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    /* APB1 timer clock is doubled unless the APB1 prescaler is 1 (PPRE1 field >= 4). */
    if (((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) >= 4U)
    {
        timclk *= 2U;
    }
    uint32_t prescaler = timclk / PWM_TIM_HZ;
    if (prescaler == 0U)
    {
        prescaler = 1U;
    }

    g_pwm_tim.Instance = TIM2;
    g_pwm_tim.Init.Prescaler = prescaler - 1U;
    g_pwm_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm_tim.Init.Period = 0xFFFFFFFFU; /* TIM2 is 32-bit; never wraps mid-period */
    g_pwm_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_pwm_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&g_pwm_tim) != HAL_OK)
    {
        printf("PWM monitor: TIM2 init failed\r\n");
        return;
    }

    TIM_IC_InitTypeDef sIC = {0};
    sIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sIC.ICSelection = TIM_ICSELECTION_DIRECTTI; /* CH1 <- TI1: period */
    sIC.ICPrescaler = TIM_ICPSC_DIV1;
    sIC.ICFilter = 0U;
    (void)HAL_TIM_IC_ConfigChannel(&g_pwm_tim, &sIC, TIM_CHANNEL_1);

    sIC.ICPolarity = TIM_ICPOLARITY_FALLING;
    sIC.ICSelection = TIM_ICSELECTION_INDIRECTTI; /* CH2 <- TI1: high time */
    (void)HAL_TIM_IC_ConfigChannel(&g_pwm_tim, &sIC, TIM_CHANNEL_2);

    TIM_SlaveConfigTypeDef sSlave = {0};
    sSlave.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlave.InputTrigger = TIM_TS_TI1FP1;
    sSlave.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    sSlave.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlave.TriggerFilter = 0U;
    (void)HAL_TIM_SlaveConfigSynchro(&g_pwm_tim, &sSlave);

    (void)HAL_TIM_IC_Start(&g_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Start(&g_pwm_tim, TIM_CHANNEL_2);
}

static void pwm_input_deinit(void)
{
    (void)HAL_TIM_IC_Stop(&g_pwm_tim, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Stop(&g_pwm_tim, TIM_CHANNEL_2);
    (void)HAL_TIM_IC_DeInit(&g_pwm_tim);
}

/* One PWM measurement. Returns 0 with freq/duty/period filled, or -1 if no full
 * PWM period is seen within PWM_MEAS_WINDOW_MS (signal lost / stuck / stalled). */
static int pwm_measure(uint32_t *freq_hz, uint32_t *duty_pct, uint32_t *period_us)
{
    uint32_t start = HAL_GetTick();

    /* Discard the first capture (partial period after we start looking), then wait
     * for a second, full period. Two edges also proves the signal actually toggles. */
    __HAL_TIM_CLEAR_FLAG(&g_pwm_tim, TIM_FLAG_CC1);
    while (!__HAL_TIM_GET_FLAG(&g_pwm_tim, TIM_FLAG_CC1))
    {
        if ((HAL_GetTick() - start) > PWM_MEAS_WINDOW_MS)
        {
            return -1;
        }
    }
    __HAL_TIM_CLEAR_FLAG(&g_pwm_tim, TIM_FLAG_CC1);
    while (!__HAL_TIM_GET_FLAG(&g_pwm_tim, TIM_FLAG_CC1))
    {
        if ((HAL_GetTick() - start) > PWM_MEAS_WINDOW_MS)
        {
            return -1;
        }
    }

    uint32_t period = HAL_TIM_ReadCapturedValue(&g_pwm_tim, TIM_CHANNEL_1);
    uint32_t high = HAL_TIM_ReadCapturedValue(&g_pwm_tim, TIM_CHANNEL_2);
    if (period == 0U)
    {
        return -1;
    }
    if (high > period)
    {
        high = period;
    }
    *period_us = period;
    *freq_hz = PWM_TIM_HZ / period;
    *duty_pct = (high * 100U) / period;
    return 0;
}

static void monitor_pwm_until_key(uint32_t period_sec)
{
    uint32_t period_ms = period_sec * 1000U;

    pwm_input_init();
    printf("monitoring PWM on PA0 (TIM2_CH1) every %lu s (press any key to stop)\r\n",
           (unsigned long)period_sec);
    printf("failure = no PWM edges within %u ms (signal lost / motor stalled)\r\n",
           (unsigned int)PWM_MEAS_WINDOW_MS);

    uint32_t next_ms = HAL_GetTick();
    uint32_t cycle = 0U;
    while (1)
    {
        if (usart1_read_byte_nonblocking(&rx_byte))
        {
            printf("\r\nPWM monitor stopped\r\n");
            break;
        }

        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_ms) >= 0)
        {
            cycle++;
            uint32_t freq = 0U;
            uint32_t duty = 0U;
            uint32_t per = 0U;
            if (pwm_measure(&freq, &duty, &per) == 0)
            {
                printf("cycle=%lu tick=%lu ms PWM ok: freq=%lu Hz duty=%lu%% period=%lu us\r\n",
                       (unsigned long)cycle,
                       (unsigned long)now,
                       (unsigned long)freq,
                       (unsigned long)duty,
                       (unsigned long)per);
            }
            else
            {
                printf("cycle=%lu tick=%lu ms failure detected: no PWM on PA0 (signal lost)\r\n",
                       (unsigned long)cycle,
                       (unsigned long)now);
                break; /* stop monitoring on failure */
            }
            next_ms += period_ms;
        }
    }
    pwm_input_deinit();
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_TIM2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_0; /* PA0 = TIM2_CH1 */
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        /* Pull-down so a disconnected input reads low (no edges = failure detected)
         * rather than floating and decoding spurious "PWM". */
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    }
}

/* Bring up ADC1 on PA0 (ADC1_IN0) for the coil-current sense, single software-
 * triggered 12-bit conversions. PA0 is shared with the TIM2 PWM monitor above;
 * "rotors step ... verify" uses this ADC path, "monitor pwm" uses the timer path,
 * and each configures/tears down PA0 on entry/exit so they never collide. */
static void adc_input_init(void)
{
    g_adc.Instance = ADC1;
    g_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc.Init.ScanConvMode = DISABLE;
    g_adc.Init.ContinuousConvMode = DISABLE;
    g_adc.Init.DiscontinuousConvMode = DISABLE;
    g_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc.Init.NbrOfConversion = 1U;
    g_adc.Init.DMAContinuousRequests = DISABLE;
    g_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&g_adc) != HAL_OK)
    {
        printf("ADC monitor: ADC1 init failed\r\n");
        return;
    }

    ADC_ChannelConfTypeDef sChannel = {0};
    sChannel.Channel = ADC_CHANNEL_0; /* PA0 = ADC1_IN0 */
    sChannel.Rank = 1U;
    sChannel.SamplingTime = ADC_SAMPLETIME_84CYCLES; /* op-amp output settles fast */
    (void)HAL_ADC_ConfigChannel(&g_adc, &sChannel);
}

static void adc_input_deinit(void)
{
    (void)HAL_ADC_DeInit(&g_adc);
}

/* One 12-bit conversion of the coil-current sense on PA0. Returns 0..4095; on a
 * conversion timeout it returns 0 (reads as "flat", which is the safe/fault side). */
static uint16_t adc_sample(void)
{
    if (HAL_ADC_Start(&g_adc) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_ADC_PollForConversion(&g_adc, 2U) != HAL_OK)
    {
        HAL_ADC_Stop(&g_adc);
        return 0U;
    }
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&g_adc);
    HAL_ADC_Stop(&g_adc);
    return v;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_ADC1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_0; /* PA0 = ADC1_IN0 */
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    }
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
    if (bmp280_probe_and_init(1U) != 0)
    {
        printf("monitor note: BMP280 init failed (reads may fail until sensor responds)\r\n");
    }
    if (vl53l0x_init(1U) != 0)
    {
        printf("monitor note: VL53L0X init failed (will retry during reads)\r\n");
    }
    printf("monitoring BMP280 + VL53L0X every 1 second (press any key to stop)\r\n");
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
                int32_t temp_centi = round_float_to_centi(g_sensor.temperature_c);
                int32_t altitude_centi = round_float_to_centi(g_sensor.altitude_m);
                int32_t pressure_pa_i = round_float_to_int(g_sensor.pressure_pa);
                uint32_t temp_abs_centi = (temp_centi < 0) ? (uint32_t)(-temp_centi) : (uint32_t)temp_centi;
                uint32_t altitude_abs_centi = (altitude_centi < 0) ? (uint32_t)(-altitude_centi) : (uint32_t)altitude_centi;
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
                printf("tick=%lu ms temp=%s%lu.%02lu C pressure=%ld Pa altitude=%s%lu.%02lu m trend=%s\r\n",
                       (unsigned long)now,
                       (temp_centi < 0) ? "-" : "",
                       (unsigned long)(temp_abs_centi / 100U),
                       (unsigned long)(temp_abs_centi % 100U),
                       (long)pressure_pa_i,
                       (altitude_centi < 0) ? "-" : "",
                       (unsigned long)(altitude_abs_centi / 100U),
                       (unsigned long)(altitude_abs_centi % 100U),
                       trend);
                last_altitude_m = g_sensor.altitude_m;
                have_prev = 1U;
            }
            else
            {
                g_sensor.read_fail_count++;
                printf("tick=%lu ms BMP280 read failed\r\n", (unsigned long)now);
                printf("BMP280 failed -> recovering I2C\r\n");
                i2c_recover();
            }
            uint16_t range_mm = 0U;
            if (vl53l0x_read_range_mm(&range_mm, 0U) == 0)
            {
                g_vl53.read_count++;
                printf("tick=%lu ms VL53L0X range=%u mm\r\n",
                       (unsigned long)now,
                       (unsigned int)range_mm);
            }
            else
            {
                g_vl53.read_fail_count++;
                printf("tick=%lu ms VL53L0X read failed\r\n", (unsigned long)now);
                printf("VL53 failed -> recovering I2C\r\n");
                i2c_recover();
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
            printf("VL53L0X init:\r\n");
            if (vl53l0x_init(1U) == 0)
            {
                uint16_t range_mm = 0U;
                if (vl53l0x_read_range_mm(&range_mm, 1U) == 0)
                {
                    g_vl53.read_count++;
                    printf("VL53L0X read OK range=%u mm\r\n", (unsigned int)range_mm);
                }
                else
                {
                    g_vl53.read_fail_count++;
                    printf("VL53L0X init OK, read FAILED\r\n");
                    printf("VL53 failed -> recovering I2C\r\n");
                    i2c_recover();
                }
            }
            else
            {
                printf("VL53L0X init FAILED\r\n");
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

static void i2c_recover(void)
{
    HAL_StatusTypeDef st = HAL_I2C_DeInit(&g_sensor.hi2c1);
    i2c_dump_hal_context("HAL_I2C_DeInit(recover)", st);
    HAL_Delay(2U);
    g_i2c1_hw_inited = 0U;
    st = HAL_I2C_Init(&g_sensor.hi2c1);
    i2c_dump_hal_context("HAL_I2C_Init(recover)", st);
    g_i2c1_hw_inited = (st == HAL_OK) ? 1U : 0U;
    if (st == HAL_OK)
    {
        vl53l0x_port_set_i2c(&g_sensor.hi2c1);
    }
    HAL_Delay(2U);
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
    uint8_t val = VL53L0X_readReg(&g_vl53_dev, IDENTIFICATION_MODEL_ID);
    g_vl53.last_model_id = val;
    if (verbose)
    {
        printf("VL53L0X reg 0x%02X read: i2c_status=%u data=0x%02X (expect 0x%02X)\r\n",
               IDENTIFICATION_MODEL_ID,
               (unsigned int)g_vl53_dev.last_status,
               val,
               VL53L0X_MODEL_ID_EXPECTED);
    }
    if (g_vl53_dev.last_status != 0U)
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

static int vl53l0x_init(uint8_t verbose)
{
    if (i2c1_ensure_init() != 0)
    {
        g_vl53.initialized = 0U;
        return -1;
    }
    g_vl53_dev.io_2v8 = true;
    g_vl53_dev.address = VL53L0X_I2C_ADDR_7B;
    g_vl53_dev.io_timeout = BMP280_TIMEOUT_MS;
    g_vl53_dev.did_timeout = false;

    if (!VL53L0X_init(&g_vl53_dev))
    {
        g_vl53.detected = 0U;
        g_vl53.initialized = 0U;
        if (verbose)
        {
            printf("VL53L0X init failed (driver)\r\n");
        }
        return -1;
    }
    if (VL53L0X_setMeasurementTimingBudget(&g_vl53_dev, 20000U) == false && verbose)
    {
        printf("VL53L0X timing budget set failed; continuing\r\n");
    }
    if (vl53l0x_read_model_id(verbose) != 0)
    {
        g_vl53.initialized = 0U;
        return -1;
    }
    g_vl53.initialized = 1U;
    if (verbose)
    {
        printf("VL53L0X init OK (third-party driver)\r\n");
    }
    return 0;
}

static int vl53l0x_read_range_mm(uint16_t *range_mm, uint8_t verbose)
{
    if (range_mm == NULL)
    {
        return -1;
    }
    if (g_vl53.initialized == 0U)
    {
        if (vl53l0x_init(verbose) != 0)
        {
            return -1;
        }
    }

    uint16_t range = VL53L0X_readRangeSingleMillimeters(&g_vl53_dev);
    bool timed_out = VL53L0X_timeoutOccurred(&g_vl53_dev);
    if (g_vl53_dev.last_status != 0U || timed_out || range == 65535U)
    {
        g_vl53.initialized = 0U;
        if (verbose)
        {
            printf("VL53L0X range read failed: i2c_status=%u timeout=%u range=%u\r\n",
                   (unsigned int)g_vl53_dev.last_status,
                   timed_out ? 1U : 0U,
                   (unsigned int)range);
        }
        return -1;
    }
    *range_mm = range;
    g_vl53.last_range_mm = *range_mm;
    if (verbose)
    {
        printf("VL53L0X range read: %u mm\r\n", (unsigned int)(*range_mm));
    }
    return 0;
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
    g_vl53.initialized = 0U;
    g_vl53.last_model_id = 0U;
    g_vl53.last_range_mm = 0U;
    g_vl53.read_count = 0U;
    g_vl53.read_fail_count = 0U;
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

static int32_t round_float_to_int(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static int32_t round_float_to_centi(float value)
{
    return round_float_to_int(value * 100.0f);
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
        vl53l0x_port_set_i2c(&g_sensor.hi2c1);
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
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    /* Enable the USART1 interrupt so RDR is drained as each byte lands (above). */
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
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
