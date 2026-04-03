// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#define HIGH_SPEED
//#define HIGH_ACCURACY


#include "stm32f1xx.h"
#include "init.h"
#include "VL53L0X.h"
#include "usart1.h"
#include <stdio.h>
#include "i2c.h"


char strbuf[25]; // for UART output via sprintf()

struct VL53L0X myTOFsensor = {.io_2v8 = true, .address = 0b0101001, .io_timeout = 500, .did_timeout = false};

int main(void){
	// Initialize system timer for 1ms ticks (else divide by 1e6 for µs ticks)
	SysTick_Config(SystemCoreClock / 1000);
	// init the USART1 peripheral to print to serial terminal
	init_USART1();
	// init the I2C1 peripheral and the SDA/SCL GPIO pins
	init_i2c1();

	USART1_transmitString("testing VL53L0X\n--------------\n");

	// GPIO pin PB5 connected to XSHUT pin of sensor
	GPIOB->CRL &=~GPIO_CRL_CNF5;
	GPIOB->CRL |= GPIO_CRL_MODE5;
	GPIOB->ODR &=~GPIO_ODR_ODR5; // shut off VL53L0X
	delay(1);
	GPIOB->ODR |= GPIO_ODR_ODR5; // power up VL53L0X again
	delay(2);

	if( VL53L0X_init(&myTOFsensor) ){
		USART1_transmitString("init successful\n");
	}else{
		USART1_transmitString("init error");
		return 0;
	}

#ifdef LONG_RANGE
	// lower the return signal rate limit (default is 0.25 MCPS)
	VL53L0X_setSignalRateLimit(&myTOFsensor, 0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodPreRange, 18);
	VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodFinalRange, 14);
#endif
#ifdef HIGH_SPEED
	// reduce timing budget to 20 ms (default is about 33 ms)
	VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20000);
	USART1_transmitString("step1\n");
#else //HIGH_ACCURACY
	// increase timing budget to 200 ms
	VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 200000);
#endif

	VL53L0X_startContinuous(&myTOFsensor, 0);

	while(1){
		uint16_t value = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
		sprintf(strbuf, "\t%d\tmm\n", value);
		USART1_transmitString(strbuf);
		 if ( VL53L0X_timeoutOccurred(&myTOFsensor) ) {
			 USART1_transmitString("TIMEOUT\n");
		 }
	}

return 1;
}
