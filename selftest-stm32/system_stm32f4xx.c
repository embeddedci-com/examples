#include "stm32f4xx.h"

uint32_t SystemCoreClock = 16000000U;
const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8]  = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
#endif
}

void SystemCoreClockUpdate(void)
{
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllp;
    uint32_t pllsource;
    uint32_t pllm;

    tmp = RCC->CFGR & RCC_CFGR_SWS;
    if (tmp == RCC_CFGR_SWS_HSI) {
        SystemCoreClock = HSI_VALUE;
    } else if (tmp == RCC_CFGR_SWS_HSE) {
        SystemCoreClock = HSE_VALUE;
    } else if (tmp == RCC_CFGR_SWS_PLL) {
        pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22U;
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
        if (pllsource != 0U) {
            pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U);
        } else {
            pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6U);
        }
        pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16U) + 1U) * 2U);
        SystemCoreClock = pllvco / pllp;
    } else {
        SystemCoreClock = HSI_VALUE;
    }

    tmp = AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> 4U];
    SystemCoreClock >>= tmp;
}
