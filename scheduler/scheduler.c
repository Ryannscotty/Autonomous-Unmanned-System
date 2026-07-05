#include "stm32h723xx.h"
#include "scheduler.h"
#include "Clocks.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


/*
 * TASK TABLE LAYOUT  
 * { function of task, function name, task period time, task priority is task enabled}
*/ 

#ifdef MOCK_LIST
/*
Scheduler_Tasks SchedulerList[MAX_SCH_CNT] = 
{
    {},
    {},
    {},
    {},
    {},
    {},
    {},
    {},
    {},
};
*/ 
#else

#endif

/*
 * ===========================================================================
 *  sysTickConfig_1KHz -- SysTick interrupt firing every 1ms (1kHz)
 * ===========================================================================
 *  SysTick's CLKSOURCE bit is documented (RM0468/PM0253) as selecting
 *  between the processor clock (HCLK) and HCLK/8, but multiple ST
 *  community reports on H7-class silicon found CLKSOURCE doesn't behave
 *  as documented -- some saw SysTick running at the raw HCLK regardless
 *  of the bit, contradicting the /8 reference-clock path entirely. To
 *  sidestep that ambiguity, this function explicitly sets CLKSOURCE=1
 *  (processor clock / HCLK direct) and computes the reload value
 *  directly from HCLK -- the well-tested, portable choice used in the
 *  overwhelming majority of real SysTick examples across the whole
 *  STM32 family, regardless of this H7-specific documentation quirk.
 *
 *  Reload math: with HPRE=/2 on a 480MHz SYSCLK (per
 *  systemClockConfig_480Mhz), HCLK = 240MHz.
 *    RELOAD = (HCLK / target_Hz) - 1 = (240,000,000 / 1000) - 1 = 239999
 *  SysTick fires on the 1 -> 0 transition, so RELOAD+1 cycles actually
 *  elapse per tick -- 240000 cycles @ 240MHz = exactly 1.000ms.
 *
 *  SYST_RVR is a 24-bit register (max 0x00FFFFFF = 16,777,215); 239999
 *  fits comfortably.
 *
 *  IMPORTANT: call this AFTER systemClockConfig_480Mhz(), not before --
 *  unlike the MCO test functions, SysTick's reload value is computed
 *  from the ASSUMPTION that HCLK is already 240MHz. Calling this first
 *  would compute the reload against whatever HCLK was at reset (HSI-
 *  derived, much slower), giving a far slower-than-intended tick rate
 *  once the real clock config later kicks in.
 *
 *  You must also define a SysTick_Handler() somewhere in your project --
 *  the NVIC/CMSIS startup file expects that exact name in the vector
 *  table. This function only configures the timer; it does not define
 *  the handler itself.
 * ===========================================================================
 */

void sysTickConfig_1KHz(void)
{
    const uint32_t HCLK_HZ = 240000000U; /* matches HPRE=/2 on 480MHz SYSCLK */
    const uint32_t TARGET_HZ = 1000U;    /* 1kHz = 1ms period */
    const uint32_t reload = (HCLK_HZ / TARGET_HZ) - 1U;
 
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; /* stop the timer while reconfiguring */
 
    SysTick->LOAD = reload;                    /* SYST_RVR: 24-bit reload value */

    SysTick->VAL  = 0U;                        /* SYST_CVR: clear current value
                                                    so the first period is full-length,
                                                    not whatever was left over */
 
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk /* processor clock (HCLK), see note above */
                  | SysTick_CTRL_TICKINT_Msk   /* enable the SysTick exception */
                  | SysTick_CTRL_ENABLE_Msk;   /* start counting */
}

static volatile uint32_t msTicks = 0U;

void DebugLed(void)
{
    RCC->AHB4ENR |= (1 << 5); /* turning on clock for port F */
    GPIOF->MODER |= (1 << 8); /* use GPIOF Pin 4 */
    GPIOF->OTYPER &= ~(1 << 4);
    GPIOF->OSPEEDR |= (0b011 << 8);

}
 
void SysTick_Handler(void)
{
    msTicks++;
    GPIOF->ODR ^= (1 << 4);
    //printf("ms tick count: %d/n",msTicks);
}
 
/* Returns the current millisecond tick count. Rollover-safe when used
 * with subtraction-based timeout comparisons (see note above). */
uint32_t millis(void)
{
    return msTicks;
}
 
/* Simple blocking delay, built on the tick counter. Rollover-safe by
 * construction since it uses subtraction, not a target value. Mainly
 * useful for bring-up/debug; prefer non-blocking millis()-based checks
 * in real application code so the CPU isn't stuck spinning. */
void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) { }
}
 












