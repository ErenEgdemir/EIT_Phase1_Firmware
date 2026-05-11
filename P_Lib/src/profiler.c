#include "profiler.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"
#include "system_stm32f4xx.h"
#include <stdint.h>

#if PROFILER_ENABLE
volatile ProfView g_prof_view[PROF_COUNT];
volatile uint32_t g_cpu_load_permille = 0;

static ProfInternal s_prof[PROF_COUNT];

void Profiler_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t Profiler_NowCycles(void)
{
    return DWT->CYCCNT;
}

static uint32_t Profiler_CyclesToUs(uint32_t cycles)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000UL;

    if(cycles_per_us == 0) {
        return 0;
    }

    return cycles / cycles_per_us;
}

void Profiler_Begin(ProfBlock block)
{
    if(block >= PROF_COUNT) return;
    s_prof[block].begin_cycles = Profiler_NowCycles();
}

void Profiler_End(ProfBlock block)
{
    if(block >= PROF_COUNT) return;

    uint32_t now = Profiler_NowCycles();
    uint32_t dt_cycles = now - s_prof[block].begin_cycles;

    s_prof[block].last_cycles = dt_cycles;

    if(dt_cycles > s_prof[block].max_cycles)
    {
        s_prof[block].max_cycles = dt_cycles;
    }

    s_prof[block].total_cycles_1s += dt_cycles;
    s_prof[block].call_count_1s++;
}

void Profiler_Update_1s(void)
{
    uint64_t active_cycles = 0;

    for(uint32_t i = 0; i < PROF_COUNT; i++)
    {
        active_cycles += s_prof[i].total_cycles_1s;
    }
    g_cpu_load_permille = 
        (uint32_t)((active_cycles * 1000ULL) / (uint64_t)SystemCoreClock);
    
    if(g_cpu_load_permille > 1000)
    {
        g_cpu_load_permille = 1000;
    }
    for(uint32_t i = 0; i < PROF_COUNT; i++)
    {
        uint64_t block_cycles = s_prof[i].total_cycles_1s;
        g_prof_view[i].last_us = Profiler_CyclesToUs(s_prof[i].last_cycles);
        g_prof_view[i].max_us = Profiler_CyclesToUs(s_prof[i].max_cycles);
        g_prof_view[i].call_count_1s = s_prof[i].call_count_1s;
        g_prof_view[i].load_permille =
            (uint32_t)((block_cycles * 1000ULL) / (uint64_t)SystemCoreClock);

        s_prof[i].total_cycles_1s = 0;
        s_prof[i].call_count_1s = 0;
    }

}
#endif