#ifndef INC_PROFILER_H_
#define INC_PROFILER_H_

#include <stdint.h>

#define PROFILER_ENABLE 1

#ifdef __cplusplus
extern "C"{
#endif

typedef enum
{
    MEAS_START = 0,
    MEAS_SWITCHING,
    MEAS_SETTLING_IRQ,
    MEAS_SETTLING,
    MEAS_INTEGRATE_TIMER_IRQ,
    MEAS_INTEGRATE,
    MEAS_LOCKIN,
    MEAS_STORE,
    MEAS_DONE,

    FSM_SEND_MEASDONE_CND,

    COM_TX_RETRY_POLL,
    COM_FRAME_PARSER_POLL,
    COM_SEND_DATA,
    COM_SEND_DEBUG,

    PLL_CAP_BLOCK_PRS,
    PLL_UPDATE,

    APP_MEASDONE_CALL,
    APP_MASDONE_CND,

    PROF_COUNT
}ProfBlock;

typedef struct 
{
    uint32_t last_us;
    uint32_t max_us;
    uint32_t load_permille;
    uint32_t call_count_1s;
}ProfView;

typedef struct
{
    uint32_t begin_cycles;
    uint32_t last_cycles;
    uint32_t max_cycles;
    uint64_t total_cycles_1s;
    uint32_t call_count_1s;
}ProfInternal;

#if PROFILER_ENABLE

void Profiler_Init(void);

void Profiler_Begin(ProfBlock block);
void Profiler_End(ProfBlock block);

void Profiler_Update_1s(void);

#else

#define Profiler_Init()      ((void)0)
#define Profiler_Begin(block)     ((void)0)
#define Profiler_End(block)       ((void)0)
#define Profiler_Update_1s() ((void)0)

#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_PROFILER_H_ */