#ifndef __MCP3564_H__
#define __MCP3564_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "driver/gpio.h"

// Custom macros
#define N_CHANNELS      6
#define N_SAMPLES       (1<<16) // 16384
#define N_SAMPLES_MASK  N_SAMPLES-1

typedef struct{
    // External flag 
    uint32_t flag_drdy;

    // history
//    uint32_t* history[N_CHANNELS];

} MCP3564_t;


void MCP3564_startUp();
int32_t MCP3564_signExtend(uint32_t Bytes);
uint32_t readDataRegister();

extern uint32_t freq;

#ifdef __cplusplus
}
#endif //__cplusplus

#endif