#ifndef INC_PLL_HPP_
#define INC_PLL_HPP_

#include "IRQ_Handler.hpp"
#include "eit_types.h"
#include <cstdint>

extern "C" {
    #include "stm32f4xx_hal.h"
    #include "stm32f4xx_hal_tim.h"
    #include "main.h"
}

struct CapSample {
     float fdds;
     uint32_t edgeTime;
 };


class PLL {
public:
    enum class State {
        ACQUIRE = 0,
        TRACK = 1,
        LOST = 2
    };

    void captureBlockProcessing();
    void halfCapture();
    void fullCapture();
    void update();
    void init(uint32_t initPhaseInc);
    void setConfig(eit_cfg_t cfg);
    uint32_t getPhaseInc();
    int32_t getPhaseCorr();
    void getDebugData(eit_debug_t* d);
    void setNcoPhase(uint32_t phase);
    bool PllLostFlag = false;

private:
    uint32_t calcPhaseInc(uint32_t sineFreq, uint32_t adcFreq, uint8_t ncoBits);
    void updateFine();
    void updateCoarse();
    void lost();
    void queuePush(CapSample pll);
    bool queuePop(CapSample *pll);
    //frequency lock functions
    float calcFrequencyError(CapSample p, uint32_t p_inc);
    int32_t applyFreqPI(float kp, float ki, float e);
    void clampF(float *fi, float max, float min);

    //phase lock functions

    void clampInt(int32_t *p, int32_t max, int32_t min);
    uint32_t calcPhasePredict(CapSample c);
    int32_t calcPhaseError(uint32_t p_ideal, uint32_t p_pred);
    void applyPhaseP(float kp, int32_t e);

    int32_t abs32(int32_t a);
    void errorBoundryCheckFine();
    void errorBoundryCheckCoarse();

    uint8_t _rbHead = 0;
    uint8_t _rbTail = 0;

    uint32_t _phase;
    uint32_t _phaseIncrement;
    int32_t _phaseCorr = 0;
    float _phaseCorrF = 0.0f;

    uint32_t _tim_ic_buf[16];
    uint32_t _dt[5];
    uint32_t _prevEdgeTime;
    bool _prevETReady = false;
    float _dtMean = 0;
    
    uint32_t _fs = 800000;
    uint16_t _edgeCounter = 0;
    
    bool _halfCapture = false;
    bool _fullCapture = false;
 
    float _fKp = 0;
    float _fKpCoarse = 0;
    float _fKi = 0;
    float _fKiCoarse = 0;
    float _fi = 0;
    float _ef = 0;
    float _efFineMax = 0;
    float _efCoarseMax = 0;
    float _efCoarseMin = 0;
    float _efAbs = 0;

    float _pKp = 0;
    float _pKpCoarse = 0;
    uint32_t _epFineMax = 0;
    uint32_t _epCoarseMax = 0;
    uint32_t _epCoarseMin = 0;
    int32_t _ep = 0;
    uint32_t _epAbs = 0;
    uint32_t _refEdge = 0;
    uint32_t _phaseRef = 0;
    bool _refAvailable = 0;
    uint32_t _phasePrediction = 0;
    uint32_t _phaseIdeal = 0;
    
    CapSample queue[16];

    State st = State::ACQUIRE;

    uint8_t _tractToAcquireCnt = 0;
    uint8_t _acquireToTrackCnt = 0;
    uint8_t _acquireToLostCnt = 0;
    uint8_t _lostToAcquireCnt = 0;

    eit_debug_t debug;


};

#endif /* INC_PLL_HPP_ */