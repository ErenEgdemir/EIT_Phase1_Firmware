#include "pll.hpp"
#include "app.hpp"
#include "eit_types.h"
#include "stm32f4xx_hal_tim.h"
#include "nco.hpp"
#include <cstdint>
#include <cmath>

#define F_TIM       60000000.0f
#define N_P_INC     134217728
extern CaptureEvents CpEvents;
void PLL::init()
{
    HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_1, _tim_ic_buf, 16);

    PllLostFlag = false;
    _rbHead = 0;
    _rbTail = 0;
    _phaseCorr = 0;
    _prevETReady = false;
    _edgeCounter = 0;
    _dtMean = 0;
    _refAvailable = false;
    _fi = 0;
    st = State::ACQUIRE;
    _tractToAcquireCnt = 0;
    _acquireToTrackCnt = 0;
    _acquireToLostCnt = 0;
    _lostToAcquireCnt = 0;
    _ef = 0;
    _ep = 0;
    _efAbs = 0;
    _epAbs = 0;
}

void PLL::setConfig(eit_cfg_t cfg)
{
    _fKp = cfg.freqKpFine;
    _fKi = cfg.freqKiFine;
    _fKpCoarse = cfg.freqKpCoarse;
    _fKiCoarse = cfg.freqKiCoarse;
    _pKp = cfg.phaseKpFine;
    _pKpCoarse = cfg.phaseKpCoarse;
    _phaseIdeal = cfg.PhaseIdeal;
    _fs = cfg.fs_hz;
    _efFineMax = cfg.FreqErrorMaxFine;
    _epFineMax = cfg.PhaseErrorMaxFine;
    _efCoarseMax = cfg.FreqErrorMaxCoarse;
    _efCoarseMin = cfg.FreqErrorMinCoarse;
    _epCoarseMax = cfg.PhaseErrorMaxCoarse;
    _epCoarseMin = cfg.PhaseErrorMinCoarse;
}
uint32_t PLL::calcPhaseInc(uint32_t sineFreq, uint32_t adcFreq, uint8_t ncoBits)
{
    if(ncoBits > 32) ncoBits = 32;
	uint32_t phaseInc = (uint32_t)(((uint64_t)sineFreq << ncoBits) / adcFreq);

	return phaseInc;
}

void PLL::halfCapture()
{
    _halfCapture = true;
}

void PLL::fullCapture()
{
    _fullCapture = true;
}

void PLL::queuePush(CapSample cap)
{
    queue[_rbHead] = cap;
    _rbHead = (_rbHead + 1) & 0xF;
    if(_rbHead == _rbTail){
        _rbTail = (_rbTail + 1) & 0xF;
    }
}

bool PLL::queuePop(CapSample *cap)
{
    if(_rbHead == _rbTail) return false;
    *cap = queue[_rbTail];
    _rbTail = (_rbTail + 1) & 0xF;
    return true;
}

void PLL::captureBlockProcessing()
{
    CapSample cap;
    if(CpEvents.halfCapCallback_pll){
        for(int i = 0; i < 8; i ++){

            if(!_prevETReady){
                _prevEdgeTime = _tim_ic_buf[i];
                _prevETReady = true;
                continue;
            }
            _dt[_edgeCounter] = _tim_ic_buf[i] - _prevEdgeTime;
            _prevEdgeTime = _tim_ic_buf[i];
            _edgeCounter++;
            
            if(_edgeCounter == 5){
                for(int k = 0; k < 5; k++){
                    _dtMean += _dt[k];
                }
                _dtMean = (float)_dtMean / 5;
                // if(_dtMean == 0) return;
                cap.fdds = F_TIM / _dtMean;
                 
                cap.edgeTime = _tim_ic_buf[i];
                // if(cap.fdds < 20000 || cap.fdds > 30000) return; 
                queuePush(cap);
                _edgeCounter = 0;
                _dtMean = 0;
            }
        }
        CpEvents.halfCapCallback_pll = false;
    }
    if(CpEvents.fullCapCallback_pll){
        for(int i = 8; i < 16; i++){
            if(!_prevETReady){
                _prevEdgeTime = _tim_ic_buf[i];
                _prevETReady = true;
                continue;
            }
            _dt[_edgeCounter] = _tim_ic_buf[i] - _prevEdgeTime;
            _prevEdgeTime = _tim_ic_buf[i];
            _edgeCounter++;
            if(_edgeCounter == 5){
                for(int k = 0; k < 5; k++){
                    _dtMean += _dt[k];
                }
                _dtMean = (float)_dtMean / 5;
                // if(_dtMean == 0) return;
                cap.fdds = F_TIM / _dtMean;
                
                cap.edgeTime = _tim_ic_buf[i];
                // if(cap.fdds < 20000 || cap.fdds > 30000) return; 
                queuePush(cap);
                _edgeCounter = 0;
                _dtMean = 0;
            }

        }
        CpEvents.fullCapCallback_pll = false;
    }
    
}
float PLL::calcFrequencyError(CapSample c, uint32_t p_inc)
{
    uint32_t targetPhaseInc = calcPhaseInc((uint32_t)c.fdds, _fs, 32);
    int64_t diff = (int64_t)targetPhaseInc - (int64_t)p_inc;
    return (float)diff;
}   
void PLL::clampF(float *fi, float max, float min)
{
    if(*fi > max) *fi = max;
    if(*fi < min) *fi = min;
}
int32_t PLL::applyFreqPI(float kp, float ki, float e)
{
    float fi_max = 0;
    
    if(ki){
        _fi += e;
        fi_max = (0.02 * N_P_INC) / ki;
        clampF(&_fi, fi_max, -fi_max);
    }
    int32_t pInc = kp * e + ki * _fi;

    return pInc;
}
uint32_t PLL::calcPhasePredict(CapSample c)
{
    uint32_t edgeTick = c.edgeTime - _refEdge; 
    float nSample = ((float)edgeTick * (float)_fs) /(float)F_TIM;
    uint32_t pPred = _phaseRef + (nSample * _phaseIncrement) + _phaseCorr;

    return pPred;
}
int32_t PLL::calcPhaseError(uint32_t p_ideal, uint32_t p_pred)
{
    uint32_t diff = p_ideal - p_pred;
    int32_t e = (int32_t)diff;

    return e;
}
void PLL::clampInt(int32_t *p, int32_t max, int32_t min)
{
    if(*p > max) *p = max;
    if(*p < min) *p = min;
}
void PLL::applyPhaseP(float kp, int32_t e)
{
    
    _phaseCorr += kp * e;
    clampInt(&_phaseCorr, +(1 << 26), -(1 << 26));

}
int32_t PLL::abs32(int32_t a)
{
    if(a < 0) a = -a;
    return a;
}

void PLL::errorBoundryCheckFine()
{
    _efAbs = fabsf(_ef);
    _epAbs = abs32(_ep);
   if(_efAbs > _efFineMax || _epAbs > _epFineMax) {
        if(_tractToAcquireCnt == 5) {
            st = State::ACQUIRE;
            _tractToAcquireCnt = 0;
            return;
        }
        _tractToAcquireCnt++;
    }else{
        _tractToAcquireCnt = 0;
    }
}

void PLL::errorBoundryCheckCoarse()
{
    _efAbs = fabsf(_ef);
    _epAbs = abs32(_ep);

    if(PllLostFlag) return;

    if(_efAbs < _efCoarseMin && _epAbs < _epCoarseMin) {
        
        if(_acquireToTrackCnt == 5) {
            st = State::TRACK;
            _acquireToTrackCnt = 0;
            return;
        }
        _acquireToTrackCnt++;
    }else{
        _acquireToTrackCnt = 0;
    }
    if(_efAbs > _efCoarseMax || _epAbs > _epCoarseMax){
        
        if(_acquireToLostCnt == 5) {
            st = State::LOST;
            _acquireToLostCnt = 0;
            return;
        }
        _acquireToLostCnt++;
    }else {
        _acquireToLostCnt = 0;
    }
}

void PLL::updateFine()
{
    CapSample capCatche;
    if(!queuePop(&capCatche)) return;
    if(capCatche.fdds < 20000 || capCatche.fdds > 30000) return;
    //Frequency lock
    debug.measuredFreq = capCatche.fdds;
    _ef = calcFrequencyError(capCatche, _phaseIncrement);
    debug.freqError = _ef;

    _phaseIncrement += applyFreqPI(_fKp, _fKi, _ef);
    debug.phaseInc = _phaseIncrement;
    debug.freqi = _fi;
    //Phase Lock
    if(!_refAvailable){
        _refEdge = capCatche.edgeTime;
        _phaseRef = _phase;
        _refAvailable = true;
        return;
    }
    _phasePrediction = calcPhasePredict(capCatche);
    _ep = calcPhaseError(_phaseIdeal, _phasePrediction);
    debug.phaseError = _ep;
    applyPhaseP(_pKp, _ep);
    //Referance Update
    _refEdge = capCatche.edgeTime;
    _phaseRef = _phasePrediction;

    errorBoundryCheckFine();
    
}

void PLL::updateCoarse()
{
    CapSample capCatche;
    if(!queuePop(&capCatche)) return;
    if(capCatche.fdds < 20000 || capCatche.fdds > 30000) return;
    debug.measuredFreq = capCatche.fdds;
    _ef = calcFrequencyError(capCatche, _phaseIncrement);
    _phaseIncrement += applyFreqPI(_fKpCoarse, _fKiCoarse, _ef);

     if(!_refAvailable){
        _refEdge = capCatche.edgeTime;
        _phaseRef = _phase;
        _refAvailable = true;
        return;
    }
    _phasePrediction = calcPhasePredict(capCatche);
    _ep = calcPhaseError(_phaseIdeal, _phasePrediction);
    debug.phaseError = _ep;
    applyPhaseP(_pKpCoarse, _ep);
    debug.phaseCorr = _phaseCorr;
    debug.phase = _phasePrediction;
    debug.phaseInc = _phaseIncrement;
    //Referance Update
    _refEdge = capCatche.edgeTime;
    _phaseRef = _phasePrediction;

    errorBoundryCheckCoarse();
}
void PLL::lost()
{
    if(_efAbs < _efCoarseMax && _epAbs < _epCoarseMax){
     
        if(_lostToAcquireCnt == 5) {
            st = State::ACQUIRE;
            _lostToAcquireCnt = 0;
            PllLostFlag = false;
            return;
        }
        _lostToAcquireCnt++;
    }else {
        PllLostFlag = true;
        _lostToAcquireCnt = 0;
    }
}

void PLL::update()
{
    debug.st = (uint8_t)st;
    switch(st){
    case State::ACQUIRE:
        updateCoarse();
    break;
    case State::TRACK:
        updateFine();
    break;
    case State::LOST:
        updateCoarse();
        lost();
    break;
    default:
    break; 
    }
}

void PLL::getDebugData(eit_debug_t* d)
{
    if(d){
        *d = debug;
    }
    
}

uint32_t PLL::getPhaseInc() { return _phaseIncrement; }
int32_t PLL::getPhaseCorr() { return _phaseCorr; }
