#ifndef INC_NCO_HPP_
#define INC_NCO_HPP_


#include <cstdint>


#define PI      3.14159265358979323846f

class NCO {
public:
    void init();
    void run();
    uint32_t calcPhaseIncrement(uint32_t sineFreq, uint32_t adcFreq, uint8_t ncoBit);
    uint32_t getPhase();
    void setPhaseInc(uint32_t phaseInc);
    void setPhaseCorr(int32_t phaseCorr);
    float getSine();
    float getCos();
    void reset();
private:
    void generateLut(float *lut, uint16_t lutSize);
    uint32_t _phase = 0;
    uint32_t _phaseInc = 0;
    int32_t _phaseCorr = 0;
    uint32_t _phaseUse = 0;
    uint32_t _sinIndx = 0;
    uint32_t _cosIndx = 0;

    float LUT[4096] = {};
    
    
};




#endif /* INC_NCO_HPP_ */