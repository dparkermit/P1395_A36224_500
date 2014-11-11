#ifndef __ETM_SCALE_H
#define __ETM_SCALE_H

unsigned int ETMScaleFactor2(unsigned int value, unsigned int scale_factor_0_2, signed int offset);
/* 
   This function multiplies "value" by "scale_factor" and returns the result.
   Scale factor is a Usigned Q 1.15 (range 0->2)   
*/


unsigned int ETMScaleFactor16(unsigned int value, unsigned int scale_factor_0_16, signed int offset);
/*
  This function multiplies "value" by "scale_factor" and returns the result.
  Scale factor is a Usigned Q 4.12 (range 0->16)   
*/


extern unsigned int etm_scale_saturation_etmscalefactor2_count;
extern unsigned int etm_scale_saturation_etmscalefactor16_count;


#define MACRO_DEC_TO_SCALE_FACTOR_16(X)     (X*4096)
#define MACRO_DEC_TO_CAL_FACTOR_2(X)       (X*32768)


#endif
