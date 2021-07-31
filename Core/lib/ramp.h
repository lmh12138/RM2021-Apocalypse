#ifndef _RAMP_H_
#define _RAMP_H_

#include "stm32f4xx.h"

struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
};

#define RAMP_DEFAULT \
{ \
  .count = 0, \
  .scale = 0, \
  .out = 0, \
  .init = &RampInit, \
  .calc = &RampCalc, \
} \

extern void RampInit(struct ramp_t *ramp, int32_t scale);
extern float RampCalc(struct ramp_t *ramp);

float RAMP_float( float final, float now, float ramp );
#endif
