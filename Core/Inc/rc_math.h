#ifndef RC_RC_MATH_H
#define RC_RC_MATH_H

#include <stdbool.h>
#include <stdint.h>

static inline float Limit(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static inline int ILimit(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);}
static inline int Sign(int v){return (v>0)-(v<0);}
static inline bool Reached(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
#endif /* RC_RC_MATH_H */
