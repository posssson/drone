/*
 * File: NelderMead.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

#ifndef NELDERMEAD_H
#define NELDERMEAD_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "nouvelle_trajectoire_types.h"

/* Function Declarations */
extern void NelderMead(const float b_P0[2], const float Pminmax[4], Lgcb42vx *h,
  float P[2], float *C);

#endif

/*
 * File trailer for NelderMead.h
 *
 * [EOF]
 */
