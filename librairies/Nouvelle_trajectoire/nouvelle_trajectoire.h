/*
 * File: nouvelle_trajectoire.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

#ifndef NOUVELLE_TRAJECTOIRE_H
#define NOUVELLE_TRAJECTOIRE_H

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
extern void nouvelle_trajectoire(const float xyh0[3], float Vw, const float
  etat[5], float t, float ua, float b_psiw, const float uvww0t[3], long initopt,
  emxArray_real32_T *tref, emxArray_real32_T *psiref, emxArray_real32_T *posref,
  float *val, float *outopt, float par[2]);

#endif

/*
 * File trailer for nouvelle_trajectoire.h
 *
 * [EOF]
 */
