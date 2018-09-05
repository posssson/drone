/*
 * File: nouvelle_trajectoire_emxutil.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

#ifndef NOUVELLE_TRAJECTOIRE_EMXUTIL_H
#define NOUVELLE_TRAJECTOIRE_EMXUTIL_H

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
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);
extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions);
extern void emxInit_real32_T1(emxArray_real32_T **pEmxArray, int b_numDimensions);

#endif

/*
 * File trailer for nouvelle_trajectoire_emxutil.h
 *
 * [EOF]
 */
