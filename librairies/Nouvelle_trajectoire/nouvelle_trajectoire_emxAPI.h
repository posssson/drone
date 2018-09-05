/*
 * File: nouvelle_trajectoire_emxAPI.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

#ifndef NOUVELLE_TRAJECTOIRE_EMXAPI_H
#define NOUVELLE_TRAJECTOIRE_EMXAPI_H

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
extern emxArray_real32_T *emxCreateND_real32_T(int b_numDimensions, int *b_size);
extern emxArray_real32_T *emxCreateWrapperND_real32_T(float *b_data, int
  b_numDimensions, int *b_size);
extern emxArray_real32_T *emxCreateWrapper_real32_T(float *b_data, int rows, int
  cols);
extern emxArray_real32_T *emxCreate_real32_T(int rows, int cols);
extern void emxDestroyArray_real32_T(emxArray_real32_T *emxArray);
void emxInitArray_real32_T(emxArray_real32_T **pEmxArray, int
  b_numDimensions);
  void coucou();

#endif

/*
 * File trailer for nouvelle_trajectoire_emxAPI.h
 *
 * [EOF]
 */
