/*
 * File: nouvelle_trajectoire_emxAPI.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "nouvelle_trajectoire_emxAPI.h"
#include "nouvelle_trajectoire_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : int b_numDimensions
 *                int *b_size
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateND_real32_T(int b_numDimensions, int *b_size)
{
  emxArray_real32_T *emx;
  int numEl;
  int i;
  emxInit_real32_T1(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (float *)calloc((unsigned int)numEl, sizeof(float));
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : float *b_data
 *                int b_numDimensions
 *                int *b_size
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateWrapperND_real32_T(float *b_data, int
  b_numDimensions, int *b_size)
{
  emxArray_real32_T *emx;
  int numEl;
  int i;
  emxInit_real32_T1(&emx, b_numDimensions);
  numEl = 1;
  for (i = 0; i < b_numDimensions; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = b_numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : float *b_data
 *                int rows
 *                int cols
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreateWrapper_real32_T(float *b_data, int rows, int cols)
{
  emxArray_real32_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  emxInit_real32_T1(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = b_data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : int rows
 *                int cols
 * Return Type  : emxArray_real32_T *
 */
emxArray_real32_T *emxCreate_real32_T(int rows, int cols)
{
  emxArray_real32_T *emx;
  int b_size[2];
  int numEl;
  int i;
  b_size[0] = rows;
  b_size[1] = cols;
  emxInit_real32_T1(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= b_size[i];
    emx->size[i] = b_size[i];
  }

  emx->data = (float *)calloc((unsigned int)numEl, sizeof(float));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : emxArray_real32_T *emxArray
 * Return Type  : void
 */
void emxDestroyArray_real32_T(emxArray_real32_T *emxArray)
{
  emxFree_real32_T(&emxArray);
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int b_numDimensions
 * Return Type  : void
 */
void emxInitArray_real32_T(emxArray_real32_T **pEmxArray, int b_numDimensions)
{
  emxInit_real32_T1(pEmxArray, b_numDimensions);
}
void coucou()
{
}
/*
 * File trailer for nouvelle_trajectoire_emxAPI.c
 *
 * [EOF]
 */
