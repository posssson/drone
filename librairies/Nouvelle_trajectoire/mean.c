/*
 * File: mean.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "mean.h"

/* Function Definitions */

/*
 * Arguments    : const float x[1001]
 * Return Type  : float
 */
float mean(const float x[1001])
{
  float y;
  int k;
  y = x[0];
  for (k = 0; k < 1000; k++) {
    y += x[k + 1];
  }

  y /= 1001.0F;
  return y;
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
