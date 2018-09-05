/*
 * File: sum.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "sum.h"

/* Function Definitions */

/*
 * Arguments    : const float x[4]
 *                float y[2]
 * Return Type  : void
 */
void b_sum(const float x[4], float y[2])
{
  int i;
  for (i = 0; i < 2; i++) {
    y[i] = x[i << 1] + x[(i << 1) + 1];
  }
}

/*
 * Arguments    : const float x[1000]
 * Return Type  : float
 */
float sum(const float x[1000])
{
  float y;
  int k;
  y = x[0];
  for (k = 0; k < 999; k++) {
    y += x[k + 1];
  }

  return y;
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
