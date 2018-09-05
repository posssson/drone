/*
 * File: abs.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "abs.h"

/* Function Definitions */

/*
 * Arguments    : const float x[1001]
 *                float y[1001]
 * Return Type  : void
 */
void b_abs(const float x[1001], float y[1001])
{
  int k;
  for (k = 0; k < 1001; k++) {
    y[k] = (float)fabs(x[k]);
  }
}

/*
 * Arguments    : const float x[4]
 *                float y[4]
 * Return Type  : void
 */
void c_abs(const float x[4], float y[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    y[k] = (float)fabs(x[k]);
  }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
