/*
 * File: diag.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "diag.h"

/* Function Definitions */

/*
 * Arguments    : const float v[2]
 *                float d[4]
 * Return Type  : void
 */
void diag(const float v[2], float d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 2; j++) {
    d[j + (j << 1)] = v[j];
  }
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
