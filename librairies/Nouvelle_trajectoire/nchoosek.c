/*
 * File: nchoosek.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "nchoosek.h"

/* Function Declarations */
static float rt_roundf_snf(float u);

/* Function Definitions */

/*
 * Arguments    : float u
 * Return Type  : float
 */
static float rt_roundf_snf(float u)
{
  float y;
  if ((float)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (float)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (float)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : float n
 *                float k
 * Return Type  : float
 */
float nCk(float n, float k)
{
  float y;
  float nmk;
  int j;
  if (k > n / 2.0F) {
    k = n - k;
  }

  y = 1.0F;
  nmk = n - k;
  for (j = 0; j < (int)k; j++) {
    y *= ((1.0F + (float)j) + nmk) / (1.0F + (float)j);
  }

  return rt_roundf_snf(y);
}

/*
 * File trailer for nchoosek.c
 *
 * [EOF]
 */
