/*
 * File: sort1.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "sort1.h"

/* Function Definitions */

/*
 * Arguments    : float x[3]
 *                int idx[3]
 * Return Type  : void
 */
void sort(float x[3], int idx[3])
{
  float b_x[3];
  int i2;
  boolean_T p;
  for (i2 = 0; i2 < 3; i2++) {
    b_x[i2] = x[i2];
  }

  if ((x[0] <= x[1]) || rtIsNaNF(x[1])) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
    if ((x[1] <= x[2]) || rtIsNaNF(x[2])) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      idx[0] = 1;
      idx[1] = 2;
      idx[2] = 3;
    } else {
      if ((x[0] <= x[2]) || rtIsNaNF(x[2])) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        idx[0] = 1;
        idx[1] = 3;
        idx[2] = 2;
        b_x[1] = x[2];
        b_x[2] = x[1];
      } else {
        idx[0] = 3;
        idx[1] = 1;
        idx[2] = 2;
        b_x[2] = x[1];
        b_x[1] = x[0];
        b_x[0] = x[2];
      }
    }
  } else {
    if ((x[0] <= x[2]) || rtIsNaNF(x[2])) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      idx[0] = 2;
      idx[1] = 1;
      idx[2] = 3;
      b_x[0] = x[1];
      b_x[1] = x[0];
    } else {
      if ((x[1] <= x[2]) || rtIsNaNF(x[2])) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        idx[0] = 2;
        idx[1] = 3;
        idx[2] = 1;
        b_x[0] = x[1];
        b_x[1] = x[2];
        b_x[2] = x[0];
      } else {
        idx[0] = 3;
        idx[1] = 2;
        idx[2] = 1;
        b_x[0] = x[2];
        b_x[2] = x[0];
      }
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    x[i2] = b_x[i2];
  }
}

/*
 * File trailer for sort1.c
 *
 * [EOF]
 */
