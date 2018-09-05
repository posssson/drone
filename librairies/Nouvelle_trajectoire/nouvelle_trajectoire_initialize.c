/*
 * File: nouvelle_trajectoire_initialize.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "nouvelle_trajectoire_initialize.h"
#include "nouvelle_trajectoire_data.h"

/* Named Constants */
#define b_nfev                         (3.0F)
#define b_nsev                         (3.0F)

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void nouvelle_trajectoire_initialize(void)
{
  rt_InitInfAndNaN(8U);
  nsev = b_nsev;
  nfev = b_nfev;
}

/*
 * File trailer for nouvelle_trajectoire_initialize.c
 *
 * [EOF]
 */
