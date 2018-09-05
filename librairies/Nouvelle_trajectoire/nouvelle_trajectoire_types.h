/*
 * File: nouvelle_trajectoire_types.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

#ifndef NOUVELLE_TRAJECTOIRE_TYPES_H
#define NOUVELLE_TRAJECTOIRE_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_Lgcb42vx
#define typedef_Lgcb42vx

typedef struct {
  float T0;
  float P0[3];
  float psi0;
  float psip0;
  float Pt[3];
  float psiw;
  float Vh;
  float Vv;
  float Tapp;
  float Vh0;
  float Vh2;
  float Vv2;
  float Tt;
  float Tturn;
  float x1;
  float y1;
  float z1;
  float x2;
  float y2;
  float cpsiw;
  float spsiw;
  float Wx;
  float Wy;
  float Wz;
  float cpsi0;
  float spsi0;
  float Par[2];
  float tj[1001];
  float xj[1001];
  float yj[1001];
  float psij[1001];
  float Vhj[1001];
  float Vvj[1001];
  float Tfar;
} Lgcb42vx;

#endif                                 /*typedef_Lgcb42vx*/

#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray__common*/

#ifndef typedef_emxArray__common
#define typedef_emxArray__common

typedef struct emxArray__common emxArray__common;

#endif                                 /*typedef_emxArray__common*/

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real32_T*/

#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T

typedef struct emxArray_real32_T emxArray_real32_T;

#endif                                 /*typedef_emxArray_real32_T*/
#endif

/*
 * File trailer for nouvelle_trajectoire_types.h
 *
 * [EOF]
 */
