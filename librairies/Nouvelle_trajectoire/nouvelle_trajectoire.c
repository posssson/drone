/*
 * File: nouvelle_trajectoire.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "nouvelle_trajectoire_emxutil.h"
#include "sum.h"
#include "pbern.h"
#include "NelderMead.h"
#include "nouvelle_trajectoire_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const float xyh0[3]
 *                float Vw
 *                const float etat[5]
 *                float t
 *                float ua
 *                float b_psiw
 *                const float uvww0t[3]
 *                long initopt
 *                emxArray_real32_T *tref
 *                emxArray_real32_T *psiref
 *                emxArray_real32_T *posref
 *                float *val
 *                float *outopt
 *                float par[2]
 * Return Type  : void
 */
void nouvelle_trajectoire(const float xyh0[3], float Vw, const float etat[5],
  float t, float ua, float b_psiw, const float uvww0t[3], long initopt,
  emxArray_real32_T *tref, emxArray_real32_T *psiref, emxArray_real32_T *posref,
  float *val, float *outopt, float par[2])
{
  int i0;
  static Lgcb42vx lg;
  float uvww0_idx_0;
  float uvww0_idx_1;
  float a[5];
  float d1;
  float fcnOutput;
  int k;
  static const float b_a[5] = { 0.0F, -1.6689F, -0.2392F, 0.0F, 0.0F };

  static const float c_a[25] = { -0.6533F, -7.1735F, 6.9283F, 0.0F, 0.0F, 0.0F,
    -0.748F, 1.1949F, 1.0F, 0.0F, -1.0F, -1.4495F, -0.2516F, 0.0F, 1.0F, 1.2253F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F };

  float x;
  float b_x;
  float Par1[2];
  float d2;
  float xc11;
  float yc11;
  float xc21;
  float yc21;
  float xc22;
  static float B5[6006];
  float B4[5005];
  float b_xc11[5];
  float b_xyh0[5];
  float b_B4[1001];
  float c_B4[1001];
  float xp[1001];
  float b_yc11[5];
  float c_xyh0[5];
  float yp[1001];
  float psi[1001];
  float dtj[1000];
  int i;
  float d_xyh0[6];
  float e_xyh0[6];
  static const float fv0[4] = { 0.0F, 0.0F, 100.0F, 100.0F };

  float c_xc11[5];
  float b_lg[5];
  float c_yc11[5];
  float c_lg[5];
  float d_lg[6];
  float e_lg[6];
  float fv1[1001];
  double ndbl;
  double apnd;
  double cdiff;
  int n;
  emxArray_real32_T *x_;
  emxArray_real32_T *y_;
  emxArray_real32_T *h_;
  emxArray_real32_T *Vvj_;
  emxArray_real32_T *b_tref;
  emxArray_real32_T *b_x_;
  emxArray_real32_T *c_tref;
  float f_lg[3];
  (void)uvww0t;

  /*  Cible */
  /*  OBJET GUIDAGEï¿œ */
  for (i0 = 0; i0 < 2; i0++) {
    lg.Par[i0] = 0.0F;
  }

  for (i0 = 0; i0 < 3; i0++) {
    lg.Pt[i0] = 0.0F;
  }

  lg.Vh = 8.0F;
  lg.Vv = 2.5F;
  lg.Tapp = 1.0F;

  /*  Etat initial */
  /*  Tapp=0 W=5 */
  /*  OBJET PARAFOIL */
  /* C = [0 0 0 1 0]; % Phi */
  /*  Psi */
  uvww0_idx_0 = Vw * (float)cos(b_psiw);
  uvww0_idx_1 = Vw * (float)sin(b_psiw);

  /*  %% ON ENELVE PF=  Modpfc5 car ca bug */
  /* pf = Modpfc5(A,B,C,D); */
  /* pf.data(Vh, Vv); */
  /* xa = pf.init_state(pos0,etat); */
  /* [xyh0, Vh, Vv, psi, psip, beta, p, r, phi] = pf.outputs(t, ua, xa, uvww0t); */
  /*  */
  /*  CONSTRUCTOR */
  /*   persistent hh  ; */
  /*  hh = h; */
  /*  DATA */
  /*  ETAT_INIT */
  /*     Tturn = Tt - Tapp; */
  /*  CRIT_OPT */
  /*      d1 = Pd(1)/2; */
  /*      d2 = Pd(2)/2; */
  /*     Tturn = (P0(3)-Pt(3)-Vv*Tapp)/Vvmoy; */
  /*  x1 xc11 xc12 xc21 xc21 x2 */
  /*  Recalcul de xc12 et yc12 pour assurer la continuit� de psip0 */
  /*  dt variable */
  /*             hh = zeros(N,1); */
  /*         dtj(j-1) = sqrt( ((x(j)-x(j-1))^2+(y(j)-y(j-1))^2) / Vj2 ); */
  /*         psi(j) = atan2((y(j)-y(j-1))/dtj(j-1), (x(j)-x(j-1))/dtj(j-1)); */
  /*          if j<N */
  /*              psi(j) = atan2(y(j+1)-y(j-1), x(j+1)-x(j-1)); */
  /*          else */
  /*              psi(j) = atan2(y(j)-y(j-1), x(j)-x(j-1)); */
  /*          end */
  /*         [(x(j)-x(j-1))/dt xp(j)] */
  /*      psip(1) = psip0; */
  /*      psip(end) = psip(end-1); */
  /*             dz = hh.Vv*(hh.Tfar-hh.Tturn); */
  /*     C = dz^2 + 0*(0.01*psipmax)^2; */
  /*     C = (Tfar-Tturn); */
  /*     C = (Tfar-Tturn)^2 + 1*psipmax; */
  /*  OPT_TRAJ */
  lg.T0 = t;
  for (i0 = 0; i0 < 3; i0++) {
    lg.P0[i0] = xyh0[i0];
  }

  lg.psi0 = etat[4];
  for (i0 = 0; i0 < 5; i0++) {
    d1 = 0.0F;
    for (k = 0; k < 5; k++) {
      d1 += c_a[i0 + 5 * k] * etat[k];
    }

    a[i0] = d1 + b_a[i0] * ua;
  }

  lg.psip0 = a[4];
  fcnOutput = rt_atan2f_snf(uvww0_idx_1, uvww0_idx_0);
  lg.psiw = rt_atan2f_snf(uvww0_idx_1, uvww0_idx_0);
  lg.Wx = uvww0_idx_0;
  lg.Wy = uvww0_idx_1;
  lg.Wz = 0.0F;
  lg.x1 = xyh0[0];
  lg.y1 = xyh0[1];
  lg.z1 = xyh0[2];
  lg.cpsi0 = (float)cos(etat[4]);
  lg.spsi0 = (float)sin(etat[4]);
  x = (float)cos(lg.psiw);
  lg.cpsiw = (float)cos(fcnOutput);
  b_x = (float)sin(lg.psiw);
  lg.spsiw = (float)sin(fcnOutput);
  lg.Tturn = ((xyh0[2] - lg.Pt[2]) - 2.5F) / 2.5F;
  lg.Vh0 = 8.0F;
  lg.Vh2 = 8.0F;
  lg.Vv2 = 2.5F;
  for (i0 = 0; i0 < 2; i0++) {
    Par1[i0] = 0.0F;
  }

  if (initopt == 1L) {
    Par1[0] = 100.0F;
    Par1[1] = lg.Tturn;
  } else {
    if (initopt == 0L) {
      for (i0 = 0; i0 < 2; i0++) {
        Par1[i0] = lg.Par[i0];
      }
    }
  }

  d1 = (float)fabs(lg.Par[0]) / 2.0F;
  d2 = (float)fabs(lg.Par[0]) / 2.0F;
  fcnOutput = (float)fabs(lg.Par[1]);
  lg.Tturn = (float)fabs(lg.Par[1]);
  lg.Tt = lg.Tturn + 1.0F;
  lg.x2 = ((lg.Pt[0] + 8.0F * lg.cpsiw) - uvww0_idx_0) - uvww0_idx_0 * fcnOutput;
  lg.y2 = ((lg.Pt[1] + 8.0F * lg.spsiw) - uvww0_idx_1) - uvww0_idx_1 * fcnOutput;
  xc11 = xyh0[0] + d1 * (float)cos(etat[4]);
  yc11 = xyh0[1] + d1 * (float)sin(etat[4]);
  xc21 = lg.x2 + d2 * x;
  yc21 = lg.y2 + d2 * b_x;
  xc22 = lg.x2 + 2.0F * d2 * x;
  d1 = lg.y2 + 2.0F * d2 * b_x;
  uvww0_idx_0 = 5.0F * (xc11 - xyh0[0]);
  uvww0_idx_1 = 5.0F * (yc11 - xyh0[1]);
  x = (float)sqrt(64.0F / (uvww0_idx_0 * uvww0_idx_0 + uvww0_idx_1 * uvww0_idx_1));
  uvww0_idx_1 = (-8.0F * (float)sin(etat[4]) * lg.psip0 / 20.0F / (x * x) + 2.0F
                 * xc11) - xyh0[0];
  uvww0_idx_0 = (8.0F * (float)cos(etat[4]) * lg.psip0 / 20.0F / (x * x) + 2.0F *
                 yc11) - xyh0[1];
  pbern(B5);
  b_pbern(B4);
  b_xc11[0] = xc11;
  b_xc11[1] = uvww0_idx_1;
  b_xc11[2] = xc22;
  b_xc11[3] = xc21;
  b_xc11[4] = lg.x2;
  b_xyh0[0] = xyh0[0];
  b_xyh0[1] = xc11;
  b_xyh0[2] = uvww0_idx_1;
  b_xyh0[3] = xc22;
  b_xyh0[4] = xc21;
  for (i0 = 0; i0 < 1001; i0++) {
    b_B4[i0] = 0.0F;
    c_B4[i0] = 0.0F;
    for (k = 0; k < 5; k++) {
      b_B4[i0] += B4[i0 + 1001 * k] * b_xc11[k];
      c_B4[i0] += B4[i0 + 1001 * k] * b_xyh0[k];
    }

    xp[i0] = 5.0F * (b_B4[i0] - c_B4[i0]);
  }

  b_yc11[0] = yc11;
  b_yc11[1] = uvww0_idx_0;
  b_yc11[2] = d1;
  b_yc11[3] = yc21;
  b_yc11[4] = lg.y2;
  c_xyh0[0] = xyh0[1];
  c_xyh0[1] = yc11;
  c_xyh0[2] = uvww0_idx_0;
  c_xyh0[3] = d1;
  c_xyh0[4] = yc21;
  for (i0 = 0; i0 < 1001; i0++) {
    b_B4[i0] = 0.0F;
    c_B4[i0] = 0.0F;
    for (k = 0; k < 5; k++) {
      b_B4[i0] += B4[i0 + 1001 * k] * b_yc11[k];
      c_B4[i0] += B4[i0 + 1001 * k] * c_xyh0[k];
    }

    yp[i0] = 5.0F * (b_B4[i0] - c_B4[i0]);
  }

  memset(&lg.Vhj[0], 0, 1001U * sizeof(float));
  memset(&lg.Vvj[0], 0, 1001U * sizeof(float));
  memset(&psi[0], 0, 1001U * sizeof(float));
  psi[0] = etat[4];
  lg.Vhj[0] = 8.0F;
  lg.Vvj[0] = 2.5F;
  for (i = 0; i < 1000; i++) {
    dtj[i] = 0.001F * (float)sqrt(xp[i + 1] * xp[i + 1] + yp[i + 1] * yp[i + 1])
      / lg.Vhj[i];
    psi[i + 1] = rt_atan2f_snf(yp[i + 1], xp[i + 1]);
    if (psi[i + 1] > psi[i] + 3.14159274F) {
      psi[i + 1] -= 6.28318548F;
    }

    if (psi[i + 1] < psi[i] - 3.14159274F) {
      psi[i + 1] += 6.28318548F;
    }

    lg.Vhj[i + 1] = 8.0F;
    lg.Vvj[i + 1] = 2.5F;
  }

  d_xyh0[0] = xyh0[0];
  d_xyh0[1] = xc11;
  d_xyh0[2] = uvww0_idx_1;
  d_xyh0[3] = xc22;
  d_xyh0[4] = xc21;
  d_xyh0[5] = lg.x2;
  for (i0 = 0; i0 < 1001; i0++) {
    lg.xj[i0] = 0.0F;
    for (k = 0; k < 6; k++) {
      lg.xj[i0] += B5[i0 + 1001 * k] * d_xyh0[k];
    }
  }

  e_xyh0[0] = xyh0[1];
  e_xyh0[1] = yc11;
  e_xyh0[2] = uvww0_idx_0;
  e_xyh0[3] = d1;
  e_xyh0[4] = yc21;
  e_xyh0[5] = lg.y2;
  for (i0 = 0; i0 < 1001; i0++) {
    lg.yj[i0] = 0.0F;
    for (k = 0; k < 6; k++) {
      lg.yj[i0] += B5[i0 + 1001 * k] * e_xyh0[k];
    }
  }

  memcpy(&lg.psij[0], &psi[0], 1001U * sizeof(float));
  lg.Tfar = sum(dtj);
  for (k = 0; k < 999; k++) {
    dtj[k + 1] += dtj[k];
  }

  lg.tj[0] = t;
  for (i = 0; i < 1000; i++) {
    lg.tj[i + 1] = t + dtj[i];
  }

  NelderMead(Par1, fv0, &lg, par, &b_x);

  /* [par, val, flag, outopt] =fminsearch(@hh.optcrit,Par1); */
  /*        Par = fminsearch(crit_opt, Par1) */
  /*   [par, val, flag, outopt] =patternsearch(@hh.optcrit, Par1,options,hh); */
  /*                  if val>0.01, */
  /*                      flag = 0; */
  /*                  end */
  d1 = (float)fabs(par[0]) / 2.0F;
  d2 = (float)fabs(par[0]) / 2.0F;
  fcnOutput = (float)fabs(par[1]);
  lg.Tt = (float)fabs(par[1]) + lg.Tapp;
  lg.x2 = ((lg.Pt[0] + lg.Vh2 * lg.cpsiw * lg.Tapp) - lg.Wx * lg.Tapp) - lg.Wx *
    fcnOutput;
  lg.y2 = ((lg.Pt[1] + lg.Vh2 * lg.spsiw * lg.Tapp) - lg.Wy * lg.Tapp) - lg.Wy *
    fcnOutput;
  xc11 = lg.x1 + d1 * lg.cpsi0;
  yc11 = lg.y1 + d1 * lg.spsi0;
  xc21 = lg.x2 + d2 * lg.cpsiw;
  yc21 = lg.y2 + d2 * lg.spsiw;
  xc22 = lg.x2 + 2.0F * d2 * lg.cpsiw;
  d1 = lg.y2 + 2.0F * d2 * lg.spsiw;
  uvww0_idx_0 = 5.0F * (xc11 - lg.x1);
  uvww0_idx_1 = 5.0F * (yc11 - lg.y1);
  x = (float)sqrt(lg.Vh0 * lg.Vh0 / (uvww0_idx_0 * uvww0_idx_0 + uvww0_idx_1 *
    uvww0_idx_1));
  uvww0_idx_1 = (-lg.Vh0 * (float)sin(lg.psi0) * lg.psip0 / 20.0F / (x * x) +
                 2.0F * xc11) - lg.x1;
  uvww0_idx_0 = (lg.Vh0 * (float)cos(lg.psi0) * lg.psip0 / 20.0F / (x * x) +
                 2.0F * yc11) - lg.y1;
  pbern(B5);
  b_pbern(B4);
  c_xc11[0] = xc11;
  c_xc11[1] = uvww0_idx_1;
  c_xc11[2] = xc22;
  c_xc11[3] = xc21;
  c_xc11[4] = lg.x2;
  b_lg[0] = lg.x1;
  b_lg[1] = xc11;
  b_lg[2] = uvww0_idx_1;
  b_lg[3] = xc22;
  b_lg[4] = xc21;
  for (i0 = 0; i0 < 1001; i0++) {
    b_B4[i0] = 0.0F;
    c_B4[i0] = 0.0F;
    for (k = 0; k < 5; k++) {
      b_B4[i0] += B4[i0 + 1001 * k] * c_xc11[k];
      c_B4[i0] += B4[i0 + 1001 * k] * b_lg[k];
    }

    xp[i0] = 5.0F * (b_B4[i0] - c_B4[i0]);
  }

  c_yc11[0] = yc11;
  c_yc11[1] = uvww0_idx_0;
  c_yc11[2] = d1;
  c_yc11[3] = yc21;
  c_yc11[4] = lg.y2;
  c_lg[0] = lg.y1;
  c_lg[1] = yc11;
  c_lg[2] = uvww0_idx_0;
  c_lg[3] = d1;
  c_lg[4] = yc21;
  for (i0 = 0; i0 < 1001; i0++) {
    b_B4[i0] = 0.0F;
    c_B4[i0] = 0.0F;
    for (k = 0; k < 5; k++) {
      b_B4[i0] += B4[i0 + 1001 * k] * c_yc11[k];
      c_B4[i0] += B4[i0 + 1001 * k] * c_lg[k];
    }

    yp[i0] = 5.0F * (b_B4[i0] - c_B4[i0]);
  }

  memset(&lg.Vhj[0], 0, 1001U * sizeof(float));
  memset(&lg.Vvj[0], 0, 1001U * sizeof(float));
  memset(&psi[0], 0, 1001U * sizeof(float));
  psi[0] = lg.psi0;
  lg.Vhj[0] = lg.Vh;
  lg.Vvj[0] = lg.Vv;
  for (i = 0; i < 1000; i++) {
    dtj[i] = 0.001F * (float)sqrt(xp[i + 1] * xp[i + 1] + yp[i + 1] * yp[i + 1])
      / lg.Vhj[i];
    psi[i + 1] = rt_atan2f_snf(yp[i + 1], xp[i + 1]);
    if (psi[i + 1] > psi[i] + 3.14159274F) {
      psi[i + 1] -= 6.28318548F;
    }

    if (psi[i + 1] < psi[i] - 3.14159274F) {
      psi[i + 1] += 6.28318548F;
    }

    lg.Vhj[i + 1] = lg.Vh;
    lg.Vvj[i + 1] = lg.Vv;
  }

  d_lg[0] = lg.x1;
  d_lg[1] = xc11;
  d_lg[2] = uvww0_idx_1;
  d_lg[3] = xc22;
  d_lg[4] = xc21;
  d_lg[5] = lg.x2;
  for (i0 = 0; i0 < 1001; i0++) {
    lg.xj[i0] = 0.0F;
    for (k = 0; k < 6; k++) {
      lg.xj[i0] += B5[i0 + 1001 * k] * d_lg[k];
    }
  }

  e_lg[0] = lg.y1;
  e_lg[1] = yc11;
  e_lg[2] = uvww0_idx_0;
  e_lg[3] = d1;
  e_lg[4] = yc21;
  e_lg[5] = lg.y2;
  for (i0 = 0; i0 < 1001; i0++) {
    lg.yj[i0] = 0.0F;
    for (k = 0; k < 6; k++) {
      lg.yj[i0] += B5[i0 + 1001 * k] * e_lg[k];
    }
  }

  uvww0_idx_1 = sum(dtj);
  for (k = 0; k < 999; k++) {
    dtj[k + 1] += dtj[k];
  }

  fv1[0] = 0.0F;
  memcpy(&fv1[1], &dtj[0], 1000U * sizeof(float));
  for (i0 = 0; i0 < 1001; i0++) {
    lg.tj[i0] = lg.T0 + fv1[i0];
  }

  /*     [T, P, Psi, Ttur] = traj_ref(Par); */
  *val = b_x;
  *outopt = 0.0F;

  /*  TRAJ_REF */
  d1 = fcnOutput + lg.Tapp;
  if (rtIsNaNF(lg.Tt)) {
    i0 = tref->size[0] * tref->size[1];
    tref->size[0] = 1;
    tref->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)tref, i0, (int)sizeof(float));
    tref->data[0] = ((real32_T)rtNaN);
  } else if (lg.Tt < 0.0F) {
    i0 = tref->size[0] * tref->size[1];
    tref->size[0] = 1;
    tref->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)tref, i0, (int)sizeof(float));
  } else if (rtIsInfF(lg.Tt) && (0.0F == lg.Tt)) {
    i0 = tref->size[0] * tref->size[1];
    tref->size[0] = 1;
    tref->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)tref, i0, (int)sizeof(float));
    tref->data[0] = ((real32_T)rtNaN);
  } else {
    ndbl = floor(d1 / 0.0099999997764825821 + 0.5);
    apnd = ndbl * 0.0099999997764825821;
    cdiff = apnd - lg.Tt;
    if (fabs(cdiff) < 2.38418579E-7F * (float)fabs(d1)) {
      ndbl++;
      d1 = lg.Tt;
    } else if (cdiff > 0.0) {
      d1 = (float)((ndbl - 1.0) * 0.0099999997764825821);
    } else {
      ndbl++;
      d1 = (float)apnd;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }

    i0 = tref->size[0] * tref->size[1];
    tref->size[0] = 1;
    tref->size[1] = n;
    emxEnsureCapacity((emxArray__common *)tref, i0, (int)sizeof(float));
    if (n > 0) {
      tref->data[0] = 0.0F;
      if (n > 1) {
        tref->data[n - 1] = d1;
        i = (n - 1) / 2;
        for (k = 1; k < i; k++) {
          uvww0_idx_0 = (float)k * 0.01F;
          tref->data[k] = uvww0_idx_0;
          tref->data[(n - k) - 1] = d1 - uvww0_idx_0;
        }

        if (i << 1 == n - 1) {
          tref->data[i] = d1 / 2.0F;
        } else {
          uvww0_idx_0 = (float)i * 0.01F;
          tref->data[i] = uvww0_idx_0;
          tref->data[i + 1] = d1 - uvww0_idx_0;
        }
      }
    }
  }

  i0 = tref->size[0] * tref->size[1];
  tref->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)tref, i0, (int)sizeof(float));
  i = tref->size[0];
  k = tref->size[1];
  i *= k;
  for (i0 = 0; i0 < i; i0++) {
    tref->data[i0] += lg.T0;
  }

  emxInit_real32_T(&x_, 1);
  i0 = x_->size[0];
  x_->size[0] = (int)(float)tref->size[1];
  emxEnsureCapacity((emxArray__common *)x_, i0, (int)sizeof(float));
  i = (int)(float)tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    x_->data[i0] = 0.0F;
  }

  emxInit_real32_T(&y_, 1);
  i0 = y_->size[0];
  y_->size[0] = (int)(float)tref->size[1];
  emxEnsureCapacity((emxArray__common *)y_, i0, (int)sizeof(float));
  i = (int)(float)tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    y_->data[i0] = 0.0F;
  }

  emxInit_real32_T(&h_, 1);
  i0 = h_->size[0];
  h_->size[0] = (int)(float)tref->size[1];
  emxEnsureCapacity((emxArray__common *)h_, i0, (int)sizeof(float));
  i = (int)(float)tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    h_->data[i0] = 0.0F;
  }

  i0 = psiref->size[0];
  psiref->size[0] = (int)(float)tref->size[1];
  emxEnsureCapacity((emxArray__common *)psiref, i0, (int)sizeof(float));
  i = (int)(float)tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    psiref->data[i0] = 0.0F;
  }

  emxInit_real32_T(&Vvj_, 1);
  i0 = Vvj_->size[0];
  Vvj_->size[0] = (int)(float)tref->size[1];
  emxEnsureCapacity((emxArray__common *)Vvj_, i0, (int)sizeof(float));
  i = (int)(float)tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    Vvj_->data[i0] = 0.0F;
  }

  x_->data[0] = lg.xj[0];
  y_->data[0] = lg.yj[0];
  h_->data[0] = lg.z1;
  psiref->data[0] = psi[0];
  Vvj_->data[0] = lg.Vvj[0];
  i = 0;
  for (k = 0; k < (int)((float)tref->size[1] + -1.0F); k++) {
    if (tref->data[(int)(2.0F + (float)k) - 1] - lg.T0 < uvww0_idx_1) {
      while ((lg.tj[i + 1] < tref->data[(int)(2.0F + (float)k) - 1]) && (i + 2 <
              1001)) {
        i++;
      }

      d1 = (tref->data[(int)(2.0F + (float)k) - 1] - lg.tj[i]) / (lg.tj[i + 1] -
        lg.tj[i]);
      x_->data[(int)(2.0F + (float)k) - 1] = lg.xj[i] + d1 * (lg.xj[i + 1] -
        lg.xj[i]);
      y_->data[(int)(2.0F + (float)k) - 1] = lg.yj[i] + d1 * (lg.yj[i + 1] -
        lg.yj[i]);
      psiref->data[(int)(2.0F + (float)k) - 1] = psi[i] + d1 * (psi[i + 1] -
        psi[i]);
      Vvj_->data[(int)(2.0F + (float)k) - 1] = lg.Vvj[i] + d1 * (lg.Vvj[i + 1] -
        lg.Vvj[i]);
      h_->data[(int)(2.0F + (float)k) - 1] = h_->data[(int)((2.0F + (float)k) -
        1.0F) - 1] - Vvj_->data[(int)((2.0F + (float)k) - 1.0F) - 1] * 0.01F;
    } else {
      x_->data[(int)(2.0F + (float)k) - 1] = x_->data[(int)((2.0F + (float)k) -
        1.0F) - 1] - lg.Vh * lg.cpsiw * 0.01F;
      y_->data[(int)(2.0F + (float)k) - 1] = y_->data[(int)((2.0F + (float)k) -
        1.0F) - 1] - lg.Vh * lg.spsiw * 0.01F;
      psiref->data[(int)(2.0F + (float)k) - 1] = psiref->data[(int)((2.0F +
        (float)k) - 1.0F) - 1];
    }

    Vvj_->data[(int)(2.0F + (float)k) - 1] = lg.Vv2;
    h_->data[(int)(2.0F + (float)k) - 1] = h_->data[(int)((2.0F + (float)k) -
      1.0F) - 1] - lg.Vv * 0.01F;
  }

  emxFree_real32_T(&Vvj_);
  emxInit_real32_T(&b_tref, 1);
  i0 = b_tref->size[0];
  b_tref->size[0] = tref->size[1];
  emxEnsureCapacity((emxArray__common *)b_tref, i0, (int)sizeof(float));
  i = tref->size[1];
  for (i0 = 0; i0 < i; i0++) {
    b_tref->data[i0] = tref->data[tref->size[0] * i0] - lg.T0;
  }

  emxInit_real32_T1(&b_x_, 2);
  emxInit_real32_T1(&c_tref, 2);
  f_lg[0] = lg.Wx;
  f_lg[1] = lg.Wy;
  f_lg[2] = -lg.Wz;
  i0 = b_x_->size[0] * b_x_->size[1];
  b_x_->size[0] = x_->size[0];
  b_x_->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)b_x_, i0, (int)sizeof(float));
  i = x_->size[0];
  for (i0 = 0; i0 < i; i0++) {
    b_x_->data[i0] = x_->data[i0];
  }

  emxFree_real32_T(&x_);
  i = y_->size[0];
  for (i0 = 0; i0 < i; i0++) {
    b_x_->data[i0 + b_x_->size[0]] = y_->data[i0];
  }

  emxFree_real32_T(&y_);
  i = h_->size[0];
  for (i0 = 0; i0 < i; i0++) {
    b_x_->data[i0 + (b_x_->size[0] << 1)] = h_->data[i0];
  }

  emxFree_real32_T(&h_);
  i0 = c_tref->size[0] * c_tref->size[1];
  c_tref->size[0] = b_tref->size[0];
  c_tref->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)c_tref, i0, (int)sizeof(float));
  i = b_tref->size[0];
  for (i0 = 0; i0 < i; i0++) {
    for (k = 0; k < 3; k++) {
      c_tref->data[i0 + c_tref->size[0] * k] = b_tref->data[i0] * f_lg[k];
    }
  }

  emxFree_real32_T(&b_tref);
  i0 = posref->size[0] * posref->size[1];
  posref->size[0] = b_x_->size[0];
  posref->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)posref, i0, (int)sizeof(float));
  for (i0 = 0; i0 < 3; i0++) {
    i = b_x_->size[0];
    for (k = 0; k < i; k++) {
      posref->data[k + posref->size[0] * i0] = b_x_->data[k + b_x_->size[0] * i0]
        + c_tref->data[k + c_tref->size[0] * i0];
    }
  }

  emxFree_real32_T(&c_tref);
  emxFree_real32_T(&b_x_);
}

/*
 * File trailer for nouvelle_trajectoire.c
 *
 * [EOF]
 */
