/*
 * File: NelderMead.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 21-Jul-2017 11:43:03
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "nouvelle_trajectoire.h"
#include "NelderMead.h"
#include "abs.h"
#include "sum.h"
#include "mean.h"
#include "pbern.h"
#include "sort1.h"
#include "diag.h"
#include "nouvelle_trajectoire_rtwutil.h"
#include "nouvelle_trajectoire_data.h"

/* Function Declarations */
static void Seval(const float S[6], Lgcb42vx *h, float FV[3]);
static void init2(const float b_P0[2], float P[6]);

/* Function Definitions */

/*
 * Arguments    : const float S[6]
 *                Lgcb42vx *h
 *                float FV[3]
 * Return Type  : void
 */
static void Seval(const float S[6], Lgcb42vx *h, float FV[3])
{
  int i;
  Lgcb42vx *hh;
  float d1;
  float d2;
  float a;
  float b;
  float b_b;
  float b_a;
  float c_a;
  float c_b;
  float d_a;
  float d_b;
  float e_a;
  float xc11;
  float yc11;
  float xc21;
  float yc21;
  float xc22;
  float yc22;
  float x;
  float b_x;
  float f_a;
  float e_b;
  float yc12;
  float B5[6006];
  float f_b[6];
  float g_b[6];
  float B4[5005];
  float h_b[5];
  float i_b[5];
  float b_B4[1001];
  float c_B4[1001];
  float xp[1001];
  int b_i;
  int ix;
  float yp[1001];
  float psi[1001];
  float psip[1001];
  float dtj[1000];
  boolean_T exitg1;
  float b_d2;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  disp('évaluation'); */
  /*  S */
  for (i = 0; i < 3; i++) {
    /*   F(i)=0; */
    /*  FV(i) = feval(fnc,S(:,i),h); */
    hh = h;
    d1 = (float)fabs(S[i << 1]) / 2.0F;
    d2 = (float)fabs(S[i << 1]) / 2.0F;
    hh->Tturn = (float)fabs(S[1 + (i << 1)]);
    a = hh->Tturn;
    b = hh->Tapp;
    hh->Tt = a + b;
    a = hh->Vh2;
    b = hh->cpsiw;
    b_b = hh->Tapp;
    b_a = hh->Pt[0];
    c_a = hh->Wx;
    c_b = hh->Tapp;
    d_a = hh->Wx;
    d_b = hh->Tturn;
    hh->x2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
    a = hh->Vh2;
    b = hh->spsiw;
    b_b = hh->Tapp;
    b_a = hh->Pt[1];
    c_a = hh->Wy;
    c_b = hh->Tapp;
    d_a = hh->Wy;
    d_b = hh->Tturn;
    hh->y2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
    a = hh->Vv2;
    b = hh->Wz;
    b_b = hh->Tapp;
    b_a = hh->Pt[2];
    c_a = hh->Wz;
    c_b = hh->Tturn;
    d_a = hh->P0[2];
    d_b = hh->cpsi0;
    e_a = hh->x1;
    xc11 = e_a + d1 * d_b;
    d_b = hh->spsi0;
    e_a = hh->y1;
    yc11 = e_a + d1 * d_b;
    d_b = hh->cpsiw;
    e_a = hh->x2;
    xc21 = e_a + d2 * d_b;
    d_b = hh->spsiw;
    e_a = hh->y2;
    yc21 = e_a + d2 * d_b;
    d_b = hh->cpsiw;
    e_a = hh->x2;
    xc22 = e_a + 2.0F * d2 * d_b;
    d_b = hh->spsiw;
    e_a = hh->y2;
    yc22 = e_a + 2.0F * d2 * d_b;
    x = hh->psi0;
    e_a = -hh->Vh0;
    d_b = hh->psip0;
    b_x = hh->psi0;
    f_a = hh->Vh0;
    e_b = hh->psip0;
    d1 = hh->x1;
    d2 = 5.0F * (xc11 - d1);
    d1 = hh->y1;
    d1 = 5.0F * (yc11 - d1);
    yc12 = hh->Vh0;
    yc12 = (float)sqrt(yc12 * yc12 / (d2 * d2 + d1 * d1));
    d1 = hh->x1;
    d1 = (e_a * (float)sin(x) * d_b / 20.0F / (yc12 * yc12) + 2.0F * xc11) - d1;
    d_b = hh->y1;
    yc12 = (f_a * (float)cos(b_x) * e_b / 20.0F / (yc12 * yc12) + 2.0F * yc11) -
      d_b;
    pbern(B5);
    f_b[0] = hh->x1;
    f_b[1] = xc11;
    f_b[2] = d1;
    f_b[3] = xc22;
    f_b[4] = xc21;
    f_b[5] = hh->x2;
    g_b[0] = hh->y1;
    g_b[1] = yc11;
    g_b[2] = yc12;
    g_b[3] = yc22;
    g_b[4] = yc21;
    g_b[5] = hh->y2;
    b_pbern(B4);
    h_b[0] = xc11;
    h_b[1] = d1;
    h_b[2] = xc22;
    h_b[3] = xc21;
    h_b[4] = hh->x2;
    i_b[0] = hh->x1;
    i_b[1] = xc11;
    i_b[2] = d1;
    i_b[3] = xc22;
    i_b[4] = xc21;
    for (b_i = 0; b_i < 1001; b_i++) {
      b_B4[b_i] = 0.0F;
      c_B4[b_i] = 0.0F;
      for (ix = 0; ix < 5; ix++) {
        b_B4[b_i] += B4[b_i + 1001 * ix] * h_b[ix];
        c_B4[b_i] += B4[b_i + 1001 * ix] * i_b[ix];
      }

      xp[b_i] = 5.0F * (b_B4[b_i] - c_B4[b_i]);
    }

    h_b[0] = yc11;
    h_b[1] = yc12;
    h_b[2] = yc22;
    h_b[3] = yc21;
    h_b[4] = hh->y2;
    i_b[0] = hh->y1;
    i_b[1] = yc11;
    i_b[2] = yc12;
    i_b[3] = yc22;
    i_b[4] = yc21;
    for (b_i = 0; b_i < 1001; b_i++) {
      b_B4[b_i] = 0.0F;
      c_B4[b_i] = 0.0F;
      for (ix = 0; ix < 5; ix++) {
        b_B4[b_i] += B4[b_i + 1001 * ix] * h_b[ix];
        c_B4[b_i] += B4[b_i + 1001 * ix] * i_b[ix];
      }

      yp[b_i] = 5.0F * (b_B4[b_i] - c_B4[b_i]);
    }

    for (b_i = 0; b_i < 1001; b_i++) {
      hh->Vhj[b_i] = 0.0F;
    }

    for (b_i = 0; b_i < 1001; b_i++) {
      hh->Vvj[b_i] = 0.0F;
    }

    memset(&psi[0], 0, 1001U * sizeof(float));
    memset(&psip[0], 0, 1001U * sizeof(float));
    psi[0] = hh->psi0;
    psip[0] = hh->psip0;
    hh->Vhj[0] = hh->Vh;
    hh->Vvj[0] = hh->Vv;
    for (b_i = 0; b_i < 1000; b_i++) {
      d1 = hh->Vhj[b_i];
      yc12 = 0.001F * (float)sqrt(xp[b_i + 1] * xp[b_i + 1] + yp[b_i + 1] *
        yp[b_i + 1]) / d1;
      psi[b_i + 1] = rt_atan2f_snf(yp[b_i + 1], xp[b_i + 1]);
      if (psi[b_i + 1] > psi[b_i] + 3.14159274F) {
        psi[b_i + 1] -= 6.28318548F;
      }

      if (psi[b_i + 1] < psi[b_i] - 3.14159274F) {
        psi[b_i + 1] += 6.28318548F;
      }

      psip[b_i + 1] = (psi[b_i + 1] - psi[b_i]) / yc12;
      hh->Vhj[b_i + 1] = hh->Vh;
      hh->Vvj[b_i + 1] = hh->Vv;
      dtj[b_i] = yc12;
    }

    for (b_i = 0; b_i < 1001; b_i++) {
      hh->xj[b_i] = 0.0F;
      for (ix = 0; ix < 6; ix++) {
        hh->xj[b_i] += B5[b_i + 1001 * ix] * f_b[ix];
      }
    }

    for (b_i = 0; b_i < 1001; b_i++) {
      hh->yj[b_i] = 0.0F;
      for (ix = 0; ix < 6; ix++) {
        hh->yj[b_i] += B5[b_i + 1001 * ix] * g_b[ix];
      }
    }

    for (b_i = 0; b_i < 1001; b_i++) {
      hh->psij[b_i] = psi[b_i];
    }

    d1 = mean(hh->Vvj);
    hh->Tfar = sum(dtj);
    for (b_i = 0; b_i < 999; b_i++) {
      dtj[b_i + 1] += dtj[b_i];
    }

    e_a = hh->T0;
    hh->tj[0] = e_a;
    for (b_i = 0; b_i < 1000; b_i++) {
      hh->tj[b_i + 1] = e_a + dtj[b_i];
    }

    b_abs(psip, xp);
    b_i = 1;
    yc12 = xp[0];
    if (rtIsNaNF(xp[0])) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix < 1002)) {
        b_i = ix;
        if (!rtIsNaNF(xp[ix - 1])) {
          yc12 = xp[ix - 1];
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (b_i < 1001) {
      while (b_i + 1 < 1002) {
        if (xp[b_i] > yc12) {
          yc12 = xp[b_i];
        }

        b_i++;
      }
    }

    d2 = yc12 * 180.0F / 3.14159274F;
    d_b = hh->Tfar;
    yc12 = (d_a - ((b_a + (a + b) * b_b) + c_a * c_b)) - d1 * d_b;
    a = hh->Tfar;
    b = hh->Tturn;
    d1 = a - b;
    if (d2 - 30.0F >= 0.0F) {
      b_d2 = d2 - 30.0F;
    } else {
      b_d2 = 0.0F;
    }

    FV[i] = (yc12 * yc12 + d1 * d1) + b_d2;

    /* F(i) = FV(i); */
    nfev++;
    nsev++;

    /*      figure(1); */
    /*      defplot(3,1,1,1); */
    /*      plot(nsev,FV(i),'o'); */
    /*      hold on; */
    /*      set(gca,'XLim',[1 10]); */
    /*  */
    /*      defplot(3,1,2,1); */
    /*      plot(nsev,FC(i,1),'o'); */
    /*      hold on; */
    /*      set(gca,'XLim',[1 10]); */
    /*  */
    /*      defplot(3,1,3,1); */
    /*      plot(nsev,F(i),'o'); */
    /*      hold on; */
    /*      set(gca,'XLim',[1 10]); */
  }

  /* pause */
}

/*
 * Arguments    : const float b_P0[2]
 *                float P[6]
 * Return Type  : void
 */
static void init2(const float b_P0[2], float P[6])
{
  int i1;
  int i;
  float Pi[2];
  for (i1 = 0; i1 < 2; i1++) {
    P[i1] = b_P0[i1];
  }

  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 2; i1++) {
      Pi[i1] = b_P0[i1];
    }

    Pi[i] = 1.05F * b_P0[i];
    for (i1 = 0; i1 < 2; i1++) {
      P[i1 + ((i + 1) << 1)] = Pi[i1];
    }
  }
}

/*
 * Arguments    : const float b_P0[2]
 *                const float Pminmax[4]
 *                Lgcb42vx *h
 *                float P[2]
 *                float *C
 * Return Type  : void
 */
void NelderMead(const float b_P0[2], const float Pminmax[4], Lgcb42vx *h, float
                P[2], float *C)
{
  float maxval[2];
  int ix;
  float value[4];
  float S0[6];
  float F[3];
  int k;
  int exitg1;
  int iidx[3];
  float b_S0[6];
  int ixstart;
  float b_value[4];
  float c_S0[4];
  int i;
  int b_ix;
  float d1;
  boolean_T exitg5;
  boolean_T exitg6;
  float Pc[2];
  float Pr[2];
  Lgcb42vx *hh;
  float d2;
  float a;
  float b;
  float b_b;
  float b_a;
  float c_a;
  float c_b;
  float d_a;
  float d_b;
  float e_a;
  float xc11;
  float yc11;
  float xc21;
  float yc21;
  float xc22;
  float yc22;
  float x;
  float b_x;
  float f_a;
  float e_b;
  float xp0_;
  float B5[6006];
  float f_b[6];
  float g_b[6];
  float B4[5005];
  float h_b[5];
  float i_b[5];
  float b_B4[1001];
  float c_B4[1001];
  float xp[1001];
  float yp[1001];
  float psi[1001];
  float psip[1001];
  float dtj[1000];
  boolean_T exitg4;
  float b_xp0_;
  float Cr;
  boolean_T exitg3;
  boolean_T exitg2;
  float c_xp0_;
  float d_xp0_;

  /*  Méthode de Nelder Mead */
  /*  pénalisation adaptative */
  /*  projection sur les bornes */
  /*  fnc : fonction à maximiser */
  /*  P0 : valeur initiale des paramètres */
  /*  Pminmax : bornes des paramètres [min max] */
  /*  nit : nombre maximal d'itérations */
  /*  kml : coefficient d'adaptation du multiplicateur de Lagrange */
  /*  P : paramètres optimisés */
  /*  C : valeur optimale du critère */
  /*  HP : historique des paramètres */
  /*  HC : historique du critère */
  /*  Constantes de la méthode */
  /*  Taille du simplex initial */
  /* a = 0.5;        % Taille du simplex initial */
  /* a = 1.4; */
  /*  Taille minimale du simplex */
  /*  Bornes */
  for (ix = 0; ix < 2; ix++) {
    maxval[ix] = 1.0F / (Pminmax[2 + ix] - Pminmax[ix]);
  }

  diag(maxval, value);

  /*  Simplex initial */
  init2(b_P0, S0);

  /*  Initialisation des variables */
  /* disp('Simplex initial') */
  /* disp('paramètres :'); */
  /* S */
  nfev = 0.0F;
  nsev = 0.0F;

  /*  figure(1); */
  /*  clf; */
  /*  figure(2); */
  /*  clf; */
  /*  Calcul du critère du simplex initial */
  /* [FV, FC, F] = Seval(fnc, S); */
  Seval(S0, h, F);

  /*  nombre de paramètres */
  /*  nombre de contraintes */
  /* nc = size(FC,2); */
  /*  Multiplicateurs de Lagrange */
  /* ml = zeros(1,nc); */
  /* F = Lagrang(FV, FC, ml, kml); */
  /* disp('Simplex initial'); */
  /* disp('critère :'); */
  /* F */
  /* , FC, ml, F */
  /* pause; */
  k = 1;

  /* tic; */
  do {
    exitg1 = 0;

    /*    Tri selon les valeurs croissantes du critères */
    sort(F, iidx);
    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 2; ixstart++) {
        b_S0[ixstart + (ix << 1)] = S0[ixstart + (((int)(float)iidx[ix] - 1) <<
          1)];
      }
    }

    for (ix = 0; ix < 3; ix++) {
      for (ixstart = 0; ixstart < 2; ixstart++) {
        S0[ixstart + (ix << 1)] = b_S0[ixstart + (ix << 1)];
      }
    }

    /*    FV = FV(is); */
    /*     FC = FC(is,:); */
    /*     F = F(is); */
    /*     %%    HP(k,:) = S(:,1)'; */
    /*     %%    HC(k) = F(1); */
    nsev = 3.0F;

    /*       fprintf(1,'\nItération %d  ', k); */
    /*       fprintf(1,'  Evaluation %d  ', nfev); */
    /*       fprintf(1,'  Critère %10.4f\n', F(1)); */
    k++;
    if (k > 100) {
      exitg1 = 1;
    } else {
      /*    Critère d'arret */
      /*     if norm([std(S')]'./DP) < smin, */
      for (ix = 0; ix < 2; ix++) {
        for (ixstart = 0; ixstart < 2; ixstart++) {
          c_S0[ix + (ixstart << 1)] = S0[ix + ((1 + ixstart) << 1)] - S0[ix];
        }
      }

      for (ix = 0; ix < 2; ix++) {
        for (ixstart = 0; ixstart < 2; ixstart++) {
          b_value[ix + (ixstart << 1)] = 0.0F;
          for (b_ix = 0; b_ix < 2; b_ix++) {
            b_value[ix + (ixstart << 1)] += value[ix + (b_ix << 1)] * c_S0[b_ix
              + (ixstart << 1)];
          }
        }
      }

      c_abs(b_value, c_S0);
      for (i = 0; i < 2; i++) {
        b_ix = i << 1;
        ixstart = (i << 1) + 1;
        d1 = c_S0[b_ix];
        if (rtIsNaNF(c_S0[b_ix])) {
          ix = ixstart + 1;
          exitg6 = false;
          while ((!exitg6) && (ix <= b_ix + 2)) {
            ixstart = ix;
            if (!rtIsNaNF(c_S0[ix - 1])) {
              d1 = c_S0[ix - 1];
              exitg6 = true;
            } else {
              ix++;
            }
          }
        }

        if (ixstart < b_ix + 2) {
          while (ixstart + 1 <= b_ix + 2) {
            if (c_S0[ixstart] > d1) {
              d1 = c_S0[ixstart];
            }

            ixstart++;
          }
        }

        maxval[i] = d1;
      }

      ixstart = 1;
      d1 = maxval[0];
      if (rtIsNaNF(maxval[0])) {
        b_ix = 2;
        exitg5 = false;
        while ((!exitg5) && (b_ix < 3)) {
          ixstart = 2;
          if (!rtIsNaNF(maxval[1])) {
            d1 = maxval[1];
            exitg5 = true;
          } else {
            b_ix = 3;
          }
        }
      }

      if ((ixstart < 2) && (maxval[1] > d1)) {
        d1 = maxval[1];
      }

      if (d1 < 0.0001F) {
        /*         disp('Convergence du simplex'); */
        exitg1 = 1;
      } else {
        /*    Test de degenerescence */
        /*      [whatconv,loopstop] = checkconv(S, F, Pminmax); */
        /*      if loopstop == 1, */
        /*          whatconv */
        /*          pause */
        /*      end */
        /*    Centroid */
        for (ix = 0; ix < 2; ix++) {
          for (ixstart = 0; ixstart < 2; ixstart++) {
            c_S0[ixstart + (ix << 1)] = S0[ix + (ixstart << 1)];
          }
        }

        b_sum(c_S0, maxval);

        /*    Reflexion */
        /*     Pr = Pc + 1*(Pc - S(:,np+1)); */
        for (ix = 0; ix < 2; ix++) {
          d1 = 0.5F * maxval[ix];
          Pr[ix] = 2.0F * d1 - S0[4 + ix];
          Pc[ix] = d1;
        }

        /*     Pr = Slimit(Pr, Pminmax); */
        /*     [FVr, FCr, Cr] = Seval(fnc,Pr,ml); */
        /*     Cr = Seval(fnc,Pr); */
        /*  Cr = feval(fnc,Pr,h); */
        hh = h;
        d1 = (float)fabs(Pr[0]) / 2.0F;
        d2 = (float)fabs(Pr[0]) / 2.0F;
        hh->Tturn = (float)fabs(Pr[1]);
        a = hh->Tturn;
        b = hh->Tapp;
        hh->Tt = a + b;
        a = hh->Vh2;
        b = hh->cpsiw;
        b_b = hh->Tapp;
        b_a = hh->Pt[0];
        c_a = hh->Wx;
        c_b = hh->Tapp;
        d_a = hh->Wx;
        d_b = hh->Tturn;
        hh->x2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
        a = hh->Vh2;
        b = hh->spsiw;
        b_b = hh->Tapp;
        b_a = hh->Pt[1];
        c_a = hh->Wy;
        c_b = hh->Tapp;
        d_a = hh->Wy;
        d_b = hh->Tturn;
        hh->y2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
        a = hh->Vv2;
        b = hh->Wz;
        b_b = hh->Tapp;
        b_a = hh->Pt[2];
        c_a = hh->Wz;
        c_b = hh->Tturn;
        d_a = hh->P0[2];
        d_b = hh->cpsi0;
        e_a = hh->x1;
        xc11 = e_a + d1 * d_b;
        d_b = hh->spsi0;
        e_a = hh->y1;
        yc11 = e_a + d1 * d_b;
        d_b = hh->cpsiw;
        e_a = hh->x2;
        xc21 = e_a + d2 * d_b;
        d_b = hh->spsiw;
        e_a = hh->y2;
        yc21 = e_a + d2 * d_b;
        d_b = hh->cpsiw;
        e_a = hh->x2;
        xc22 = e_a + 2.0F * d2 * d_b;
        d_b = hh->spsiw;
        e_a = hh->y2;
        yc22 = e_a + 2.0F * d2 * d_b;
        x = hh->psi0;
        e_a = -hh->Vh0;
        d_b = hh->psip0;
        b_x = hh->psi0;
        f_a = hh->Vh0;
        e_b = hh->psip0;
        d1 = hh->x1;
        xp0_ = 5.0F * (xc11 - d1);
        d1 = hh->y1;
        d1 = 5.0F * (yc11 - d1);
        d2 = hh->Vh0;
        d2 = (float)sqrt(d2 * d2 / (xp0_ * xp0_ + d1 * d1));
        d1 = hh->x1;
        xp0_ = (e_a * (float)sin(x) * d_b / 20.0F / (d2 * d2) + 2.0F * xc11) -
          d1;
        d_b = hh->y1;
        d1 = (f_a * (float)cos(b_x) * e_b / 20.0F / (d2 * d2) + 2.0F * yc11) -
          d_b;
        pbern(B5);
        f_b[0] = hh->x1;
        f_b[1] = xc11;
        f_b[2] = xp0_;
        f_b[3] = xc22;
        f_b[4] = xc21;
        f_b[5] = hh->x2;
        g_b[0] = hh->y1;
        g_b[1] = yc11;
        g_b[2] = d1;
        g_b[3] = yc22;
        g_b[4] = yc21;
        g_b[5] = hh->y2;
        b_pbern(B4);
        h_b[0] = xc11;
        h_b[1] = xp0_;
        h_b[2] = xc22;
        h_b[3] = xc21;
        h_b[4] = hh->x2;
        i_b[0] = hh->x1;
        i_b[1] = xc11;
        i_b[2] = xp0_;
        i_b[3] = xc22;
        i_b[4] = xc21;
        for (ix = 0; ix < 1001; ix++) {
          b_B4[ix] = 0.0F;
          c_B4[ix] = 0.0F;
          for (ixstart = 0; ixstart < 5; ixstart++) {
            b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
            c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
          }

          xp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
        }

        h_b[0] = yc11;
        h_b[1] = d1;
        h_b[2] = yc22;
        h_b[3] = yc21;
        h_b[4] = hh->y2;
        i_b[0] = hh->y1;
        i_b[1] = yc11;
        i_b[2] = d1;
        i_b[3] = yc22;
        i_b[4] = yc21;
        for (ix = 0; ix < 1001; ix++) {
          b_B4[ix] = 0.0F;
          c_B4[ix] = 0.0F;
          for (ixstart = 0; ixstart < 5; ixstart++) {
            b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
            c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
          }

          yp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
        }

        for (i = 0; i < 1001; i++) {
          hh->Vhj[i] = 0.0F;
        }

        for (i = 0; i < 1001; i++) {
          hh->Vvj[i] = 0.0F;
        }

        memset(&psi[0], 0, 1001U * sizeof(float));
        memset(&psip[0], 0, 1001U * sizeof(float));
        psi[0] = hh->psi0;
        psip[0] = hh->psip0;
        hh->Vhj[0] = hh->Vh;
        hh->Vvj[0] = hh->Vv;
        for (ixstart = 0; ixstart < 1000; ixstart++) {
          d1 = hh->Vhj[ixstart];
          d1 = 0.001F * (float)sqrt(xp[ixstart + 1] * xp[ixstart + 1] +
            yp[ixstart + 1] * yp[ixstart + 1]) / d1;
          psi[ixstart + 1] = rt_atan2f_snf(yp[ixstart + 1], xp[ixstart + 1]);
          if (psi[ixstart + 1] > psi[ixstart] + 3.14159274F) {
            psi[ixstart + 1] -= 6.28318548F;
          }

          if (psi[ixstart + 1] < psi[ixstart] - 3.14159274F) {
            psi[ixstart + 1] += 6.28318548F;
          }

          psip[ixstart + 1] = (psi[ixstart + 1] - psi[ixstart]) / d1;
          hh->Vhj[ixstart + 1] = hh->Vh;
          hh->Vvj[ixstart + 1] = hh->Vv;
          dtj[ixstart] = d1;
        }

        for (ix = 0; ix < 1001; ix++) {
          hh->xj[ix] = 0.0F;
          for (ixstart = 0; ixstart < 6; ixstart++) {
            hh->xj[ix] += B5[ix + 1001 * ixstart] * f_b[ixstart];
          }
        }

        for (ix = 0; ix < 1001; ix++) {
          hh->yj[ix] = 0.0F;
          for (ixstart = 0; ixstart < 6; ixstart++) {
            hh->yj[ix] += B5[ix + 1001 * ixstart] * g_b[ixstart];
          }
        }

        for (i = 0; i < 1001; i++) {
          hh->psij[i] = psi[i];
        }

        d2 = mean(hh->Vvj);
        hh->Tfar = sum(dtj);
        for (ixstart = 0; ixstart < 999; ixstart++) {
          dtj[ixstart + 1] += dtj[ixstart];
        }

        e_a = hh->T0;
        hh->tj[0] = e_a;
        for (i = 0; i < 1000; i++) {
          hh->tj[i + 1] = e_a + dtj[i];
        }

        b_abs(psip, xp);
        ixstart = 1;
        d1 = xp[0];
        if (rtIsNaNF(xp[0])) {
          b_ix = 2;
          exitg4 = false;
          while ((!exitg4) && (b_ix < 1002)) {
            ixstart = b_ix;
            if (!rtIsNaNF(xp[b_ix - 1])) {
              d1 = xp[b_ix - 1];
              exitg4 = true;
            } else {
              b_ix++;
            }
          }
        }

        if (ixstart < 1001) {
          while (ixstart + 1 < 1002) {
            if (xp[ixstart] > d1) {
              d1 = xp[ixstart];
            }

            ixstart++;
          }
        }

        xp0_ = d1 * 180.0F / 3.14159274F;
        d_b = hh->Tfar;
        d2 = (d_a - ((b_a + (a + b) * b_b) + c_a * c_b)) - d2 * d_b;
        a = hh->Tfar;
        b = hh->Tturn;
        d1 = a - b;
        if (xp0_ - 30.0F >= 0.0F) {
          b_xp0_ = xp0_ - 30.0F;
        } else {
          b_xp0_ = 0.0F;
        }

        Cr = (d2 * d2 + d1 * d1) + b_xp0_;
        if (Cr < F[0]) {
          /*        Expansion */
          /*         Pe = Pc + 2*(Pr-Pc); */
          for (ix = 0; ix < 2; ix++) {
            Pc[ix] = 2.0F * Pr[ix] - Pc[ix];
          }

          /*         Pe = Slimit(Pe, Pminmax); */
          /*  Ce = Seval(fnc,Pe); */
          hh = h;
          d1 = (float)fabs(Pc[0]) / 2.0F;
          d2 = (float)fabs(Pc[0]) / 2.0F;
          hh->Tturn = (float)fabs(Pc[1]);
          a = hh->Tturn;
          b = hh->Tapp;
          hh->Tt = a + b;
          a = hh->Vh2;
          b = hh->cpsiw;
          b_b = hh->Tapp;
          b_a = hh->Pt[0];
          c_a = hh->Wx;
          c_b = hh->Tapp;
          d_a = hh->Wx;
          d_b = hh->Tturn;
          hh->x2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
          a = hh->Vh2;
          b = hh->spsiw;
          b_b = hh->Tapp;
          b_a = hh->Pt[1];
          c_a = hh->Wy;
          c_b = hh->Tapp;
          d_a = hh->Wy;
          d_b = hh->Tturn;
          hh->y2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
          a = hh->Vv2;
          b = hh->Wz;
          b_b = hh->Tapp;
          b_a = hh->Pt[2];
          c_a = hh->Wz;
          c_b = hh->Tturn;
          d_a = hh->P0[2];
          d_b = hh->cpsi0;
          e_a = hh->x1;
          xc11 = e_a + d1 * d_b;
          d_b = hh->spsi0;
          e_a = hh->y1;
          yc11 = e_a + d1 * d_b;
          d_b = hh->cpsiw;
          e_a = hh->x2;
          xc21 = e_a + d2 * d_b;
          d_b = hh->spsiw;
          e_a = hh->y2;
          yc21 = e_a + d2 * d_b;
          d_b = hh->cpsiw;
          e_a = hh->x2;
          xc22 = e_a + 2.0F * d2 * d_b;
          d_b = hh->spsiw;
          e_a = hh->y2;
          yc22 = e_a + 2.0F * d2 * d_b;
          x = hh->psi0;
          e_a = -hh->Vh0;
          d_b = hh->psip0;
          b_x = hh->psi0;
          f_a = hh->Vh0;
          e_b = hh->psip0;
          d1 = hh->x1;
          xp0_ = 5.0F * (xc11 - d1);
          d1 = hh->y1;
          d1 = 5.0F * (yc11 - d1);
          d2 = hh->Vh0;
          d2 = (float)sqrt(d2 * d2 / (xp0_ * xp0_ + d1 * d1));
          d1 = hh->x1;
          xp0_ = (e_a * (float)sin(x) * d_b / 20.0F / (d2 * d2) + 2.0F * xc11) -
            d1;
          d_b = hh->y1;
          d1 = (f_a * (float)cos(b_x) * e_b / 20.0F / (d2 * d2) + 2.0F * yc11) -
            d_b;
          pbern(B5);
          f_b[0] = hh->x1;
          f_b[1] = xc11;
          f_b[2] = xp0_;
          f_b[3] = xc22;
          f_b[4] = xc21;
          f_b[5] = hh->x2;
          g_b[0] = hh->y1;
          g_b[1] = yc11;
          g_b[2] = d1;
          g_b[3] = yc22;
          g_b[4] = yc21;
          g_b[5] = hh->y2;
          b_pbern(B4);
          h_b[0] = xc11;
          h_b[1] = xp0_;
          h_b[2] = xc22;
          h_b[3] = xc21;
          h_b[4] = hh->x2;
          i_b[0] = hh->x1;
          i_b[1] = xc11;
          i_b[2] = xp0_;
          i_b[3] = xc22;
          i_b[4] = xc21;
          for (ix = 0; ix < 1001; ix++) {
            b_B4[ix] = 0.0F;
            c_B4[ix] = 0.0F;
            for (ixstart = 0; ixstart < 5; ixstart++) {
              b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
              c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
            }

            xp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
          }

          h_b[0] = yc11;
          h_b[1] = d1;
          h_b[2] = yc22;
          h_b[3] = yc21;
          h_b[4] = hh->y2;
          i_b[0] = hh->y1;
          i_b[1] = yc11;
          i_b[2] = d1;
          i_b[3] = yc22;
          i_b[4] = yc21;
          for (ix = 0; ix < 1001; ix++) {
            b_B4[ix] = 0.0F;
            c_B4[ix] = 0.0F;
            for (ixstart = 0; ixstart < 5; ixstart++) {
              b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
              c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
            }

            yp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
          }

          for (i = 0; i < 1001; i++) {
            hh->Vhj[i] = 0.0F;
          }

          for (i = 0; i < 1001; i++) {
            hh->Vvj[i] = 0.0F;
          }

          memset(&psi[0], 0, 1001U * sizeof(float));
          memset(&psip[0], 0, 1001U * sizeof(float));
          psi[0] = hh->psi0;
          psip[0] = hh->psip0;
          hh->Vhj[0] = hh->Vh;
          hh->Vvj[0] = hh->Vv;
          for (ixstart = 0; ixstart < 1000; ixstart++) {
            d1 = hh->Vhj[ixstart];
            d1 = 0.001F * (float)sqrt(xp[ixstart + 1] * xp[ixstart + 1] +
              yp[ixstart + 1] * yp[ixstart + 1]) / d1;
            psi[ixstart + 1] = rt_atan2f_snf(yp[ixstart + 1], xp[ixstart + 1]);
            if (psi[ixstart + 1] > psi[ixstart] + 3.14159274F) {
              psi[ixstart + 1] -= 6.28318548F;
            }

            if (psi[ixstart + 1] < psi[ixstart] - 3.14159274F) {
              psi[ixstart + 1] += 6.28318548F;
            }

            psip[ixstart + 1] = (psi[ixstart + 1] - psi[ixstart]) / d1;
            hh->Vhj[ixstart + 1] = hh->Vh;
            hh->Vvj[ixstart + 1] = hh->Vv;
            dtj[ixstart] = d1;
          }

          for (ix = 0; ix < 1001; ix++) {
            hh->xj[ix] = 0.0F;
            for (ixstart = 0; ixstart < 6; ixstart++) {
              hh->xj[ix] += B5[ix + 1001 * ixstart] * f_b[ixstart];
            }
          }

          for (ix = 0; ix < 1001; ix++) {
            hh->yj[ix] = 0.0F;
            for (ixstart = 0; ixstart < 6; ixstart++) {
              hh->yj[ix] += B5[ix + 1001 * ixstart] * g_b[ixstart];
            }
          }

          for (i = 0; i < 1001; i++) {
            hh->psij[i] = psi[i];
          }

          d2 = mean(hh->Vvj);
          hh->Tfar = sum(dtj);
          for (ixstart = 0; ixstart < 999; ixstart++) {
            dtj[ixstart + 1] += dtj[ixstart];
          }

          e_a = hh->T0;
          hh->tj[0] = e_a;
          for (i = 0; i < 1000; i++) {
            hh->tj[i + 1] = e_a + dtj[i];
          }

          b_abs(psip, xp);
          ixstart = 1;
          d1 = xp[0];
          if (rtIsNaNF(xp[0])) {
            b_ix = 2;
            exitg3 = false;
            while ((!exitg3) && (b_ix < 1002)) {
              ixstart = b_ix;
              if (!rtIsNaNF(xp[b_ix - 1])) {
                d1 = xp[b_ix - 1];
                exitg3 = true;
              } else {
                b_ix++;
              }
            }
          }

          if (ixstart < 1001) {
            while (ixstart + 1 < 1002) {
              if (xp[ixstart] > d1) {
                d1 = xp[ixstart];
              }

              ixstart++;
            }
          }

          xp0_ = d1 * 180.0F / 3.14159274F;
          d_b = hh->Tfar;
          d2 = (d_a - ((b_a + (a + b) * b_b) + c_a * c_b)) - d2 * d_b;
          a = hh->Tfar;
          b = hh->Tturn;
          d1 = a - b;
          if (xp0_ - 30.0F >= 0.0F) {
            c_xp0_ = xp0_ - 30.0F;
          } else {
            c_xp0_ = 0.0F;
          }

          d1 = (d2 * d2 + d1 * d1) + c_xp0_;

          /*    Ce = feval(Pe,h); */
          if (d1 < Cr) {
            for (ix = 0; ix < 2; ix++) {
              S0[4 + ix] = Pc[ix];
            }

            F[2] = d1;
          } else {
            for (ix = 0; ix < 2; ix++) {
              S0[4 + ix] = Pr[ix];
            }

            F[2] = Cr;
          }
        } else if (Cr > F[1]) {
          if (Cr < F[2]) {
            for (ix = 0; ix < 2; ix++) {
              S0[4 + ix] = Pr[ix];
            }
          } else {
            for (ix = 0; ix < 2; ix++) {
              Pr[ix] = S0[4 + ix];
            }

            Cr = F[2];
          }

          /*            Contraction */
          /*             Pk = Pc + 0.5*(Pr-Pc); */
          for (ix = 0; ix < 2; ix++) {
            Pr[ix] = 0.5F * (Pr[ix] + Pc[ix]);
          }

          /*           Pc, Pr, Pk */
          /*             Pk = Slimit(Pk, Pminmax); */
          /*             [FVk, FCk, Ck] = Seval(fnc,Pk, ml); */
          /*             Ck = Seval(fnc,Pk); */
          /*     Ck = feval(fnc,Pk,h); */
          hh = h;
          d1 = (float)fabs(Pr[0]) / 2.0F;
          d2 = (float)fabs(Pr[0]) / 2.0F;
          hh->Tturn = (float)fabs(Pr[1]);
          a = hh->Tturn;
          b = hh->Tapp;
          hh->Tt = a + b;
          a = hh->Vh2;
          b = hh->cpsiw;
          b_b = hh->Tapp;
          b_a = hh->Pt[0];
          c_a = hh->Wx;
          c_b = hh->Tapp;
          d_a = hh->Wx;
          d_b = hh->Tturn;
          hh->x2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
          a = hh->Vh2;
          b = hh->spsiw;
          b_b = hh->Tapp;
          b_a = hh->Pt[1];
          c_a = hh->Wy;
          c_b = hh->Tapp;
          d_a = hh->Wy;
          d_b = hh->Tturn;
          hh->y2 = ((b_a + a * b * b_b) - c_a * c_b) - d_a * d_b;
          a = hh->Vv2;
          b = hh->Wz;
          b_b = hh->Tapp;
          b_a = hh->Pt[2];
          c_a = hh->Wz;
          c_b = hh->Tturn;
          d_a = hh->P0[2];
          d_b = hh->cpsi0;
          e_a = hh->x1;
          xc11 = e_a + d1 * d_b;
          d_b = hh->spsi0;
          e_a = hh->y1;
          yc11 = e_a + d1 * d_b;
          d_b = hh->cpsiw;
          e_a = hh->x2;
          xc21 = e_a + d2 * d_b;
          d_b = hh->spsiw;
          e_a = hh->y2;
          yc21 = e_a + d2 * d_b;
          d_b = hh->cpsiw;
          e_a = hh->x2;
          xc22 = e_a + 2.0F * d2 * d_b;
          d_b = hh->spsiw;
          e_a = hh->y2;
          yc22 = e_a + 2.0F * d2 * d_b;
          x = hh->psi0;
          e_a = -hh->Vh0;
          d_b = hh->psip0;
          b_x = hh->psi0;
          f_a = hh->Vh0;
          e_b = hh->psip0;
          d1 = hh->x1;
          xp0_ = 5.0F * (xc11 - d1);
          d1 = hh->y1;
          d1 = 5.0F * (yc11 - d1);
          d2 = hh->Vh0;
          d2 = (float)sqrt(d2 * d2 / (xp0_ * xp0_ + d1 * d1));
          d1 = hh->x1;
          xp0_ = (e_a * (float)sin(x) * d_b / 20.0F / (d2 * d2) + 2.0F * xc11) -
            d1;
          d_b = hh->y1;
          d1 = (f_a * (float)cos(b_x) * e_b / 20.0F / (d2 * d2) + 2.0F * yc11) -
            d_b;
          pbern(B5);
          f_b[0] = hh->x1;
          f_b[1] = xc11;
          f_b[2] = xp0_;
          f_b[3] = xc22;
          f_b[4] = xc21;
          f_b[5] = hh->x2;
          g_b[0] = hh->y1;
          g_b[1] = yc11;
          g_b[2] = d1;
          g_b[3] = yc22;
          g_b[4] = yc21;
          g_b[5] = hh->y2;
          b_pbern(B4);
          h_b[0] = xc11;
          h_b[1] = xp0_;
          h_b[2] = xc22;
          h_b[3] = xc21;
          h_b[4] = hh->x2;
          i_b[0] = hh->x1;
          i_b[1] = xc11;
          i_b[2] = xp0_;
          i_b[3] = xc22;
          i_b[4] = xc21;
          for (ix = 0; ix < 1001; ix++) {
            b_B4[ix] = 0.0F;
            c_B4[ix] = 0.0F;
            for (ixstart = 0; ixstart < 5; ixstart++) {
              b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
              c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
            }

            xp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
          }

          h_b[0] = yc11;
          h_b[1] = d1;
          h_b[2] = yc22;
          h_b[3] = yc21;
          h_b[4] = hh->y2;
          i_b[0] = hh->y1;
          i_b[1] = yc11;
          i_b[2] = d1;
          i_b[3] = yc22;
          i_b[4] = yc21;
          for (ix = 0; ix < 1001; ix++) {
            b_B4[ix] = 0.0F;
            c_B4[ix] = 0.0F;
            for (ixstart = 0; ixstart < 5; ixstart++) {
              b_B4[ix] += B4[ix + 1001 * ixstart] * h_b[ixstart];
              c_B4[ix] += B4[ix + 1001 * ixstart] * i_b[ixstart];
            }

            yp[ix] = 5.0F * (b_B4[ix] - c_B4[ix]);
          }

          for (i = 0; i < 1001; i++) {
            hh->Vhj[i] = 0.0F;
          }

          for (i = 0; i < 1001; i++) {
            hh->Vvj[i] = 0.0F;
          }

          memset(&psi[0], 0, 1001U * sizeof(float));
          memset(&psip[0], 0, 1001U * sizeof(float));
          psi[0] = hh->psi0;
          psip[0] = hh->psip0;
          hh->Vhj[0] = hh->Vh;
          hh->Vvj[0] = hh->Vv;
          for (ixstart = 0; ixstart < 1000; ixstart++) {
            d1 = hh->Vhj[ixstart];
            d1 = 0.001F * (float)sqrt(xp[ixstart + 1] * xp[ixstart + 1] +
              yp[ixstart + 1] * yp[ixstart + 1]) / d1;
            psi[ixstart + 1] = rt_atan2f_snf(yp[ixstart + 1], xp[ixstart + 1]);
            if (psi[ixstart + 1] > psi[ixstart] + 3.14159274F) {
              psi[ixstart + 1] -= 6.28318548F;
            }

            if (psi[ixstart + 1] < psi[ixstart] - 3.14159274F) {
              psi[ixstart + 1] += 6.28318548F;
            }

            psip[ixstart + 1] = (psi[ixstart + 1] - psi[ixstart]) / d1;
            hh->Vhj[ixstart + 1] = hh->Vh;
            hh->Vvj[ixstart + 1] = hh->Vv;
            dtj[ixstart] = d1;
          }

          for (ix = 0; ix < 1001; ix++) {
            hh->xj[ix] = 0.0F;
            for (ixstart = 0; ixstart < 6; ixstart++) {
              hh->xj[ix] += B5[ix + 1001 * ixstart] * f_b[ixstart];
            }
          }

          for (ix = 0; ix < 1001; ix++) {
            hh->yj[ix] = 0.0F;
            for (ixstart = 0; ixstart < 6; ixstart++) {
              hh->yj[ix] += B5[ix + 1001 * ixstart] * g_b[ixstart];
            }
          }

          for (i = 0; i < 1001; i++) {
            hh->psij[i] = psi[i];
          }

          d2 = mean(hh->Vvj);
          hh->Tfar = sum(dtj);
          for (ixstart = 0; ixstart < 999; ixstart++) {
            dtj[ixstart + 1] += dtj[ixstart];
          }

          e_a = hh->T0;
          hh->tj[0] = e_a;
          for (i = 0; i < 1000; i++) {
            hh->tj[i + 1] = e_a + dtj[i];
          }

          b_abs(psip, xp);
          ixstart = 1;
          d1 = xp[0];
          if (rtIsNaNF(xp[0])) {
            b_ix = 2;
            exitg2 = false;
            while ((!exitg2) && (b_ix < 1002)) {
              ixstart = b_ix;
              if (!rtIsNaNF(xp[b_ix - 1])) {
                d1 = xp[b_ix - 1];
                exitg2 = true;
              } else {
                b_ix++;
              }
            }
          }

          if (ixstart < 1001) {
            while (ixstart + 1 < 1002) {
              if (xp[ixstart] > d1) {
                d1 = xp[ixstart];
              }

              ixstart++;
            }
          }

          xp0_ = d1 * 180.0F / 3.14159274F;
          d_b = hh->Tfar;
          d2 = (d_a - ((b_a + (a + b) * b_b) + c_a * c_b)) - d2 * d_b;
          a = hh->Tfar;
          b = hh->Tturn;
          d1 = a - b;
          if (xp0_ - 30.0F >= 0.0F) {
            d_xp0_ = xp0_ - 30.0F;
          } else {
            d_xp0_ = 0.0F;
          }

          d1 = (d2 * d2 + d1 * d1) + d_xp0_;
          if (d1 > Cr) {
            /*                Réduction */
            for (ix = 0; ix < 2; ix++) {
              Pc[ix] = S0[ix];
            }

            for (i = 0; i < 3; i++) {
              for (ix = 0; ix < 2; ix++) {
                S0[ix + (i << 1)] = (S0[ix + (i << 1)] + Pc[ix]) / 2.0F;
              }
            }

            /*                 [FV, FC, F] = Seval(fnc, S, ml); */
            Seval(S0, h, F);
          } else {
            for (ix = 0; ix < 2; ix++) {
              S0[4 + ix] = Pr[ix];
            }

            F[2] = d1;
          }
        } else {
          for (ix = 0; ix < 2; ix++) {
            S0[4 + ix] = Pr[ix];
          }

          F[2] = Cr;
        }

        /*      toc; */
        /*      pause; */
      }
    }
  } while (exitg1 == 0);

  /* toc; */
  for (ix = 0; ix < 2; ix++) {
    P[ix] = S0[ix];
  }

  *C = F[0];
}

/*
 * File trailer for NelderMead.c
 *
 * [EOF]
 */
