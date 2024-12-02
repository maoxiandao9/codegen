/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: computeSFO.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2024-12-02 19:45:56
 */

/* Include Files */
#include "computeSFO.h"
#include "computeSFO_emxutil.h"
#include "computeSFO_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static boolean_T isInitialized_computeSFO = false;

/* Function Declarations */
static void b_polyfit(const emxArray_real_T *x, const emxArray_real_T *y, double p[2]);

static int div_nde_s32_floor(int numerator);

static void polyfit(const double y_data[], double p[2]);

static double rt_roundd_snf(double u);

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *x
 *                const emxArray_real_T *y
 *                double p[2]
 * Return Type  : void
 */
static void b_polyfit(const emxArray_real_T *x, const emxArray_real_T *y, double p[2])
{
    emxArray_real_T *B;
    emxArray_real_T *V;
    double tau_data[2];
    double vn1[2];
    double vn2[2];
    double work[2];
    const double *x_data;
    const double *y_data;
    double absxk;
    double atmp;
    double scale;
    double t;
    double temp;
    double *B_data;
    double *V_data;
    int b_i;
    int exitg1;
    int i;
    int iac;
    int ii;
    int ip1;
    int ix;
    int jA;
    int k;
    int kend;
    int m;
    int mmi;
    int pvt;
    int u0;
    signed char jpvt[2];
    y_data = y->data;
    x_data = x->data;
    emxInit_real_T(&V, 2);
    m = x->size[1];
    i = V->size[0] * V->size[1];
    V->size[0] = x->size[1];
    V->size[1] = 2;
    emxEnsureCapacity_real_T(V, i);
    V_data = V->data;
    if (x->size[1] != 0) {
        for (k = 0; k < m; k++) {
            V_data[k + V->size[0]] = 1.0;
            V_data[k] = x_data[k];
        }
    }
    emxInit_real_T(&B, 1);
    kend = y->size[1];
    i = B->size[0];
    B->size[0] = y->size[1];
    emxEnsureCapacity_real_T(B, i);
    B_data = B->data;
    for (i = 0; i < kend; i++) {
        B_data[i] = y_data[i];
    }
    u0 = V->size[0];
    if (u0 > 2) {
        u0 = 2;
    }
    if (u0 - 1 >= 0) {
        memset(&tau_data[0], 0, (unsigned int)u0 * sizeof(double));
    }
    if (V->size[0] == 0) {
        jpvt[0] = 1;
        jpvt[1] = 2;
    } else {
        for (k = 0; k < 2; k++) {
            jpvt[k] = (signed char)(k + 1);
            work[k] = 0.0;
            ix = k * m;
            temp = 0.0;
            if (m >= 1) {
                if (m == 1) {
                    temp = fabs(V_data[ix]);
                } else {
                    scale = 3.3121686421112381E-170;
                    kend = ix + m;
                    for (jA = ix + 1; jA <= kend; jA++) {
                        absxk = fabs(V_data[jA - 1]);
                        if (absxk > scale) {
                            t = scale / absxk;
                            temp = temp * t * t + 1.0;
                            scale = absxk;
                        } else {
                            t = absxk / scale;
                            temp += t * t;
                        }
                    }
                    temp = scale * sqrt(temp);
                }
            }
            vn1[k] = temp;
            vn2[k] = temp;
        }
        for (b_i = 0; b_i < u0; b_i++) {
            ip1 = b_i + 2;
            jA = b_i * m;
            ii = jA + b_i;
            mmi = (m - b_i) - 1;
            kend = 0;
            if ((2 - b_i > 1) && (vn1[1] > vn1[b_i])) {
                kend = 1;
            }
            pvt = b_i + kend;
            if (pvt != b_i) {
                ix = pvt * m;
                for (k = 0; k < m; k++) {
                    kend = ix + k;
                    temp = V_data[kend];
                    i = jA + k;
                    V_data[kend] = V_data[i];
                    V_data[i] = temp;
                }
                kend = jpvt[pvt];
                jpvt[pvt] = jpvt[b_i];
                jpvt[b_i] = (signed char)kend;
                vn1[pvt] = vn1[b_i];
                vn2[pvt] = vn2[b_i];
            }
            if (b_i + 1 < m) {
                atmp = V_data[ii];
                ix = ii + 2;
                tau_data[b_i] = 0.0;
                if (mmi + 1 > 0) {
                    temp = 0.0;
                    if (mmi >= 1) {
                        if (mmi == 1) {
                            temp = fabs(V_data[ii + 1]);
                        } else {
                            scale = 3.3121686421112381E-170;
                            kend = (ii + mmi) + 1;
                            for (k = ix; k <= kend; k++) {
                                absxk = fabs(V_data[k - 1]);
                                if (absxk > scale) {
                                    t = scale / absxk;
                                    temp = temp * t * t + 1.0;
                                    scale = absxk;
                                } else {
                                    t = absxk / scale;
                                    temp += t * t;
                                }
                            }
                            temp = scale * sqrt(temp);
                        }
                    }
                    if (temp != 0.0) {
                        scale = fabs(V_data[ii]);
                        if (scale < temp) {
                            scale /= temp;
                            temp *= sqrt(scale * scale + 1.0);
                        } else if (scale > temp) {
                            temp /= scale;
                            temp = scale * sqrt(temp * temp + 1.0);
                        } else if (rtIsNaN(temp)) {
                            temp = rtNaN;
                        } else {
                            temp = scale * 1.4142135623730951;
                        }
                        if (V_data[ii] >= 0.0) {
                            temp = -temp;
                        }
                        if (fabs(temp) < 1.0020841800044864E-292) {
                            kend = 0;
                            i = (ii + mmi) + 1;
                            do {
                                kend++;
                                for (k = ix; k <= i; k++) {
                                    V_data[k - 1] *= 9.9792015476736E+291;
                                }
                                temp *= 9.9792015476736E+291;
                                atmp *= 9.9792015476736E+291;
                            } while ((fabs(temp) < 1.0020841800044864E-292) && (kend < 20));
                            temp = 0.0;
                            if (mmi >= 1) {
                                if (mmi == 1) {
                                    temp = fabs(V_data[ii + 1]);
                                } else {
                                    scale = 3.3121686421112381E-170;
                                    for (k = ix; k <= i; k++) {
                                        absxk = fabs(V_data[k - 1]);
                                        if (absxk > scale) {
                                            t = scale / absxk;
                                            temp = temp * t * t + 1.0;
                                            scale = absxk;
                                        } else {
                                            t = absxk / scale;
                                            temp += t * t;
                                        }
                                    }
                                    temp = scale * sqrt(temp);
                                }
                            }
                            scale = fabs(atmp);
                            if (scale < temp) {
                                scale /= temp;
                                temp *= sqrt(scale * scale + 1.0);
                            } else if (scale > temp) {
                                temp /= scale;
                                temp = scale * sqrt(temp * temp + 1.0);
                            } else if (rtIsNaN(temp)) {
                                temp = rtNaN;
                            } else {
                                temp = scale * 1.4142135623730951;
                            }
                            if (atmp >= 0.0) {
                                temp = -temp;
                            }
                            tau_data[b_i] = (temp - atmp) / temp;
                            scale = 1.0 / (atmp - temp);
                            for (k = ix; k <= i; k++) {
                                V_data[k - 1] *= scale;
                            }
                            for (k = 0; k < kend; k++) {
                                temp *= 1.0020841800044864E-292;
                            }
                            atmp = temp;
                        } else {
                            tau_data[b_i] = (temp - V_data[ii]) / temp;
                            scale = 1.0 / (V_data[ii] - temp);
                            i = (ii + mmi) + 1;
                            for (k = ix; k <= i; k++) {
                                V_data[k - 1] *= scale;
                            }
                            atmp = temp;
                        }
                    }
                }
                V_data[ii] = atmp;
            } else {
                tau_data[b_i] = 0.0;
            }
            if (b_i + 1 < 2) {
                atmp = V_data[ii];
                V_data[ii] = 1.0;
                jA = (ii + m) + 1;
                if (tau_data[0] != 0.0) {
                    pvt = mmi;
                    kend = ii + mmi;
                    while ((pvt + 1 > 0) && (V_data[kend] == 0.0)) {
                        pvt--;
                        kend--;
                    }
                    ix = 1;
                    k = jA;
                    do {
                        exitg1 = 0;
                        if (k <= jA + pvt) {
                            if (V_data[k - 1] != 0.0) {
                                exitg1 = 1;
                            } else {
                                k++;
                            }
                        } else {
                            ix = 0;
                            exitg1 = 1;
                        }
                    } while (exitg1 == 0);
                } else {
                    pvt = -1;
                    ix = 0;
                }
                if (pvt + 1 > 0) {
                    if (ix != 0) {
                        work[0] = 0.0;
                        kend = 0;
                        for (iac = jA; m < 0 ? iac >= jA : iac <= jA; iac += m) {
                            temp = 0.0;
                            i = iac + pvt;
                            for (k = iac; k <= i; k++) {
                                temp += V_data[k - 1] * V_data[(ii + k) - iac];
                            }
                            work[kend] += temp;
                            kend++;
                        }
                    }
                    if (!(-tau_data[0] == 0.0)) {
                        for (iac = 0; iac < ix; iac++) {
                            if (work[0] != 0.0) {
                                temp = work[0] * -tau_data[0];
                                i = pvt + jA;
                                for (kend = jA; kend <= i; kend++) {
                                    V_data[kend - 1] += V_data[(ii + kend) - jA] * temp;
                                }
                            }
                            jA += m;
                        }
                    }
                }
                V_data[ii] = atmp;
            }
            for (iac = ip1; iac < 3; iac++) {
                kend = (b_i + m) + 1;
                if (vn1[1] != 0.0) {
                    temp = fabs(V_data[kend - 1]) / vn1[1];
                    temp = 1.0 - temp * temp;
                    if (temp < 0.0) {
                        temp = 0.0;
                    }
                    scale = vn1[1] / vn2[1];
                    scale = temp * (scale * scale);
                    if (scale <= 1.4901161193847656E-8) {
                        if (b_i + 1 < m) {
                            ix = kend + 1;
                            temp = 0.0;
                            if (mmi >= 1) {
                                if (mmi == 1) {
                                    temp = fabs(V_data[kend]);
                                } else {
                                    scale = 3.3121686421112381E-170;
                                    kend += mmi;
                                    for (k = ix; k <= kend; k++) {
                                        absxk = fabs(V_data[k - 1]);
                                        if (absxk > scale) {
                                            t = scale / absxk;
                                            temp = temp * t * t + 1.0;
                                            scale = absxk;
                                        } else {
                                            t = absxk / scale;
                                            temp += t * t;
                                        }
                                    }
                                    temp = scale * sqrt(temp);
                                }
                            }
                            vn1[1] = temp;
                            vn2[1] = temp;
                        } else {
                            vn1[1] = 0.0;
                            vn2[1] = 0.0;
                        }
                    } else {
                        vn1[1] *= sqrt(temp);
                    }
                }
            }
        }
    }
    ix = 0;
    if (u0 > 0) {
        for (k = 0; k < u0; k++) {
            if (V_data[k + V->size[0] * k] != 0.0) {
                ix++;
            }
        }
    }
    p[0] = 0.0;
    p[1] = 0.0;
    for (iac = 0; iac < u0; iac++) {
        if (tau_data[iac] != 0.0) {
            temp = B_data[iac];
            i = iac + 2;
            for (b_i = i; b_i <= m; b_i++) {
                temp += V_data[(b_i + V->size[0] * iac) - 1] * B_data[b_i - 1];
            }
            temp *= tau_data[iac];
            if (temp != 0.0) {
                B_data[iac] -= temp;
                for (b_i = i; b_i <= m; b_i++) {
                    B_data[b_i - 1] -= V_data[(b_i + V->size[0] * iac) - 1] * temp;
                }
            }
        }
    }
    for (b_i = 0; b_i < ix; b_i++) {
        p[jpvt[b_i] - 1] = B_data[b_i];
    }
    emxFree_real_T(&B);
    for (iac = ix; iac >= 1; iac--) {
        kend = jpvt[iac - 1] - 1;
        p[kend] /= V_data[(iac + V->size[0] * (iac - 1)) - 1];
        for (b_i = 0; b_i <= iac - 2; b_i++) {
            p[jpvt[0] - 1] -= p[kend] * V_data[V->size[0] * (iac - 1)];
        }
    }
    emxFree_real_T(&V);
}

/*
 * Arguments    : int numerator
 * Return Type  : int
 */
static int div_nde_s32_floor(int numerator)
{
    int quotient;
    if ((numerator < 0) && (numerator % 56 != 0)) {
        quotient = -1;
    } else {
        quotient = 0;
    }
    quotient += numerator / 56;
    return quotient;
}

/*
 * Arguments    : const double y_data[]
 *                double p[2]
 * Return Type  : void
 */
static void polyfit(const double y_data[], double p[2])
{
    static const int b_A[112] = {
        -8750000, -8437500, -8125000, -7812500, -7500000, -7187500, -6875000, -6562500, -6250000, -5937500, -5625000,
        -5312500, -5000000, -4687500, -4375000, -4062500, -3750000, -3437500, -3125000, -2812500, -2500000, -2187500,
        -1875000, -1562500, -1250000, -937500,  -625000,  -312500,  312500,   625000,   937500,   1250000,  1562500,
        1875000,  2187500,  2500000,  2812500,  3125000,  3437500,  3750000,  4062500,  4375000,  4687500,  5000000,
        5312500,  5625000,  5937500,  6250000,  6562500,  6875000,  7187500,  7500000,  7812500,  8125000,  8437500,
        8750000,  1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1};
    static const int iv[112] = {
        -8750000, -8437500, -8125000, -7812500, -7500000, -7187500, -6875000, -6562500, -6250000, -5937500, -5625000,
        -5312500, -5000000, -4687500, -4375000, -4062500, -3750000, -3437500, -3125000, -2812500, -2500000, -2187500,
        -1875000, -1562500, -1250000, -937500,  -625000,  -312500,  312500,   625000,   937500,   1250000,  1562500,
        1875000,  2187500,  2500000,  2812500,  3125000,  3437500,  3750000,  4062500,  4375000,  4687500,  5000000,
        5312500,  5625000,  5937500,  6250000,  6562500,  6875000,  7187500,  7500000,  7812500,  8125000,  8437500,
        8750000,  1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,        1,
        1,        1};
    double A[112];
    double B_data[56];
    double tau[2];
    double vn1[2];
    double vn2[2];
    double work[2];
    double atmp;
    double b_absxk;
    double scale;
    double t;
    double temp;
    int absxk;
    int assumedRank;
    int b_i;
    int exitg1;
    int i;
    int ii;
    int ip1;
    int j;
    int k;
    int kend;
    int lastc;
    int pvt;
    signed char jpvt[2];
    memcpy(&B_data[0], &y_data[0], 56U * sizeof(double));
    jpvt[0] = 1;
    tau[0] = 0.0;
    jpvt[1] = 2;
    tau[1] = 0.0;
    for (i = 0; i < 112; i++) {
        A[i] = b_A[i];
    }
    for (j = 0; j < 2; j++) {
        work[j] = 0.0;
        assumedRank = j * 56;
        temp = 0.0;
        scale = 3.3121686421112381E-170;
        kend = assumedRank + 56;
        for (k = assumedRank + 1; k <= kend; k++) {
            absxk = (int)fabs((double)iv[k - 1]);
            if (absxk > scale) {
                t = scale / (double)absxk;
                temp = temp * t * t + 1.0;
                scale = absxk;
            } else {
                t = (double)absxk / scale;
                temp += t * t;
            }
        }
        scale *= sqrt(temp);
        vn1[j] = scale;
        vn2[j] = scale;
    }
    for (b_i = 0; b_i < 2; b_i++) {
        ip1 = b_i + 2;
        ii = b_i * 56 + b_i;
        kend = 0;
        if ((2 - b_i > 1) && (vn1[1] > vn1[b_i])) {
            kend = 1;
        }
        pvt = b_i + kend;
        if (pvt != b_i) {
            kend = pvt * 56;
            absxk = b_i * 56;
            for (k = 0; k < 56; k++) {
                assumedRank = kend + k;
                temp = A[assumedRank];
                lastc = absxk + k;
                A[assumedRank] = A[lastc];
                A[lastc] = temp;
            }
            kend = jpvt[pvt];
            jpvt[pvt] = jpvt[b_i];
            jpvt[b_i] = (signed char)kend;
            vn1[pvt] = vn1[b_i];
            vn2[pvt] = vn2[b_i];
        }
        atmp = A[ii];
        assumedRank = ii + 2;
        tau[b_i] = 0.0;
        temp = 0.0;
        scale = 3.3121686421112381E-170;
        kend = (ii - b_i) + 56;
        for (k = assumedRank; k <= kend; k++) {
            b_absxk = fabs(A[k - 1]);
            if (b_absxk > scale) {
                t = scale / b_absxk;
                temp = temp * t * t + 1.0;
                scale = b_absxk;
            } else {
                t = b_absxk / scale;
                temp += t * t;
            }
        }
        temp = scale * sqrt(temp);
        if (temp != 0.0) {
            scale = A[ii];
            t = fabs(scale);
            if (t < temp) {
                t /= temp;
                temp *= sqrt(t * t + 1.0);
            } else if (t > temp) {
                temp /= t;
                temp = t * sqrt(temp * temp + 1.0);
            } else if (rtIsNaN(temp)) {
                temp = rtNaN;
            } else {
                temp = t * 1.4142135623730951;
            }
            if (scale >= 0.0) {
                temp = -temp;
            }
            if (fabs(temp) < 1.0020841800044864E-292) {
                absxk = 0;
                do {
                    absxk++;
                    for (k = assumedRank; k <= kend; k++) {
                        A[k - 1] *= 9.9792015476736E+291;
                    }
                    temp *= 9.9792015476736E+291;
                    atmp *= 9.9792015476736E+291;
                } while ((fabs(temp) < 1.0020841800044864E-292) && (absxk < 20));
                temp = 0.0;
                scale = 3.3121686421112381E-170;
                for (k = assumedRank; k <= kend; k++) {
                    b_absxk = fabs(A[k - 1]);
                    if (b_absxk > scale) {
                        t = scale / b_absxk;
                        temp = temp * t * t + 1.0;
                        scale = b_absxk;
                    } else {
                        t = b_absxk / scale;
                        temp += t * t;
                    }
                }
                temp = scale * sqrt(temp);
                t = fabs(atmp);
                if (t < temp) {
                    t /= temp;
                    temp *= sqrt(t * t + 1.0);
                } else if (t > temp) {
                    temp /= t;
                    temp = t * sqrt(temp * temp + 1.0);
                } else if (rtIsNaN(temp)) {
                    temp = rtNaN;
                } else {
                    temp = t * 1.4142135623730951;
                }
                if (atmp >= 0.0) {
                    temp = -temp;
                }
                tau[b_i] = (temp - atmp) / temp;
                t = 1.0 / (atmp - temp);
                for (k = assumedRank; k <= kend; k++) {
                    A[k - 1] *= t;
                }
                for (k = 0; k < absxk; k++) {
                    temp *= 1.0020841800044864E-292;
                }
                atmp = temp;
            } else {
                tau[b_i] = (temp - scale) / temp;
                t = 1.0 / (scale - temp);
                for (k = assumedRank; k <= kend; k++) {
                    A[k - 1] *= t;
                }
                atmp = temp;
            }
        }
        A[ii] = atmp;
        if (b_i + 1 < 2) {
            A[ii] = 1.0;
            absxk = ii + 57;
            if (tau[0] != 0.0) {
                pvt = 56;
                kend = ii + 55;
                while ((pvt > 0) && (A[kend] == 0.0)) {
                    pvt--;
                    kend--;
                }
                lastc = 1;
                kend = ii + 56;
                do {
                    exitg1 = 0;
                    if (kend + 1 <= (ii + pvt) + 56) {
                        if (A[kend] != 0.0) {
                            exitg1 = 1;
                        } else {
                            kend++;
                        }
                    } else {
                        lastc = 0;
                        exitg1 = 1;
                    }
                } while (exitg1 == 0);
            } else {
                pvt = 0;
                lastc = 0;
            }
            if (pvt > 0) {
                if (lastc != 0) {
                    work[0] = 0.0;
                    for (assumedRank = absxk; assumedRank <= absxk; assumedRank += 56) {
                        temp = 0.0;
                        i = assumedRank + pvt;
                        for (kend = assumedRank; kend < i; kend++) {
                            temp += A[kend - 1] * A[(ii + kend) - assumedRank];
                        }
                        kend = div_nde_s32_floor((assumedRank - ii) - 57);
                        work[kend] += temp;
                    }
                }
                if (!(-tau[0] == 0.0)) {
                    kend = ii;
                    for (j = 0; j < lastc; j++) {
                        if (work[0] != 0.0) {
                            temp = work[0] * -tau[0];
                            i = kend + 57;
                            absxk = pvt + kend;
                            for (assumedRank = i; assumedRank <= absxk + 56; assumedRank++) {
                                A[assumedRank - 1] += A[((ii + assumedRank) - kend) - 57] * temp;
                            }
                        }
                        kend += 56;
                    }
                }
            }
            A[ii] = atmp;
        }
        for (j = ip1; j < 3; j++) {
            if (vn1[1] != 0.0) {
                temp = fabs(A[b_i + 56]) / vn1[1];
                temp = 1.0 - temp * temp;
                if (temp < 0.0) {
                    temp = 0.0;
                }
                scale = vn1[1] / vn2[1];
                scale = temp * (scale * scale);
                if (scale <= 1.4901161193847656E-8) {
                    assumedRank = b_i + 58;
                    temp = 0.0;
                    scale = 3.3121686421112381E-170;
                    for (k = assumedRank; k < 113; k++) {
                        b_absxk = fabs(A[k - 1]);
                        if (b_absxk > scale) {
                            t = scale / b_absxk;
                            temp = temp * t * t + 1.0;
                            scale = b_absxk;
                        } else {
                            t = b_absxk / scale;
                            temp += t * t;
                        }
                    }
                    scale *= sqrt(temp);
                    vn1[1] = scale;
                    vn2[1] = scale;
                } else {
                    vn1[1] *= sqrt(temp);
                }
            }
        }
    }
    assumedRank = 0;
    for (k = 0; k < 2; k++) {
        if (A[k + 56 * k] != 0.0) {
            assumedRank++;
        }
        p[k] = 0.0;
        if (tau[k] != 0.0) {
            temp = B_data[k];
            i = k + 2;
            for (b_i = i; b_i < 57; b_i++) {
                temp += A[(b_i + 56 * k) - 1] * B_data[b_i - 1];
            }
            temp *= tau[k];
            if (temp != 0.0) {
                B_data[k] -= temp;
                for (b_i = i; b_i < 57; b_i++) {
                    B_data[b_i - 1] -= A[(b_i + 56 * k) - 1] * temp;
                }
            }
        }
    }
    for (b_i = 0; b_i < assumedRank; b_i++) {
        p[jpvt[b_i] - 1] = B_data[b_i];
    }
    for (j = assumedRank; j >= 1; j--) {
        kend = jpvt[j - 1] - 1;
        absxk = 56 * (j - 1);
        p[kend] /= A[(j + absxk) - 1];
        for (b_i = 0; b_i <= j - 2; b_i++) {
            p[jpvt[0] - 1] -= p[kend] * A[absxk];
        }
    }
}

/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
    double y;
    if (fabs(u) < 4.503599627370496E+15) {
        if (u >= 0.5) {
            y = floor(u + 0.5);
        } else if (u > -0.5) {
            y = u * 0.0;
        } else {
            y = ceil(u - 0.5);
        }
    } else {
        y = u;
    }
    return y;
}

/*
 * 输入: buffer (1D array of uint16)
 *  输出: phaseSlopes (1D array of double), sfo (1D array of double)
 *
 * Arguments    : const emxArray_real_T *buffer
 * Return Type  : double
 */
double computeSFO(const emxArray_real_T *buffer)
{
    emxArray_cint32_T *b_csi;
    emxArray_cint32_T *c_csi;
    emxArray_cint32_T *csi;
    emxArray_cint32_T *d_csi;
    emxArray_cint32_T *side_info;
    emxArray_int16_T *b_y;
    emxArray_int16_T *y;
    emxArray_real_T *b;
    emxArray_real_T *b_buffer;
    emxArray_real_T *b_phase;
    emxArray_real_T *b_rx_timestamp;
    emxArray_real_T *b_vwork;
    emxArray_real_T *phase;
    emxArray_real_T *vwork;
    emxArray_uint16_T *b_b;
    emxArray_uint16_T *x;
    emxArray_uint32_T *rx_timestamp;
    cint32_T *b_csi_data;
    cint32_T *csi_data;
    cint32_T *side_info_data;
    double b_phase_data[56];
    double dv[2];
    const double *buffer_data;
    double clockOffset;
    double cumsum_dp_corr;
    double dp_corr;
    double pkm1;
    double *b_data;
    double *phase_data;
    double *vwork_data;
    int b_loop_ub;
    int ep;
    int exitg1;
    int i;
    int j;
    int k;
    int len_a;
    int loop_ub;
    int npages;
    int num_side_info_tmp;
    int result_tmp;
    int vlen_tmp;
    int vspread;
    int vstride;
    unsigned int *rx_timestamp_data;
    unsigned short *b_b_data;
    short *b_y_data;
    unsigned short *x_data;
    short *y_data;
    signed char input_sizes_idx_0;
    signed char sizes_idx_0;
    boolean_T empty_non_axis_sizes;
    if (!isInitialized_computeSFO) {
        computeSFO_initialize();
    }
    buffer_data = buffer->data;
    len_a = (int)floor((double)buffer->size[0] / 4.0) << 2;
    emxInit_real_T(&b_buffer, 1);
    npages = b_buffer->size[0];
    b_buffer->size[0] = len_a;
    emxEnsureCapacity_real_T(b_buffer, npages);
    phase_data = b_buffer->data;
    for (npages = 0; npages < len_a; npages++) {
        phase_data[npages] = buffer_data[npages];
    }
    len_a = (int)((double)len_a / 4.0);
    emxInit_real_T(&b, 2);
    npages = b->size[0] * b->size[1];
    b->size[0] = len_a;
    b->size[1] = 4;
    emxEnsureCapacity_real_T(b, npages);
    b_data = b->data;
    for (npages = 0; npages < 4; npages++) {
        for (vspread = 0; vspread < len_a; vspread++) {
            b_data[vspread + b->size[0] * npages] = phase_data[npages + 4 * vspread];
        }
    }
    emxFree_real_T(&b_buffer);
    num_side_info_tmp = (int)floor((double)b->size[0] / 58.0);
    emxInit_cint32_T(&side_info);
    npages = side_info->size[0] * side_info->size[1];
    side_info->size[0] = 58;
    side_info->size[1] = num_side_info_tmp;
    emxEnsureCapacity_cint32_T(side_info, npages);
    side_info_data = side_info->data;
    ep = 58 * num_side_info_tmp;
    for (npages = 0; npages < ep; npages++) {
        side_info_data[npages].re = 0;
        side_info_data[npages].im = 0;
    }
    emxInit_cint32_T(&csi);
    npages = csi->size[0] * csi->size[1];
    csi->size[0] = 56;
    csi->size[1] = num_side_info_tmp;
    emxEnsureCapacity_cint32_T(csi, npages);
    csi_data = csi->data;
    ep = 56 * num_side_info_tmp;
    for (npages = 0; npages < ep; npages++) {
        csi_data[npages].re = 0;
        csi_data[npages].im = 0;
    }
    emxInit_uint16_T(&b_b, 2);
    npages = b_b->size[0] * b_b->size[1];
    b_b->size[0] = len_a;
    b_b->size[1] = 4;
    emxEnsureCapacity_uint16_T(b_b, npages);
    b_b_data = b_b->data;
    ep = b->size[0] << 2;
    for (npages = 0; npages < ep; npages++) {
        b_b_data[npages] = (unsigned short)rt_roundd_snf(b_data[npages]);
    }
    emxFree_real_T(&b);
    emxInit_uint32_T(&rx_timestamp);
    npages = rx_timestamp->size[0] * rx_timestamp->size[1];
    rx_timestamp->size[0] = 1;
    rx_timestamp->size[1] = num_side_info_tmp;
    emxEnsureCapacity_uint32_T(rx_timestamp, npages);
    rx_timestamp_data = rx_timestamp->data;
    emxInit_int16_T(&y);
    emxInit_uint16_T(&x, 1);
    emxInit_int16_T(&b_y);
    for (i = 0; i < num_side_info_tmp; i++) {
        len_a = i * 58;
        ep = (i + 1) * 58;
        /*  提取高位32位作为tx时间戳 */
        /*  提取低位32位作为rx时间戳 */
        rx_timestamp_data[i] = b_b_data[len_a] + ((unsigned int)b_b_data[len_a + b_b->size[0]] << 16);
        if (len_a + 1 > ep) {
            len_a = 0;
            ep = 0;
            npages = 0;
            vspread = 0;
        } else {
            npages = len_a;
            vspread = ep;
        }
        loop_ub = ep - len_a;
        vstride = x->size[0];
        x->size[0] = loop_ub;
        emxEnsureCapacity_uint16_T(x, vstride);
        x_data = x->data;
        for (vstride = 0; vstride < loop_ub; vstride++) {
            x_data[vstride] = b_b_data[len_a + vstride];
        }
        vstride = y->size[0];
        y->size[0] = loop_ub;
        emxEnsureCapacity_int16_T(y, vstride);
        y_data = y->data;
        memcpy((void *)&y_data[0], (void *)&x_data[0], (unsigned int)((size_t)(ep - len_a) * sizeof(short)));
        vstride = x->size[0];
        x->size[0] = loop_ub;
        emxEnsureCapacity_uint16_T(x, vstride);
        x_data = x->data;
        for (vstride = 0; vstride < loop_ub; vstride++) {
            x_data[vstride] = b_b_data[(npages + vstride) + b_b->size[0]];
        }
        vstride = b_y->size[0];
        b_y->size[0] = loop_ub;
        emxEnsureCapacity_int16_T(b_y, vstride);
        b_y_data = b_y->data;
        memcpy((void *)&b_y_data[0], (void *)&x_data[0], (unsigned int)((size_t)(vspread - npages) * sizeof(short)));
        for (npages = 0; npages < 58; npages++) {
            vspread = npages + 58 * i;
            side_info_data[vspread].re = y_data[npages];
            side_info_data[vspread].im = b_y_data[npages];
        }
        for (npages = 0; npages < 56; npages++) {
            csi_data[npages + 56 * i] = side_info_data[(npages + 58 * i) + 2];
        }
    }
    emxFree_int16_T(&b_y);
    emxFree_uint16_T(&x);
    emxFree_int16_T(&y);
    emxFree_uint16_T(&b_b);
    emxFree_cint32_T(&side_info);
    if (csi->size[1] != 0) {
        result_tmp = csi->size[1];
    } else {
        result_tmp = 0;
    }
    empty_non_axis_sizes = (result_tmp == 0);
    if (empty_non_axis_sizes || (csi->size[1] != 0)) {
        input_sizes_idx_0 = 28;
    } else {
        input_sizes_idx_0 = 0;
    }
    if (empty_non_axis_sizes || (csi->size[1] != 0)) {
        sizes_idx_0 = 28;
    } else {
        sizes_idx_0 = 0;
    }
    emxInit_cint32_T(&b_csi);
    npages = b_csi->size[0] * b_csi->size[1];
    b_csi->size[0] = 28;
    b_csi->size[1] = num_side_info_tmp;
    emxEnsureCapacity_cint32_T(b_csi, npages);
    side_info_data = b_csi->data;
    ep = input_sizes_idx_0;
    emxInit_cint32_T(&c_csi);
    npages = c_csi->size[0] * c_csi->size[1];
    c_csi->size[0] = 28;
    c_csi->size[1] = num_side_info_tmp;
    emxEnsureCapacity_cint32_T(c_csi, npages);
    b_csi_data = c_csi->data;
    for (npages = 0; npages < num_side_info_tmp; npages++) {
        for (vspread = 0; vspread < 28; vspread++) {
            vstride = vspread + 56 * npages;
            len_a = vspread + 28 * npages;
            side_info_data[len_a] = csi_data[vstride + 28];
            b_csi_data[len_a] = csi_data[vstride];
        }
    }
    emxFree_cint32_T(&csi);
    len_a = sizes_idx_0;
    emxInit_cint32_T(&d_csi);
    npages = d_csi->size[0] * d_csi->size[1];
    d_csi->size[0] = input_sizes_idx_0 + sizes_idx_0;
    d_csi->size[1] = result_tmp;
    emxEnsureCapacity_cint32_T(d_csi, npages);
    csi_data = d_csi->data;
    for (npages = 0; npages < result_tmp; npages++) {
        for (vspread = 0; vspread < ep; vspread++) {
            csi_data[vspread + d_csi->size[0] * npages] = side_info_data[vspread + input_sizes_idx_0 * npages];
        }
        for (vspread = 0; vspread < len_a; vspread++) {
            csi_data[(vspread + input_sizes_idx_0) + d_csi->size[0] * npages] =
                b_csi_data[vspread + sizes_idx_0 * npages];
        }
    }
    emxFree_cint32_T(&c_csi);
    emxFree_cint32_T(&b_csi);
    /*  计算CSI的相位 */
    vstride = d_csi->size[0] * d_csi->size[1];
    emxInit_real_T(&phase, 2);
    loop_ub = d_csi->size[0];
    npages = phase->size[0] * phase->size[1];
    phase->size[0] = d_csi->size[0];
    b_loop_ub = d_csi->size[1];
    phase->size[1] = d_csi->size[1];
    emxEnsureCapacity_real_T(phase, npages);
    phase_data = phase->data;
    for (k = 0; k < vstride; k++) {
        len_a = csi_data[k].im;
        ep = csi_data[k].re;
        if (ep == 0) {
            if (len_a > 0) {
                phase_data[k] = RT_PI / 2.0;
            } else if (len_a < 0) {
                phase_data[k] = -(RT_PI / 2.0);
            } else {
                phase_data[k] = 0.0;
            }
        } else {
            phase_data[k] = atan2(len_a, ep);
        }
    }
    len_a = 1;
    if (phase->size[0] != 1) {
        len_a = 0;
    }
    vlen_tmp = phase->size[len_a];
    emxInit_real_T(&vwork, 1);
    npages = vwork->size[0];
    vwork->size[0] = vlen_tmp;
    emxEnsureCapacity_real_T(vwork, npages);
    vwork_data = vwork->data;
    vstride = 1;
    for (k = 0; k < len_a; k++) {
        vstride *= phase->size[0];
    }
    vspread = (vlen_tmp - 1) * vstride;
    npages = 1;
    len_a += 2;
    for (k = len_a; k < 3; k++) {
        npages *= phase->size[1];
    }
    len_a = 0;
    for (i = 0; i < npages; i++) {
        ep = len_a - 1;
        len_a += vspread;
        for (j = 0; j < vstride; j++) {
            ep++;
            len_a++;
            for (k = 0; k < vlen_tmp; k++) {
                vwork_data[k] = phase_data[ep + k * vstride];
            }
            cumsum_dp_corr = 0.0;
            k = 0;
            while ((k + 1 < vlen_tmp) && (rtIsInf(vwork_data[k]) || rtIsNaN(vwork_data[k]))) {
                k++;
            }
            if (k + 1 < vwork->size[0]) {
                pkm1 = vwork_data[k];
                do {
                    exitg1 = 0;
                    k++;
                    while ((k + 1 <= vlen_tmp) && (rtIsInf(vwork_data[k]) || rtIsNaN(vwork_data[k]))) {
                        k++;
                    }
                    if (k + 1 > vlen_tmp) {
                        exitg1 = 1;
                    } else {
                        pkm1 = vwork_data[k] - pkm1;
                        dp_corr = pkm1 / 6.2831853071795862;
                        if (rtIsNaN(dp_corr) || rtIsInf(dp_corr)) {
                            clockOffset = rtNaN;
                        } else {
                            clockOffset = fmod(dp_corr, 1.0);
                        }
                        if (fabs(clockOffset) <= 0.5) {
                            if (dp_corr < 0.0) {
                                dp_corr = ceil(dp_corr);
                            } else {
                                dp_corr = floor(dp_corr);
                            }
                        } else if (fabs(dp_corr) < 4.503599627370496E+15) {
                            if (dp_corr >= 0.5) {
                                dp_corr = floor(dp_corr + 0.5);
                            } else if (dp_corr > -0.5) {
                                dp_corr *= 0.0;
                            } else {
                                dp_corr = ceil(dp_corr - 0.5);
                            }
                        }
                        if (fabs(pkm1) >= 3.1415926535897931) {
                            cumsum_dp_corr += dp_corr;
                        }
                        pkm1 = vwork_data[k];
                        vwork_data[k] -= 6.2831853071795862 * cumsum_dp_corr;
                    }
                } while (exitg1 == 0);
            }
            for (k = 0; k < vlen_tmp; k++) {
                phase_data[ep + k * vstride] = vwork_data[k];
            }
        }
    }
    emxInit_real_T(&b_phase, 2);
    npages = b_phase->size[0] * b_phase->size[1];
    b_phase->size[0] = d_csi->size[1];
    b_phase->size[1] = d_csi->size[0];
    emxEnsureCapacity_real_T(b_phase, npages);
    b_data = b_phase->data;
    for (npages = 0; npages < loop_ub; npages++) {
        for (vspread = 0; vspread < b_loop_ub; vspread++) {
            b_data[vspread + b_phase->size[0] * npages] = phase_data[npages + phase->size[0] * vspread];
        }
    }
    npages = phase->size[0] * phase->size[1];
    phase->size[0] = d_csi->size[1];
    phase->size[1] = d_csi->size[0];
    emxFree_cint32_T(&d_csi);
    emxEnsureCapacity_real_T(phase, npages);
    phase_data = phase->data;
    ep = b_phase->size[0] * b_phase->size[1];
    for (npages = 0; npages < ep; npages++) {
        phase_data[npages] = b_data[npages];
    }
    emxFree_real_T(&b_phase);
    /*  初始化相位斜率数组 */
    npages = vwork->size[0];
    vwork->size[0] = result_tmp;
    emxEnsureCapacity_real_T(vwork, npages);
    vwork_data = vwork->data;
    /* phase_y = phase_unwrapped(); */
    /* p = polyfit(phase_x, phase_y, 1); */
    for (i = 0; i < result_tmp; i++) {
        vwork_data[i] = 0.0;
        for (npages = 0; npages < loop_ub; npages++) {
            b_phase_data[npages] = phase_data[i + phase->size[0] * npages];
        }
        polyfit(b_phase_data, dv);
        vwork_data[i] = dv[0];
    }
    emxFree_real_T(&phase);
    /*  计算相位斜率 */
    /* phaseSlope = p(1); */
    /*  unwrap along the first dimension */
    for (npages = 0; npages < result_tmp; npages++) {
        vwork_data[npages] = vwork_data[npages] / 6.2831853071795862 / 2.5E-8 * 6.2831853071795862;
    }
    cumsum_dp_corr = 0.0;
    k = 0;
    while ((k + 1 < result_tmp) && (rtIsInf(vwork_data[k]) || rtIsNaN(vwork_data[k]))) {
        k++;
    }
    if (k + 1 < vwork->size[0]) {
        pkm1 = vwork_data[k];
        do {
            exitg1 = 0;
            k++;
            while ((k + 1 <= result_tmp) && (rtIsInf(vwork_data[k]) || rtIsNaN(vwork_data[k]))) {
                k++;
            }
            if (k + 1 > result_tmp) {
                exitg1 = 1;
            } else {
                pkm1 = vwork_data[k] - pkm1;
                dp_corr = pkm1 / 6.2831853071795862;
                if (rtIsNaN(dp_corr) || rtIsInf(dp_corr)) {
                    clockOffset = rtNaN;
                } else {
                    clockOffset = fmod(dp_corr, 1.0);
                }
                if (fabs(clockOffset) <= 0.5) {
                    if (dp_corr < 0.0) {
                        dp_corr = ceil(dp_corr);
                    } else {
                        dp_corr = floor(dp_corr);
                    }
                } else if (fabs(dp_corr) < 4.503599627370496E+15) {
                    if (dp_corr >= 0.5) {
                        dp_corr = floor(dp_corr + 0.5);
                    } else if (dp_corr > -0.5) {
                        dp_corr *= 0.0;
                    } else {
                        dp_corr = ceil(dp_corr - 0.5);
                    }
                }
                if (fabs(pkm1) >= 3.1415926535897931) {
                    cumsum_dp_corr += dp_corr;
                }
                pkm1 = vwork_data[k];
                vwork_data[k] -= 6.2831853071795862 * cumsum_dp_corr;
            }
        } while (exitg1 == 0);
    }
    /*  固定间隔积累的时间偏移 */
    /*  直接计算缩放后的时间偏移 */
    emxInit_real_T(&b_rx_timestamp, 2);
    npages = b_rx_timestamp->size[0] * b_rx_timestamp->size[1];
    b_rx_timestamp->size[0] = 1;
    b_rx_timestamp->size[1] = num_side_info_tmp;
    emxEnsureCapacity_real_T(b_rx_timestamp, npages);
    b_data = b_rx_timestamp->data;
    for (npages = 0; npages < num_side_info_tmp; npages++) {
        b_data[npages] = (double)rx_timestamp_data[npages] * 25.0 * 1.0E-9;
    }
    emxFree_uint32_T(&rx_timestamp);
    emxInit_real_T(&b_vwork, 2);
    npages = b_vwork->size[0] * b_vwork->size[1];
    b_vwork->size[0] = 1;
    b_vwork->size[1] = result_tmp;
    emxEnsureCapacity_real_T(b_vwork, npages);
    b_data = b_vwork->data;
    for (npages = 0; npages < result_tmp; npages++) {
        b_data[npages] = vwork_data[npages] / 6.2831853071795862 * 2.5E-8;
    }
    emxFree_real_T(&vwork);
    b_polyfit(b_rx_timestamp, b_vwork, dv);
    emxFree_real_T(&b_vwork);
    emxFree_real_T(&b_rx_timestamp);
    return 2.0E+7 / (1.0 - 2.0E+7 * (dv[0] / 2.0E+7)) - 2.0E+7;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void computeSFO_initialize(void)
{
    rt_InitInfAndNaN();
    isInitialized_computeSFO = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void computeSFO_terminate(void)
{
    isInitialized_computeSFO = false;
}

/*
 * File trailer for computeSFO.c
 *
 * [EOF]
 */
