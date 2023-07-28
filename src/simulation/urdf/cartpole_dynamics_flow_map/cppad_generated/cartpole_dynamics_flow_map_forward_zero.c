#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void cartpole_dynamics_flow_map_forward_zero(double const *const * in,
                                             double*const * out,
                                             struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[41];

   v[0] = cos(x[2]);
   v[1] = cos(x[3]);
   v[2] = cos(x[4]);
   v[3] = 0.114295509084676 * v[2];
   v[4] = sin(x[4]);
   v[5] = 0.2 * v[4];
   v[6] = v[3] * v[2] + v[5] * v[4];
   v[7] = 0.5 * v[6];
   v[8] = 0.05 + v[7];
   v[9] = 0.02917 + 0.5 * v[7];
   v[10] = 1 / v[9];
   v[11] = v[8] * v[10];
   v[6] = 0.2 + v[6] - v[11] * v[8];
   v[12] = sin(x[3]);
   v[13] = 0.2 * v[2];
   v[14] = 0 - v[4];
   v[15] = 0.114295509084676 * v[14];
   v[5] = v[3] * v[14] + v[5] * v[2];
   v[3] = 0.5 * v[5];
   v[16] = v[3] * v[10];
   v[17] = v[13] * v[4] + v[15] * v[2] - v[16] * v[8];
   v[18] = v[1] * v[6] + v[12] * v[17];
   v[19] = 0 - v[12];
   v[5] = v[5] - v[11] * v[3];
   v[15] = 0.2 + v[13] * v[2] + v[15] * v[14] - v[16] * v[3];
   v[13] = v[1] * v[5] + v[12] * v[15];
   v[20] = v[18] * v[19] + v[13] * v[1];
   v[13] = v[18] * v[1] + v[13] * v[12];
   v[18] = v[8] - v[11] * v[9];
   v[21] = v[3] - v[16] * v[9];
   v[22] = v[1] * v[18] + v[12] * v[21];
   v[23] = v[13] + v[22];
   v[24] = 0.1 + v[23];
   v[25] = v[9] * v[10];
   v[23] = 1 / (0.06667 + v[9] + v[22] + v[23] - v[25] * v[9]);
   v[22] = v[24] * v[23];
   v[21] = v[1] * v[21] + v[19] * v[18] + v[20];
   v[20] = v[20] - v[22] * v[21];
   v[18] = sin(x[2]);
   v[9] = v[19] * v[5] + v[1] * v[15];
   v[26] = v[19] * v[6] + v[1] * v[17];
   v[27] = v[21] * v[23];
   v[21] = 0.2 + v[9] * v[1] + v[26] * v[19] - v[27] * v[21];
   v[13] = 0.2 + v[13] - v[22] * v[24];
   v[26] = v[9] * v[12] + v[26] * v[1] - v[27] * v[24];
   v[9] = 1 / (1 + (v[0] * v[20] + v[18] * v[21]) * v[18] + (v[0] * v[13] + v[18] * v[26]) * v[0]);
   v[24] = 0 - x[8];
   v[28] = x[7] + x[6];
   v[29] = v[0] * x[5];
   v[30] = x[6] + v[29];
   v[31] = v[18] * x[5];
   v[32] = v[1] * v[30] + v[19] * v[31];
   v[33] = 0.5 * v[28] + v[32];
   v[30] = v[12] * v[30] + v[1] * v[31];
   v[34] = v[2] * v[33] + v[14] * v[30];
   v[35] = x[8] + v[28];
   v[36] = 0.2 * (0 - 0.25 * v[35]);
   v[34] = 0.2 * (0 - v[24]) * v[34] - v[35] * (0.2 * v[34] - v[36]);
   v[33] = v[4] * v[33] + v[2] * v[30];
   v[36] = 0 - v[33] * v[36];
   v[37] = 0 - v[36];
   v[24] = v[24] * v[33];
   v[33] = 1.71408981830648 * v[37] + 0.114295509084676 * v[24] + v[35] * 0.2 * v[33];
   v[35] = 0 - x[7];
   v[38] = (0 - v[35]) * v[32];
   v[35] = v[35] * v[30];
   v[4] = v[2] * v[33] + v[4] * v[34];
   v[39] = 0.2 * (0 - 0.25 * v[28]);
   v[36] = v[37] + v[36] + 0.5 * v[4] - v[30] * v[39];
   v[40] = 0 - v[36];
   v[39] = v[2] * v[34] + v[14] * v[33] + v[15] * v[38] + v[17] * v[35] + v[16] * v[40] - v[28] * (0.2 * v[32] - v[39]);
   v[4] = v[4] + v[28] * 0.2 * v[30] + v[5] * v[38] + v[6] * v[35] + v[11] * v[40];
   v[30] = 0 - x[6];
   v[28] = (0 - v[30]) * v[29];
   v[30] = v[30] * v[31];
   v[5] = 0.2 * (0 - 0.5 * x[6]);
   v[6] = v[1] * v[4] + v[12] * v[39];
   v[36] = v[31] * v[5] - (v[3] - v[25] * v[3]) * v[38] - (0.05 + v[7] - v[25] * v[8]) * v[35] - v[25] * v[40] - v[36] - v[6];
   v[3] = 0 - v[18];
   y[4] = v[9] * (x[9] - v[18] * (v[1] * v[39] + v[19] * v[4] + v[21] * v[28] + v[26] * v[30] + v[27] * v[36] - x[6] * (0.2 * v[29] - v[5])) - v[0] * (v[6] + x[6] * 0.2 * v[31] + v[20] * v[28] + v[13] * v[30] + v[22] * v[36])) - 9.81 * ((v[3] * v[13] + v[0] * v[26]) * v[0] + (v[3] * v[20] + v[0] * v[21]) * v[18]) * v[9];
   v[28] = 9.81 * v[0] + v[18] * y[4] + v[28];
   v[3] = 9.81 * v[3] + v[0] * y[4] + v[30];
   y[5] = v[23] * v[36] - v[27] * v[28] - v[22] * v[3];
   v[3] = y[5] + v[3];
   v[38] = v[1] * v[28] + v[12] * v[3] + v[38];
   v[3] = v[19] * v[28] + v[1] * v[3] + v[35];
   y[6] = v[10] * v[40] - v[16] * v[38] - v[11] * v[3] - v[25] * y[5];
   v[40] = y[5] + y[6];
   y[7] = 34.2817963661296 * v[37] - v[40] - 1.71408981830648 * (v[14] * v[38] + v[2] * (0.5 * v[40] + v[3]) + v[24]);
   // dependent variables without operations
   y[0] = x[5];
   y[1] = x[6];
   y[2] = x[7];
   y[3] = x[8];
}

