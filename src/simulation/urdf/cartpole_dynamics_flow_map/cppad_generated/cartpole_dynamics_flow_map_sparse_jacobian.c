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

void cartpole_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[109];

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
   v[16] = v[3] * v[14] + v[5] * v[2];
   v[17] = 0.5 * v[16];
   v[18] = v[17] * v[10];
   v[19] = v[13] * v[4] + v[15] * v[2] - v[18] * v[8];
   v[20] = v[1] * v[6] + v[12] * v[19];
   v[21] = 0 - v[12];
   v[16] = v[16] - v[11] * v[17];
   v[22] = 0.2 + v[13] * v[2] + v[15] * v[14] - v[18] * v[17];
   v[23] = v[1] * v[16] + v[12] * v[22];
   v[24] = v[20] * v[21] + v[23] * v[1];
   v[25] = v[20] * v[1] + v[23] * v[12];
   v[26] = v[8] - v[11] * v[9];
   v[27] = v[17] - v[18] * v[9];
   v[28] = v[1] * v[26] + v[12] * v[27];
   v[29] = v[25] + v[28];
   v[30] = 0.1 + v[29];
   v[31] = v[9] * v[10];
   v[29] = 0.06667 + v[9] + v[28] + v[29] - v[31] * v[9];
   v[28] = 1 / v[29];
   v[32] = v[30] * v[28];
   v[33] = v[1] * v[27] + v[21] * v[26] + v[24];
   v[24] = v[24] - v[32] * v[33];
   v[34] = sin(x[2]);
   v[35] = v[21] * v[16] + v[1] * v[22];
   v[36] = v[21] * v[6] + v[1] * v[19];
   v[37] = v[33] * v[28];
   v[38] = 0.2 + v[35] * v[1] + v[36] * v[21] - v[37] * v[33];
   v[39] = v[0] * v[24] + v[34] * v[38];
   v[25] = 0.2 + v[25] - v[32] * v[30];
   v[40] = v[35] * v[12] + v[36] * v[1] - v[37] * v[30];
   v[41] = v[0] * v[25] + v[34] * v[40];
   v[42] = 1 + v[39] * v[34] + v[41] * v[0];
   jac[11] = 1 / v[42];
   v[43] = 0 - x[8];
   v[44] = 0 - v[43];
   v[45] = -1 * sin(x[2]);
   v[46] = v[45] * x[5];
   v[47] = cos(x[2]);
   v[48] = v[47] * x[5];
   v[49] = v[1] * v[46] + v[21] * v[48];
   v[50] = v[12] * v[46] + v[1] * v[48];
   v[51] = v[2] * v[49] + v[14] * v[50];
   v[52] = x[7] + x[6];
   v[53] = x[8] + v[52];
   v[51] = 0.2 * v[44] * v[51] - v[53] * 0.2 * v[51];
   v[54] = v[4] * v[49] + v[2] * v[50];
   v[55] = 0.2 * (0 - 0.25 * v[53]);
   v[56] = - v[54] * v[55];
   v[57] = - v[56];
   v[58] = v[43] * v[54];
   v[54] = 1.71408981830648 * v[57] + 0.114295509084676 * v[58] + v[53] * 0.2 * v[54];
   v[59] = 0 - x[7];
   v[60] = 0 - v[59];
   v[61] = v[60] * v[49];
   v[62] = v[59] * v[50];
   v[63] = v[2] * v[54] + v[4] * v[51];
   v[64] = 0.2 * (0 - 0.25 * v[52]);
   v[56] = v[57] + v[56] + 0.5 * v[63] - v[50] * v[64];
   v[65] = - v[56];
   v[54] = v[2] * v[51] + v[14] * v[54] + v[22] * v[61] + v[19] * v[62] + v[18] * v[65] - v[52] * 0.2 * v[49];
   v[63] = v[63] + v[52] * 0.2 * v[50] + v[16] * v[61] + v[6] * v[62] + v[11] * v[65];
   v[50] = 0 - x[6];
   v[51] = 0 - v[50];
   v[49] = v[51] * v[46];
   v[66] = v[50] * v[48];
   v[67] = 0.2 * (0 - 0.5 * x[6]);
   v[68] = v[17] - v[31] * v[17];
   v[7] = 0.05 + v[7] - v[31] * v[8];
   v[69] = v[1] * v[63] + v[12] * v[54];
   v[56] = v[48] * v[67] - v[68] * v[61] - v[7] * v[62] - v[31] * v[65] - v[56] - v[69];
   v[70] = v[0] * x[5];
   v[71] = x[6] + v[70];
   v[72] = v[34] * x[5];
   v[73] = v[1] * v[71] + v[21] * v[72];
   v[74] = 0.5 * v[52] + v[73];
   v[75] = v[12] * v[71] + v[1] * v[72];
   v[76] = v[2] * v[74] + v[14] * v[75];
   v[77] = 0.2 * v[76] - v[55];
   v[78] = 0.2 * v[44] * v[76] - v[53] * v[77];
   v[79] = v[4] * v[74] + v[2] * v[75];
   v[80] = 0 - v[79] * v[55];
   v[81] = 0 - v[80];
   v[82] = 0.2 * v[79];
   v[83] = 1.71408981830648 * v[81] + 0.114295509084676 * v[43] * v[79] + v[53] * v[82];
   v[84] = v[60] * v[73];
   v[85] = v[59] * v[75];
   v[86] = v[2] * v[83] + v[4] * v[78];
   v[81] = v[81] + v[80] + 0.5 * v[86] - v[75] * v[64];
   v[80] = 0 - v[81];
   v[87] = 0.2 * v[73] - v[64];
   v[88] = v[2] * v[78] + v[14] * v[83] + v[22] * v[84] + v[19] * v[85] + v[18] * v[80] - v[52] * v[87];
   v[89] = 0.2 * v[75];
   v[86] = v[86] + v[52] * v[89] + v[16] * v[84] + v[6] * v[85] + v[11] * v[80];
   v[90] = v[51] * v[70];
   v[91] = v[50] * v[72];
   v[92] = v[1] * v[86] + v[12] * v[88];
   v[81] = v[72] * v[67] - v[68] * v[84] - v[7] * v[85] - v[31] * v[80] - v[81] - v[92];
   v[93] = 0.2 * v[70] - v[67];
   v[94] = v[1] * v[88] + v[21] * v[86] + v[38] * v[90] + v[40] * v[91] + v[37] * v[81] - x[6] * v[93];
   v[95] = 0.2 * v[72];
   v[92] = v[92] + x[6] * v[95] + v[24] * v[90] + v[25] * v[91] + v[32] * v[81];
   v[41] = ((- jac[11]) * (v[39] * v[47] + (v[45] * v[24] + v[47] * v[38]) * v[34] + v[41] * v[45] + (v[45] * v[25] + v[47] * v[40]) * v[0])) / v[42];
   v[39] = x[9] - v[34] * v[94] - v[0] * v[92];
   v[96] = 0 - v[34];
   v[97] = v[96] * v[25] + v[0] * v[40];
   v[98] = v[96] * v[24] + v[0] * v[38];
   v[99] = v[97] * v[0] + v[98] * v[34];
   v[100] = - v[47];
   jac[4] = jac[11] * (0 - (v[34] * (v[1] * v[54] + v[21] * v[63] + v[38] * v[49] + v[40] * v[66] + v[37] * v[56] - x[6] * 0.2 * v[46]) + v[47] * v[94]) - (v[0] * (v[69] + x[6] * 0.2 * v[48] + v[24] * v[49] + v[25] * v[66] + v[32] * v[56]) + v[45] * v[92])) + v[41] * v[39] - 9.81 * (v[99] * v[41] + (v[97] * v[45] + (v[100] * v[25] + v[45] * v[40]) * v[0] + v[98] * v[47] + (v[100] * v[24] + v[45] * v[38]) * v[34]) * jac[11]);
   v[98] = -1 * sin(x[3]);
   v[97] = cos(x[3]);
   v[41] = - v[97];
   v[92] = v[98] * v[71] + v[41] * v[72];
   v[71] = v[97] * v[71] + v[98] * v[72];
   v[94] = v[2] * v[92] + v[14] * v[71];
   v[94] = 0.2 * v[44] * v[94] - v[53] * 0.2 * v[94];
   v[69] = v[4] * v[92] + v[2] * v[71];
   v[63] = - v[69] * v[55];
   v[54] = - v[63];
   v[48] = v[43] * v[69];
   v[69] = 1.71408981830648 * v[54] + 0.114295509084676 * v[48] + v[53] * 0.2 * v[69];
   v[46] = v[60] * v[92];
   v[101] = v[59] * v[71];
   v[102] = v[2] * v[69] + v[4] * v[94];
   v[63] = v[54] + v[63] + 0.5 * v[102] - v[71] * v[64];
   v[103] = - v[63];
   v[69] = v[2] * v[94] + v[14] * v[69] + v[22] * v[46] + v[19] * v[101] + v[18] * v[103] - v[52] * 0.2 * v[92];
   v[102] = v[102] + v[52] * 0.2 * v[71] + v[16] * v[46] + v[6] * v[101] + v[11] * v[103];
   v[71] = v[41] * v[16] + v[98] * v[22];
   v[94] = v[41] * v[6] + v[98] * v[19];
   v[92] = v[98] * v[6] + v[97] * v[19];
   v[104] = v[98] * v[16] + v[97] * v[22];
   v[105] = v[20] * v[41] + v[92] * v[21] + v[23] * v[98] + v[104] * v[1];
   v[106] = v[98] * v[27] + v[41] * v[26] + v[105];
   v[27] = v[98] * v[26] + v[97] * v[27];
   v[104] = v[20] * v[98] + v[92] * v[1] + v[23] * v[97] + v[104] * v[12];
   v[92] = v[104] + v[27];
   v[27] = ((- v[28]) * (v[27] + v[92])) / v[29];
   v[23] = v[33] * v[27] + v[106] * v[28];
   v[20] = v[35] * v[98] + v[71] * v[1] + v[36] * v[41] + v[94] * v[21] - (v[37] * v[106] + v[23] * v[33]);
   v[94] = v[35] * v[97] + v[71] * v[12] + v[36] * v[98] + v[94] * v[1] - (v[37] * v[92] + v[23] * v[30]);
   v[71] = v[1] * v[102] + v[98] * v[86] + v[12] * v[69] + v[97] * v[88];
   v[63] = 0 - v[68] * v[46] - v[7] * v[101] - v[31] * v[103] - v[63] - v[71];
   v[36] = v[30] * v[27] + v[92] * v[28];
   v[106] = v[105] - (v[32] * v[106] + v[36] * v[33]);
   v[92] = v[104] - (v[32] * v[92] + v[36] * v[30]);
   v[104] = ((- jac[11]) * ((v[0] * v[106] + v[34] * v[20]) * v[34] + (v[0] * v[92] + v[34] * v[94]) * v[0])) / v[42];
   jac[5] = jac[11] * (0 - v[34] * (v[1] * v[69] + v[98] * v[88] + v[21] * v[102] + v[41] * v[86] + v[20] * v[90] + v[94] * v[91] + v[37] * v[63] + v[23] * v[81]) - v[0] * (v[71] + v[106] * v[90] + v[92] * v[91] + v[32] * v[63] + v[36] * v[81])) + v[104] * v[39] - 9.81 * (v[99] * v[104] + ((v[96] * v[92] + v[0] * v[94]) * v[0] + (v[96] * v[106] + v[0] * v[20]) * v[34]) * jac[11]);
   v[104] = -1 * sin(x[4]);
   v[92] = cos(x[4]);
   v[106] = - v[92];
   v[71] = v[104] * v[74] + v[106] * v[75];
   v[71] = 0.2 * v[44] * v[71] - v[53] * 0.2 * v[71];
   v[74] = v[92] * v[74] + v[104] * v[75];
   v[94] = - v[74] * v[55];
   v[20] = - v[94];
   v[102] = v[43] * v[74];
   v[74] = 1.71408981830648 * v[20] + 0.114295509084676 * v[102] + v[53] * 0.2 * v[74];
   v[69] = 0.2 * v[104];
   v[86] = 0.114295509084676 * v[106];
   v[88] = 0.114295509084676 * v[104];
   v[105] = 0.2 * v[92];
   v[35] = v[3] * v[106] + v[88] * v[14] + v[5] * v[104] + v[105] * v[2];
   v[26] = 0.5 * v[35];
   v[105] = v[3] * v[104] + v[88] * v[2] + v[5] * v[92] + v[105] * v[4];
   v[88] = 0.5 * v[105];
   v[5] = 0.5 * v[88];
   v[3] = ((- v[10]) * v[5]) / v[9];
   v[107] = v[17] * v[3] + v[26] * v[10];
   v[108] = v[13] * v[104] + v[69] * v[2] + v[15] * v[106] + v[86] * v[14] - (v[18] * v[26] + v[107] * v[17]);
   v[86] = v[13] * v[92] + v[69] * v[4] + v[15] * v[104] + v[86] * v[2] - (v[18] * v[88] + v[107] * v[8]);
   v[92] = v[2] * v[74] + v[104] * v[83] + v[4] * v[71] + v[92] * v[78];
   v[94] = v[20] + v[94] + 0.5 * v[92];
   v[69] = - v[94];
   v[74] = v[2] * v[71] + v[104] * v[78] + v[14] * v[74] + v[106] * v[83] + v[108] * v[84] + v[86] * v[85] + v[18] * v[69] + v[107] * v[80];
   v[71] = v[8] * v[3] + v[88] * v[10];
   v[35] = v[35] - (v[11] * v[26] + v[71] * v[17]);
   v[105] = v[105] - (v[11] * v[88] + v[71] * v[8]);
   v[92] = v[92] + v[35] * v[84] + v[105] * v[85] + v[11] * v[69] + v[71] * v[80];
   v[83] = v[21] * v[35] + v[1] * v[108];
   v[78] = v[21] * v[105] + v[1] * v[86];
   v[15] = v[26] - (v[18] * v[5] + v[107] * v[9]);
   v[13] = v[88] - (v[11] * v[5] + v[71] * v[9]);
   v[105] = v[1] * v[105] + v[12] * v[86];
   v[35] = v[1] * v[35] + v[12] * v[108];
   v[108] = v[105] * v[21] + v[35] * v[1];
   v[86] = v[1] * v[15] + v[21] * v[13] + v[108];
   v[13] = v[1] * v[13] + v[12] * v[15];
   v[35] = v[105] * v[1] + v[35] * v[12];
   v[105] = v[35] + v[13];
   v[15] = v[9] * v[3] + v[5] * v[10];
   v[13] = ((- v[28]) * (v[5] + v[13] + v[105] - (v[31] * v[5] + v[15] * v[9]))) / v[29];
   v[5] = v[33] * v[13] + v[86] * v[28];
   v[29] = v[83] * v[1] + v[78] * v[21] - (v[37] * v[86] + v[5] * v[33]);
   v[78] = v[83] * v[12] + v[78] * v[1] - (v[37] * v[105] + v[5] * v[30]);
   v[83] = v[1] * v[92] + v[12] * v[74];
   v[94] = 0 - (v[26] - (v[31] * v[26] + v[15] * v[17])) * v[84] - (v[88] - (v[31] * v[88] + v[15] * v[8])) * v[85] - (v[31] * v[69] + v[15] * v[80]) - v[94] - v[83];
   v[88] = v[30] * v[13] + v[105] * v[28];
   v[86] = v[108] - (v[32] * v[86] + v[88] * v[33]);
   v[105] = v[35] - (v[32] * v[105] + v[88] * v[30]);
   v[42] = ((- jac[11]) * ((v[0] * v[86] + v[34] * v[29]) * v[34] + (v[0] * v[105] + v[34] * v[78]) * v[0])) / v[42];
   jac[6] = jac[11] * (0 - v[34] * (v[1] * v[74] + v[21] * v[92] + v[29] * v[90] + v[78] * v[91] + v[37] * v[94] + v[5] * v[81]) - v[0] * (v[83] + v[86] * v[90] + v[105] * v[91] + v[32] * v[94] + v[88] * v[81])) + v[42] * v[39] - 9.81 * (v[99] * v[42] + ((v[96] * v[105] + v[0] * v[78]) * v[0] + (v[96] * v[86] + v[0] * v[29]) * v[34]) * jac[11]);
   v[42] = v[1] * v[0] + v[21] * v[34];
   v[105] = v[12] * v[0] + v[1] * v[34];
   v[86] = v[2] * v[42] + v[14] * v[105];
   v[86] = 0.2 * v[44] * v[86] - v[53] * 0.2 * v[86];
   v[83] = v[4] * v[42] + v[2] * v[105];
   v[78] = - v[83] * v[55];
   v[29] = - v[78];
   v[92] = v[43] * v[83];
   v[83] = 1.71408981830648 * v[29] + 0.114295509084676 * v[92] + v[53] * 0.2 * v[83];
   v[74] = v[60] * v[42];
   v[35] = v[59] * v[105];
   v[30] = v[2] * v[83] + v[4] * v[86];
   v[78] = v[29] + v[78] + 0.5 * v[30] - v[105] * v[64];
   v[108] = - v[78];
   v[83] = v[2] * v[86] + v[14] * v[83] + v[22] * v[74] + v[19] * v[35] + v[18] * v[108] - v[52] * 0.2 * v[42];
   v[30] = v[30] + v[52] * 0.2 * v[105] + v[16] * v[74] + v[6] * v[35] + v[11] * v[108];
   v[51] = v[51] * v[0];
   v[50] = v[50] * v[34];
   v[105] = v[1] * v[30] + v[12] * v[83];
   v[78] = v[34] * v[67] - v[68] * v[74] - v[7] * v[35] - v[31] * v[108] - v[78] - v[105];
   jac[7] = jac[11] * (0 - v[34] * (v[1] * v[83] + v[21] * v[30] + v[38] * v[51] + v[40] * v[50] + v[37] * v[78] - x[6] * 0.2 * v[0]) - v[0] * (v[105] + x[6] * 0.2 * v[34] + v[24] * v[51] + v[25] * v[50] + v[32] * v[78]));
   v[105] = 0.5 + v[1];
   v[30] = v[2] * v[105] + v[14] * v[12];
   v[30] = 0.2 * v[44] * v[30] - (v[53] * (0.2 * v[30] + 0.05) + v[77]);
   v[105] = v[4] * v[105] + v[2] * v[12];
   v[83] = -(v[79] * -0.05 + v[105] * v[55]);
   v[67] = - v[83];
   v[86] = v[43] * v[105];
   v[105] = 1.71408981830648 * v[67] + 0.114295509084676 * v[86] + v[53] * 0.2 * v[105] + v[82];
   v[60] = v[60] * v[1];
   v[59] = v[59] * v[12];
   v[42] = v[2] * v[105] + v[4] * v[30];
   v[83] = v[67] + v[83] + 0.5 * v[42] - (v[75] * -0.05 + v[12] * v[64]);
   v[64] = - v[83];
   v[105] = v[2] * v[30] + v[14] * v[105] + v[22] * v[60] + v[19] * v[59] + v[18] * v[64] - (v[52] * (0.2 * v[1] + 0.05) + v[87]);
   v[42] = v[42] + v[52] * 0.2 * v[12] + v[89] + v[16] * v[60] + v[6] * v[59] + v[11] * v[64];
   v[30] = -1 * v[72];
   v[33] = v[1] * v[42] + v[12] * v[105];
   v[83] = v[72] * -0.1 - v[68] * v[60] - v[7] * v[59] - v[31] * v[64] - v[83] - v[33];
   jac[8] = jac[11] * (0 - v[34] * (v[1] * v[105] + v[21] * v[42] + v[38] * v[70] + v[40] * v[30] + v[37] * v[83] - (x[6] * 0.1 + v[93])) - v[0] * (v[33] + v[95] + v[24] * v[70] + v[25] * v[30] + v[32] * v[83]));
   v[33] = v[2] * 0.5;
   v[33] = 0.2 * v[44] * v[33] - (v[53] * (0.2 * v[33] + 0.05) + v[77]);
   v[44] = v[4] * 0.5;
   v[55] = -(v[79] * -0.05 + v[44] * v[55]);
   v[42] = - v[55];
   v[43] = v[43] * v[44];
   v[44] = 1.71408981830648 * v[42] + 0.114295509084676 * v[43] + v[53] * 0.2 * v[44] + v[82];
   v[105] = -1 * v[75];
   v[95] = v[2] * v[44] + v[4] * v[33];
   v[55] = v[42] + v[55] + 0.5 * v[95] - v[75] * -0.05;
   v[75] = - v[55];
   v[44] = v[2] * v[33] + v[14] * v[44] + v[22] * v[73] + v[19] * v[105] + v[18] * v[75] - (v[52] * 0.05 + v[87]);
   v[95] = v[95] + v[89] + v[16] * v[73] + v[6] * v[105] + v[11] * v[75];
   v[89] = v[1] * v[95] + v[12] * v[44];
   v[55] = 0 - v[68] * v[73] - v[7] * v[105] - v[31] * v[75] - v[55] - v[89];
   jac[9] = jac[11] * (0 - v[34] * (v[1] * v[44] + v[21] * v[95] + v[37] * v[55]) - v[0] * (v[89] + v[32] * v[55]));
   v[77] = 0.2 * v[76] - (v[53] * 0.05 + v[77]);
   v[76] = - v[79] * -0.05;
   v[53] = - v[76];
   v[79] = -1 * v[79];
   v[82] = 1.71408981830648 * v[53] + 0.114295509084676 * v[79] + v[82];
   v[4] = v[2] * v[82] + v[4] * v[77];
   v[76] = v[53] + v[76] + 0.5 * v[4];
   v[89] = - v[76];
   v[82] = v[2] * v[77] + v[14] * v[82] + v[18] * v[89];
   v[4] = v[4] + v[11] * v[89];
   v[77] = v[1] * v[4] + v[12] * v[82];
   v[76] = 0 - v[31] * v[89] - v[76] - v[77];
   jac[10] = jac[11] * (0 - v[34] * (v[1] * v[82] + v[21] * v[4] + v[37] * v[76]) - v[0] * (v[77] + v[32] * v[76]));
   v[99] = jac[11] * v[39] - 9.81 * v[99] * jac[11];
   v[49] = 9.81 * v[45] + v[34] * jac[4] + v[47] * v[99] + v[49];
   v[100] = 9.81 * v[100] + v[0] * jac[4] + v[45] * v[99] + v[66];
   jac[12] = v[28] * v[56] - v[37] * v[49] - v[32] * v[100];
   v[56] = v[34] * jac[5];
   v[90] = 9.81 * v[0] + v[34] * v[99] + v[90];
   v[66] = v[0] * jac[5];
   v[99] = 9.81 * v[96] + v[0] * v[99] + v[91];
   jac[13] = v[28] * v[63] + v[27] * v[81] - (v[37] * v[56] + v[23] * v[90]) - (v[32] * v[66] + v[36] * v[99]);
   v[36] = v[34] * jac[6];
   v[63] = v[0] * jac[6];
   jac[14] = v[28] * v[94] + v[13] * v[81] - (v[37] * v[36] + v[5] * v[90]) - (v[32] * v[63] + v[88] * v[99]);
   v[51] = v[34] * jac[7] + v[51];
   v[50] = v[0] * jac[7] + v[50];
   jac[15] = v[28] * v[78] - v[37] * v[51] - v[32] * v[50];
   v[70] = v[34] * jac[8] + v[70];
   v[30] = v[0] * jac[8] + v[30];
   jac[16] = v[28] * v[83] - v[37] * v[70] - v[32] * v[30];
   v[83] = v[34] * jac[9];
   v[78] = v[0] * jac[9];
   jac[17] = v[28] * v[55] - v[37] * v[83] - v[32] * v[78];
   v[55] = v[34] * jac[10];
   v[88] = v[0] * jac[10];
   jac[18] = v[28] * v[76] - v[37] * v[55] - v[32] * v[88];
   v[34] = v[34] * jac[11];
   v[0] = v[0] * jac[11];
   jac[19] = 0 - v[37] * v[34] - v[32] * v[0];
   v[100] = jac[12] + v[100];
   v[61] = v[1] * v[49] + v[12] * v[100] + v[61];
   v[100] = v[21] * v[49] + v[1] * v[100] + v[62];
   jac[20] = v[10] * v[65] - v[18] * v[61] - v[11] * v[100] - v[31] * jac[12];
   v[66] = jac[13] + v[66];
   v[81] = v[28] * v[81] - v[37] * v[90] - v[32] * v[99];
   v[99] = v[81] + v[99];
   v[46] = v[1] * v[56] + v[98] * v[90] + v[12] * v[66] + v[97] * v[99] + v[46];
   v[66] = v[21] * v[56] + v[41] * v[90] + v[1] * v[66] + v[98] * v[99] + v[101];
   jac[21] = v[10] * v[103] - v[18] * v[46] - v[11] * v[66] - v[31] * jac[13];
   v[63] = jac[14] + v[63];
   v[103] = v[1] * v[36] + v[12] * v[63];
   v[84] = v[1] * v[90] + v[12] * v[99] + v[84];
   v[63] = v[21] * v[36] + v[1] * v[63];
   v[99] = v[21] * v[90] + v[1] * v[99] + v[85];
   jac[22] = v[10] * v[69] + v[3] * v[80] - (v[18] * v[103] + v[107] * v[84]) - (v[11] * v[63] + v[71] * v[99]) - (v[31] * jac[14] + v[15] * v[81]);
   v[50] = jac[15] + v[50];
   v[74] = v[1] * v[51] + v[12] * v[50] + v[74];
   v[50] = v[21] * v[51] + v[1] * v[50] + v[35];
   jac[23] = v[10] * v[108] - v[18] * v[74] - v[11] * v[50] - v[31] * jac[15];
   v[30] = jac[16] + v[30];
   v[60] = v[1] * v[70] + v[12] * v[30] + v[60];
   v[30] = v[21] * v[70] + v[1] * v[30] + v[59];
   jac[24] = v[10] * v[64] - v[18] * v[60] - v[11] * v[30] - v[31] * jac[16];
   v[78] = jac[17] + v[78];
   v[73] = v[1] * v[83] + v[12] * v[78] + v[73];
   v[78] = v[21] * v[83] + v[1] * v[78] + v[105];
   jac[25] = v[10] * v[75] - v[18] * v[73] - v[11] * v[78] - v[31] * jac[17];
   v[88] = jac[18] + v[88];
   v[75] = v[1] * v[55] + v[12] * v[88];
   v[88] = v[21] * v[55] + v[1] * v[88];
   jac[26] = v[10] * v[89] - v[18] * v[75] - v[11] * v[88] - v[31] * jac[18];
   v[0] = jac[19] + v[0];
   v[12] = v[1] * v[34] + v[12] * v[0];
   v[0] = v[21] * v[34] + v[1] * v[0];
   jac[27] = 0 - v[18] * v[12] - v[11] * v[0] - v[31] * jac[19];
   v[34] = jac[12] + jac[20];
   jac[28] = 34.2817963661296 * v[57] - v[34] - 1.71408981830648 * (v[14] * v[61] + v[2] * (0.5 * v[34] + v[100]) + v[58]);
   v[34] = jac[13] + jac[21];
   jac[29] = 34.2817963661296 * v[54] - v[34] - 1.71408981830648 * (v[14] * v[46] + v[2] * (0.5 * v[34] + v[66]) + v[48]);
   v[34] = jac[14] + jac[22];
   jac[30] = 34.2817963661296 * v[20] - v[34] - 1.71408981830648 * (v[14] * v[103] + v[106] * v[84] + v[2] * (0.5 * v[34] + v[63]) + v[104] * (0.5 * (v[81] + v[10] * v[80] - v[18] * v[84] - v[11] * v[99] - v[31] * v[81]) + v[99]) + v[102]);
   v[34] = jac[15] + jac[23];
   jac[31] = 34.2817963661296 * v[29] - v[34] - 1.71408981830648 * (v[14] * v[74] + v[2] * (0.5 * v[34] + v[50]) + v[92]);
   v[34] = jac[16] + jac[24];
   jac[32] = 34.2817963661296 * v[67] - v[34] - 1.71408981830648 * (v[14] * v[60] + v[2] * (0.5 * v[34] + v[30]) + v[86]);
   v[34] = jac[17] + jac[25];
   jac[33] = 34.2817963661296 * v[42] - v[34] - 1.71408981830648 * (v[14] * v[73] + v[2] * (0.5 * v[34] + v[78]) + v[43]);
   v[34] = jac[18] + jac[26];
   jac[34] = 34.2817963661296 * v[53] - v[34] - 1.71408981830648 * (v[14] * v[75] + v[2] * (0.5 * v[34] + v[88]) + v[79]);
   v[34] = jac[19] + jac[27];
   jac[35] = 0 - v[34] - 1.71408981830648 * (v[14] * v[12] + v[2] * (0.5 * v[34] + v[0]));
   // dependent variables without operations
   jac[0] = 1;
   jac[1] = 1;
   jac[2] = 1;
   jac[3] = 1;
}
