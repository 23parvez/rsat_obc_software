#ifndef ADCS_COMMONROUTINES_H_INCLUDED
#define ADCS_COMMONROUTINES_H_INCLUDED

int i_comr, j_comr, k_comr;
///Rotation matrices
double Rx[3][3], Ry[3][3], Rz[3][3];

///mat inverse
double determinant, Invmatout33[3][3];

///mat multiplication
double Matout31[3], Matout33[3][3], Matout441[4];
int i, j;

///Cross product
double Cross_Product[3];

///Vector Normalization
double Norm_out[3], vecnorm_mag;

double out_Quat_norm[4]; ///Global variable (Normalized Quaternion)
double mat_mult[3][3]; ///Global variable (matrix multipication of 2 matrices[3][3])
double mat_adj[3][3]; ///Global variable (Adjoint of matrix[3][3])
double DCM2Q_out[4]; ///Global variable (RM to Quaternion Conversion)
double out_quatMult[4]; ///Global variable (Multiplication of 2 Quaternions)

///Function declaration
void rMatMul3x3(double mat331[3][3], double mat332[3][3]);
void rMatMul3x1(double mat33[3][3], double vec31[3]);
void rMatMul44x1(double mat44[4][4], double vec41[4]);
void rCross_Product(double u[3], double v[3]);
void rRM_to_Quat(double DCM2Q_in[3][3]);
void rQs_Normalization(double in_Quat_norm[4]);
void rVectorNorm(double vecnorm_in[3]);
void rQs_Normalization(double in_Quat_norm[4]);
void rMat_adjoint3(double in_mat_adj[3][3]);
void rRM_to_Quat(double DCM2Q_in[3][3]);
void rQuat_multiply(double in_quat1[4],double in_quat2[4]);
void rQs_Multiplication(double in_Quat1[4],double in_Quat2[4]);


void rMatMul34x1(double mat34[3][4], double vec41[4]);
double Matout341[3];


void rMatMul43x1(double mat43[4][3], double vec31[4]);
double Matout431[4];

double out_Quat_mult[4]; ///Global Variable of the Out_Quat from Q Multiplication Routine


#endif // ADCS_COMMONROUTINES_H_INCLUDED






