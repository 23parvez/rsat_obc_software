#ifndef ADCS_COMMONROUTINES_H_INCLUDED
#define ADCS_COMMONROUTINES_H_INCLUDED

extern int i_comr, j_comr, k_comr;
//Rotation matrices
extern double Rx[3][3], Ry[3][3], Rz[3][3];

//mat inverse
extern double determinant, Invmatout33[3][3];

//mat multiplication
extern double Matout31[3], Matout33[3][3], Matout441[4];
//extern int i, j;

//Cross product
extern double Cross_Product[3];

//Vector Normalization
extern double Norm_out[3], vecnorm_mag;

extern double out_Quat_norm[4]; //Global variable (Normalized Quaternion)
extern double mat_mult[3][3]; //Global variable (matrix multipication of 2 matrices[3][3])
extern double mat_adj[3][3]; //Global variable (Adjoint of matrix[3][3])
extern double DCM2Q_out[4]; //Global variable (RM to Quaternion Conversion)
extern double Matout431[4];
extern double Matout341[3];
extern double out_Quat_mult[4]; //Global Variable of the Out_Quat from Q Multiplication Routine

//Function declaration
extern void rMatMul3x3(double mat331[3][3], double mat332[3][3]);
extern void rMatMul3x1(double mat33[3][3], double const vec31[3]);
extern void rMatMul44x1(double mat44[4][4], double const vec41[4]);
extern void rCross_Product(double const u[3], double const v[3]);
extern void rRM_to_Quat(double DCM2Q_in[3][3]);
extern void rQs_Normalization(double const in_Quat_norm[4]);
extern void rVectorNorm(double const vecnorm_in[3]);
extern void rQs_Normalization(double const in_Quat_norm[4]);
extern void rMat_adjoint3(double in_mat_adj[3][3]);
extern void rRM_to_Quat(double DCM2Q_in[3][3]);
extern void rQs_Multiplication(double const in_Quat1[4],double const in_Quat2[4]);
extern void rMatInv(double mat2inv[3][3]);
extern void rxRot(double const th);
extern void ryRot(double const th);
extern void rzRot(double const th);

extern void rMatMul34x1(double mat34[3][4], double const vec41[4]);


extern void rMatMul43x1(double mat43[4][3], double const vec31[3]);


double rsign_f(double const sign_var);


#endif // ADCS_COMMONROUTINES_H_INCLUDED






