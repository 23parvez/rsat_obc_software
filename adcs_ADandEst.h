#ifndef ADCS_ADANDEST_H_INCLUDED
#define ADCS_ADANDEST_H_INCLUDED

//----------AD and Estimation---------------

// Quest Data Processing
extern int sc_qst; //sample count for magAD data processing(MC)
extern int dc_qst; //data count for magAD data processing (sec) NOTE: NOT MC
extern int wc_qst; //wait period count for magAD data processing (sec) NOTE: NOT MC
extern int mat_mag_DataCounter, mat_sm_DataCounter; //data counter for magAD and sunmagAD to track the filling matrices
extern int TC_wp_QDP; //Telecommand for Wait period selection in QDP
extern double NMB_mag[3][24]; //Measurement Matrix (Magnetic field) for magAD
extern double NRB_mag[3][24]; //Reference Matrix (Magnetic field) for magAD
extern double NMB_sunmag[3][8]; //Measurement Matrix (Magnetic field) for sunmagAD
extern double NRB_sunmag[3][8]; //Reference Matrix (Magnetic field) for sunmagAD
extern double NMS[3][8]; //Measurement Matrix (sun sensor) for sunmagAD
extern double NRS[3][8];  //Reference Matrix (sun model) for sunmagAD
extern int f_DataSort_MAG,f_DataSort_SUNMAG; //Flags that are raised to tell data matrices are ready for Quaternion computation in QUEST
extern int i_QDP;
extern int OBC_Quest_update, Quest_update_available, w_q_update_satisfy;
extern int TC_SunMagAD;

//DAD Quest
extern double Bcap_DAD[3][3], Bt_DAD[3][3];
extern double NRBt_mag[24][3], NRSt[8][3];
extern double NRBt_sunmag[8][3];
extern double wkm, wks;
extern double S_DAD[3][3], St_DAD[3][3], S_sqr_DAD[3][3];
extern double Z_DAD[3], Zt_DAD[3];
extern double sigma_DAD, delta_DAD, a_DAD, b_DAD,c_DAD,d_DAD, k_DAD;

extern double root_DAD, func_DAD, Dfunc_DAD, func_prev_DAD;
extern double alpha_DAD, beta_DAD, gamma_DAD;
extern double X_DAD[3];

extern double Q_quest_DAD[4];

extern int wAD_updatecount;
extern double Qquest_update[4];

extern int I_DAD, J_DAD, K_DADAD;
extern double NMB_NRBt[3][3], NMS_NRSt[3][3];
extern double Zt_Z_DAD, Zt_S_Z_DAD, Zt_Ssqr_Z_DAD;
extern double alpha_I_DAD[3][3], beta_S_DAD[3][3], sumX_DAD[3][3];

extern double W_cross[3][3];
extern double big_omega[4][4];
extern double Delta_by2;
extern double W_norm;
extern double Delta;
extern double cos_of_W_norm_and_Delta_by2;
extern double cos_of_W_norm_Delta_by2_with_I[4][4];
extern double sin_of_W_norm_and_Delta_by2;
extern double sin_of_W_norm_Delta_by2_and_W_norm;
extern double sin_of_W_norm_Delta_by2_W_norm_and_big_omega[4][4];
extern double cos_and_sin[4][4];
extern double trace_S_DAD, adj_S_DAD[3][3];


// Qgyro

extern double Del_Y_theta, Del_R_theta, Del_P_theta; //Small angle obtained by integrating Gyro data over a periof of time (mc/MC)
extern double Del_Q[4]; //Delta Qs
extern double q_prop_out[4],Qprop_prev[4];
extern double Qbody[4];

//Kalman Filter
extern int i_kf, j_kf, k_kf;
extern double sigma_v;
extern double sigma_u;
extern double sigma_m;
extern double Qk_temp1;
extern double Qk_temp2;
extern double Qk_temp3;
extern double Qk_temp4;
extern double qk_DCM[3][3];
extern double ref_cross[3][3];
extern double Hk[3][9];
extern double pk_HkT[9][3];
extern double pk_plus_one[9][9];
extern double phi_k_pk_plus[9][9];
extern double phi_k_pk_plus_phi_k_T[9][9];
extern double rk_Qk[9][9];
extern double rk_Qk_rk_T[9][9];
extern double Qk[9][9];
extern double  phi_k_T[9][9];
extern double phi_k[9][9];
extern double F_k1[3][3], mod_of_w_k, jw_cross[3][3], F_k4_temp1[3][3], F_k4_temp2[3][3], F_k4[3][3], F_k[9][9];
extern double phi_12[3][3];
extern double phi_12_temp2[3][3];
extern double Result_of_sine_Delta_cube;
extern double Result_of_sine_Delta;
extern double phi_12_temp1[3][3];
extern double phi_11[3][3];
extern double phi_11_temp4[3][3];
extern double phi_11_temp3[3][3];
extern double cos_of_Wk1_Delta_by_Wk1_square;
extern double Q_quest_DAD_den;
extern double mod_of_Wk1_square;
extern double mod_of_Wk1_cube;
extern double cos_of_Wk1_Delta;
extern double phi_11_temp2[3][3];
extern double phi_11_temp1[3][3];
extern double Wk1_one[3][3];
extern double sine_of_Wk1_Delta_by_Wk1;
extern double sine_of_Wk1_Delta;
extern double mod_of_Wk1;
extern double pk_plus[9][9];
extern double Kk_Hk_with_minus_one[9][9];
extern double Kk_Hk[9][9];
extern double qk_plus[4], qk_plus_conj[4];
extern double qk_plus_temp[4];
extern double Yk[3];
extern double Xk[9];
extern double q_by_two[4][3];
extern double Xk_plus_temp[3];
extern double Xk_plus[9];
extern double Yk_minus_hk_Kk[9];
extern double Yk_minus_hk[3];
extern double Kk_matrix[9][9];
extern double K_temp[3][3];
extern double pk_HkT_Hk_Rk[3][3];
extern double pk_HkT_Hk[3][3];
extern double pk[9][9];
extern double Rk[3][3];
extern double Hk_temp[9][3];
extern double s_hk[3];
extern double Wk1[3][3];
extern double phi_12_temp3[3][3];
extern double qk_minus[4], hk_xk_minus[3];
extern int TC_KalmanFilter_ENABLE;
extern double sun_noise;
extern double mag_noise;

// Extended Kalman Filter 2

extern int ecl_ekf, kfmeas, tor_counterk;

extern double T_RW_NETk[3], T_MTk[3], Mk[3], T_NETk[3], w_k[3], RWSpeedk[4];
extern double b_k[3], lpfx_magbk, lpfy_magbk, lpfz_magbk, lpfb_k[3], lpfb_k_prev[3];
extern double pbk, qqbk;

extern double kfk1[4], kfq_dot[4], kfg1[3], kfw_dot[3], kfv1[4], kfv_dot[4], kfq01[4], kfw01[3], kfv01[4];
extern double kfk2[4], kfg2[3], kfv2[4], kfq02[4], kfw02[3], kfv02[4];
extern double kfk3[4], kfg3[3], kfv3[4], kfq03[4], kfw03[3], kfv03[4];
extern double kfk4[4], kfg4[3], kfv4[4];

extern double kfom[4][4], kfq_dot[4], kfv_dot[4], kfT_RW_NET_temp[4], kfT_RW_NET[3], kfH_wh_ind[4], kfH_wh2body[3], kfI_MAT_w0[3], kfI_MAT_w0_HW2B[3], kfT_NET[3], kfw0crossI_MAT_w0_HW2B[3], kfw_dot_temp[3], kfw_dot[3];
extern int f_EKF2_prop_en,f_EKF2_mag_bias_dis;

//Function Declarations

extern void rDAD_quest(void);
extern void rErrorComputation(void);
extern void rExtendedKalmanFilter1_p1(void);
extern void rExtendedKalmanFilter1_p2(void);
extern void rQuestDataProcessing(void);

extern void rExtendedKalmanFilter2_p1(void);
extern void rExtendedKalmanFilter2_Prop(void);
extern void rExtendedKalmanFilter2_p2(void);
/*static void rEKFDynamics(void);
static void rEKF_dy_int(double kfq_dy[4], double kfw_dy[3], double kfv_dy[4]);
static void rQ_Propagation(double q_prop[4], double w_prop[3]);*/

#endif // ADCS_ADANDEST_H_INCLUDED
