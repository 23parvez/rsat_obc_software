#ifndef ADCS_LINEARCONTROLLER_H_INCLUDED
#define ADCS_LINEARCONTROLLER_H_INCLUDED

extern double TC_KR[3];
extern double TC_KP[3];
extern float RW_Wheel_Speed[4];
extern double TC_wh_speed_cutoff;

//Dutycycle Generation
extern int DutyCycleGenEnable, TorquerPolaritySetFlag;
extern int Roll_MTR_Pol_Reversal, Pitch_MTR_Pol_Reversal, Yaw_MTR_Pol_Reversal;
extern int DPM_Polarity[3], Ton[3], Toff[3], DPM_Pol_prev[3];
extern int Roll_MTREnable, Pitch_MTREnable, Yaw_MTREnable;
extern unsigned int MTR_ActuationCycle;

extern int i_lict, j_lict;
extern unsigned int TorquerDutyCycle[3];


//Angular Momentum Dumping
extern int f_Momentum_Dumping;
extern int dumping_on;
extern double delta_HB[3];
extern double DPM[3];
extern double TC_MDk;
extern double TC_Hmax;

//Speed based Momentum Dumping
extern double TC_max_whsp_spdump, TC_min_whsp_spdump, TC_SpeedDumpLimit, temp_lic;
extern double TC_SpeedDumpTime;

extern int check_dump_wh[4];
extern double H_retn;

extern double T_RW_sdump[4];
extern int wh_sdump_start[4];
extern double TB_sMD[3], u_parl[3], T_RW_sdump[4], T_RWB[3];
extern double del_Vw[4];
extern double T_RWBn[3], tau_ms, min_TW;
extern double u_perp[3];
extern double Pse_Inv_Dist_Mat[4][3];



//Linear Controller

extern double Tc[3],Qerror[4];
extern double RWSpeed_RPM[4], RWSpeed_RAD[4];
extern double H_wh[4], HB[3];
extern double T_RW[4];

extern double B2wh_mat[4][3], wh2B_mat[3][4];


//wheel dynamic friction

extern double speedDFCch;
extern int TachoHyst[4], DFCCount[4], FirstSpeed[4];
extern double DFCGainHigh, LPFK1HighSL, LPFK2HighSL, DFCGainLow, LPFK1LowSL, LPFK2LowSL;
extern double ExWhMom[4], DFCGainSL[4], LPFK1SL[4], LPFK2SL[4],
DFCGainSL[4], LPFK1SL[4], LPFK2SL[4], WhMom0[4], ActWhMom[4], LossWhMom[4], LossWhMomFilt[4], DFCTorq[4];
extern int DFCCountLimSL[4], DFCCountLimSL[4], DFCcountHigh, DFCcountLow;


//wheel spin updown
extern int wheel_spin_logic, spin_up_avg_count, spin_up_avg_count_2;
extern double del_v0[4], del_v0a, T_RW_spin[4];
extern float TC_RW_Nominal[4];

// for dynamics
extern int tor_counter;

//wheel auto reconfiguration
extern int wheel_index[4], wheel_index_ARCsum, Wheel_Config, RW_ARC_Logic, RW_ARC_Count, count_arc_w0, count_arc_w1, count_arc_w2, count_arc_w3;
extern int TC_ARC_Time_Cycle;
extern double pres_exp_whsp_ch[4], exp_whsp_ch[4], ch_obs_whsp[4], prev_obs_whsp_ch[4], diff_obs_exp_ch[4];
extern double TC_ARC_RPM_Thres;

extern double Twof_veh[3][3], Tv_wof[3][3], Tcv_temp[3], Tcv[3], Tdeficit_temp1[3], Tdeficit_temp2[3], Tdeficit[3], Tdefnorm;
extern double alpha_2RW_temp[3], alpha_2RW_tempnorm, alpha_2RW, B_Tef_ANG, mu_m_temp[3], mu_m[3];
extern double Tdes[3], T_MT_2RW[3], Trwa_mt_comp[3], Trwa[4], T_SD_RW[4], Tmm_actual[3];

extern double TC_AngMomDump_Thrsld;
extern double TC_momentum_dumping_gain;


/*static void rWheel_Auto_Reconfiguration(void);
static void rWheel_Spin_updown(void);
static void rSpeedBasedMomentumDumping(void);*/
extern void rWheel_Dither_Torque(void);
extern void rWheel_Dynamic_Friction(void);
extern void rLinearController(void);
extern void rAngularMomentumDumping(void);
extern void rDutyCycleGeneration(void);
extern void rTorquer_Polarity_Check(void);
extern void rTwo_RW_control(void);

#endif // ADCS_LINEARCONTROLLER_H_INCLUDED
