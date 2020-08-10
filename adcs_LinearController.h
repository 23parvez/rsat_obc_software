#ifndef ADCS_LINEARCONTROLLER_H_INCLUDED
#define ADCS_LINEARCONTROLLER_H_INCLUDED

///Dutycycle Generation
int DutyCycleGenEnable, TorquerPolaritySetFlag;
double MR, MP, MY;
int Roll_MTR_Pol_Reversal, Pitch_MTR_Pol_Reversal, Yaw_MTR_Pol_Reversal;
int MR_Polarity, MP_Polarity, MY_Polarity, DPM_Polarity[3], Ton[3], Toff[3];
int Roll_MTREnable, Pitch_MTREnable, Yaw_MTREnable;
int MTR_ActuationCycle;

int i_lict, j_lict;
int CB_Detumbling_Mode;
double TorquerDutyCycle[3];
///double DutyCycle_Roll;
///double DutyCycle_Pitch;
///double DutyCycle_Yaw;
float ActuationCycle;

///Angular Momentum Dumping
int f_Momentum_Dumping;
int dumping_on;
double delta_HB[3];
double DPM[3];
double TC_MDk;

///Speed based Momentum Dumping
double MOI_wh;

int check_dump_wh[4];
double RWSpeed[4], H_retn;

double T_RW_sdump[4];
int wh_sdump_start[4], speed_based_torquer_control;
double TB_sMD[3], u_parl[3], Tc[3], T_RW[4], T_RW_sdump[4], T_RWB[3];
double del_Vw[4];
double T_RWBn[3], tau_ms, min_TW;
double u_perp[3];
double Pse_Inv_Dist_Mat[4][3];



///Linear Controller

double Tc[3],Qerror[4];
double RWSpeed[4], v0_rad_sec[4];
double H_wh[4], HB[3];
double T_RW[4];
int CB_Wheel_OverSpeed_TorqueCutOff, DFriction_Compensation;

double B2wh_mat[4][3], wh2B_mat[3][4];


///wheel dynamic friction
int k;
double speedDFCch;
int TachoHyst[4], DFCCount[4], FirstSpeed[4];
double DFCGainHigh, LPFK1HighSL, LPFK2HighSL, DFCGainLow, LPFK1LowSL, LPFK2LowSL;
double ExWhMom[4], DFCGainSL[4], LPFK1SL[4], LPFK2SL[4],
DFCGainSL[4], LPFK1SL[4], LPFK2SL[4], WhMom0[4], ActWhMom[4], LossWhMom[4], LossWhMomFilt[4], DFCTorq[4];
int DFCCountLimSL[4], DFCCountLimSL[4], DFCcountHigh, DFCcountLow;


///wheel spin updown
int wheel_spin_logic, spin_up_avg_count, spin_up_avg_count_2;
double del_v0[4], v0c[4], del_v0a, T_RW_spin[4];

/// for dynamics
int tor_counter;

///wheel auto reconfiguration
int wheel_index[4], wheel_index_ARCsum, Wheel_Config, RW_ARC_Logic, RW_ARC_Count, count_arc_w0, count_arc_w1, count_arc_w2, count_arc_w3;
double pres_exp_whsp_ch[4], exp_whsp_ch[4], ch_obs_whsp[4], prev_obs_whsp_ch[4], diff_obs_exp_ch[4];

void rWheel_Auto_Reconfiguration(void);
void rWheel_Spin_updown(void);
void rWheel_Dither_Torque(void);
void rWheel_Dynamic_Friction(void);
void rLinearController(void);
void rAngularMomentumDumping(void);
void rSpeedBasedMomentumDumping(void);
void rDutyCycleGeneration(void);
void rTorquer_Polarity_Check(void);
void rTwo_RW_control(void);

///Two RW control

double Twof_veh[3][3], Tv_wof[3][3], Tcv_temp[3], Tcv[3], Tdeficit_temp1[3], Tdeficit_temp2[3], Tdeficit[3], Tdefnorm;
double alpha_2RW_temp[3], alpha_2RW_tempnorm, alpha_2RW, B_Tef_ANG, mu_m_temp[3], mu_m[3];
double Tdes[3], T_MT_2RW[3], Trwa_mt_comp[3], Trwa[4], T_SD_RW[4], Tmm_actual[3];

#endif // ADCS_LINEARCONTROLLER_H_INCLUDED
