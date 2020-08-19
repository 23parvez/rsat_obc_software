#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "adcs_VarDeclarations.h"

#include "Telecommand.h"
#include "TC_List.h"


void rADCS_Pon_vars(void)
{

	CB_Detumbling_Mode = 0;
	CB_OrbitModel = 0;
	CB_Q_propagation = 0;
	CB_DAD_quest = 0;
	CB_QuestDataProcessing = 0;
	CB_ErrorComputation = 0;
	CB_ExtendedKalmanFilter = 0;
	CB_LinearController = 0;
	CB_Wheel_OverSpeed_TorqueCutOff = 0;
	CB_DutyCycleGeneration = 0;
	CB_AngularMomentumDumping = 0;
	CB_SpeedBasedMomentumDumping = 0;
	CB_Wheel_Dynamic_Friction = 0;
	CB_Wheel_Spin_updown = 0;
	CB_Wheel_Auto_Reconfiguration = 0;
	CB_Two_RW_control = 0;
	CB_MagFieldComp = 0;
	CB_Sun_model = 0;
	CB_ReferenceQuatComputation = 0;
	CB_RefVectorGeneration = 0;
	CB_RefRate_Computation = 0;
	CB_Sl_Ecl_OnBrd_detection = 0;
	CB_BDOT_Computation = 0;

	/// IMU ///////////////////////////////////////////////////////////

	for (i_pini=0; i_pini<3; i_pini++)
	{
		w_BODY[i_pini] = 0.0;
		w_BODYdeg[i_pini] = 0.0;
		B_BODY[i_pini] = 0.0;
		B_BODYn[i_pini] = 0.0;
		B_BODYtesla[i_pini] = 0.0;
		DelThta_rawdata[i_pini] = 0.0;
		w_ABC[i_pini] = 0.0;
		w_BiasCor[i_pini] = 0.0;
		w_ASC[i_pini] = 0.0;
		w_AMSC[i_pini] = 0.0;
		w_RPY[i_pini] = 0.0;
		w_LPF[i_pini] = 0.0;
		B_BiasCor[i_pini] = 0.0;
		B_ASC[i_pini] = 0.0;
		B_AMSC[i_pini] = 0.0;
		B_RPY[i_pini] = 0.0;
		B_LPF[i_pini] = 0.0;
		w_BODY_IMU1[i_pini] = 0.0;
		w_BODY_IMU2[i_pini] = 0.0;
		B_BODY_IMU1[i_pini] = 0.0;
		B_BODY_IMU2[i_pini] = 0.0;
		IMU_prcd_data[i_pini] = 0.0;
	}

    /*IMU1_Corr.DB_w_BiasCor[0] = 0.0;
    IMU1_Corr.DB_w_BiasCor[1] = 0.0;
    IMU1_Corr.DB_w_BiasCor[2] = 0.0;*/

    IMU1_Corr.DB_w_BiasCor[0] = 0.0;
    IMU1_Corr.DB_w_BiasCor[1] = 0.0;
    IMU1_Corr.DB_w_BiasCor[2] = 0.0;

    IMU1_Corr.DB_wSFC_Neg[0] = 1.000133267;
    IMU1_Corr.DB_wSFC_Neg[1] = 1.010407138;
    IMU1_Corr.DB_wSFC_Neg[2] = 1.004991424;

    IMU1_Corr.DB_wSFC_Pos[0] = 1.000133267;
    IMU1_Corr.DB_wSFC_Pos[1] = 1.010407138;
    IMU1_Corr.DB_wSFC_Pos[2] = 1.004991424;

    /*IMU1_Corr.DB_wSFC_Neg[0] = 1.0;
    IMU1_Corr.DB_wSFC_Neg[1] = 1.0;
    IMU1_Corr.DB_wSFC_Neg[2] = 1.0;

    IMU1_Corr.DB_wSFC_Pos[0] = 1.0;
    IMU1_Corr.DB_wSFC_Pos[1] = 1.0;
    IMU1_Corr.DB_wSFC_Pos[2] = 1.0;*/

    IMU1_Corr.DB_B_BiasCor[0] = 0.0;
    IMU1_Corr.DB_B_BiasCor[1] = 0.0;
    IMU1_Corr.DB_B_BiasCor[2] = 0.0;

    IMU1_Corr.DB_BSFC_Neg[0] = 1.0;
    IMU1_Corr.DB_BSFC_Neg[1] = 1.0;
    IMU1_Corr.DB_BSFC_Neg[2] = 1.0;

    IMU1_Corr.DB_BSFC_Pos[0] = 1.0;
    IMU1_Corr.DB_BSFC_Pos[1] = 1.0;
    IMU1_Corr.DB_BSFC_Pos[2] = 1.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			IMU_Sen2Bdy[i_pini][j_pini] = 0.0;

		}
	}

	IMU_Sen2Bdy[0][2] = -1.0;
	IMU_Sen2Bdy[1][0] = -1.0;
	IMU_Sen2Bdy[2][1] = 1.0;

	Bsq = 0.0;

	IMU1_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU1_Corr.DB_w_MisCor[0][1] = 0.000225;
	IMU1_Corr.DB_w_MisCor[0][2] = 0.00001391;
	IMU1_Corr.DB_w_MisCor[1][0] = -0.000227;
	IMU1_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU1_Corr.DB_w_MisCor[1][2] = 0.000063;
	IMU1_Corr.DB_w_MisCor[2][0] = -0.000207;
	IMU1_Corr.DB_w_MisCor[2][1] = -0.000063;
	IMU1_Corr.DB_w_MisCor[2][2] = 1.0;

	/*IMU1_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU1_Corr.DB_w_MisCor[0][1] = 0.0;
	IMU1_Corr.DB_w_MisCor[0][2] = 0.0;
	IMU1_Corr.DB_w_MisCor[1][0] = 0.0;
	IMU1_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU1_Corr.DB_w_MisCor[1][2] = 0.0;
	IMU1_Corr.DB_w_MisCor[2][0] = 0.0;
	IMU1_Corr.DB_w_MisCor[2][1] = 0.0;
	IMU1_Corr.DB_w_MisCor[2][2] = 1.0;*/

	IMU1_Corr.DB_B_MisCor[0][0] = 1.0;
	IMU1_Corr.DB_B_MisCor[0][1] = 0.0;
	IMU1_Corr.DB_B_MisCor[0][2] = 0.0;
	IMU1_Corr.DB_B_MisCor[1][0] = 0.0;
	IMU1_Corr.DB_B_MisCor[1][1] = 1.0;
	IMU1_Corr.DB_B_MisCor[1][2] = 0.0;
	IMU1_Corr.DB_B_MisCor[2][0] = 0.0;
	IMU1_Corr.DB_B_MisCor[2][1] = 0.0;
	IMU1_Corr.DB_B_MisCor[2][2] = 1.0;


    IMU1_Corr.DB_GyroLPF[0] = 0.0;
	IMU1_Corr.DB_GyroLPF[1] = 1.0;

	IMU1_Corr.DB_MagLPF[0] = 0.0;
	IMU1_Corr.DB_MagLPF[1] = 1.0;

	// IMU2

	IMU2_Corr.DB_w_BiasCor[0] = 0.0;
	IMU2_Corr.DB_w_BiasCor[1] = 0.0;
	IMU2_Corr.DB_w_BiasCor[2] = 0.0;

	IMU2_Corr.DB_wSFC_Neg[0] = 1.000133267;
	IMU2_Corr.DB_wSFC_Neg[1] = 1.010407138;
	IMU2_Corr.DB_wSFC_Neg[2] = 1.004991424;

	IMU2_Corr.DB_wSFC_Pos[0] = 1.000133267;
	IMU2_Corr.DB_wSFC_Pos[1] = 1.010407138;
	IMU2_Corr.DB_wSFC_Pos[2] = 1.004991424;

	/*IMU2_Corr.DB_wSFC_Neg[0] = 1.0;
	IMU2_Corr.DB_wSFC_Neg[1] = 1.0;
	IMU2_Corr.DB_wSFC_Neg[2] = 1.0;

	IMU2_Corr.DB_wSFC_Pos[0] = 1.0;
	IMU2_Corr.DB_wSFC_Pos[1] = 1.0;
	IMU2_Corr.DB_wSFC_Pos[2] = 1.0;*/

	IMU2_Corr.DB_B_BiasCor[0] = 0.0;
	IMU2_Corr.DB_B_BiasCor[1] = 0.0;
	IMU2_Corr.DB_B_BiasCor[2] = 0.0;

	IMU2_Corr.DB_BSFC_Neg[0] = 1.0;
	IMU2_Corr.DB_BSFC_Neg[1] = 1.0;
	IMU2_Corr.DB_BSFC_Neg[2] = 1.0;

	IMU2_Corr.DB_BSFC_Pos[0] = 1.0;
	IMU2_Corr.DB_BSFC_Pos[1] = 1.0;
	IMU2_Corr.DB_BSFC_Pos[2] = 1.0;

	Bsq = 0.0;

	IMU2_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU2_Corr.DB_w_MisCor[0][1] = 0.000225;
	IMU2_Corr.DB_w_MisCor[0][2] = 0.00001391;
	IMU2_Corr.DB_w_MisCor[1][0] = -0.000227;
	IMU2_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU2_Corr.DB_w_MisCor[1][2] = 0.000063;
	IMU2_Corr.DB_w_MisCor[2][0] = -0.000207;
	IMU2_Corr.DB_w_MisCor[2][1] = -0.000063;
	IMU2_Corr.DB_w_MisCor[2][2] = 1.0;

	/*IMU2_Corr.DB_w_MisCor[0][0] = 1.0;
	IMU2_Corr.DB_w_MisCor[0][1] = 0.0;
	IMU2_Corr.DB_w_MisCor[0][2] = 0.0;
	IMU2_Corr.DB_w_MisCor[1][0] = 0.0;
	IMU2_Corr.DB_w_MisCor[1][1] = 1.0;
	IMU2_Corr.DB_w_MisCor[1][2] = 0.0;
	IMU2_Corr.DB_w_MisCor[2][0] = 0.0;
	IMU2_Corr.DB_w_MisCor[2][1] = 0.0;
	IMU2_Corr.DB_w_MisCor[2][2] = 1.0;*/

	IMU2_Corr.DB_B_MisCor[0][0] = 1.0;
	IMU2_Corr.DB_B_MisCor[0][1] = 0.0;
	IMU2_Corr.DB_B_MisCor[0][2] = 0.0;
	IMU2_Corr.DB_B_MisCor[1][0] = 0.0;
	IMU2_Corr.DB_B_MisCor[1][1] = 1.0;
	IMU2_Corr.DB_B_MisCor[1][2] = 0.0;
	IMU2_Corr.DB_B_MisCor[2][0] = 0.0;
	IMU2_Corr.DB_B_MisCor[2][1] = 0.0;
	IMU2_Corr.DB_B_MisCor[2][2] = 1.0;


	IMU2_Corr.DB_GyroLPF[0] = 0.0;
	IMU2_Corr.DB_GyroLPF[1] = 1.0;

	IMU2_Corr.DB_MagLPF[0] = 0.0;
	IMU2_Corr.DB_MagLPF[1] = 1.0;

	/// Sun Sensors ///////////////////////////////////////////////////////////

	SS_Main_2Exe_DB.DB_Imax_RPD = 7.4536;//1.073296595956431;
	SS_Main_2Exe_DB.DB_Imax_RND = 6.8237;//1.172387413685503;
	SS_Main_2Exe_DB.DB_Imax_RPND = 6.9977;//1.143227326266195;
	SS_Main_2Exe_DB.DB_Imax_RNND = 7.1851;//1.113413888039156;
	SS_Main_2Exe_DB.DB_Imax_PP = 8.0;//1.0;
	SS_Main_2Exe_DB.DB_Imax_PN = 7.0977;//1.127119300147093;
	SS_Main_2Exe_DB.DB_Imax_YP = 7.5456;//1.060224293620740;
	SS_Main_2Exe_DB.DB_Imax_YN = 7.4365;//1.075774928880186;

	SS_Redundant_2Exe_DB.DB_Imax_RPD = 7.2932;//1.096912051131744;
	SS_Redundant_2Exe_DB.DB_Imax_RND = 7.0657;//1.132238986590674;
	SS_Redundant_2Exe_DB.DB_Imax_RPND = 7.10883;//1.125363752764521;
	SS_Redundant_2Exe_DB.DB_Imax_RNND = 7.3405;//1.089843309660692;
	SS_Redundant_2Exe_DB.DB_Imax_PP	= 8.0;//1.0;
	SS_Redundant_2Exe_DB.DB_Imax_PN = 7.4311;//1.076554757530204;
	SS_Redundant_2Exe_DB.DB_Imax_YP = 7.8809;//1.015119697606048;
	SS_Redundant_2Exe_DB.DB_Imax_YN = 6.9909;//1.144345150026632;


	/*SS_Main_2Exe_DB.DB_misaln_cor125[0][0] = 1.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[0][1] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[0][2] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][0] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][1] = 1.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][2] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][0] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][1] = 0.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][2] = 1.0;*/

	SS_Main_2Exe_DB.DB_misaln_cor125[0][0] = 1.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[0][1] = -0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[0][2] = 0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][0] = 0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][1] = 1.0;
	SS_Main_2Exe_DB.DB_misaln_cor125[1][2] = -0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][0] = -0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][1] = 0.005235963831420;
	SS_Main_2Exe_DB.DB_misaln_cor125[2][2] = 1.0;

	SS_Redundant_2Exe_DB.DB_misaln_cor125[0][0] = 1.0;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[0][1] = -0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[0][2] = 0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[1][0] = 0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[1][1] = 1.0;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[1][2] = -0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[2][0] = -0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[2][1] = 0.005235963831420;
	SS_Redundant_2Exe_DB.DB_misaln_cor125[2][2] = 1.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			SS_Main_2Exe_DB.DB_misaln_cor126[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor325[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor326[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor345[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor346[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor145[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Main_2Exe_DB.DB_misaln_cor146[i_pini][j_pini] = SS_Main_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];

		}
	}

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			SS_Redundant_2Exe_DB.DB_misaln_cor126[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor325[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor326[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor345[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor346[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor145[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];
			SS_Redundant_2Exe_DB.DB_misaln_cor146[i_pini][j_pini] = SS_Redundant_2Exe_DB.DB_misaln_cor125[i_pini][j_pini];

		}
	}

	SS_M1 = 0.0;
	SS_M2 = 0.0;
	SS_M3 = 0.0;
	SS_M4 = 0.0;
	SS_M5 = 0.0;
	SS_M6 = 0.0;
	SS_M7 = 0.0;
	SS_M8 = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		SB_MAIN[i_pini] = 0.0;
		SB_RED[i_pini] = 0.0;
		SS_prcd_data[i_pini] = 0.0;
		sun_sf[i_pini] = 0.0;
		S_BODY[i_pini] = 0.0;
		S_BODYn[i_pini] = 0.0;
	}

	SC1 = 0.0;
	SC2 = 0.0;
	SC3 = 0.0;
	SC4 = 0.0;
	SC5 = 0.0;
	SC6 = 0.0;
	SC7 = 0.0;
	SC8 = 0.0;

	SC1ImaxF = 0.0;
	SC2ImaxF = 0.0;
	SC3ImaxF = 0.0;
	SC4ImaxF = 0.0;
	SC5ImaxF = 0.0;
	SC6ImaxF = 0.0;

	comb1 = 0.0;
	comb2 = 0.0;
	temp11 = 0.0;
	temp12 = 0.0;

	ele1 = 0.0;
	az1 = 0.0;
	ele2 = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			ss2b[i_pini][j_pini] = 0.0;

		}
	}

	ss2b[0][0] = 1.0;
	ss2b[1][1] = 1.0;
	ss2b[2][2] = 1.0;

	Ang_Deviation = 0.0;

	Roll_ang_err = 0.0;
	Yaw_ang_err = 0.0;
	sun_quadrant = 0;
	f_Sunlit_Presence = 1;

	i_MatEq = 0.0;
	j_MatEq = 0.0;
	Sunlit_presence_timer = 0;

	SunNPP_SMtransit_counter = 0;
	SunNPP_SMtransit_count_limit = 0;
	SunNPP_SMtransit = 0.0;

	AngDev_SAMtransit_thrsld = 0.0;
	SunNPP_SAMtransit_counter = 0.0;
	SunNPP_SAMtransit_count_limit = 0.0;
	SunNPP_SAMtransit = 0.0;

	///BDOT Computation   ///Det gyro

	BDOT_Counter = 0;
	Det_BDOT_MC_Count = 0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		BDOT[i_pini] = 0.0;
		Bpresent[i_pini] = 0.0;
		Bprev[i_pini] = 0.0;
		gyrodet_w[i_pini] = 0.0;
		gyrodet_B[i_pini] = 0.0;
	}

	GYRO_Counter = 0.0;

	// Error Computation

	magRoll_angle = 0.0;
	magPitch_angle = 0.0;
	magYaw_angle = 0.0;

	wAD_updatecount = 0;
	w_q_update_satisfy = 0;

	PolCheck_LUT = 0;
	PolChec_LUT_res = 0;

	Quest_update_available = 0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		Qquest_update[i_pini] = 0.0;
		Qbody[i_pini] = 0.0;
		Q_REF[i_pini] = 0.0;
		Q_REF_conj[i_pini] = 0.0;
		Qerror[i_pini] = 0.0;
	}

	Qquest_update[3] = 1.0;
	Qbody[3] = 1.0;
	Q_REF[3] = 1.0;
	Q_REF_conj[3] = 1.0;
	Qerror[3] = 1.0;

	// Torquer Polarity Check

	TorquerPolaritySetFlag = 0;
	Roll_MTR_Pol_Reversal = 0;

	Pitch_MTR_Pol_Reversal = 0;

	Yaw_MTR_Pol_Reversal = 0;

	// Linear Controller

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Tc[i_pini] = 0.0;
		HB[i_pini] = 0.0;
	}

	for (i_pini=0; i_pini<4; i_pini++)
	{
		H_wh[i_pini] = 0.0;
		T_RW[i_pini] = 0.0;
	}

	CB_Wheel_OverSpeed_TorqueCutOff = 0;

	// Q propagation

	Del_R_theta = 0.0;
	Del_P_theta = 0.0;
	Del_Y_theta = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		Del_Q[i_pini] = 0.0;
		Qprop_prev[i_pini] = 0.0;
		q_prop_out[i_pini] = 0.0;
	}

	Del_Q[3] = 1.0;
	Qprop_prev[3] = 1.0;
	q_prop_out[3] = 1.0;

	// Extended Kalman Filter

	///Kalman Filter

	sigma_v = 7.56407e-5;///
	sigma_u = 4.1209e-10;///2.9688e-6;///7.56407e-10;///
	sigma_m = 1500.0e-13;
	sun_noise = 0.2*3.141592653589793/180.0;
	mag_noise = 100.0;

	Qk_temp1 = 0.0;
	Qk_temp2 = 0.0;
	Qk_temp3 = 0.0;
	Qk_temp4 = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			qk_DCM[i_pini][j_pini] = 0.0;
			ref_cross[i_pini][j_pini] = 0.0;

		}
	}

	qk_DCM[0][0] = 1.0;
	qk_DCM[1][1] = 1.0;
	qk_DCM[2][2] = 1.0;

	ref_cross[0][0] = 1.0;
	ref_cross[1][1] = 1.0;
	ref_cross[2][2] = 1.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 9; j_pini++)
		{
			Hk[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 9; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			pk_HkT[i_pini][j_pini] = 0.0;
			Hk_temp[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 9; i_pini++)
	{
		for (j_pini = 0; j_pini < 9; j_pini++)
		{
			pk_plus_one[i_pini][j_pini] = 0.0;
			phi_k_pk_plus[i_pini][j_pini] = 0.0;
			phi_k_pk_plus_phi_k_T[i_pini][j_pini] = 0.0;
			rk_Qk[i_pini][j_pini] = 0.0;
			rk_Qk_rk_T[i_pini][j_pini] = 0.0;
			Qk[i_pini][j_pini] = 0.0;
			phi_k_T[i_pini][j_pini] = 0.0;
			phi_k[i_pini][j_pini] = 0.0;
			pk_plus[i_pini][j_pini] = 0.0;
			Kk_Hk_with_minus_one[i_pini][j_pini] = 0.0;
			Kk_Hk[i_pini][j_pini] = 0.0;
			Kk_matrix[i_pini][j_pini] = 0.0;
			pk[i_pini][j_pini] = 0.0;
		}
	}

	pk[0][0] = 0.00761543549;
	pk[1][1] = 0.00761543549;
	pk[2][2] = 0.00761543549;
	pk[3][3] = 0.000000000611350;
	pk[4][4] = 0.000000000611350;
	pk[5][5] = 0.000000000611350;
	pk[6][6] = 0.00000000000025;
	pk[7][7] = 0.00000000000025;
	pk[8][8] = 0.00000000000025;


	Result_of_sine_Delta_cube = 0.0;
	Result_of_sine_Delta = 0.0;
	cos_of_Wk1_Delta_by_Wk1_square = 0.0;
	Q_quest_DAD_den = 0.0;
	mod_of_Wk1_square = 0.0;
	mod_of_Wk1_cube = 0.0;
	cos_of_Wk1_Delta = 0.0;
	sine_of_Wk1_Delta_by_Wk1 = 0.0;
	sine_of_Wk1_Delta = 0.0;
	mod_of_Wk1 = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			phi_12[i_pini][j_pini] = 0.0;
			phi_12_temp2[i_pini][j_pini] = 0.0;
			phi_12_temp1[i_pini][j_pini] = 0.0;
			phi_11[i_pini][j_pini] = 0.0;
			phi_11_temp4[i_pini][j_pini] = 0.0;
			phi_11_temp3[i_pini][j_pini] = 0.0;
			phi_11_temp2[i_pini][j_pini] = 0.0;
			phi_11_temp1[i_pini][j_pini] = 0.0;
			Wk1_one[i_pini][j_pini] = 0.0;
			K_temp[i_pini][j_pini] = 0.0;
			pk_HkT_Hk_Rk[i_pini][j_pini] = 0.0;
			pk_HkT_Hk[i_pini][j_pini] = 0.0;
			Rk[i_pini][j_pini] = 0.0;
			Wk1[i_pini][j_pini] = 0.0;
			phi_12_temp3[i_pini][j_pini] = 0.0;

		}
	}

	phi_12[0][0] = 1.0;
	phi_12[1][1] = 1.0;
	phi_12[2][2] = 1.0;

	phi_12_temp2[0][0] = 1.0;
	phi_12_temp2[1][1] = 1.0;
	phi_12_temp2[2][2] = 1.0;

	phi_12_temp1[0][0] = 1.0;
	phi_12_temp1[1][1] = 1.0;
	phi_12_temp1[2][2] = 1.0;

	phi_11[0][0] = 1.0;
	phi_11[1][1] = 1.0;
	phi_11[2][2] = 1.0;

	phi_11_temp4[0][0] = 1.0;
	phi_11_temp4[1][1] = 1.0;
	phi_11_temp4[2][2] = 1.0;

	phi_11_temp3[0][0] = 1.0;
	phi_11_temp3[1][1] = 1.0;
	phi_11_temp3[2][2] = 1.0;

	phi_11_temp2[0][0] = 1.0;
	phi_11_temp2[1][1] = 1.0;
	phi_11_temp2[2][2] = 1.0;

	phi_11_temp1[0][0] = 1.0;
	phi_11_temp1[1][1] = 1.0;
	phi_11_temp1[2][2] = 1.0;

	Wk1_one[0][0] = 1.0;
	Wk1_one[1][1] = 1.0;
	Wk1_one[2][2] = 1.0;

	K_temp[0][0] = 1.0;
	K_temp[1][1] = 1.0;
	K_temp[2][2] = 1.0;

	pk_HkT_Hk_Rk[0][0] = 1.0;
	pk_HkT_Hk_Rk[1][1] = 1.0;
	pk_HkT_Hk_Rk[2][2] = 1.0;

	pk_HkT_Hk[0][0] = 1.0;
	pk_HkT_Hk[1][1] = 1.0;
	pk_HkT_Hk[2][2] = 1.0;

	Rk[0][0] = 1.0;
	Rk[1][1] = 1.0;
	Rk[2][2] = 1.0;

	Wk1[0][0] = 1.0;
	Wk1[1][1] = 1.0;
	Wk1[2][2] = 1.0;

	phi_12_temp3[0][0] = 1.0;
	phi_12_temp3[1][1] = 1.0;
	phi_12_temp3[2][2] = 1.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		qk_plus[i_pini] = 0.0;
		qk_plus_temp[i_pini] = 0.0;
		Yk[i_pini] = 0.0;
		Xk_plus_temp[i_pini] = 0.0;
		Yk_minus_hk[i_pini] = 0.0;
		hk[i_pini] = 0.0;
		qk_minus[i_pini] = 0.0;
		hk_xk_minus[i_pini] = 0.0;
	}

	qk_plus[3] = 1.0;
	qk_plus_temp[3] = 1.0;

	for (i_pini=0; i_pini<9; i_pini++)
	{
		Xk[i_pini] = 0.0;
		Xk_plus[i_pini] = 0.0;
		Yk_minus_hk_Kk[i_pini] = 0.0;
	}

	for (i_pini = 0; i_pini < 4; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			q_by_two[i_pini][j_pini] = 0.0;

		}
	}

	qk_minus[3] = 1.0;

	// GPS DataProcessing

	///rOrbitalElements_generation_GPS

	GPS_Select = 1;
	f_GPS_Valid_Data = 0;
	i_jday = 0.0;

	for (i_pini=0; i_pini<13; i_pini++)
	{
		lmonth[i_pini] = 0.0;
	}

	year_GPS = 0.0;
	UTC_mon_GPS = 0.0;
	tempdays = 0.0;
	Numofdays = 0.0;
	UTC_day_GPS = 0.0;
	UTC_hr_GPS = 0.0;
	UTC_min_GPS = 0.0;

	epochdays_GPS = 0.0;
	UTC_sec_GPS = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Pos_ECEF_GPS[i_pini] = 0.0;
		Vel_ECEF_GPS[i_pini] = 0.0;
		Vel_ECI_temp[i_pini] = 0.0;
		Vel_ECI_GPS[i_pini] = 0.0;
		Pos_ECI_GPS[i_pini] = 0.0;
		rpef[i_pini] = 0.0;
		vpef[i_pini] = 0.0;
		wcrecef[i_pini] = 0.0;
		e_vec[i_pini] = 0.0;
		angmomentumvec[i_pini] = 0.0;
	}

	Epochyear_GPS = 0.0;


	radialdistance = 0.0;
	radialdistance_ecef = 0.0;
	velmag = 0.0;
	r_delta = 0.0;
	velmagsq = 0.0;
	rdtv = 0.0;
	ecc_temp1 = 0.0;
	ecc_temp2 = 0.0;
	ecc_temp3 = 0.0;
	ecc_temp4 = 0.0;
	ecc_temp5 = 0.0;
	ecc_temp6 = 0.0;
	ecc_temp7 = 0.0;
	ecc_GPS = 0.0;
	semi_den = 0.0;
	semimajoraxis_GPS = 0.0;
	Alti_GPS = 0.0;
	angmomentumvecmag = 0.0;
	delta_hmag = 0.0;
	invangmomentumvecmag = 0.0;
	inclination_GPS = 0.0;
	sinlongacnode = 0.0;
	coslongacnode = 0.0;
	nodeo_GPS = 0.0;
	e_vecdtr = 0.0;
	tempta = 0.0;
	trueanomoly_GPS = 0.0;
	Ndte_vec = 0.0;
	N_ecc = 0.0;
	temparg = 0.0;
	argpo_GPS = 0.0;
	omecc = 0.0;
	sine = 0.0;
	cose = 0.0;
	eccanomaly_GPS = 0.0;
	mo_GPS = 0.0;
	Orbit_Period = 0.0;
	no_GPS = 0.0;
	longitude_GPS = 0.0;
	latitude_GPS = 0.0;

	epochyr = 0.0;
	dayofyr = 0.0;
	inttemp = 0.0;

	///Orbit Initialization and Propagation
	Tsince_GPS = 0.0;
	epochdays_sel = 0.0;
	inclination_sel = 0.0;
	nodeo_sel = 0.0;
	trueanomoly_sel = 0.0;
	mo_sel = 0.0;
	argpo_sel = 0.0;
	ecc_sel = 0.0;
	no_sel = 0.0;
	ibexp_sel = 0.0;
	bstar_sel = 0.0;
	sec_sel = 0.0;
	year_sel = 0.0;
	mon_sel = 0.0;
	days_sel = 0.0;
	hr_sel = 0.0;
	minute_sel = 0.0;
	epochyr_sel = 0.0;
	GPS_Elements_Available = 0;
	TLE_Data_Available = 0;
	rtom = 0.0;
	ao_sfour = 0.0;
	pow_psisq = 0.0;
	psetasq = 0.0;
	intermediate = 0.0;
	am_den = 0.0;
	temp_temp = 0.0;
	temp1 = 0.0;
	for (i_pini=0; i_pini<3; i_pini++)
	{
		wcreci[i_pini] = 0.0;
	}
	Tsince = 0.0;
	Tsince_TLE = 0.0;
	Tsince_TLE_tc = 0.0;
	Day_Of_Year_DeltaT = 0.0;
	Delta_T = 0.0;//////////////////////////////////////
	Orbit_Period_Comp = 0.0;
	wo = 0.0;
	inclo = 0.0;
	nodeo = 0.0;
	argpo = 0.0;
	mo = 0.0;
	ecco = 0.0;
	no = 0.0;
	ibexp = 0.0;
	bstar = 0.0;

	con41 = 0.0;
	cc1 = 0.0;
	cc4 = 0.0;
	cc5 = 0.0;
	d2 = 0.0;
	d3 = 0.0;
	d4 = 0.0;
	delmo = 0.0;
	eta = 0.0;
	argpdot = 0.0;
	omgcof = 0.0;
	sinmao = 0.0;
	aycof = 0.0;
	t2cof = 0.0;
	t3cof = 0.0;

	t4cof = 0.0;
	t5cof = 0.0;
	x7thm1 = 0.0;
	mdot = 0.0;
	nodedot = 0.0;
	xmcof = 0.0;
	nodecf = 0.0;
	cc3 = 0.0;
	var = 0.0;

	uuu = 0.0;
	temp_prop = 0.0;
	var1 = 0.0;
	eccsq = 0.0;
	omeosq = 0.0;
	rteosq = 0.0;
	cosio = 0.0;
	cosio2 = 0.0;

	ak = 0.0;
	d1 = 0.0;
	del = 0.0;
	adel = 0.0;
	ao = 0.0;
	sinio = 0.0;
	po = 0.0;
	con42 = 0.0;
	posq = 0.0;
	rp = 0.0;

	sfour = 0.0;
	qzms24 = 0.0;

	pinvsq = 0.0;
	tsi = 0.0;
	etasq = 0.0;
	eeta = 0.0;
	psisq = 0.0;
	coef = 0.0;
	cc2 = 0.0;
	coef1 = 0.0;
	qzms24_temp = 0.0;

	cosio4 = 0.0;
	temp1_or_init = 0.0;
	temp2 = 0.0;
	temp3 = 0.0;
	temp_or_init = 0.0;
	temp_om = 0.0;

	xhdot1 = 0.0;
	xlcof = 0.0;

	cc1sq = 0.0;
	ak_temp = 0.0;
	ao_temp = 0.0;
	delmo_temp = 0.0;

	x1mth2 = 0.0;
	perigee = 0.0;

	xmdf = 0.0;
	argpdf = 0.0;
	nodedf = 0.0;
	mm = 0.0;
	t2 = 0.0;
	tempa = 0.0;
	tempe = 0.0;
	delomg = 0.0;
	delm = 0.0;
	t3 = 0.0;
	t4 = 0.0;
	nm = 0.0;
	em = 0.0;
	inclm = 0.0;
	ecose = 0.0;
	am = 0.0;
	argpm = 0.0;
	nodem = 0.0;
	xlm = 0.0;
	sinim = 0.0;
	cosim = 0.0;
	ep = 0.0;
	xl = 0.0;
	sinip = 0.0;
	cosip = 0.0;
	xincp = 0.0;
	argpp = 0.0;
	nodep = 0.0;
	mp = 0.0;
	axnl = 0.0;
	aynl = 0.0;
	esine = 0.0;
	r1 = 0.0;
	betal = 0.0;
	sinu = 0.0;
	cosu = 0.0;
	suu = 0.0;
	sin2u = 0.0;
	cos2u = 0.0;
	mrt = 0.0;
	xnode = 0.0;
	xinc = 0.0;
	mvt = 0.0;
	rvdot = 0.0;
	emsq = 0.0;
	sinsu = 0.0;
	cossu = 0.0;
	snod = 0.0;
	cnod = 0.0;
	sini = 0.0;
	cosi = 0.0;
	xmx = 0.0;
	xmy = 0.0;
	ux = 0.0;
	uy = 0.0;
	uz = 0.0;
	vx = 0.0;
	vy = 0.0;
	vz  = 0.0;
	xsatx_eci = 0.0;
	xsaty_eci = 0.0;
	xsatz_eci = 0.0;
	vsatx_eci = 0.0;
	vsaty_eci = 0.0;
	vsatz_eci = 0.0;
	templ = 0.0;
	am_temp = 0.0;
	nm_temp = 0.0;
	mm_temp = 0.0;
	u_temp = 0.0;
	el2 = 0.0;
	pl = 0.0;
	rl = 0.0;
	rdotl = 0.0;
	rvdotl = 0.0;
	sinkp = 0.0;
	coskp = 0.0;
	eol = 0.0;
	etempe = 0.0;
	del_temp = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Pos_ECI[i_pini] = 0.0;
		Vel_ECI[i_pini] = 0.0;
		Pos_ECEF[i_pini] = 0.0;
		Vel_ECEF_temp[i_pini] = 0.0;
		Vel_ECEF[i_pini] = 0.0;
		Pos_ECIn[i_pini] = 0.0;
		Vel_ECIn[i_pini] = 0.0;
		Pos_ECEFn[i_pini] = 0.0;
		Vel_ECEFn[i_pini] = 0.0;
	}


	///Orbital elements computation
	semimajoraxis = 0.0;
	ecc = 0.0;
	Alti = 0.0;
	inclination_temp = 0.0;
	inclination = 0.0;
	RAAN = 0.0;
	trueanomoly = 0.0;
	argofperigee = 0.0;
	eccanomaly = 0.0;
	ecc_r = 0.0;
	longitude = 0.0;
	latitude_temp = 0.0;
	latitude = 0.0;
	longitude_tan_num = 0.0;
	longitude_tan_den = 0.0;


	///Ecef to ECI to ecef
	UT1 = 0.0;
	UTC = 0.0;
	TAI = 0.0;
	JDTDT = 0.0;
	M_quad = 0.0;
	sine1 = 0.0;
	sine2 = 0.0;
	TDB = 0.0;
	TDT = 0.0;
	TTDB = 0.0;
	JDTDB = 0.0;
	TTDB2 = 0.0;
	TTDB3 = 0.0;
	zeta = 0.0;
	z = 0.0;
	theta = 0.0;
	eps = 0.0;
	dpsi = 0.0;
	xin_temp = 0.0;
	tut1 = 0.0;
	gmst0 = 0.0;
	gmst_ = 0.0;
	gast = 0.0;
	ang = 0.0;
	TTDT = 0.0;
	deps = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			pre_temp[i_pini][j_pini] = 0.0;
			precession[i_pini][j_pini] = 0.0;
			nut_temp[i_pini][j_pini] = 0.0;
			nutation[i_pini][j_pini] = 0.0;
			sidereal[i_pini][j_pini] = 0.0;
			polarmotion[i_pini][j_pini] = 0.0;
			nut_sid_temp[i_pini][j_pini] = 0.0;
			ECEFtoECI[i_pini][j_pini] = 0.0;
			ECItoECEF[i_pini][j_pini] = 0.0;

			///NED to ECEF
			NEDtoECEF[i_pini][j_pini] = 0.0;

		}
	}

	GPSDataReady_NA_count = 0.0;
	Present_OBT = 0.0;
	OBT_at_TLE_epoch = 0.0;
	Delta_TLE = 0.0;

	///Julian Day
	jd_time = 0.0;
	tut = 0.0;
	pps_deltaT = 0.0;
	Julian_day = 0.0;

	///ast_args

	for (i_pini=0; i_pini<4; i_pini++)
	{
		tt[i_pini] = 0.0;
	}

	for (i_pini=0; i_pini<5; i_pini++)
	{
		f[i_pini] = 0.0;
	}

	///main
	minorcyclecount = 0.0;
	majorcyclecount = 0.0;
	k = 0.0;
	i_pini = 0.0;
	ktr = 0.0;
	TLE_Select = 0;

	/// IGRF model

	temp_magad = 0.0;
	BNorth = 0.0;
	BEast = 0.0;
	BNorth_old = 0.0;
	BDown = 0.0;
	rot_ang = 0.0;
	A1_magad = 0.0;
	A2_magad = 0.0;
	A3_magad = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		B_NED[i_pini] = 0.0;
		B_ECI[i_pini] = 0.0;
		B_ECIn[i_pini] = 0.0;
		B_ECItesla[i_pini] = 0.0;
		B_ECEF[i_pini] = 0.0;
	}

	co_dec = 0.0;
	fN_MAGAD = 0.0;
	gN_MAGAD = 0.0;
	gmm = 0.0;
	st_magad = 0.0;
	ct_magad = 0.0;
	for (i_pini=0; i_pini<14; i_pini++)
	{
		cl_magad[i_pini] = 0.0;
		sl_magad[i_pini] = 0.0;
	}

	one_magad = 0.0;
	two_magad = 0.0;
	sd_magad = 0.0;
	for (i_pini=0; i_pini<106;i_pini++)
	{
		p_magad[i_pini] = 0.0;
		q_magad[i_pini] = 0.0;
	}

	I_MFC = 0.0;
	K_MAGAD = 0.0;
	I_MAGAD = 0.0;
	J_MAGAD = 0.0;
	M_MAGAD = 0.0;
	N_MAGAD = 0.0;
	Cond1 = 0.0;
	A_R  = 0.0;
	Alti_Mod  = 0.0;
	Rho  = 0.0;
	B_Rho  = 0.0;
	cd_magad  = 0.0;
	old_cos  = 0.0;
	s_magad = 0.0;
	m_gh  = 0.0;
	n_gh  = 0.0;
	i_gh  = 0.0;
	j_gh = 0.0;

	for (i_pini = 0; i_pini < 15; i_pini++)
	{
		for (j_pini = 0; j_pini < 16; j_pini++)
		{
			g[i_pini][j_pini] = 0.0;
			h[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			REF_FRAME_DCM[i_pini][j_pini] = 0.0;
		}
	}

	DeltaT_MFC = 0.0;
	DeltaT_Updated = 0.0;

	///Sun Model

	for (i_pini=0; i_pini<3; i_pini++)
	{
		S_ECI[i_pini] = 0.0;
		S_ECIn[i_pini] = 0.0;
	}

	L_Msun = 0.0;
	Msun = 0.0;
	L_Ecliptic = 0.0;
	Sun_Dis = 0.0;
	Epsilon = 0.0;

	/// Ref quaternions
	SUN_ECI_mag = 0.0;
	X_SVO2ECI_mag  = 0.0;
	Z_SVO2ECI_mag  = 0.0;
	Y_EPO2ECI_mag  = 0.0;
	X_EPO2ECI_mag  = 0.0;
	Z_SFAO2ECI_mag  = 0.0;
	X_SFAO2ECI_mag = 0.0;
	X_SFDO2ECI_mag  = 0.0;
	Z_SFDO2ECI_mag = 0.0;
	rdotrst  = 0.0;
	SAT_ANGLE_STAT = 0.0;
	X_SPO2ECI_mag  = 0.0;
	Y_SPO2ECI_mag = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Y_SVO2ECI[i_pini] = 0.0;
		Z_SVO2ECI[i_pini] = 0.0;
		X_SVO2ECI[i_pini] = 0.0;
		STATION_ECEF[i_pini] = 0.0;
		STATION_ECI[i_pini] = 0.0;
		STATION_vector[i_pini] = 0.0;
		STATION_ECIn[i_pini] = 0.0;
		X_SPO2ECI[i_pini] = 0.0;
		Y_SPO2ECI[i_pini] = 0.0;
		Z_SPO2ECI[i_pini] = 0.0;
		Pos_ECIn[i_pini] = 0.0;
		Z_EPO2ECI[i_pini] = 0.0;
		Y_EPO2ECI[i_pini] = 0.0;
		X_EPO2ECI[i_pini] = 0.0;
		Y_SFAO2ECI[i_pini] = 0.0;
		Z_SFAO2ECI[i_pini] = 0.0;
		X_SFAO2ECI[i_pini] = 0.0;
		Y_SFDO2ECI[i_pini] = 0.0;
		Z_SFDO2ECI[i_pini] = 0.0;
		X_SFDO2ECI[i_pini] = 0.0;
		B_REF[i_pini] = 0.0;
		S_REF[i_pini] = 0.0;
		B_REFn[i_pini] = 0.0;
		S_REFn[i_pini] = 0.0;
		Q_SVO2ECI[i_pini] = 0.0;
		Q_SPO2ECI[i_pini] = 0.0;
		Q_EPO2ECI[i_pini] = 0.0;
		Q_SFAO2ECI[i_pini] = 0.0;
		Q_SFDO2ECI[i_pini] = 0.0;
		Q_REF[i_pini] = 0.0;
		Q_StP2ECI[i_pini] = 0.0;
	}


	Q_SVO2ECI[3] = 1.0;
	Q_SPO2ECI[3] = 1.0;
	Q_EPO2ECI[3] = 1.0;
	Q_SFAO2ECI[3] = 1.0;
	Q_SFDO2ECI[3] = 1.0;
	///Ref Vector Generation
	Q_REF[3]  = 1.0;
	Q_StP2ECI[3]  = 1.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			R_SVO2ECI[i_pini][j_pini] = 0.0;
			R_SPO2ECI[i_pini][j_pini] = 0.0;
			R_EPO2ECI[i_pini][j_pini] = 0.0;
			R_SFAO2ECI[i_pini][j_pini] = 0.0;
			R_SFDO2ECI[i_pini][j_pini] = 0.0;
			R_StP2ECI[i_pini][j_pini] = 0.0;

		}
	}

	/// Ref Gyro

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		w_REF[i_pini] = 0.0;
		Q_REF_pres[i_pini] = 0.0;
		Q_REF_prev[i_pini] = 0.0;
		Q_REF_prev_conj[i_pini] = 0.0;
		Q_REF_diff[i_pini] = 0.0;
		Q_axis[i_pini] = 0.0;
	}

	Q_REF_pres[3]  = 1.0;
	Q_REF_prev[3]  = 1.0;
	Q_REF_prev_conj[3] = 1.0;
	Q_REF_diff[3]  = 1.0;
	QRD_vect_norm = 0.0;
	Q_angle  = 0.0;

	/// Onboard Eclipse Algorithm

	for (i_pini=0; i_pini<3; i_pini++)
	{
		rsun[i_pini] = 0.0;
		rsat[i_pini] = 0.0;
	}

	theta1_se  = 0.0;
	theta2_se = 0.0;
	psi_sl_ecl = 0.0;

	magrsun  = 0.0;
	//tempse  = 0.0;
	//bsqrd  = 0.0;
	//asqrd  = 0.0;
	//adotb  = 0.0;
	//distsqrd  = 0.0;
	//tmin = 0.0;

	/// Quest Data Processing

	sc_qst = 0.0;
	dc_qst = 0.0;
	wc_qst = 0.0;
	mat_mag_DataCounter  = 0.0;
	mat_sm_DataCounter = 0.0;

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 24; j_pini++)
		{
			NMB_mag[i_pini][j_pini] = 0.0;
			NRB_mag[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 8; j_pini++)
		{
			NRB_sunmag[i_pini][j_pini] = 0.0;
			NMB_sunmag[i_pini][j_pini] = 0.0;
			NMS[i_pini][j_pini] = 0.0;
			NRS[i_pini][j_pini] = 0.0;

		}
	}

	f_DataSort_MAG = 0.0;
	f_DataSort_SUNMAG = 0.0;
	i_QDP = 0.0;
	CB_Q_propagation  = 0.0;

	///DAD Quest
	CB_DAD_quest = 0.0;

	for (i_pini = 0; i_pini < 24; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			NRBt_mag[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 8; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			NRSt[i_pini][j_pini] = 0.0;
			NRBt_sunmag[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			B_DAD[i_pini][j_pini] = 0.0;
			Bt_DAD[i_pini][j_pini] = 0.0;
			S_DAD[i_pini][j_pini] = 0.0;
			St_DAD[i_pini][j_pini] = 0.0;
			S_sqr_DAD[i_pini][j_pini] = 0.0;
			NMB_NRBt[i_pini][j_pini] = 0.0;
			NMS_NRSt[i_pini][j_pini] = 0.0;
			alpha_I_DAD[i_pini][j_pini] = 0.0;
			beta_S_DAD[i_pini][j_pini] = 0.0;
			sumX_DAD[i_pini][j_pini] = 0.0;
			W_cross[i_pini][j_pini] = 0.0;
			adj_S_DAD[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Z_DAD[i_pini] = 0.0;
		Zt_DAD[i_pini] = 0.0;
		Q_quest_DAD[i_pini] = 0.0;
		X_DAD[i_pini] = 0.0;
		Qquest_update[i_pini] = 0.0;
		rsat[i_pini] = 0.0;
	}


	wkm  = 0.0;
	wks = 0.0;
	sigma_DAD  = 0.0;
	delta_DAD  = 0.0;
	a_DAD  = 0.0;
	b_DAD = 0.0;
	c_DAD = 0.0;
	d_DAD  = 0.0;
	k_DAD = 0.0;

	root_DAD  = 0.0;
	func_DAD  = 0.0;
	Dfunc_DAD  = 0.0;
	func_prev_DAD = 0.0;
	alpha_DAD  = 0.0;
	beta_DAD  = 0.0;
	gamma_DAD = 0.0;

	Q_quest_DAD[3] = 1.0;

	magRoll_angle  = 0.0;
	magPitch_angle  = 0.0;
	magYaw_angle = 0.0;

	wAD_updatecount = 0;
	Qquest_update[3] = 1.0;

	I_DAD  = 0.0;
	J_DAD  = 0.0;
	K_DAD = 0.0;
	Zt_Z_DAD  = 0.0;
	Zt_S_Z_DAD  = 0.0;
	Zt_Ssqr_Z_DAD = 0.0;

	for (i_pini = 0; i_pini < 4; i_pini++)
	{
		for (j_pini = 0; j_pini < 4; j_pini++)
		{
			big_omega[i_pini][j_pini] = 0.0;
			sin_of_W_norm_Delta_by2_W_norm_and_big_omega[i_pini][j_pini] = 0.0;
			cos_and_sin[i_pini][j_pini] = 0.0;
			cos_of_W_norm_Delta_by2_with_I[i_pini][j_pini] = 0.0;

		}
	}

	Delta_by2 = 0.0;
	W_norm = 0.0;
	Delta = 0.0;
	cos_of_W_norm_and_Delta_by2 = 0.0;
	sin_of_W_norm_and_Delta_by2 = 0.0;
	sin_of_W_norm_Delta_by2_and_W_norm = 0.0;

	trace_S_DAD  = 0.0;

	///Dutycycle Generation

	for (i_pini=0; i_pini<3; i_pini++)
	{
		DPM_Polarity[i_pini] = 0;
		DPM_Pol_prev[i_pini] = 0;
		Ton[i_pini] = 0.0;
		Ton[i_pini] = 0.0;
		TorquerDutyCycle[i_pini] = 0.0;
	}

	//DutyCycleGenEnable = 0;
	//Torquer_Actuate = 0;
	//TorquerPolaritySetFlag = 0;
	MR = 0.0;
	MP = 0.0;
	MY = 0.0;
	Roll_MTR_Pol_Reversal = 0.0;
	Pitch_MTR_Pol_Reversal = 0.0;
	Yaw_MTR_Pol_Reversal = 0.0;
	MTR_ActuationCycle = 8;

	ActuationCycle = 0.0;

	///Angular Momentum Dumping

	f_Momentum_Dumping = 0;
	dumping_on = 0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		delta_HB[i_pini] = 0.0;
		DPM[i_pini] = 0.0;
	}

	///Speed based Momentum Dumping

	for (i_pini=0; i_pini<4; i_pini++)
	{
		check_dump_wh[i_pini] = 0.0;
		T_RW_sdump[i_pini] = 0.0;
		wh_sdump_start[i_pini] = 0.0;
		T_RW_sdump[i_pini] = 0.0;
	}

	H_retn = 0.0;
	speed_based_torquer_control = 0.0;

	for (i_pini=0; i_pini<3; i_pini++)
	{
		TB_sMD[i_pini] = 0.0;
		u_parl[i_pini] = 0.0;
		T_RWB[i_pini] = 0.0;
		T_RWBn[i_pini] = 0.0;
		u_perp[i_pini] = 0.0;
	}

	tau_ms = 0.0;
	min_TW = 0.0;

	///Linear Controller

	for (i_pini=0; i_pini<4; i_pini++)
	{
		RWSpeed[i_pini] = 0.0;
		v0_rad_sec[i_pini] = 0.0;
	}

	DFriction_Compensation = 0.0;

	for (i_pini = 0; i_pini < 4; i_pini++)
	{
		for (j_pini = 0; j_pini < 3; j_pini++)
		{
			Pse_Inv_Dist_Mat[i_pini][j_pini] = 0.0;
			B2wh_mat[i_pini][j_pini] = 0.0;

		}
	}

	for (i_pini = 0; i_pini < 3; i_pini++)
	{
		for (j_pini = 0; j_pini < 4; j_pini++)
		{
			wh2B_mat[i_pini][j_pini] = 0.0;

		}
	}

	///wheel dynamic friction

	for (i_pini=0; i_pini<4; i_pini++)
	{
		TachoHyst[i_pini] = 0.0;
		DFCCount[i_pini] = 0.0;
		FirstSpeed[i_pini] = 0.0;
		ExWhMom[i_pini] = 0.0;
		DFCGainSL[i_pini] = 0.0;
		LPFK1SL[i_pini] = 0.0;
		LPFK2SL[i_pini] = 0.0;
		WhMom0[i_pini] = 0.0;
		ActWhMom[i_pini] = 0.0;
		LossWhMom[i_pini] = 0.0;
		LossWhMomFilt[i_pini] = 0.0;
		DFCTorq[i_pini] = 0.0;
		DFCCountLimSL[i_pini] = 0.0;
	}

	k = 0.0;
	speedDFCch = 0.0;
	DFCGainHigh = 0.0;
	LPFK1HighSL = 0.0;
	LPFK2HighSL = 0.0;
	DFCGainLow = 0.0;
	LPFK1LowSL = 0.0;
	LPFK2LowSL = 0.0;
	DFCcountHigh = 0.0;
	DFCcountLow = 0.0;


	///wheel spin updown
	wheel_spin_logic = 0.0;
	spin_up_avg_count = 0.0;
	spin_up_avg_count_2 = 0.0;

	for (i_pini=0; i_pini<4; i_pini++)
	{
		del_v0[i_pini] = 0.0;
		T_RW_spin[i_pini] = 0.0;
	}

	del_v0a = 0.0;

	///wheel auto reconfiguration

	for (i_pini = 0; i_pini < 4; i_pini++)
	{
		wheel_index[i_pini] = 0.0;
		pres_exp_whsp_ch[i_pini] = 0.0;
		exp_whsp_ch[i_pini] = 0.0;
		ch_obs_whsp[i_pini] = 0.0;
		prev_obs_whsp_ch[i_pini] = 0.0;
		diff_obs_exp_ch[i_pini] = 0.0;
	}

	wheel_index_ARCsum = 0.0;
	Wheel_Config = 0.0;
	RW_ARC_Logic = 0.0;
	RW_ARC_Count = 0.0;
	count_arc_w0 = 0.0;
	count_arc_w1 = 0.0;
	count_arc_w2 = 0.0;
	count_arc_w3 = 0.0;

	TC_GYRO_Det_Min_Thresh = 0.2;
	GYRO_Threshold_Count = 0.0;

	///Detumbling_ModePreprocessing

	GYRO_max_threshold_count = 0.0;

	BDOT_Threshold_Count = 0.0;
	TorquerPolaritySetFlag = 0.0;

	///rDetumbling_ModePreprocessing_GYRO_Logic

	for (i_pini=0; i_pini<3; i_pini++)
	{
		Tdet[i_pini] = 0.0;
		gyrodet_w[i_pini] = 0.0;
		Bpresent[i_pini] = 0.0;
		gyrodet_w[i_pini] = 0.0;
		gyrodet_B[i_pini] = 0.0;
	}

	GYRO_Threshold_Count = 0.0;

	///////////////BDOT Computation

	///GYRO ext Computation

	GYRO_Counter = 0.0;

	///Rate reduction routine
	BDOTnorm = 0.0;

	///SunAcquisition_ModePreprocessing

	SunAcquisition2ThreeAxis_autotransit = 0;
	SunAcq_Ang_Thres = 0.01;
	SunAcq3ThreeAx_trsit_cnt = 0;
	SunAcq2ThreeAx_trsit_cnt_thres = 28125;//14063;//2344;

	SunAcq2DetMode_counter = 0;

	ThreeAxis2DetMode_counter = 0;
	f_threeaxis2safe = 0;
	f_battery_safemode = 0;

	/// Timer based sunlit/eclipse

	entrytime2eclipse = 900.0;
	orbit_time = 5688.0;
	TC_Hmin = 0.001;

	// Remote data

	ADCS_TC_data_command_Table.TC_wAD_BODYminThRoll = -0.1;
	ADCS_TC_data_command_Table.TC_wAD_BODYminThPitch = -0.2;
	ADCS_TC_data_command_Table.TC_wAD_BODYminThYaw = -0.1;
	ADCS_TC_data_command_Table.TC_wAD_updateTimeThresh = 6000;
	ADCS_TC_data_command_Table.TC_wp_QDP = 60;

	// Boolean//

	// Telecommands INIT

	TC_boolean_u.TC_Boolean_Table.TC_Sun_Ephemeris_en_dis = Enable;
	TC_boolean_u.TC_Boolean_Table.TC_Mag_Refeci_en_dis = Enable;
	TC_boolean_u.TC_Boolean_Table.TC_GPS_TLE_Select = 1;
	TC_boolean_u.TC_Boolean_Table.TC_SS_Cells_Sel = TC_Main_Cells;
	TC_boolean_u.TC_Boolean_Table.TC_Sun_Varying_Mode = Enable;
	TC_boolean_u.TC_Boolean_Table.TC_SunAq2DetMode_autotransit = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_ThreeAxis2SafeMode_autotransit = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_QuestUpdate_Enable = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_mom_dumping_ang_mom_based = Enable;

	TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable = Disable;
	TC_boolean_u.TC_Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable = Enable;
	TC_boolean_u.TC_Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable = Enable;
	TC_boolean_u.TC_Boolean_Table.TC_BIST_override = Disable;

	ADCS_TC_data_command_Table.TC_PanelD_Status_Sel = TC_All_Deployed;
	ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh = 40.0;
	ADCS_TC_data_command_Table.TC_eclipse_entrytime = 900.0;
	ADCS_TC_data_command_Table.TC_Det_BDOT_Compute_Count = 5;
	ADCS_TC_data_command_Table.TC_Det_Bprev_Count = 2;
	ADCS_TC_data_command_Table.TC_Det_GYRO_Compute_Count = 3;

	ADCS_TC_data_command_Table.TC_wAD_BODYmaxThRoll = 0.1;
	ADCS_TC_data_command_Table.TC_wAD_BODYmaxThPitch = 0.001;
	ADCS_TC_data_command_Table.TC_wAD_BODYmaxThYaw = 0.1;
	ADCS_TC_data_command_Table.TC_magMin_angle = 45.0;
	ADCS_TC_data_command_Table.TC_magMax_angle = 135.0;
	ADCS_TC_data_command_Table.TC_ECEF_stationlatitude = 0.303425;
	ADCS_TC_data_command_Table.TC_ECEF_stationLongitude = 1.300038;
	TC_boolean_u.TC_Boolean_Table.TC_Station_Tracking_Mode = 1;

	//TC_Bdot_Gain = 1.0;

	TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Enable_or_Disable = 1;
	TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Enable_or_Disable = 1;
	TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Enable_or_Disable = 1;


	//power on init
	TC_detumbling_rate_gain[0]=GAIN_DATA_SET.TC_detumbling_rate_gain_0_00;
	TC_detumbling_rate_gain[1]=GAIN_DATA_SET.TC_detumbling_rate_gain_1_00;
	TC_detumbling_rate_gain[2]=GAIN_DATA_SET.TC_detumbling_rate_gain_2_00;

	TC_KP[0]=GAIN_DATA_SET.TC_KP_0_00;
	TC_KP[1]=GAIN_DATA_SET.TC_KP_1_00;
	TC_KP[2]=GAIN_DATA_SET.TC_KP_2_00;

	TC_KR[0]=GAIN_DATA_SET.TC_KR_0_00;
	TC_KR[1]=GAIN_DATA_SET.TC_KR_1_00;
	TC_KR[2]=GAIN_DATA_SET.TC_KR_2_00;

	TC_detumbling_bdot_gain[0]=GAIN_DATA_SET.TC_detumbling_bdot_gain_0_00;
	TC_detumbling_bdot_gain[1]=GAIN_DATA_SET.TC_detumbling_bdot_gain_1_00;
	TC_detumbling_bdot_gain[2]=GAIN_DATA_SET.TC_detumbling_bdot_gain_2_00;

	IMU1_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_0_00;
	IMU1_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU1_1_00;

	IMU1_Corr.DB_MagLPF[0] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_0_00;
	IMU1_Corr.DB_MagLPF[1] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU1_1_00;

	IMU2_Corr.DB_GyroLPF[0] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_0_00;
	IMU2_Corr.DB_GyroLPF[1] =GAIN_DATA_SET.TC_Gyro_LPF_Gain_IMU2_1_00;

	IMU2_Corr.DB_MagLPF[0] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_0_00;
	IMU2_Corr.DB_MagLPF[1] =GAIN_DATA_SET.TC_Mag_LPF_Gain_IMU2_1_00;

	TC_comd_pitch_rate=GAIN_DATA_SET.TC_comd_pitch_rate_0_00;

	TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;

	TC_SpeedDumpLimit=GAIN_DATA_SET.TC_SpeedDump_TimeSelect_0_00;

	TC_AngMomDump_Thrsld=GAIN_DATA_SET.TC_AngMomDump_Thrsld_0_00;

	TC_GYRO_Det_Min_Thresh=GAIN_DATA_SET.TC_GYRO_Det_Min_Thres_0_00;

	TC_BDOT_Det_Thresh=GAIN_DATA_SET.TC_BDOT_Det_Thresh_0_00;

	TC_wh_speed_thres=GAIN_DATA_SET.TC_Wheel_Cutoff_Threshold_0_00;

	TC_momentum_dumping_gain = GAIN_DATA_SET.TC_momentum_dumping_gain_0_00;

	TC_GPS_pulse_duration=GAIN_DATA_SET.TC_GPS_pulse_duration_0_00;

	TC_RW_Nominal[0] = GAIN_DATA_SET.Tc_nominal_speed_rw1_01;
	TC_RW_Nominal[1] = GAIN_DATA_SET.Tc_nominal_speed_rw2_01;
	TC_RW_Nominal[2] = GAIN_DATA_SET.Tc_nominal_speed_rw3_01;
	TC_RW_Nominal[3] = GAIN_DATA_SET.Tc_nominal_speed_rw4_01;

	//////////////////////////////////////////////////////////////   TEST

	hils_mode_select = 1;


}
