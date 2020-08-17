#include <stdio.h>
#include <math.h>

#include "adcs_VarDeclarations.h"

#include "HAL_Global.h"
#include "Global.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"



void rScModeSelection(void)
{
	switch (Spacecraft_Mode)
	{
			case Suspended_ModePreprocessing:
				rSuspended_ModePreprocessing();
			break;

			case Detumbling_ModePreprocessing_BDOT:
				rDetumbling_ModePreprocessing_BDOT_Logic();
			break;

			case Detumbling_ModePreprocessing_GYRO:
				rDetumbling_ModePreprocessing_GYRO_Logic();
			break;

			case SunAcquisition_ModePreprocessing:
				rSunAcquisition_ModePreprocessing();
			break;

			case ThreeAxis_ModePreprocessing:
				rThreeAxis_ModePreprocessing();
			break;

			case Safe_ModePreprocessing:
				rSafeMode_Preprocessing();
			break;
			default: //Suspended_ModePreprocessing
				rSuspended_ModePreprocessing();
			break;
	}
}

/*
unsigned short Check_sum_data =0;
void rHILS_packets()
{
	int i;
	unsigned short temp_hils;
	HILS_packet.header          = 0x07e0;
	HILS_packet.len             = 0x40;
	HILS_packet.aux             = 0x01;
	HILS_packet.mode_flag 		= (char)Spacecraft_Mode; //HILS
	HILS_packet.mag_field[0] 	= (int)(B_BODY[0]/c_TM_Resol_B);
	HILS_packet.mag_field[1] 	= (int)(B_BODY[1]/c_TM_Resol_B);
	HILS_packet.mag_field[2] 	= (int)(B_BODY[2]/c_TM_Resol_B);
	temp_hils                	= input_latch_1.data;
	HILS_packet.polarity     	= (char)((temp_hils & 0xFC00)>>8);
	HILS_packet.rw_torque[0] 	= (int)(T_RW[0]/c_TM_RW_Resol);
	HILS_packet.rw_torque[1] 	= (int)(T_RW[1]/c_TM_RW_Resol);
	HILS_packet.rw_torque[2] 	= (int)(T_RW[2]/c_TM_RW_Resol);
	HILS_packet.rw_torque[3] 	= (int)(T_RW[3]/c_TM_RW_Resol);
	HILS_packet.Mic_time     	= Minor_Cycle_Count;
	HILS_packet.reserved_byte   = 0x55;
	for(i = 0; i<= 29 ; i++)
	{
		unsigned short temp;

		temp                  = HILS_packet.HILS_data_16bit[i];
		Check_sum_data        = (Check_sum_data ^ temp);
		HILS_packet.checksum  = Check_sum_data;
	}

		HILS_packet.Footer    = 0x7ffe;
	 //rHILS_payload(&HILS_packet);
}
*/

void rSuspended_ModePreprocessing(void)
{
	// Remote patch entry hook function
	rSus_mode_remote_entry_hook();

	TM.Buffer.TM_TC_Buffer[7] 	= (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[8] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[9]	= 0x0;
	TM.Buffer.TM_TC_Buffer[10]	= 0x0;
	TM.Buffer.TM_TC_Buffer[11]	= 0x0;

    if (Susp_cnt >= c_oneminute)
    {
    	if (TC_boolean_u.TC_Boolean_Table.TC_Sus2det_transit_en_dis == 1)
    	{
			Susp_cnt = 0;
			rADCS_Pon_vars();
			Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
			return;
    	}
    }
    else
    {
        Susp_cnt++;
    }

    // Remote patch exit hook function
   rSus_mode_remote_exit_hook();
}

void rDetumbling_ModePreprocessing_BDOT_Logic(void)
{

	// Remote patch entry hook function
	rDBDOT_mode_remote_entry_hook();

    //rRateReductionCheck();


	TM.Buffer.TM_TC_Buffer[8] = (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[7] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[9]	= 0x0;
	TM.Buffer.TM_TC_Buffer[10]	= 0x0;
	TM.Buffer.TM_TC_Buffer[11]	= 0x0;
    CB_Detumbling_Mode = 1;
    CB_OrbitModel = 0;
    CB_Q_propagation = 0;
    CB_DAD_quest = 0;
    CB_QuestDataProcessing = 0;
    CB_ErrorComputation = 0;
    CB_ExtendedKalmanFilter = 0;
    CB_LinearController = 0;
    CB_Wheel_OverSpeed_TorqueCutOff = 0;
    CB_DutyCycleGeneration = 1;
    CB_AngularMomentumDumping = 0;
    CB_SpeedBasedMomentumDumping = 0;
    CB_Wheel_Spin_updown = 0;
    CB_Wheel_Auto_Reconfiguration = 0;
    CB_Torquer_Polarity_Check = 1;
    CB_MagFieldComp = 0;
    CB_Sun_model = 0;
    CB_ReferenceQuatComputation = 0;
    CB_RefVectorGeneration = 0;
    CB_RefRate_Computation = 0;
    CB_Sl_Ecl_OnBrd_detection = 0;
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 0;
    CB_BDOT_Computation = 1;

    if(abs_f(w_BODYdeg[0]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh || abs_f(w_BODYdeg[0]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh || abs_f(w_BODYdeg[0]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh)
    {
        if(TC_boolean_u.TC_Boolean_Table.TC_Det_AutoTransitionBDOTtoGYRO == Enable)
        {
            GYRO_max_threshold_count++;
            if(GYRO_max_threshold_count >= c_onesecond)
            {
                GYRO_max_threshold_count = 0;
                Spacecraft_Mode = Detumbling_ModePreprocessing_GYRO;
                return;
            }
        }
    }
    else
    {
        GYRO_max_threshold_count = 0;
    }

    if((abs_f(BDOT[0]) <= TC_BDOT_Det_Thresh) && (abs_f(BDOT[1]) <= TC_BDOT_Det_Thresh)&& (abs_f(BDOT[2]) <= TC_BDOT_Det_Thresh))
    {
        BDOT_Threshold_Count++;
        if(BDOT_Threshold_Count >= c_oneminute)
        {
        	TorquerPolaritySetFlag = 0;

            if(TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition == Enable)
            {
                entrytime2eclipse = ADCS_TC_data_command_Table.TC_eclipse_entrytime - elapsed_running_timer;
                //entrytime2eclipse = 1000; /// test
                if (entrytime2eclipse < 900.0)
                {
                	Spacecraft_Mode = SunAcquisition_ModePreprocessing;
                    return;
                }
            }
        }

        else
        {
        	BDOT_Threshold_Count = 0;
			TorquerPolaritySetFlag = 1;
        }
    }
    else
    {
        BDOT_Threshold_Count = 0;
        TorquerPolaritySetFlag = 1;
    }

    DPM[0] = -1.0 * BDOT[0];
    DPM[1] = -1.0 * BDOT[1];
    DPM[2] = -1.0 * BDOT[2];
    Ton[0] = Ton[1] = Ton[2] = 8;

    TM.Buffer.TM_DPM[0] = (int)(DPM[0] / 0.001);
	TM.Buffer.TM_DPM[1] = (int)(DPM[1] / 0.001);
	TM.Buffer.TM_DPM[2] = (int)(DPM[2] / 0.001);
    tor_counter = 0; // for HAL_MTR

    rTorquer_Polarity_Check();


    // Remote patch exit hook function
    rDBDOT_mode_remote_exit_hook();
}

void rDetumbling_ModePreprocessing_GYRO_Logic(void)
{
	// Remote patch entry hook function
	rDGYRO_mode_remote_entry_hook();

    ///rRateReductionCheck();
	TM.Buffer.TM_TC_Buffer[9] = (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[8] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[7]	= 0x0;
	TM.Buffer.TM_TC_Buffer[10]	= 0x0;
	TM.Buffer.TM_TC_Buffer[11]	= 0x0;

    CB_Detumbling_Mode = 1;
    CB_OrbitModel = 0;
    CB_Q_propagation = 0;
    CB_DAD_quest = 0;
    CB_QuestDataProcessing = 0;
    CB_ErrorComputation = 0;
    CB_ExtendedKalmanFilter = 0;
    CB_LinearController = 0;
    CB_Wheel_OverSpeed_TorqueCutOff = 0;
    CB_DutyCycleGeneration = 1;
    CB_AngularMomentumDumping = 0;
    CB_SpeedBasedMomentumDumping = 0;
    CB_Wheel_Spin_updown = 0;
    CB_Wheel_Auto_Reconfiguration = 0;
    CB_Torquer_Polarity_Check = 1;
    CB_MagFieldComp = 0;
    CB_Sun_model = 0;
    CB_ReferenceQuatComputation = 0;
    CB_RefVectorGeneration = 0;
    CB_RefRate_Computation = 0;
    CB_Sl_Ecl_OnBrd_detection = 0;
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 0;
    CB_BDOT_Computation = 1;

    if(abs_f(w_BODYdeg[0]) <= TC_GYRO_Det_Min_Thresh && abs_f(w_BODYdeg[0]) <= TC_GYRO_Det_Min_Thresh && abs_f(w_BODYdeg[0]) <= TC_GYRO_Det_Min_Thresh)
    {
        GYRO_Threshold_Count++;
        if(GYRO_Threshold_Count >= c_oneminute)
        {
            TorquerPolaritySetFlag = 0;

            if(TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition == Enable)
            {
                entrytime2eclipse = ADCS_TC_data_command_Table.TC_eclipse_entrytime - elapsed_running_timer;
                //entrytime2eclipse = 1000; /// test
                if (entrytime2eclipse < 900.0)
                {
                	Spacecraft_Mode = SunAcquisition_ModePreprocessing;
                    return;
                }
            }
        }
        else
        {
        	GYRO_Threshold_Count = 0;
			TorquerPolaritySetFlag = 1;
        }
    }
    else
    {
        GYRO_Threshold_Count = 0;
        TorquerPolaritySetFlag = 1;
    }

    GYRO_Counter++;
    if(GYRO_Counter == ADCS_TC_data_command_Table.TC_Det_GYRO_Compute_Count)
    {
        gyrodet_w[0] = w_BODY[0];
        gyrodet_w[1] = w_BODY[1];
        gyrodet_w[2] = w_BODY[2];

        gyrodet_B[0] = B_BODYn[0];
        gyrodet_B[1] = B_BODYn[1];
        gyrodet_B[2] = B_BODYn[2];

        GYRO_Counter = 0;
    }

    Tdet[0] = -1.0 * sign_f(gyrodet_w[0]);
    Tdet[1] = -1.0 * sign_f(gyrodet_w[1]);
    Tdet[2] = -1.0 * sign_f(gyrodet_w[2]);

    rCross_Product(gyrodet_B, Tdet);
    DPM[0] = Cross_Product[0];
    DPM[1] = Cross_Product[1];
    DPM[2] = Cross_Product[2];
    Ton[0] = Ton[1] = Ton[2] = 8;

    TM.Buffer.TM_DPM[0] = (int)(DPM[0] / 4.65661287E-8);
	TM.Buffer.TM_DPM[1] = (int)(DPM[1] / 4.65661287E-8);
	TM.Buffer.TM_DPM[2] = (int)(DPM[2] / 4.65661287E-8);
	tor_counter = 0;
    rTorquer_Polarity_Check();

    // Remote patch exit hook function
    rDGYRO_mode_remote_exit_hook();

    return;
}

int qerror;
void rSunAcquisition_ModePreprocessing(void)
{
	// Remote patch entry hook function
	rSACQ_mode_remote_entry_hook();

    //f_Sunlit_Presence = True;    //for testing
	TM.Buffer.TM_TC_Buffer[10] = (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[8] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[9]	= 0x0;
	TM.Buffer.TM_TC_Buffer[7]	= 0x0;
	TM.Buffer.TM_TC_Buffer[11]	= 0x0;

    CB_Detumbling_Mode = 0;
    CB_OrbitModel = 1;
    CB_Q_propagation = 1;
    CB_DAD_quest = 1;
    CB_QuestDataProcessing = 1;
    CB_ErrorComputation = 0;
    CB_ExtendedKalmanFilter = 0;
    CB_LinearController = 1;
    CB_Wheel_OverSpeed_TorqueCutOff = 1;
    CB_DutyCycleGeneration = 1;
    CB_AngularMomentumDumping = 1;
    CB_SpeedBasedMomentumDumping = 1;
    CB_Wheel_Spin_updown = 0;
    CB_Wheel_Auto_Reconfiguration = 0;
    CB_Torquer_Polarity_Check = 1;
    CB_MagFieldComp = 0;
    CB_Sun_model = 1;
    CB_ReferenceQuatComputation = 1;
    CB_RefVectorGeneration = 1;
    CB_RefRate_Computation = 1;
    CB_Sl_Ecl_OnBrd_detection = 1;
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 1;
    CB_BDOT_Computation = 1;

    if (f_Sunlit_Presence == True)
    {
        Qerror[0] = Roll_ang_err;
        Qerror[1] = 0.0;
        Qerror[2] = Yaw_ang_err;

        TM.Buffer.TM_Q_Error[0] = (int)(Qerror[0]/4.65661287E-7);
        TM.Buffer.TM_Q_Error[1] = (int)(Qerror[1]/4.65661287E-7);
        TM.Buffer.TM_Q_Error[2] = (int)(Qerror[2]/4.65661287E-7);

        w_REF[0] = 0.0;
        w_REF[1] = -1.0 * TC_comd_pitch_rate * c_D2R;
        w_REF[2] = 0.0;

        TM.Buffer.TM_w_Ref[0] = (int)(w_REF[0]/c_TM_Resol_w);
		TM.Buffer.TM_w_Ref[1] = (int)(w_REF[1]/c_TM_Resol_w);
		TM.Buffer.TM_w_Ref[2] = (int)(w_REF[2]/c_TM_Resol_w);

        if (TC_boolean_u.TC_Boolean_Table.TC_SunAq2ThreeAxis_autotransit == Enable)
        {
            if ((Roll_ang_err <= SunAcq_Ang_Thres) && (Yaw_ang_err <= SunAcq_Ang_Thres) && (sun_quadrant == 6))
            {
                SunAcq3ThreeAx_trsit_cnt++;
                if (SunAcq3ThreeAx_trsit_cnt >= SunAcq2ThreeAx_trsit_cnt_thres)
                {
                	Spacecraft_Mode = ThreeAxis_ModePreprocessing;
                	Qbody[0] = Q_REF[0];
					Qbody[1] = Q_REF[1];
					Qbody[2] = Q_REF[2];
					Qbody[3] = Q_REF[3];
                }
            }
        }

        if ((TC_boolean_u.TC_Boolean_Table.TC_SunAq2DetMode_autotransit == Enable))
		{
			if(w_BODYnorm >= 2.0)
			{
				SunAcq2DetMode_counter++;

				if(SunAcq2DetMode_counter >= c_oneminute)
				{
					Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
					return;
				}
			}
			else
			{
				SunAcq2DetMode_counter = 0;
			}
		}
		else
		{
			SunAcq2DetMode_counter = 0;
		}

    }
    else
    {
        SunAcq3ThreeAx_trsit_cnt = 0;
        SunAcq2DetMode_counter = 0;
    }

    // Remote patch exit hook function
   rSACQ_mode_remote_exit_hook();

}

void rThreeAxis_ModePreprocessing(void)
{
	// Remote patch entry hook function
	r3AXIS_mode_remote_entry_hook();

	TM.Buffer.TM_TC_Buffer[11] = (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[8] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[9]	= 0x0;
	TM.Buffer.TM_TC_Buffer[10]	= 0x0;
	TM.Buffer.TM_TC_Buffer[7]	= 0x0;

    CB_Detumbling_Mode = 0;
    CB_OrbitModel = 1;
    CB_Q_propagation = 1;
    CB_DAD_quest = 1;
    CB_QuestDataProcessing = 1;
    CB_ErrorComputation = 1;
    CB_ExtendedKalmanFilter = 1;
    CB_LinearController = 1;
    CB_Wheel_OverSpeed_TorqueCutOff = 1;
    CB_DutyCycleGeneration = 1;
    CB_AngularMomentumDumping = 1;
    CB_SpeedBasedMomentumDumping = 1;
    CB_Wheel_Spin_updown = 0;
    CB_Wheel_Auto_Reconfiguration = 0;
    CB_Torquer_Polarity_Check = 1;
    CB_MagFieldComp = 1;
    CB_Sun_model = 1;
    CB_ReferenceQuatComputation = 1;
    CB_RefVectorGeneration = 1;
    CB_RefRate_Computation = 1;
    CB_Sl_Ecl_OnBrd_detection = 1; // COMMENTED FOR GROUND TESTING
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 1;
    CB_BDOT_Computation = 0;

	if(abs_f(Ang_Deviation) < c_AngDev_SMtransit_thrsld)
	{
		SunNPP_SMtransit_counter++;

		if(SunNPP_SMtransit_counter >= SunNPP_SMtransit_count_limit)
		{
			SunNPP_SMtransit = True;
			//SunNPP_SMtransit_counter = SunNPP_SMtransit_count_limit;
		}

		else
		{
			SunNPP_SMtransit = False;
		}
	}
	else
	{
		SunNPP_SMtransit = False;
		SunNPP_SMtransit_counter = 0;
	}

	if(w_BODYnorm >= 1.5)
	{
		ThreeAxis2DetMode_counter++;

		if(ThreeAxis2DetMode_counter >= c_oneminute)
		{
			ThreeAxis2DetMode_counter = 0;
			f_threeaxis2safe = True;
		}
	}
	else
	{
		ThreeAxis2DetMode_counter = 0;
	}

    if ((TC_boolean_u.TC_Boolean_Table.TC_ThreeAxis2SafeMode_autotransit == Enable))
	{
    	if ((SunNPP_SMtransit == True) || (f_threeaxis2safe == True) || (f_battery_safemode == True)) //ADD BATTERY CONDITION
    	{
    		Spacecraft_Mode = Safe_ModePreprocessing;
    		return;
    	}

	}

    // Remote patch exit hook function
   r3AXIS_mode_remote_exit_hook();

}

void rRateReductionCheck(void)
{
    BDOT_Reduction_Count++;
    if(BDOT_Reduction_Count >= c_halfhour)
    {
        if(abs_f(BDOTnorm) <= 2000.0)
        {
            BDOTNormCount++;
            BDOT_Reduction_Count = 0;
            if(BDOT_Norm_Count >= c_oneminute)
            {
                BDOT_Norm_Count = 0;
            }
        }
        else
        {
            if(Roll_MTR_Pol_Reversal == 0)
            {
                BDOT_Reduction_Count = 0;
                //rTC_Polarity_Reversal_Roll();
                //rTC_Polarity_Reversal_Pitch();
                //rTC_Polarity_Reversal_Yaw();
            }
            else
            {
                //CB_Torquer_Polarity_Check = 1; ///This condition is to check in MPP logic, if true make MR, MP, MY zero
            }
        }
    }
    return;
}

void rSafeMode_Preprocessing(void)
{
	// Remote patch entry hook function
	rSAFE_mode_remote_entry_hook();

	TM.Buffer.TM_TC_Buffer[12] = (char)Spacecraft_Mode;
	TM.Buffer.TM_TC_Buffer[8] 	= 0x0;
	TM.Buffer.TM_TC_Buffer[9]	= 0x0;
	TM.Buffer.TM_TC_Buffer[10]	= 0x0;
	TM.Buffer.TM_TC_Buffer[7]	= 0x0;
	TM.Buffer.TM_TC_Buffer[11]	= 0x0;

	if (f_battery_safemode == True) // Enable the flag when battery is less than the threshold
	{
		// Turn off loads
		if (w_BODYnorm > 1.0)
		{
			Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
		}
		else
		{
			entrytime2eclipse = ADCS_TC_data_command_Table.TC_eclipse_entrytime - elapsed_running_timer;
			//entrytime2eclipse = 1000; /// test
			if (entrytime2eclipse < 900.0)
			{
				Spacecraft_Mode = SunAcquisition_ModePreprocessing;
			}
			else
			{
				Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
			}
		}
	}

    // Remote patch exit hook function
	rSAFE_mode_remote_exit_hook();
}

