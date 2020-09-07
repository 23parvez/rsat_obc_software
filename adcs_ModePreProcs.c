#include <stdio.h>
#include <math.h>

#include "adcs_ADandEst.h"
#include "adcs_CommonRoutines.h"
#include "adcs_Constants.h"
#include "adcs_GPS_OD.h"
#include "adcs_LinearController.h"
#include "adcs_ModePreProcs.h"
#include "adcs_RefComp.h"
#include "adcs_SensorDataProcs.h"
#include "adcs_VarDeclarations.h"

#include "HAL_Global.h"
#include "Global.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"

static void rSuspended_ModePreprocessing(void);
static void rDetumbling_ModePreprocessing_GYRO_Logic(void);
static void rDetumbling_ModePreprocessing_BDOT_Logic(void);
static void rSunAcquisition_ModePreprocessing(void);
static void rThreeAxis_ModePreprocessing(void);
static void rSafeMode_Preprocessing(void);

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

			default:
					rSuspended_ModePreprocessing();
			break;
	}

	if(f_station_tracking_enabled)
	{
		TM.Buffer.TM_mode_selection = 0x06;
	}
	else
	{
		TM.Buffer.TM_mode_selection = Spacecraft_Mode;
	}

}

/*
unsigned short Check_sum_data =0;
void rHILS_packets(void)
{

	int i;
	unsigned short temp_hils;
	unsigned short temp_aux;
	HILS_packet.header               = 0x07e0;
	HILS_packet.len                  = 0x40;
	temp_aux 					     = hils_mode_select;
	HILS_packet.aux                  = (char)(temp_aux & 0x00FF);
	HILS_packet.mode_flag 		     = (char)Spacecraft_Mode;
	HILS_packet.mag_field[0] 	     = (int)(B_BODY[0]/c_TM_Resol_B);
	HILS_packet.mag_field[1] 	     = (int)(B_BODY[1]/c_TM_Resol_B);
	HILS_packet.mag_field[2] 	     = (int)(B_BODY[2]/c_TM_Resol_B);
	temp_hils                	     = input_latch_1.data;
	HILS_packet.polarity     	     = (char)((temp_hils & 0xFC00)>>8);
	HILS_packet.rw_torque[0] 	     = (int)(T_RW[0]/c_TM_RW_Resol);
	HILS_packet.rw_torque[1] 	     = (int)(T_RW[1]/c_TM_RW_Resol);
	HILS_packet.rw_torque[2] 	     = (int)(T_RW[2]/c_TM_RW_Resol);
	HILS_packet.rw_torque[3] 	     = (int)(T_RW[3]/c_TM_RW_Resol);
	HILS_packet.wbody[0] 	     	 = (int)(w_BODY[0]/c_TM_Resol_w);
	HILS_packet.wbody[1] 	     	 = (int)(w_BODY[1]/c_TM_Resol_w);
	HILS_packet.wbody[2] 	     	 = (int)(w_BODY[2]/c_TM_Resol_w);

	HILS_packet.Mic_time     	     = Minor_Cycle_Count;
	HILS_packet.reserved_byte        = 0x55;
	unsigned short Check_sum_data    = 0x0000;
	for(i = 2; i<= 59 ; i++)
	{

		unsigned char temp_1;
		unsigned short temp_2;

		temp_1                       = HILS_packet.HILS_data_8bit[i];
		temp_2                       = (unsigned short)(temp_1 & 0xFF);
		Check_sum_data               = (Check_sum_data ^ (temp_2 & 0x00FF));
		HILS_packet.checksum         = Check_sum_data;

	}
	HILS_packet.Footer    = 0x7ffe;
	rHILS_payload(&HILS_packet);
}
*/

static void rSuspended_ModePreprocessing(void)
{
	// Remote patch entry hook function
	rSus_mode_remote_entry_hook();

    if (Susp_cnt >= c_oneminute)
    {
    	if (TC_boolean_u.TC_Boolean_Table.TC_Sus2det_transit_en_dis == 1)
    	{
			Susp_cnt = 0;
			//rADCS_Pon_vars();
			Spacecraft_Mode = Detumbling_ModePreprocessing_BDOT;
			//return;
    	}
    }
    else
    {
        Susp_cnt++;
    }

    // Remote patch exit hook function
   rSus_mode_remote_exit_hook();
}

static void rDetumbling_ModePreprocessing_BDOT_Logic(void)
{

	// Remote patch entry hook function
	rDBDOT_mode_remote_entry_hook();

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
    CB_Sl_Ecl_OnBrd_detection = 1;
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 0;
    CB_BDOT_Computation = 1;

    if((fabs(w_BODYdeg[0]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh) || (fabs(w_BODYdeg[1]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh) || (fabs(w_BODYdeg[2]) >= ADCS_TC_data_command_Table.TC_GYRO_Det_Max_Thresh))
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

    if((fabs(BDOT[0]) <= TC_BDOT_Det_Thresh) && (fabs(BDOT[1]) <= TC_BDOT_Det_Thresh) && (fabs(BDOT[2]) <= TC_BDOT_Det_Thresh))
    {
        BDOT_Threshold_Count++;
        if(BDOT_Threshold_Count >= c_oneminute)
        {
        	TorquerPolaritySetFlag = 0;

            if(TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition == Enable)
            {
                entrytime2eclipse = ADCS_TC_data_command_Table.TC_eclipse_entrytime - elapsed_running_timer;
                //entrytime2eclipse = 1000; // test
                if (entrytime2eclipse > 7032)
				{
					f_RW_nominal = 1;
					//Turn on TW
					//Ping command
					//Enable all RW

					if (f_RW_control == 1)
					{
						f_RW_nominal = 0;
						Spacecraft_Mode = SunAcquisition_ModePreprocessing;
						return;
					}
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
    Ton[0] = 8;
    Ton[1] = 8;
    Ton[2] = 8;

    TM.Buffer.TM_DPM[0] = (int)(DPM[0] / 0.001);
	TM.Buffer.TM_DPM[1] = (int)(DPM[1] / 0.001);
	TM.Buffer.TM_DPM[2] = (int)(DPM[2] / 0.001);
    tor_counter = 0; // for HAL_MTR

    rTorquer_Polarity_Check();


    // Remote patch exit hook function
    rDBDOT_mode_remote_exit_hook();
}

static void rDetumbling_ModePreprocessing_GYRO_Logic(void)
{
	// Remote patch entry hook function
	rDGYRO_mode_remote_entry_hook();

    //rRateReductionCheck();

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
    CB_Sl_Ecl_OnBrd_detection = 1;
    CB_IMUDataProcessing = 1;
    CB_SunSensorDataProcessing = 0;
    CB_BDOT_Computation = 1;

    if((fabs(w_BODYdeg[0]) <= TC_GYRO_Det_Min_Thresh) && (fabs(w_BODYdeg[1]) <= TC_GYRO_Det_Min_Thresh) && (fabs(w_BODYdeg[2]) <= TC_GYRO_Det_Min_Thresh))
    {
        GYRO_Threshold_Count++;
        if(GYRO_Threshold_Count >= c_oneminute)
        {
            TorquerPolaritySetFlag = 0;

            if(TC_boolean_u.TC_Boolean_Table.TC_AutoTransit_Det2SunAquisition == Enable)
            {
                entrytime2eclipse = ADCS_TC_data_command_Table.TC_eclipse_entrytime - elapsed_running_timer;
                //entrytime2eclipse = 1000; // test
                if (entrytime2eclipse > 7032)
				{
					f_RW_nominal = 1;
					//Turn on TW
					//Ping command
					//Enable all RW

					if (f_RW_control == 1)
					{
						f_RW_nominal = 0;
						Spacecraft_Mode = SunAcquisition_ModePreprocessing;
						return;
					}
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

    Tdet[0] = -1.0 * rsign_f(gyrodet_w[0]);
    Tdet[1] = -1.0 * rsign_f(gyrodet_w[1]);
    Tdet[2] = -1.0 * rsign_f(gyrodet_w[2]);

    rCross_Product(gyrodet_B, Tdet);
    DPM[0] = Cross_Product[0];
    DPM[1] = Cross_Product[1];
    DPM[2] = Cross_Product[2];
    Ton[0] = 8;
	Ton[1] = 8;
	Ton[2] = 8;

    TM.Buffer.TM_DPM[0] = (int)(DPM[0] / 4.65661287E-8);
	TM.Buffer.TM_DPM[1] = (int)(DPM[1] / 4.65661287E-8);
	TM.Buffer.TM_DPM[2] = (int)(DPM[2] / 4.65661287E-8);
	tor_counter = 0;
    rTorquer_Polarity_Check();

    // Remote patch exit hook function
    rDGYRO_mode_remote_exit_hook();
}

static void rSunAcquisition_ModePreprocessing(void)
{
	// Remote patch entry hook function
	rSACQ_mode_remote_entry_hook();

    //f_Sunlit_Presence = True;    //for testing

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
            if ((fabs(Roll_ang_err) <= SunAcq_Ang_Thres) && (fabs(Yaw_ang_err) <= SunAcq_Ang_Thres) && (sun_quadrant == 6))
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

        Qerror[0] = 0.0;
		Qerror[1] = 0.0;
		Qerror[2] = 0.0;

		w_REF[0] = 0.0;
		w_REF[1] = -1.0 * TC_comd_pitch_rate * c_D2R;
		w_REF[2] = 0.0;
    }

    // Remote patch exit hook function
   rSACQ_mode_remote_exit_hook();

}

static void rThreeAxis_ModePreprocessing(void)
{
	// Remote patch entry hook function
	r3AXIS_mode_remote_entry_hook();

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
    CB_Wheel_Auto_Reconfiguration = 1;
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

	if(fabs(Ang_Deviation) < c_AngDev_SMtransit_thrsld)
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
    		//Spacecraft_Mode = Safe_ModePreprocessing;
    		return;
    	}

	}

    // Remote patch exit hook function
   r3AXIS_mode_remote_exit_hook();

}

static void rSafeMode_Preprocessing(void)
{
	// Remote patch entry hook function
	rSAFE_mode_remote_entry_hook();

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
			//entrytime2eclipse = 1000; // test
			if (entrytime2eclipse < 7032) //15mins
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

