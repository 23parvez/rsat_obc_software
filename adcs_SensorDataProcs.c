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

#include "Global.h"
#include "HAL_Global.h"
#include "HAL_IMU.h"
#include "HAL_Address.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "Telecommand.h"
#include "TC_List.h"

static void rSS_Read_Data(const unsigned long int *ADC_Data_Addr);
static void rSunSensorVectorComp(const double SS_ARRAY[8], struct SunSensor_Database *SS_2Exe_Addr);
static void rIMUDataCorrection(struct IMU_READ_DATA* IMU_DATA_ADDRS,struct IMU_Database* IMU_CORR);
static double rTheta_Limit(const double var_theta_lim);

static void rIMUDataCorrection(struct IMU_READ_DATA* IMU_DATA_ADDRS,struct IMU_Database* IMU_CORR)
{
	DelThta_cat_x = (IMU_DATA_ADDRS->DelThta_MSB_x << 16) | (IMU_DATA_ADDRS->DelThta_LSB_x & c_FFFF);
	DelThta_cat_y = (IMU_DATA_ADDRS->DelThta_MSB_y << 16) | (IMU_DATA_ADDRS->DelThta_LSB_y & c_FFFF);
	DelThta_cat_z = (IMU_DATA_ADDRS->DelThta_MSB_z << 16) | (IMU_DATA_ADDRS->DelThta_LSB_z & c_FFFF);

	// Magnetometer data being converted to Double precision floating point and multiplied by resolution
	B_rawdata[0] = (int)(IMU_DATA_ADDRS->Mag_x);
	B_rawdata[1] = (int)(IMU_DATA_ADDRS->Mag_y);
	B_rawdata[2] = (int)(IMU_DATA_ADDRS->Mag_z);

	B_rawdata[0] = B_rawdata[0] & 0XFFFF;
	B_rawdata[1] = B_rawdata[1] & 0XFFFF;
	B_rawdata[2] = B_rawdata[2] & 0XFFFF;

	B_IMU[0] = (double) ((B_rawdata[0] << 16) * c_Resol_B);
	B_IMU[1] = (double) ((B_rawdata[1] << 16) * c_Resol_B);
	B_IMU[2] = (double) ((B_rawdata[2] << 16) * c_Resol_B);

	// Delta Theta data being converted to Double precision floating point and multiplied by resolution
	DelThta_rawdata[0] = (double) (DelThta_cat_x * c_Resol_DelThta);
	DelThta_rawdata[1] = (double) (DelThta_cat_y * c_Resol_DelThta);
	DelThta_rawdata[2] = (double) (DelThta_cat_z * c_Resol_DelThta);

	Thta_rawdata[0] = Thta_rawdata[0] + DelThta_rawdata[0];
	Thta_rawdata[1] = Thta_rawdata[1] + DelThta_rawdata[1];
	Thta_rawdata[2] = Thta_rawdata[2] + DelThta_rawdata[2];

	Thta_rawdata[0] = rTheta_Limit(Thta_rawdata[0]);		// Avoiding the overflow of thta_rawdata beyond 540 degrees
	Thta_rawdata[1] = rTheta_Limit(Thta_rawdata[1]);
	Thta_rawdata[2] = rTheta_Limit(Thta_rawdata[2]);

	w_ABC[0] = (DelThta_rawdata[0]) / c_MiC;
	w_ABC[1] = (DelThta_rawdata[1]) / c_MiC;
	w_ABC[2] = (DelThta_rawdata[2]) / c_MiC;

    //Bias correction for theta
    w_BiasCor[0] = w_ABC[0] -  IMU_CORR->DB_w_BiasCor[0];
    w_BiasCor[1] = w_ABC[1] -  IMU_CORR->DB_w_BiasCor[1];
    w_BiasCor[2] = w_ABC[2] -  IMU_CORR->DB_w_BiasCor[2];

    // Scale Factor correction for Gyrodata
    if(w_BiasCor[0] < 0.0)
    {
        (w_ASC[0] = IMU_CORR->DB_wSFC_Neg[0] * w_BiasCor[0]);
    }
    else
    {
        (w_ASC[0] = IMU_CORR->DB_wSFC_Pos[0] * w_BiasCor[0]);
    }
    if(w_BiasCor[1] < 0.0)
    {
        (w_ASC[1] = IMU_CORR->DB_wSFC_Neg[1] * w_BiasCor[1]);
    }
    else
    {
        (w_ASC[1] = IMU_CORR->DB_wSFC_Pos[1] * w_BiasCor[1]);
    }
    if(w_BiasCor[2] < 0.0)
    {
        (w_ASC[2] = IMU_CORR->DB_wSFC_Neg[2] * w_BiasCor[2]);
    }
    else
    {
        (w_ASC[2] = IMU_CORR->DB_wSFC_Pos[2] * w_BiasCor[2]);
    }

    //Misalignment correction for Gyroscope
    w_AMSC[0] = ((IMU_CORR->DB_w_MisCor[0][0] * w_ASC[0]) + (IMU_CORR->DB_w_MisCor[0][1] * w_ASC[1]) + (IMU_CORR->DB_w_MisCor[0][2] * w_ASC[2]));
    w_AMSC[1] = ((IMU_CORR->DB_w_MisCor[1][0] * w_ASC[0]) + (IMU_CORR->DB_w_MisCor[1][1] * w_ASC[1]) + (IMU_CORR->DB_w_MisCor[1][2] * w_ASC[2]));
    w_AMSC[2] = ((IMU_CORR->DB_w_MisCor[2][0] * w_ASC[0]) + (IMU_CORR->DB_w_MisCor[2][1] * w_ASC[1]) + (IMU_CORR->DB_w_MisCor[2][2] * w_ASC[2]));

    //Bring sensor data from sensor frame to body frame
    w_RPY[0] = ((IMU_Sen2Bdy[0][0] * w_AMSC[0]) + (IMU_Sen2Bdy[0][1] * w_AMSC[1]) + (IMU_Sen2Bdy[0][2] * w_AMSC[2]));
    w_RPY[1] = ((IMU_Sen2Bdy[1][0] * w_AMSC[0]) + (IMU_Sen2Bdy[1][1] * w_AMSC[1]) + (IMU_Sen2Bdy[1][2] * w_AMSC[2]));
    w_RPY[2] = ((IMU_Sen2Bdy[2][0] * w_AMSC[0]) + (IMU_Sen2Bdy[2][1] * w_AMSC[1]) + (IMU_Sen2Bdy[2][2] * w_AMSC[2]));

    // Low pass filter for IMU
    w_LPF[0] = ((IMU_CORR->DB_GyroLPF[0] * w_LPF[0]) + (IMU_CORR->DB_GyroLPF[1] * w_RPY[0]));
    w_LPF[1] = ((IMU_CORR->DB_GyroLPF[0] * w_LPF[1]) + (IMU_CORR->DB_GyroLPF[1] * w_RPY[1]));
    w_LPF[2] = ((IMU_CORR->DB_GyroLPF[0] * w_LPF[2]) + (IMU_CORR->DB_GyroLPF[1] * w_RPY[2]));

    IMU_prcd_data[0] = w_LPF[0];
    IMU_prcd_data[1] = w_LPF[1];
    IMU_prcd_data[2] = w_LPF[2];

    //Bias correction for Magnetometer

    B_BiasCor[0] = B_IMU[0] - IMU_CORR->DB_B_BiasCor[0];
    B_BiasCor[1] = B_IMU[1] - IMU_CORR->DB_B_BiasCor[1];
    B_BiasCor[2] = B_IMU[2] - IMU_CORR->DB_B_BiasCor[2];

    // Scale Factor correction for Magdata
    if(B_BiasCor[0] < 0.0)
    {
        (B_ASC[0] = IMU_CORR->DB_BSFC_Neg[0] * B_BiasCor[0]);
    }
    else
    {
        (B_ASC[0] = IMU_CORR->DB_BSFC_Pos[0] * B_BiasCor[0]);
    }
    if(B_BiasCor[1] < 0.0)
    {
        (B_ASC[1] = IMU_CORR->DB_BSFC_Neg[1] * B_BiasCor[1]);
    }
    else
    {
        (B_ASC[1] = IMU_CORR->DB_BSFC_Pos[1] * B_BiasCor[1]);
    }
    if(B_BiasCor[2] < 0.0)
    {
        (B_ASC[2] = IMU_CORR->DB_BSFC_Neg[2] * B_BiasCor[2]);
    }
    else
    {
        (B_ASC[2] = IMU_CORR->DB_BSFC_Pos[2] * B_BiasCor[2]);
    }

    //Misalignment-Scale Factor correction for Magnetometer
    B_AMSC[0] = ((IMU_CORR->DB_B_MisCor[0][0] * B_ASC[0]) + (IMU_CORR->DB_B_MisCor[0][1] * B_ASC[1]) + (IMU_CORR->DB_B_MisCor[0][2] * B_ASC[2]));
    B_AMSC[1] = ((IMU_CORR->DB_B_MisCor[1][0] * B_ASC[0]) + (IMU_CORR->DB_B_MisCor[1][1] * B_ASC[1]) + (IMU_CORR->DB_B_MisCor[1][2] * B_ASC[2]));
    B_AMSC[2] = ((IMU_CORR->DB_B_MisCor[2][0] * B_ASC[0]) + (IMU_CORR->DB_B_MisCor[2][1] * B_ASC[1]) + (IMU_CORR->DB_B_MisCor[2][2] * B_ASC[2]));

    B_RPY[0] = ((IMU_Sen2Bdy[0][0] * B_AMSC[0]) + (IMU_Sen2Bdy[0][1] * B_AMSC[1]) + (IMU_Sen2Bdy[0][2] * B_AMSC[2]));
    B_RPY[1] = ((IMU_Sen2Bdy[1][0] * B_AMSC[0]) + (IMU_Sen2Bdy[1][1] * B_AMSC[1]) + (IMU_Sen2Bdy[1][2] * B_AMSC[2]));
    B_RPY[2] = ((IMU_Sen2Bdy[2][0] * B_AMSC[0]) + (IMU_Sen2Bdy[2][1] * B_AMSC[1]) + (IMU_Sen2Bdy[2][2] * B_AMSC[2]));

    B_LPF[0] = ((IMU_CORR->DB_MagLPF[0] * B_LPF[0]) + (IMU_CORR->DB_MagLPF[1] * B_RPY[0]));
    B_LPF[1] = ((IMU_CORR->DB_MagLPF[0] * B_LPF[1]) + (IMU_CORR->DB_MagLPF[1] * B_RPY[1]));
    B_LPF[2] = ((IMU_CORR->DB_MagLPF[0] * B_LPF[2]) + (IMU_CORR->DB_MagLPF[1] * B_RPY[2]));

    IMU_prcd_data[3] = B_LPF[0];
    IMU_prcd_data[4] = B_LPF[1];
    IMU_prcd_data[5] = B_LPF[2];
}

void rIMU_Angle_Reset(void)
{
	Thta_rawdata[0] = 0.0;
	Thta_rawdata[1] = 0.0;
	Thta_rawdata[2] = 0.0;

	Thta_BODY_IMU1[0] = 0.0;
	Thta_BODY_IMU1[1] = 0.0;
	Thta_BODY_IMU1[2] = 0.0;

	Thta_BODY_IMU2[0] = 0.0;
	Thta_BODY_IMU2[1] = 0.0;
	Thta_BODY_IMU2[2] = 0.0;
}

void rIMUDataProcessing(void)
{
	HAL_IMU_Read(IMU_1,(IMU1_DATA.IMU_DATA));
	rIMUDataCorrection(&IMU1_DATA,&IMU1_Corr);

	w_BODY_IMU1[0] = IMU_prcd_data[0];
	w_BODY_IMU1[1] = IMU_prcd_data[1];
	w_BODY_IMU1[2] = IMU_prcd_data[2];
	B_BODY_IMU1[0] = IMU_prcd_data[3];
	B_BODY_IMU1[1] = IMU_prcd_data[4];
	B_BODY_IMU1[2] = IMU_prcd_data[5];

	Thta_BODY_IMU1[0] = Thta_BODY_IMU1[0] + (w_BODY_IMU1[0] * c_MiC);
	Thta_BODY_IMU1[1] = Thta_BODY_IMU1[1] + (w_BODY_IMU1[1] * c_MiC);
	Thta_BODY_IMU1[2] = Thta_BODY_IMU1[2] + (w_BODY_IMU1[2] * c_MiC);

	Thta_BODY_IMU1[0] = rTheta_Limit(Thta_BODY_IMU1[0]);		// Avoiding the overflow of Thta_BODY_IMU1 beyond 540 degrees
	Thta_BODY_IMU1[1] = rTheta_Limit(Thta_BODY_IMU1[1]);
	Thta_BODY_IMU1[2] = rTheta_Limit(Thta_BODY_IMU1[2]);

	TM_IMU1.TM_Theta_IMU[0] = (int)(Thta_rawdata[0] / c_TM_Resol_DelThta);
	TM_IMU1.TM_Theta_IMU[1] = (int)(Thta_rawdata[1] / c_TM_Resol_DelThta);
	TM_IMU1.TM_Theta_IMU[2] = (int)(Thta_rawdata[2] / c_TM_Resol_DelThta);

	TM_IMU1.TM_B_IMU[0] = (int)(B_rawdata[0]);
	TM_IMU1.TM_B_IMU[1] = (int)(B_rawdata[1]);
	TM_IMU1.TM_B_IMU[2] = (int)(B_rawdata[2]);

	TM_IMU1.TM_IMU_Temp     = (unsigned short)(IMU1_DATA.IMU_Temp); //IMU Temperature
	//IMU_temperature         = (unsigned short)(IMU1_DATA.IMU_Temp);
	TM_IMU1.TM_IMU_Diag_STS = (unsigned short)(IMU1_DATA.IMU_Diag_STS); //IMU DIAG STATUS
	TM_IMU1.TM_IMU_Diag_REG = (unsigned short)(IMU1_DATA.IMU_Diag_REG); //IMU DIAG REGISTER

	/*TM.Buffer.TM_w_IMU[0] = (int)(w_ABC[0] / c_TM_Resol_w);
	TM.Buffer.TM_w_IMU[1] = (int)(w_ABC[1] / c_TM_Resol_w);
	TM.Buffer.TM_w_IMU[2] = (int)(w_ABC[2] / c_TM_Resol_w);*/

	HAL_IMU_Read(IMU_2,(IMU2_DATA.IMU_DATA));					//Data read of IMU2 from FPGA Buffer
	rIMUDataCorrection(&IMU2_DATA,&IMU2_Corr);	//IMU1 Data processing

	w_BODY_IMU2[0] = IMU_prcd_data[0];
	w_BODY_IMU2[1] = IMU_prcd_data[1];
	w_BODY_IMU2[2] = IMU_prcd_data[2];
	B_BODY_IMU2[0] = IMU_prcd_data[3];
	B_BODY_IMU2[1] = IMU_prcd_data[4];
	B_BODY_IMU2[2] = IMU_prcd_data[5];

	Thta_BODY_IMU2[0] = Thta_BODY_IMU2[0] + (w_BODY_IMU2[0] * c_MiC);
	Thta_BODY_IMU2[1] = Thta_BODY_IMU2[1] + (w_BODY_IMU2[1] * c_MiC);
	Thta_BODY_IMU2[2] = Thta_BODY_IMU2[2] + (w_BODY_IMU2[2] * c_MiC);

	Thta_BODY_IMU2[0] = rTheta_Limit(Thta_BODY_IMU2[0]);		// Avoiding the overflow of Thta_BODY_IMU2 beyond 540 degrees
	Thta_BODY_IMU2[1] = rTheta_Limit(Thta_BODY_IMU2[1]);
	Thta_BODY_IMU2[2] = rTheta_Limit(Thta_BODY_IMU2[2]);

	TM_IMU2.TM_Theta_IMU[0] = (int)(Thta_rawdata[0] / c_TM_Resol_DelThta);
	TM_IMU2.TM_Theta_IMU[1] = (int)(Thta_rawdata[1] / c_TM_Resol_DelThta);
	TM_IMU2.TM_Theta_IMU[2] = (int)(Thta_rawdata[2] / c_TM_Resol_DelThta);

	TM_IMU2.TM_B_IMU[0] = (int)(B_rawdata[0]);
	TM_IMU2.TM_B_IMU[1] = (int)(B_rawdata[1]);
	TM_IMU2.TM_B_IMU[2] = (int)(B_rawdata[2]);

	TM_IMU2.TM_IMU_Temp     = (unsigned short)(IMU2_DATA.IMU_Temp); //IMU Temperature
	TM_IMU2.TM_IMU_Diag_STS = (unsigned short)(IMU2_DATA.IMU_Diag_STS); //IMU DIAG STATUS
	TM_IMU2.TM_IMU_Diag_REG = (unsigned short)(IMU2_DATA.IMU_Diag_REG); //IMU DIAG REGISTER

	/*TM.Buffer.TM_w_IMU[0] = (int)(w_ABC[0] / c_TM_Resol_w);
	TM.Buffer.TM_w_IMU[1] = (int)(w_ABC[1] / c_TM_Resol_w);
	TM.Buffer.TM_w_IMU[2] = (int)(w_ABC[2] / c_TM_Resol_w);*/

	TM.Buffer.TM_IMU1 = TM_IMU1;
	TM.Buffer.TM_IMU2 = TM_IMU2;

	if (TC_boolean_u.TC_Boolean_Table.TC_IMU_Select == IMU1)  //IMU1
	{
		w_BODY[0] = w_BODY_IMU1[0];
		w_BODY[1] = w_BODY_IMU1[1];
		w_BODY[2] = w_BODY_IMU1[2];

		B_BODY[0] = B_BODY_IMU1[0];
		B_BODY[1] = B_BODY_IMU1[1];
		B_BODY[2] = B_BODY_IMU1[2];

		ST_normal.ST_NM_Buffer.TM_IMU_1_Temp     = (unsigned short)(IMU1_DATA.IMU_Temp); //IMU Temperature
		ST_special.ST_SP_Buffer.TM_IMU_1_Temp     = (unsigned short)(IMU1_DATA.IMU_Temp); //IMU Temperature
		ST_normal.ST_NM_Buffer.TM_IMU_1_Diag_STS = (unsigned short)(IMU1_DATA.IMU_Diag_STS); //IMU DIAG STATUS

		TM.Buffer.TM_IMU_SELECTED = TM_IMU1;
		IMU_temperature         = IMU1_DATA.IMU_Temp;

		if (TC_boolean_u.TC_Boolean_Table.TC_GND_Drift_Compensation_Enable_or_Disable == Enable)
		{
			w_BODY[0] = w_BODY[0] - (double)TC_drift_compensation_IMU1[0];
			w_BODY[1] = w_BODY[1] - (double)TC_drift_compensation_IMU1[1];
			w_BODY[2] = w_BODY[2] - (double)TC_drift_compensation_IMU1[2];
		}

		if ((TC_boolean_u.TC_Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable == Enable) && (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == Enable))
		{
			w_BODY[0] = w_BODY[0] - Xk[3];
			w_BODY[1] = w_BODY[1] - Xk[4];
			w_BODY[2] = w_BODY[2] - Xk[5];
		}


		w_BODYdeg[0] = w_BODY[0] * c_R2D;
		w_BODYdeg[1] = w_BODY[1] * c_R2D;
		w_BODYdeg[2] = w_BODY[2] * c_R2D;

		if(TC_boolean_u.TC_Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable == Enable)
		{
			if ((TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Enable_or_Disable == Enable) || (TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Enable_or_Disable == Enable) || (TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Enable_or_Disable == Enable))  // Software..hw enable/disable
			{
				for(i_MatEq = 0; i_MatEq < 27; i_MatEq++)
				{
					if((DPM_Polarity[0] == c_DPM_Pol_LookUpTable[i_MatEq][0]) && (DPM_Polarity[1] == c_DPM_Pol_LookUpTable[i_MatEq][1]) && (DPM_Polarity[2] == c_DPM_Pol_LookUpTable[i_MatEq][2]))  // Actual polarity status from rHAL_MTR
					{
						PolCheck_LUT = i_MatEq;
						break;
					}
				}

				B_BODY[0] -= c_MagBias_act_LUT[PolCheck_LUT][0];
				B_BODY[1] -= c_MagBias_act_LUT[PolCheck_LUT][1];
				B_BODY[2] -= c_MagBias_act_LUT[PolCheck_LUT][2];

			}
		}

		B_BODY_LUT[0] = B_BODY[0];
		B_BODY_LUT[1] = B_BODY[1];
		B_BODY_LUT[2] = B_BODY[2];

		if (TC_boolean_u.TC_Boolean_Table.TC_GND_MagBias_Compensation_Enable_or_Disable == Enable)
		{
			B_BODY[0] = B_BODY[0] - (double)TC_magbias_compensation_IMU1[0];
			B_BODY[1] = B_BODY[1] - (double)TC_magbias_compensation_IMU1[1];
			B_BODY[2] = B_BODY[2] - (double)TC_magbias_compensation_IMU1[2];
		}

		if (TC_boolean_u.TC_Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable == Enable)
		{
			if ((TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 0) && (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == Enable))
			{
				B_BODY[0] = B_BODY[0] - (Xk[6] * 1.0e9);
				B_BODY[1] = B_BODY[1] - (Xk[7] * 1.0e9);
				B_BODY[2] = B_BODY[2] - (Xk[8] * 1.0e9);
			}
			else
				{
					if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == Enable)
					{
						if ((f_station_tracking_enabled == 0) && (Sunlit_presence_timer > 4650))
						{
							B_BODY[0] = B_BODY[0] - (lpfb_k[0] * 1.0e9);
							B_BODY[1] = B_BODY[1] - (lpfb_k[1] * 1.0e9);
							B_BODY[2] = B_BODY[2] - (lpfb_k[2] * 1.0e9);
						}
						else// if (f_station_tracking_enabled == 1 || Sunlit_presence_timer < 9375)
						{
							B_BODY[0] = B_BODY[0] - (lpfb_k_prev[0] * 1.0e9);
							B_BODY[1] = B_BODY[1] - (lpfb_k_prev[1] * 1.0e9);
							B_BODY[2] = B_BODY[2] - (lpfb_k_prev[2] * 1.0e9);
						}
					}

				}
		}

		B_BODYtesla[0] = B_BODY[0] * 1.0E-9;
		B_BODYtesla[1] = B_BODY[1] * 1.0E-9;
		B_BODYtesla[2] = B_BODY[2] * 1.0E-9;

		Bsq = ((B_BODYtesla[0]*B_BODYtesla[0]) + (B_BODYtesla[1]*B_BODYtesla[1]) + (B_BODYtesla[2]*B_BODYtesla[2]));

		w_BODYnorm = sqrt((w_BODYdeg[0]*w_BODYdeg[0]) + (w_BODYdeg[1]*w_BODYdeg[1]) + (w_BODYdeg[2]*w_BODYdeg[2])) * c_R2D;

		rVectorNorm(B_BODY);
		B_BODYn[0] = Norm_out[0];
		B_BODYn[1] = Norm_out[1];
		B_BODYn[2] = Norm_out[2];

		magRoll_angle  = acos(B_BODYn[0])*c_R2D;
		magPitch_angle = acos(B_BODYn[1])*c_R2D;
		magYaw_angle   = acos(B_BODYn[2])*c_R2D;

		TM.Buffer.TM_w_BODY[0]=(int)(w_BODY[0]/c_TM_Resol_w);
		TM.Buffer.TM_w_BODY[1]=(int)(w_BODY[1]/c_TM_Resol_w);
		TM.Buffer.TM_w_BODY[2]=(int)(w_BODY[2]/c_TM_Resol_w);

		ST_normal.ST_NM_Buffer.TM_w_BODY[0] = (int)(w_BODY[0]/c_TM_Resol_w);
		ST_normal.ST_NM_Buffer.TM_w_BODY[1] = (int)(w_BODY[1]/c_TM_Resol_w);
		ST_normal.ST_NM_Buffer.TM_w_BODY[2] = (int)(w_BODY[2]/c_TM_Resol_w);

		ST_special.ST_SP_Buffer.TM_w_BODY[0] = (int)(w_BODY[0]/c_TM_Resol_w);
		ST_special.ST_SP_Buffer.TM_w_BODY[1] = (int)(w_BODY[1]/c_TM_Resol_w);
		ST_special.ST_SP_Buffer.TM_w_BODY[2] = (int)(w_BODY[2]/c_TM_Resol_w);

		TM.Buffer.TM_B_BODY[0]=(int)(B_BODY[0]/c_TM_Resol_B);
		TM.Buffer.TM_B_BODY[1]=(int)(B_BODY[1]/c_TM_Resol_B);
		TM.Buffer.TM_B_BODY[2]=(int)(B_BODY[2]/c_TM_Resol_B);

		ST_normal.ST_NM_Buffer.TM_B_BODY[0] = (int)(B_BODY[0]/c_TM_Resol_B);
		ST_normal.ST_NM_Buffer.TM_B_BODY[1] = (int)(B_BODY[1]/c_TM_Resol_B);
		ST_normal.ST_NM_Buffer.TM_B_BODY[2] = (int)(B_BODY[2]/c_TM_Resol_B);

		ST_special.ST_SP_Buffer.TM_B_BODY[0] = (int)(B_BODY[0]/c_TM_Resol_B);
		ST_special.ST_SP_Buffer.TM_B_BODY[1] = (int)(B_BODY[1]/c_TM_Resol_B);
		ST_special.ST_SP_Buffer.TM_B_BODY[2] = (int)(B_BODY[2]/c_TM_Resol_B);

		TM.Buffer.TM_Thta_BODY[0] = (int)(Thta_BODY_IMU1[0] / c_TM_Resol_DelThta);
		TM.Buffer.TM_Thta_BODY[1] = (int)(Thta_BODY_IMU1[1] / c_TM_Resol_DelThta);
		TM.Buffer.TM_Thta_BODY[2] = (int)(Thta_BODY_IMU1[2] / c_TM_Resol_DelThta);
	}
	else if (TC_boolean_u.TC_Boolean_Table.TC_IMU_Select == IMU2)  // IMU2
	{

		w_BODY[0] = w_BODY_IMU2[0];
		w_BODY[1] = w_BODY_IMU2[1];
		w_BODY[2] = w_BODY_IMU2[2];

		B_BODY[0] = B_BODY_IMU2[0];
		B_BODY[1] = B_BODY_IMU2[1];
		B_BODY[2] = B_BODY_IMU2[2];

		TM.Buffer.TM_IMU_SELECTED = TM_IMU2;
		IMU_temperature         = IMU2_DATA.IMU_Temp;

		ST_normal.ST_NM_Buffer.TM_IMU_2_Temp     = (unsigned short)(IMU2_DATA.IMU_Temp); //IMU Temperature
		ST_special.ST_SP_Buffer.TM_IMU_2_Temp     = (unsigned short)(IMU2_DATA.IMU_Temp); //IMU Temperature
		ST_normal.ST_NM_Buffer.TM_IMU_2_Diag_STS = (unsigned short)(IMU2_DATA.IMU_Diag_STS); //IMU DIAG STATUS

		if (TC_boolean_u.TC_Boolean_Table.TC_GND_Drift_Compensation_Enable_or_Disable == Enable)
		{
			w_BODY[0] = w_BODY[0] - (double)TC_drift_compensation_IMU2[0];
			w_BODY[1] = w_BODY[1] - (double)TC_drift_compensation_IMU2[1];
			w_BODY[2] = w_BODY[2] - (double)TC_drift_compensation_IMU2[2];
		}

		if (TC_boolean_u.TC_Boolean_Table.TC_EKF_Drift_Compensation_Enable_or_Disable == Enable)
		{
			w_BODY[0] = w_BODY[0] - Xk[3];
			w_BODY[1] = w_BODY[1] - Xk[4];
			w_BODY[2] = w_BODY[2] - Xk[5];
		}


		w_BODYdeg[0] = w_BODY[0] * c_R2D;
		w_BODYdeg[1] = w_BODY[1] * c_R2D;
		w_BODYdeg[2] = w_BODY[2] * c_R2D;


		if(TC_boolean_u.TC_Boolean_Table.TC_Mag_Torquer_Bias_Enable_or_Disable == Enable)
		{
			if ((TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Enable_or_Disable == Enable) || (TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Enable_or_Disable == Enable) || (TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Enable_or_Disable == Enable))  // Software..hw enable/disable
			{
				for(i_MatEq = 0; i_MatEq < 27; i_MatEq++)
				{
					if((DPM_Polarity[0] == c_DPM_Pol_LookUpTable[i_MatEq][0]) && (DPM_Polarity[1] == c_DPM_Pol_LookUpTable[i_MatEq][1]) && (DPM_Polarity[2] == c_DPM_Pol_LookUpTable[i_MatEq][2]))  // Actual polarity status from rHAL_MTR
					{
						PolCheck_LUT = i_MatEq;
						break;
					}
				}

				B_BODY[0] -= c_MagBias_act_LUT[PolCheck_LUT][0];
				B_BODY[1] -= c_MagBias_act_LUT[PolCheck_LUT][1];
				B_BODY[2] -= c_MagBias_act_LUT[PolCheck_LUT][2];

			}
		}

		B_BODY_LUT[0] = B_BODY[0];
		B_BODY_LUT[1] = B_BODY[1];
		B_BODY_LUT[2] = B_BODY[2];

		if (TC_boolean_u.TC_Boolean_Table.TC_GND_MagBias_Compensation_Enable_or_Disable == Enable)
		{
			B_BODY[0] = B_BODY[0] - (double)TC_magbias_compensation_IMU2[0];
			B_BODY[1] = B_BODY[1] - (double)TC_magbias_compensation_IMU2[1];
			B_BODY[2] = B_BODY[2] - (double)TC_magbias_compensation_IMU2[2];
		}

		if (TC_boolean_u.TC_Boolean_Table.TC_EKF_MagBias_Compensation_Enable_or_Disable == Enable)
		{
			if ((TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 0) && (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == Enable))
			{
				B_BODY[0] = B_BODY[0] - (Xk[6] * 1.0e9);
				B_BODY[1] = B_BODY[1] - (Xk[7] * 1.0e9);
				B_BODY[2] = B_BODY[2] - (Xk[8] * 1.0e9);
			}
			else
				{
					if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == Enable)
					{
						if ((f_station_tracking_enabled == 0) && (Sunlit_presence_timer > 4650))
						{
							B_BODY[0] = B_BODY[0] - (lpfb_k[0] * 1.0e9);
							B_BODY[1] = B_BODY[1] - (lpfb_k[1] * 1.0e9);
							B_BODY[2] = B_BODY[2] - (lpfb_k[2] * 1.0e9);
						}
						else// if (f_station_tracking_enabled == 1 || Sunlit_presence_timer < 9375)
						{
							B_BODY[0] = B_BODY[0] - (lpfb_k_prev[0] * 1.0e9);
							B_BODY[1] = B_BODY[1] - (lpfb_k_prev[1] * 1.0e9);
							B_BODY[2] = B_BODY[2] - (lpfb_k_prev[2] * 1.0e9);
						}
					}

				}
		}

		B_BODYtesla[0] = B_BODY[0] * 1.0E-9;
		B_BODYtesla[1] = B_BODY[1] * 1.0E-9;
		B_BODYtesla[2] = B_BODY[2] * 1.0E-9;

		Bsq = ((B_BODYtesla[0]*B_BODYtesla[0]) + (B_BODYtesla[1]*B_BODYtesla[1]) + (B_BODYtesla[2]*B_BODYtesla[2]));

		w_BODYnorm = sqrt((w_BODYdeg[0]*w_BODYdeg[0]) + (w_BODYdeg[1]*w_BODYdeg[1]) + (w_BODYdeg[2]*w_BODYdeg[2])) * c_R2D;

		rVectorNorm(B_BODY);
		B_BODYn[0] = Norm_out[0];
		B_BODYn[1] = Norm_out[1];
		B_BODYn[2] = Norm_out[2];

		magRoll_angle  = acos(B_BODYn[0])*c_R2D;
		magPitch_angle = acos(B_BODYn[1])*c_R2D;
		magYaw_angle   = acos(B_BODYn[2])*c_R2D;

		TM.Buffer.TM_w_BODY[0]=(int)(w_BODY[0]/c_TM_Resol_w);
		TM.Buffer.TM_w_BODY[1]=(int)(w_BODY[1]/c_TM_Resol_w);
		TM.Buffer.TM_w_BODY[2]=(int)(w_BODY[2]/c_TM_Resol_w);

		TM.Buffer.TM_B_BODY[0]= (int)(B_BODY[0]/c_TM_Resol_B);
		TM.Buffer.TM_B_BODY[1]= (int)(B_BODY[1]/c_TM_Resol_B);
		TM.Buffer.TM_B_BODY[2]= (int)(B_BODY[2]/c_TM_Resol_B);

		TM.Buffer.TM_Thta_BODY[0] = (int)(Thta_BODY_IMU2[0] / c_TM_Resol_DelThta);
		TM.Buffer.TM_Thta_BODY[1] = (int)(Thta_BODY_IMU2[1] / c_TM_Resol_DelThta);
		TM.Buffer.TM_Thta_BODY[2] = (int)(Thta_BODY_IMU2[2] / c_TM_Resol_DelThta);
	}
	else
	{

	}
}

static double rTheta_Limit(const double var_theta_lim)
{
	if(var_theta_lim >= 9.424777961)					        // (9.424777961 rad = 540 deg)
	{
        inter_theta = var_theta_lim - 6.283185307;	        // (6.283185307 rad = 360 deg)
	}

	else if(var_theta_lim <= -9.424777961)					// (9.424777961 rad = 540 deg)
	{
        inter_theta = var_theta_lim + 6.283185307;	        // (6.283185307 rad = 360 deg)
	}

	else
	{
		//
	}
	return inter_theta;
}

void rBDOT_Computation(void)
{
    if (CB_BDOT_Computation == Enable)
    {
        BDOT_Counter++;
        if(BDOT_Counter == ADCS_TC_data_command_Table.TC_Det_BDOT_Compute_Count)
        {
            Bpresent[0] = B_BODY[0];
            Bpresent[1] = B_BODY[1];
            Bpresent[2] = B_BODY[2];

            BDOT_deltaT = ((double)(ADCS_TC_data_command_Table.TC_Det_BDOT_Compute_Count-ADCS_TC_data_command_Table.TC_Det_Bprev_Count))*c_MaC;

            BDOT[0] = (Bpresent[0] - Bprev[0])/(BDOT_deltaT);
            BDOT[1] = (Bpresent[1] - Bprev[1])/(BDOT_deltaT);
            BDOT[2] = (Bpresent[2] - Bprev[2])/(BDOT_deltaT);

            BDOTnorm = ((BDOT[0]*BDOT[0]) + (BDOT[1]*BDOT[1]) + (BDOT[2]*BDOT[2]));

            TM.Buffer.TM_B_DOT[0] = (int)(BDOT[0] / c_TM_Resol_B);
			TM.Buffer.TM_B_DOT[1] = (int)(BDOT[1] / c_TM_Resol_B);
			TM.Buffer.TM_B_DOT[2] = (int)(BDOT[2] / c_TM_Resol_B);



            BDOT_Counter = 0;
        }

        else if(BDOT_Counter == ADCS_TC_data_command_Table.TC_Det_Bprev_Count)
        {
            Bprev[0] = B_BODY[0];
            Bprev[1] = B_BODY[1];
            Bprev[2] = B_BODY[2];
        }

        else
        {
            //
        }
    }
}

// Sun Sensor

void rSunSensorDataProcessing(void)
{
	if (f_Sunlit_Presence == 1)
	{
		rSS_Read_Data(ADC_Buffer);

		for (i_MatEq = 0; i_MatEq < 8;i_MatEq++)
		{
			SSMAIN_ARRAY[i_MatEq] = SS_Data[i_MatEq];
		}

		for (i_MatEq = 8; i_MatEq < 15;i_MatEq++)
		{
			SSREDT_ARRAY[i_MatEq-8] = SS_Data[i_MatEq];
		}

		rSunSensorVectorComp(SSMAIN_ARRAY,&SS_Main_2Exe_DB);
		SB_MAIN[0] = SS_prcd_data[0];
		SB_MAIN[1] = SS_prcd_data[1];
		SB_MAIN[2] = SS_prcd_data[2];

		rSunSensorVectorComp(SSREDT_ARRAY,&SS_Main_2Exe_DB);
		SB_RED[0] = SS_prcd_data[0];
		SB_RED[1] = SS_prcd_data[1];
		SB_RED[2] = SS_prcd_data[2];

		TM.Buffer.TM_S_BODY_Main[0] = (int)(SB_MAIN[0]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Main[1] = (int)(SB_MAIN[1]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Main[2] = (int)(SB_MAIN[2]/4.65661287E-7);

		ST_normal.ST_NM_Buffer.TM_S_BODY_Main[0] = (int)(SB_MAIN[0]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_S_BODY_Main[1] = (int)(SB_MAIN[1]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_S_BODY_Main[2] = (int)(SB_MAIN[2]/4.65661287E-7);

		//for special_str s_body datatype(int) as to be changed to datatype(short)

		ST_special.ST_SP_Buffer.TM_S_BODY_Main[0] = (int)(SB_MAIN[0]/4.65661287E-7);
		ST_special.ST_SP_Buffer.TM_S_BODY_Main[1] = (int)(SB_MAIN[1]/4.65661287E-7);
		ST_special.ST_SP_Buffer.TM_S_BODY_Main[2] = (int)(SB_MAIN[2]/4.65661287E-7);

		//--------------------------------------------------------------------

		/*TM.Buffer.TM_S_BODY_Red[0] = (int)(SB_RED[0]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Red[1] = (int)(SB_RED[1]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Red[2] = (int)(SB_RED[2]/4.65661287E-7);*/

		if (TC_boolean_u.TC_Boolean_Table.TC_SS_Cells_Sel == TC_Main_Cells)
		{
			S_BODY[0] = SB_MAIN[0];
			S_BODY[1] = SB_MAIN[1];
			S_BODY[2] = SB_MAIN[2];
		}
		else
		{
			S_BODY[0] = SB_RED[0];
			S_BODY[1] = SB_RED[1];
			S_BODY[2] = SB_RED[2];
		}

		rVectorNorm(S_BODY);
		S_BODYn[0] = Norm_out[0];
		S_BODYn[1] = Norm_out[1];
		S_BODYn[2] = Norm_out[2];

		//Roll and Yaw angle errors' computation

		if(fabs(S_BODYn[1]) <= c_dividebyzerovalue)
		{
			S_BODYn[1] = c_dividebyzerovalue;
		}

		Roll_ang_err = atan2(S_BODYn[2], -S_BODYn[1]);
		Yaw_ang_err = ((-1.0) * atan2(S_BODYn[0], -S_BODYn[1]));

		TM.Buffer.TM_SunSens_Roll_Error = (int)(Roll_ang_err/0.01);
		ST_normal.ST_NM_Buffer.TM_SunSens_Roll_Error = (int)(Roll_ang_err/0.01);

		TM.Buffer.TM_SunSens_Yaw_Error = (int)(Yaw_ang_err/0.01);
		ST_normal.ST_NM_Buffer.TM_SunSens_Yaw_Error = (int)(Yaw_ang_err/0.01);

		//Angle Deviation Computation
		Ang_Deviation = acos((-1.0) * (S_BODYn[1]));
		TM.Buffer.TM_SunSens_Pitch_Error = (int)Ang_Deviation;
		ST_normal.ST_NM_Buffer.TM_SunSens_Pitch_Error = (int)Ang_Deviation;


		if(fabs(Ang_Deviation) < c_AngDev_SAMtransit_thrsld)
		{
			SunNPP_SAMtransit_counter++;

			if(SunNPP_SAMtransit_counter >= SunNPP_SAMtransit_count_limit)
			{
				SunNPP_SAMtransit = True;
				SunNPP_SAMtransit_counter = SunNPP_SAMtransit_count_limit;
			}

			else
			{
				SunNPP_SAMtransit = False;
			}
		}
		else
		{
			SunNPP_SAMtransit = False;

			SunNPP_SAMtransit_counter = 0;
		}
	}
	else
	{
		S_BODY[0] = 0.0;
		S_BODYn[0] = 0.0;
		S_BODY[1] = 0.0;
		S_BODYn[1] = 0.0;
		S_BODY[2] = 0.0;
		S_BODYn[2] = 0.0;
	}
}

static void rSunSensorVectorComp(const double SS_ARRAY[8], struct SunSensor_Database *SS_2Exe_Addr)
{



	SS_M1 = SS_ARRAY[0];			//SSCAT11//SSCAT51:://SS M1//SS M9
	SS_M2 = SS_ARRAY[1];			//SSCAT12//SSCAT52:://SS M2//SS M10
	SS_M3 = SS_ARRAY[2];			//SSCAT21//SSCAT61:://SS M3//SS M11
	SS_M4 = SS_ARRAY[3];			//SSCAT22//SSCAT62:://SS M4//SS M12
	SS_M5 = SS_ARRAY[4];			//SSCAT31//SSCAT71:://SS M5//SS M13
	SS_M6 = SS_ARRAY[5];			//SSCAT32//SSCAT72:://SS M6//SS M14
	SS_M7 = SS_ARRAY[6];			//SSCAT41//SSCAT81:://SS M7//SS M15
	SS_M8 = SS_ARRAY[7];				//SSCAT42//SSCAT82:://SS M8//SS M16

	if (SS_M1 > SS_M3)
	{
		SS_M3 = 0.0;
	}
	else
	{
		SS_M1 = 0.0;
	}

	if (SS_M2 > SS_M4)
	{
		SS_M4 = 0.0;
	}
	else
	{
		SS_M2 = 0.0;
	}

	if (SS_M5 > SS_M6)
	{
		SS_M6 = 0.0;
	}
	else
	{
		SS_M5 = 0.0;
	}

	// Different deployment selections
	 if(panel_deploy_sts == TC_Not_Deployed)//Both the panels are Not deployed
	{
		SC1 = (SS_M7);
		SC3 = (SS_M8);

		SC1ImaxF = SS_2Exe_Addr->DB_Imax_RPND;
		SC3ImaxF = SS_2Exe_Addr->DB_Imax_RNND;
	}

	 else if(panel_deploy_sts == TC_All_Deployed)//Both the panels deployed
	{
		SC1 = (SS_M1);
		SC3 = (SS_M3);

		SC1ImaxF = SS_2Exe_Addr->DB_Imax_RPD;
		SC3ImaxF = SS_2Exe_Addr->DB_Imax_RND;
	}

	 else if(panel_deploy_sts == TC_PosR_Deployed)//Positive Roll Panel deployed
	{
		SC1 = (SS_M1);
		SC3 = (SS_M8);

		SC1ImaxF = SS_2Exe_Addr->DB_Imax_RPD;
		SC3ImaxF = SS_2Exe_Addr->DB_Imax_RNND;
	}

	else // elseif(TC_panel_Deployment == TC_NegR_Deployed) Negative Roll panel deployed
	{
		SC1 = (SS_M7);
		SC3 = (SS_M3);

		SC1ImaxF = SS_2Exe_Addr->DB_Imax_RPND;
		SC3ImaxF = SS_2Exe_Addr->DB_Imax_RND;
	}

	//Assignment of variables for different sensors (Yaw and Pitch)

	SC2 = SS_M2;

	if ((TC_boolean_u.TC_Boolean_Table.TC_H8Backup_H4_Main == 1) || (TC_boolean_u.TC_Boolean_Table.TC_H8Backup_H4_Redt == 1))
	{
		SC4 = SS_M8;
	}
	else if ((TC_boolean_u.TC_Boolean_Table.TC_H7Backup_H4_Main == 1) || (TC_boolean_u.TC_Boolean_Table.TC_H7Backup_H4_Redt == 1))
	{
		SC4 = SS_M7;
	}
	else
	{
		SC4 = SS_M4;
	}

	//SC4 = SS_M4;
	SC5 = SS_M5;
	SC6 = SS_M6;

	SC2ImaxF = SS_2Exe_Addr->DB_Imax_PP;
	SC4ImaxF = SS_2Exe_Addr->DB_Imax_PN;
	SC5ImaxF = SS_2Exe_Addr->DB_Imax_YP;
	SC6ImaxF = SS_2Exe_Addr->DB_Imax_YN;

	//Calculation of Azimuth and Elevation of Sun
	if((SC1  > c_SSThrsld) && (SC2 > c_SSThrsld) && (SC5 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------

		SC1 = (SS_2Exe_Addr->DB_misaln_cor125[0][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor125[0][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor125[0][2]*SC5);
		SC2 = (SS_2Exe_Addr->DB_misaln_cor125[1][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor125[1][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor125[1][2]*SC5);
		SC5 = (SS_2Exe_Addr->DB_misaln_cor125[2][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor125[2][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor125[2][2]*SC5);

		//-------------------------------------------------------------------------

		comb1 = (((SC2 * SC2ImaxF)- (SC1 * SC1ImaxF))/((SC2 * SC2ImaxF)+ (SC1 * SC1ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC5 * SC5ImaxF)- (SC1 * SC1ImaxF))/((SC5 * SC5ImaxF)+ (SC1 * SC1ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = atan(cos(az1)*tan(ele1));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b1256[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 1;

	}

	if((SC1  > c_SSThrsld) && (SC2 > c_SSThrsld) && (SC6 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------

		SC1 = (SS_2Exe_Addr->DB_misaln_cor126[0][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor126[0][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor126[0][2]*SC6);
		SC2 = (SS_2Exe_Addr->DB_misaln_cor126[1][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor126[1][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor126[1][2]*SC6);
		SC6 = (SS_2Exe_Addr->DB_misaln_cor126[2][0]*SC1) + (SS_2Exe_Addr->DB_misaln_cor126[2][1]*SC2) + (SS_2Exe_Addr->DB_misaln_cor126[2][2]*SC6);

		//-------------------------------------------------------------------------

		comb1 = (((SC2 * SC2ImaxF)- (SC1 * SC1ImaxF))/((SC2 * SC2ImaxF)+ (SC1 * SC1ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC6 * SC6ImaxF)- (SC1 * SC1ImaxF))/((SC6 * SC6ImaxF)+ (SC1 * SC1ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = ((-1.0) * atan(cos(az1)*tan(ele1)));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b1256[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 2;
	}

	if((SC3  > c_SSThrsld) && (SC2 > c_SSThrsld) && (SC5 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------

		SC2 = (SS_2Exe_Addr->DB_misaln_cor325[0][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor325[0][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor325[0][2]*SC5);
		SC3 = (SS_2Exe_Addr->DB_misaln_cor325[1][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor325[1][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor325[1][2]*SC5);
		SC5 = (SS_2Exe_Addr->DB_misaln_cor325[2][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor325[2][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor325[2][2]*SC5);


		//-------------------------------------------------------------------------

		comb1 = (((SC3 * SC3ImaxF)- (SC2 * SC2ImaxF))/((SC3 * SC3ImaxF)+ (SC2 * SC2ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC5 * SC5ImaxF)- (SC2 * SC2ImaxF))/((SC5 * SC5ImaxF)+ (SC2 * SC2ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = atan(cos(az1)*tan(ele1));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b2356[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 3;
	}

	if((SC3  > c_SSThrsld) && (SC2 > c_SSThrsld) && (SC6 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------
		SC2 = (SS_2Exe_Addr->DB_misaln_cor326[0][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor326[0][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor326[0][2]*SC6);
		SC3 = (SS_2Exe_Addr->DB_misaln_cor326[1][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor326[1][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor326[1][2]*SC6);
		SC6 = (SS_2Exe_Addr->DB_misaln_cor326[2][0]*SC2) + (SS_2Exe_Addr->DB_misaln_cor326[2][1]*SC3) + (SS_2Exe_Addr->DB_misaln_cor326[2][2]*SC6);

		//-------------------------------------------------------------------------

		comb1 = (((SC3 * SC3ImaxF)- (SC2 * SC2ImaxF))/((SC3 * SC3ImaxF)+ (SC2 * SC2ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC6 * SC6ImaxF)- (SC2 * SC2ImaxF))/((SC6 * SC6ImaxF)+ (SC2 * SC2ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = ((-1.0) * atan(cos(az1)*tan(ele1)));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b2356[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 4;
	}

	if((SC3  > c_SSThrsld) && (SC4 > c_SSThrsld) && (SC5 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------
		SC3 = (SS_2Exe_Addr->DB_misaln_cor345[0][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[0][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[0][2]*SC5);
		SC4 = (SS_2Exe_Addr->DB_misaln_cor345[1][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[1][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[1][2]*SC5);
		SC5 = (SS_2Exe_Addr->DB_misaln_cor345[2][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[2][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[2][2]*SC5);


		//-------------------------------------------------------------------------

		comb1 = (((SC4 * SC4ImaxF)- (SC3 * SC3ImaxF))/((SC4 * SC4ImaxF)+ (SC3 * SC3ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC5 * SC5ImaxF)- (SC3 * SC3ImaxF))/((SC5 * SC5ImaxF)+ (SC3 * SC3ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = atan(cos(az1)*tan(ele1));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b3456[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 5;
	}

	if((SC3  > c_SSThrsld) && (SC4 > c_SSThrsld) && (SC6 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------
		SC3 = (SS_2Exe_Addr->DB_misaln_cor345[0][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[0][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[0][2]*SC6);
		SC4 = (SS_2Exe_Addr->DB_misaln_cor345[1][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[1][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[1][2]*SC6);
		SC6 = (SS_2Exe_Addr->DB_misaln_cor345[2][0]*SC3) + (SS_2Exe_Addr->DB_misaln_cor345[2][1]*SC4) + (SS_2Exe_Addr->DB_misaln_cor345[2][2]*SC6);


		//-------------------------------------------------------------------------

		comb1 = (((SC4 * SC4ImaxF)- (SC3 * SC3ImaxF))/((SC4 * SC4ImaxF)+ (SC3 * SC3ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC6 * SC6ImaxF)- (SC3 * SC3ImaxF))/((SC6 * SC6ImaxF)+ (SC3 * SC3ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = ((-1.0) * atan(cos(az1)*tan(ele1)));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b3456[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 6;
	}

	if((SC1  > c_SSThrsld) && (SC4 > c_SSThrsld) && (SC5 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------
		SC4 = (SS_2Exe_Addr->DB_misaln_cor145[0][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[0][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[0][2]*SC5);
		SC1 = (SS_2Exe_Addr->DB_misaln_cor145[1][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[1][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[1][2]*SC5);
		SC5 = (SS_2Exe_Addr->DB_misaln_cor145[2][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[2][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[2][2]*SC5);

		//-------------------------------------------------------------------------

		comb1 = (((SC1 * SC1ImaxF)- (SC4 * SC4ImaxF))/((SC1 * SC1ImaxF)+ (SC4 * SC4ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC5 * SC5ImaxF)- (SC4 * SC4ImaxF))/((SC5 * SC5ImaxF)+ (SC4 * SC4ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = atan(cos(az1)*tan(ele1));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b4156[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 8;
	}

	if((SC1  > c_SSThrsld) && (SC4 > c_SSThrsld) && (SC6 > c_SSThrsld))
	{
		//-------------------------------------------------------------------------
		SC4 = (SS_2Exe_Addr->DB_misaln_cor145[0][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[0][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[0][2]*SC6);
		SC1 = (SS_2Exe_Addr->DB_misaln_cor145[1][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[1][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[1][2]*SC6);
		SC6 = (SS_2Exe_Addr->DB_misaln_cor145[2][0]*SC4) + (SS_2Exe_Addr->DB_misaln_cor145[2][1]*SC1) + (SS_2Exe_Addr->DB_misaln_cor145[2][2]*SC6);


		//-------------------------------------------------------------------------

		comb1 = (((SC1 * SC1ImaxF)- (SC4 * SC4ImaxF))/((SC1 * SC1ImaxF)+ (SC4 * SC4ImaxF)));
		temp11 = atan(comb1);
		az1 = ((45.0*c_D2R) + temp11);

		comb2 = (((SC6 * SC6ImaxF)- (SC4 * SC4ImaxF))/((SC6 * SC6ImaxF)+ (SC4 * SC4ImaxF)));
		temp12 = atan(comb2);
		ele1 = ((45.0*c_D2R) + temp12);

		ele2 = ((-1.0) * atan(cos(az1)*tan(ele1)));

		//Assigning Sensor to Body Transformation Matrix
		for(i_MatEq = 0; i_MatEq < 3; i_MatEq++)
		{
			for(j_MatEq = 0; j_MatEq < 3; j_MatEq++)
			{
				ss2b[i_MatEq][j_MatEq] = c_ss2b4156[i_MatEq][j_MatEq];
			}
		}
		sun_quadrant = 8;
	}

	//sunsensor data in Sensor frame
	sun_sf[0] = (cos(az1)*cos(ele2));
	sun_sf[1] = (sin(az1)*cos(ele2));
	sun_sf[2] = sin(ele2);

	rMatMul3x1(ss2b, sun_sf);
	//Conversion from sensor frame to body frame (ss2b ---> (sun sensor to body frame))
	SS_prcd_data[0] = Matout31[0];
	SS_prcd_data[1] = Matout31[1];
	SS_prcd_data[2] = Matout31[2];

}

static void rSS_Read_Data(const unsigned long int *ADC_Data_Addr)
{
	for(inter_sunsensor_i =0;inter_sunsensor_i<=15;inter_sunsensor_i++){
		SS_Data[inter_sunsensor_i] = (double)(ADC_Data_Addr[14 + inter_sunsensor_i] * 5.92145E-3);
	}
}
