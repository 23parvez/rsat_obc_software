#include <stdio.h>
#include <math.h>

#include "Global.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"

#include "adcs_ADandEst.h"
#include "adcs_CommonRoutines.h"
#include "adcs_Constants.h"
#include "adcs_GPS_OD.h"
#include "adcs_LinearController.h"
#include "adcs_ModePreProcs.h"
#include "adcs_RefComp.h"
#include "adcs_SensorDataProcs.h"
#include "adcs_VarDeclarations.h"

static void rWheel_Auto_Reconfiguration(void);
static void rWheel_Spin_updown(void);
static void rSpeedBasedMomentumDumping(void);

void rLinearController(void)
{
	double temp_data_1[3];
	unsigned int temp_data_2[3];

    if (CB_LinearController == Enable)
    {
    	if ((TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 1) && (TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable == 1))
		{
			Tc[0] = ((TC_KP[0]*Qerror[0]) + (TC_KR[0]*(w_k[0]-w_REF[0])));
			Tc[1] = ((TC_KP[1]*Qerror[1]) + (TC_KR[1]*(w_k[1]-w_REF[1])));
			Tc[2] = ((TC_KP[2]*Qerror[2]) + (TC_KR[2]*(w_k[2]-w_REF[2])));
		}
		else
		{
			Tc[0] = ((TC_KP[0]*Qerror[0]) + (TC_KR[0]*(w_BODY[0]-w_REF[0])));
			Tc[1] = ((TC_KP[1]*Qerror[1]) + (TC_KR[1]*(w_BODY[1]-w_REF[1])));
			Tc[2] = ((TC_KP[2]*Qerror[2]) + (TC_KR[2]*(w_BODY[2]-w_REF[2])));
		}

        ST_special.ST_SP_Buffer.cntrl_torque[0] = Tc[0];
        ST_special.ST_SP_Buffer.cntrl_torque[1] = Tc[1];
        ST_special.ST_SP_Buffer.cntrl_torque[2] = Tc[2];

        if (fabs(Tc[0]) >= TC_T_RW_MAX )
        {
            Tc[0] = rsign_f(Tc[0])*TC_T_RW_MAX;
        }


        if (fabs(Tc[1]) >= TC_T_RW_MAX )
        {
            Tc[1] = rsign_f(Tc[1])*TC_T_RW_MAX;
        }


        if (fabs(Tc[2]) >= TC_T_RW_MAX )
        {
            Tc[2] = rsign_f(Tc[2])*TC_T_RW_MAX;
        }

        rWheel_Auto_Reconfiguration();

        if ((Wheel_Config == 0) || (Wheel_Config == 1))
        {

            rMatMul43x1(B2wh_mat,Tc);
            T_RW[0] = Matout431[0];
            T_RW[1] = Matout431[1];
            T_RW[2] = Matout431[2];
            T_RW[3] = Matout431[3];

            RWSpeed_RAD[0] = (double)RW_Wheel_Speed[RWHEEL0];
            RWSpeed_RAD[1] = (double)RW_Wheel_Speed[RWHEEL1];
            RWSpeed_RAD[2] = (double)RW_Wheel_Speed[RWHEEL2];
            RWSpeed_RAD[3] = (double)RW_Wheel_Speed[RWHEEL3];

			H_wh[0] = (c_MOI_wh)* RWSpeed_RAD[0];
			H_wh[1] = (c_MOI_wh)* RWSpeed_RAD[1];
			H_wh[2] = (c_MOI_wh)* RWSpeed_RAD[2];
			H_wh[3] = (c_MOI_wh)* RWSpeed_RAD[3];

			rMatMul34x1(wh2B_mat,H_wh);
			HB[0] = Matout341[0];
			HB[1] = Matout341[1];
			HB[2] = Matout341[2];

			temp_data_1[0] = HB[0];
			temp_data_1[1] = HB[1];
			temp_data_1[2] = HB[2];

			temp_data_2[0] = (unsigned int)((temp_data_1[0] * 1000)/10);
			temp_data_2[1] = (unsigned int)((temp_data_1[1] * 1000)/10);
			temp_data_2[2] = (unsigned int)((temp_data_1[2] * 1000)/10);


			ST_normal.ST_NM_Buffer.ang_momtm[0] = (unsigned short)temp_data_2[0];
			ST_normal.ST_NM_Buffer.ang_momtm[1] = (unsigned short)temp_data_2[1];
			ST_normal.ST_NM_Buffer.ang_momtm[2] = (unsigned short)temp_data_2[2];

            RWSpeed_RPM[0] = RWSpeed_RAD[0] * c_RADps2RPM;
            RWSpeed_RPM[1] = RWSpeed_RAD[1] * c_RADps2RPM;
            RWSpeed_RPM[2] = RWSpeed_RAD[2] * c_RADps2RPM;
            RWSpeed_RPM[3] = RWSpeed_RAD[3] * c_RADps2RPM;

            rSpeedBasedMomentumDumping();
            rWheel_Spin_updown();

            if (CB_Wheel_OverSpeed_TorqueCutOff == Enable)
            {
                if  (((RWSpeed_RPM[0]) >= TC_wh_speed_cutoff) && (T_RW[0] > 0.0))
                {
                    T_RW[0] = 0.0;
                }

                if (((RWSpeed_RPM[0]) <= -TC_wh_speed_cutoff) && (T_RW[0] < 0.0))
                {
                    T_RW[0] = 0.0;
                }

                if  (((RWSpeed_RPM[1]) >= TC_wh_speed_cutoff) && (T_RW[1] > 0.0))
                {
                    T_RW[1] = 0.0;
                }

                if (((RWSpeed_RPM[1]) <= -TC_wh_speed_cutoff) && (T_RW[1] < 0.0))
                {
                    T_RW[1] = 0.0;
                }

                if  (((RWSpeed_RPM[2]) >= TC_wh_speed_cutoff) && (T_RW[2] > 0.0))
                {
                    T_RW[2] = 0.0;
                }

                if (((RWSpeed_RPM[2]) <= -TC_wh_speed_cutoff) && (T_RW[2] < 0.0))
                {
                    T_RW[2] = 0.0;
                }

                if  (((RWSpeed_RPM[3]) >= TC_wh_speed_cutoff) && (T_RW[3] > 0.0))
                {
                    T_RW[3] = 0.0;
                }

                if (((RWSpeed_RPM[3]) <= -TC_wh_speed_cutoff) && (T_RW[3] < 0.0))
                {
                    T_RW[3] = 0.0;
                }
            }

            if (fabs(T_RW[0]) >= TC_T_RW_MAX )
            {
                T_RW[0] = rsign_f(T_RW[0])*TC_T_RW_MAX;
            }


            if (fabs(T_RW[1]) >= TC_T_RW_MAX )
            {
                T_RW[1] = rsign_f(T_RW[1])*TC_T_RW_MAX;
            }


            if (fabs(T_RW[2]) >= TC_T_RW_MAX )
            {
                T_RW[2] = rsign_f(T_RW[2])*TC_T_RW_MAX;
            }


            if (fabs(T_RW[3]) >= TC_T_RW_MAX )
            {
                T_RW[3] = rsign_f(T_RW[3])*TC_T_RW_MAX;
            }

            del_Vw[0] = T_RW[0] * (c_MiC/c_MOI_wh);
			del_Vw[1] = T_RW[1] * (c_MiC/c_MOI_wh);
			del_Vw[2] = T_RW[2] * (c_MiC/c_MOI_wh);
			del_Vw[3] = T_RW[3] * (c_MiC/c_MOI_wh);

			TM.Buffer.TM_RW_DeltaSpeed[0] = (int)(del_Vw[0] * (c_RADps2RPM/c_TM_RW_Resol));
			TM.Buffer.TM_RW_DeltaSpeed[1] = (int)(del_Vw[1] * (c_RADps2RPM/c_TM_RW_Resol));
			TM.Buffer.TM_RW_DeltaSpeed[2] = (int)(del_Vw[2] * (c_RADps2RPM/c_TM_RW_Resol));
			TM.Buffer.TM_RW_DeltaSpeed[3] = (int)(del_Vw[3] * (c_RADps2RPM/c_TM_RW_Resol));

        }
    }
}

void rDutyCycleGeneration(void)
{
    if(CB_DutyCycleGeneration == Enable)
    {
        if (DutyCycleGenEnable == 1)
        {
            TorquerDutyCycle[0] = (unsigned int)(fabs(DPM[0])/c_DPMMAX);
            Ton[0] = floor(TorquerDutyCycle[0] * MTR_ActuationCycle);
            //Toff[0] = MTR_ActuationCycle – (Ton[0]);

            TorquerDutyCycle[1] = (unsigned int)(fabs(DPM[1])/c_DPMMAX);
            Ton[1] = floor(TorquerDutyCycle[1] * MTR_ActuationCycle);
            //Toff[1] = MTR_ActuationCycle – (Ton[1]);

            TorquerDutyCycle[2] = (unsigned int)(fabs(DPM[2])/c_DPMMAX);
            Ton[2] = floor(TorquerDutyCycle[2] * MTR_ActuationCycle);
            //Toff[2] = MTR_ActuationCycle – (Ton[2]);
            DutyCycleGenEnable = 0;
        }
    }
    else
	{
		Ton[0] = 0;
		Ton[1] = 0;
		Ton[2] = 0;
	}
}

void rAngularMomentumDumping(void)
{
	double angmom_h[3];
    if (CB_AngularMomentumDumping == Enable)
    {
        if(TC_boolean_u.TC_Boolean_Table.TC_mom_dumping_ang_mom_based == Enable)
        {
            if ((f_Momentum_Dumping == Enable) && (f_Sunlit_Presence == 1) && (f_aft_statn_wait == 0))
            {
                if (dumping_on == 0)
                {
                    if ((fabs(HB[0]) > TC_AngMomDump_Thrsld) || (fabs(HB[1]) > TC_AngMomDump_Thrsld) || (fabs(HB[2]) > TC_AngMomDump_Thrsld))
                    {
                        dumping_on = 1;
                    }
                }
                else //if (dumping_on == 1)
                {
                    if ((fabs(HB[0]) > TC_Hmin) || (fabs(HB[1]) > TC_Hmin) || (fabs(HB[2]) > TC_Hmin))
                    {
                        if (fabs(HB[0]) > TC_Hmin)
                        {
                            delta_HB[0] = (HB[0] - (rsign_f(HB[0])*TC_Hmin));
                        }

                        else
                        {
                            delta_HB[0] = 0.0;
                        }


                        if (fabs(HB[1]) > TC_Hmin)
                        {
                            delta_HB[1] = (HB[1] - (rsign_f(HB[1])*TC_Hmin));
                        }

                        else
                        {
                            delta_HB[1] = 0.0;
                        }


                        if (fabs(HB[2]) > TC_Hmin)
                        {
                            delta_HB[2] = (HB[2] - (rsign_f(HB[2])*TC_Hmin));
                        }

                        else
                        {
                            delta_HB[2] = 0.0;
                        }

                        delta_HB[0] = -delta_HB[0];
                        delta_HB[1] = -delta_HB[1];
                        delta_HB[2] = -delta_HB[2];

                        rCross_Product(B_BODYtesla,delta_HB);
                        DPM[0] = Cross_Product[0];
                        DPM[1] = Cross_Product[1];
                        DPM[2] = Cross_Product[2];

                        if(fabs(Bsq) <= c_dividebyzerovalue)
                        {
                            Bsq = c_dividebyzerovalue;
                        }

                        DPM[0] = TC_momentum_dumping_gain * (DPM[0] /Bsq);
                        DPM[1] = TC_momentum_dumping_gain * (DPM[1] /Bsq);
                        DPM[2] = TC_momentum_dumping_gain * (DPM[2] /Bsq);

                        TM.Buffer.TM_DPM[0] = (int)(DPM[0] / 4.65661287E-8);
						TM.Buffer.TM_DPM[1] = (int)(DPM[1] / 4.65661287E-8);
						TM.Buffer.TM_DPM[2] = (int)(DPM[2] / 4.65661287E-8);

                        DutyCycleGenEnable = 1;
                        tor_counter = 0;
                        tor_counterk = 0;
                        TorquerPolaritySetFlag = 1;
                    }
                    else
                    {
                        dumping_on = 0;
                        TorquerPolaritySetFlag = 0;
						DutyCycleGenEnable = 0;
						DPM[0] = 0.0;
						DPM[1] = 0.0;
						DPM[2] = 0.0;
                    }
                }
            }
            else
            {
            	TorquerPolaritySetFlag = 0;
				DutyCycleGenEnable = 0;
				DPM[0] = 0.0;
				DPM[1] = 0.0;
				DPM[2] = 0.0;
            }
            rTorquer_Polarity_Check();
        }
    }
}

void rTorquer_Polarity_Check(void)
{
    if(CB_Torquer_Polarity_Check == 1)
    {
    	DPM_Pol_prev[0] = DPM_Polarity[0];
		DPM_Pol_prev[1] = DPM_Polarity[1];
		DPM_Pol_prev[2] = DPM_Polarity[2];

		if (TorquerPolaritySetFlag == 1)
		{
			if(TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Enable_or_Disable == 1) // 1 - Enable, 0 - Disable
			{
				if(DPM[0] == 0.0)
				{
					DPM_Polarity[0] = No_Current;
				}
				else if(DPM[0] > 0.0)
				{
					DPM_Polarity[0] = Cur_Positive;

				}
				else
				{
					DPM_Polarity[0] = Cur_Negative;
				}
			}
			else
			{
				DPM[0] = 0.0;
				DPM_Polarity[0]  = No_Current;
			}

			if(TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Enable_or_Disable == 1)
			{
				if(DPM[1] == 0.0)
				{
					DPM_Polarity[1] = No_Current;
				}
				else if(DPM[1] > 0.0)
				{
					DPM_Polarity[1] = Cur_Positive;
				}
				else
				{
					DPM_Polarity[1] = Cur_Negative;
				}
			}
			else
			{
				DPM[1] = 0.0;
				DPM_Polarity[1] = No_Current;
			}

			if(TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Enable_or_Disable ==1)
			{
				if(DPM[2] == 0.0)
				{
					DPM_Polarity[2] = No_Current;
				}
				else if(DPM[2] > 0.0)
				{
					DPM_Polarity[2] = Cur_Positive;
				}
				else
				{
					DPM_Polarity[2] = Cur_Negative;
				}
			}
			else
			{
				DPM[2] = 0.0;
				DPM_Polarity[2] = No_Current;
			}
		}
		else
		{
			DPM_Polarity[0] = No_Current;
			DPM_Polarity[1] = No_Current;
			DPM_Polarity[2] = No_Current;
		}
    }
}


static void rSpeedBasedMomentumDumping(void)
{
    if (CB_SpeedBasedMomentumDumping == Enable)
    {
        //TC_SpeedAngularMomemtumDumping = 1;
        if(TC_boolean_u.TC_Boolean_Table.TC_Speed_Dumping == 1)
        {
            for(i_lict=0; i_lict<4; i_lict++)
            {
                if(TC_SpeedDumpTime <= 0.0)
                {
                    TC_SpeedDumpTime = 921.6;
                }

                if ((RWSpeed_RPM[i_lict] > TC_max_whsp_spdump) && (check_dump_wh[i_lict] == 1))
                {
                    check_dump_wh[i_lict] = 0;

                    T_RW_sdump[i_lict] = (-TC_SpeedDumpLimit*c_Pi/30.0)*(c_MOI_wh/TC_SpeedDumpTime); //TC_SpeedDumpLimit = 500
                }
                if ((RWSpeed_RPM[i_lict] < TC_min_whsp_spdump) && (check_dump_wh[i_lict] == 1))
                {
                    check_dump_wh[i_lict] = 0;
                    T_RW_sdump[i_lict] = (TC_SpeedDumpLimit*c_Pi/30.0)*(c_MOI_wh/TC_SpeedDumpTime);
                }

                if (check_dump_wh[i_lict] == 0)
                {
                    wh_sdump_start[i_lict] = wh_sdump_start[i_lict] + 1;
                    if (wh_sdump_start[i_lict] >= (int)(TC_SpeedDumpTime/c_MaC))
                    {
                        T_RW_sdump[i_lict] = 0.0;
                        wh_sdump_start[i_lict] = 0;
                        check_dump_wh[i_lict] = 1;
                    }
                }
            }
//            rMatMul34x1(wh2B_mat, T_RW_sdump);
//            TB_sMD[0] = Matout341[0];
//            TB_sMD[1] = Matout341[1];
//            TB_sMD[2] = Matout341[2];

            if ((check_dump_wh[0] == 0) || (check_dump_wh[1] == 0) || (check_dump_wh[2] == 0) || (check_dump_wh[3] == 0))
            {
				u_parl[0] = (B_BODYn[0]*B_BODYn[0]*Tc[0]) + (B_BODYn[0]*B_BODYn[1]*Tc[1]) + (B_BODYn[0]*B_BODYn[2]*Tc[2]);
				u_parl[1] = (B_BODYn[1]*B_BODYn[0]*Tc[0]) + (B_BODYn[1]*B_BODYn[1]*Tc[1]) + (B_BODYn[1]*B_BODYn[2]*Tc[2]);
				u_parl[2] = (B_BODYn[2]*B_BODYn[0]*Tc[0]) + (B_BODYn[2]*B_BODYn[1]*Tc[1]) + (B_BODYn[1]*B_BODYn[2]*Tc[2]);

				rMatMul43x1(B2wh_mat,u_parl);
				T_RW[0] = Matout431[0];
				T_RW[1] = Matout431[1];
				T_RW[2] = Matout431[2];
				T_RW[3] = Matout431[3];

				T_RW[0] = T_RW[0] + T_RW_sdump[0];
				T_RW[1] = T_RW[1] + T_RW_sdump[1];
				T_RW[2] = T_RW[2] + T_RW_sdump[2];
				T_RW[3] = T_RW[3] + T_RW_sdump[3];

				rMatMul34x1(wh2B_mat,T_RW);
				T_RWB[0] = Matout341[0];
				T_RWB[1] = Matout341[1];
				T_RWB[2] = Matout341[2];

			    u_perp[0] = (((B_BODYn[2]*B_BODYn[2])+(B_BODYn[1]*B_BODYn[1]))*(-T_RWB[0] + Tc[0])) + ((-B_BODYn[1]*B_BODYn[0])*(-T_RWB[1] + Tc[1])) + ((-B_BODYn[2]*B_BODYn[0])*(-T_RWB[2] + Tc[2]));
				u_perp[1] = ((-B_BODYn[1]*B_BODYn[0])*(-T_RWB[0] + Tc[0])) + (((B_BODYn[2]*B_BODYn[2])+(B_BODYn[0]*B_BODYn[0]))*(-T_RWB[1] + Tc[1])) + ((-B_BODYn[2]*B_BODYn[1])*(-T_RWB[2] + Tc[2]));
				u_perp[2] = ((-B_BODYn[0]*B_BODYn[2])*(-T_RWB[0] + Tc[0])) + ((-B_BODYn[2]*B_BODYn[1])*(-T_RWB[1] + Tc[1])) + (((B_BODYn[1]*B_BODYn[1])+(B_BODYn[0]*B_BODYn[0]))*(-T_RWB[2] + Tc[2]));

				rCross_Product(B_BODYtesla, u_perp);
				DPM[0] = Cross_Product[0]/Bsq; //Bsq dividebyzero check is done already
				DPM[1] = Cross_Product[1]/Bsq;
				DPM[2] = Cross_Product[2]/Bsq;

				if(fabs(T_RWB[0]) <= c_dividebyzerovalue)
				{
					T_RWB[0] = c_dividebyzerovalue;
				}

				if(fabs(T_RWB[1]) <= c_dividebyzerovalue)
				{
					T_RWB[1] = c_dividebyzerovalue;
				}


				if(fabs(T_RWB[2]) <= c_dividebyzerovalue)
				{
					T_RWB[2] = c_dividebyzerovalue;
				}

				T_RWBn[0] = fabs(0.001/(1.732050807568877*T_RWB[0]));
				T_RWBn[1] = fabs(0.001/(1.732050807568877*T_RWB[1]));
				T_RWBn[2] = fabs(0.001/(1.732050807568877*T_RWB[2]));

				temp_lic = (T_RWBn[0] < T_RWBn[1]) ? T_RWBn[0] : T_RWBn[1];
				H_retn = (T_RWBn[2] < temp_lic) ? T_RWBn[2] : temp_lic;

				//H_retn = min_fun3(T_RWBn[0], T_RWBn[1], T_RWBn[2]);

				DPM[0] = fabs(c_DPMMAX/DPM[0]);
				DPM[1] = fabs(c_DPMMAX/DPM[1]);
				DPM[2] = fabs(c_DPMMAX/DPM[2]);

				//tau_ms = min_fun3(DPM[0], DPM[1], DPM[2]);

				temp_lic = (DPM[0] < DPM[1]) ? DPM[0] : DPM[1];
				tau_ms = (DPM[2] < temp_lic) ? DPM[2] : temp_lic;

				//min_TW = min_fun2(H_retn, tau_ms);

				min_TW = (H_retn < tau_ms) ? H_retn : tau_ms;

				if (min_TW < 1.0)
				{
					u_perp[0] = u_perp[0] * min_TW;
					u_perp[1] = u_perp[1] * min_TW;
					u_perp[2] = u_perp[2] * min_TW;

					rCross_Product(B_BODYtesla,u_perp);
					DPM[0] = Cross_Product[0]/Bsq;
					DPM[1] = Cross_Product[1]/Bsq;
					DPM[2] = Cross_Product[2]/Bsq;

					T_RWB[0] = T_RWB[0] * min_TW;
					T_RWB[1] = T_RWB[1] * min_TW;
					T_RWB[2] = T_RWB[2] * min_TW;

					rMatMul43x1(B2wh_mat,T_RWB);
					T_RW[0] = Matout431[0];
					T_RW[1] = Matout431[1];
					T_RW[2] = Matout431[2];
					T_RW[3] = Matout431[3];
				}
				DutyCycleGenEnable = 1;
			    tor_counter = 0; // for dynamics
			    tor_counterk = 0;
			    TorquerPolaritySetFlag = 1;
			    rTorquer_Polarity_Check();

			   //speed_based_torquer_control = 1;
            }

			else
			{
			   DutyCycleGenEnable = 0;
			   tor_counter = 0; // for dynamics
			   tor_counterk = 0;
			   TorquerPolaritySetFlag = 0;
			   DPM[0] = 0.0;
			   DPM[1] = 0.0;
			   DPM[2] = 0.0;
			}
        }
    }
}

static void rWheel_Spin_updown(void)
{
    if (CB_Wheel_Spin_updown == Enable)
    {
        if (TC_boolean_u.TC_Boolean_Table.TC_Wheel_SpinUpDown_Logic == Enable)
        {
            if (Wheel_Config == 0)
            {
                del_v0a = 0.0;
                for (i_lict=0; i_lict<=3; i_lict++)
                {
                    del_v0[i_lict] = (TC_RW_Nominal[i_lict]) - (double)RW_Wheel_Speed[i_lict];//RW_Nominal[i_lict] should be assigned with Telecommanded value for nominal speed
                    del_v0a = del_v0a + fabs(del_v0[i_lict]);
                }
                del_v0a = del_v0a/4.0;
                if (del_v0a >= 300.0)// 300 TC
                {
                    spin_up_avg_count = spin_up_avg_count + 1;
                }

                if (spin_up_avg_count > 344)
                {
                    if (((del_v0[0] < 0.0) && (del_v0[1] > 0.0) && (del_v0[2] < 0.0) && (del_v0[3] > 0.0))||((del_v0[0] > 0.0) && (del_v0[1] < 0.0)&& (del_v0[2] > 0.0) && (del_v0[3] < 0.0)))
                    {
                        T_RW_spin[0] = 0.0001 * rsign_f(del_v0[0]);
                        T_RW_spin[1] = 0.0001 * rsign_f(del_v0[1]);
                        T_RW_spin[2] = 0.0001 * rsign_f(del_v0[2]);
                        T_RW_spin[3] = 0.0001 * rsign_f(del_v0[3]);
                        spin_up_avg_count_2 = spin_up_avg_count_2 + 1;
                        if (spin_up_avg_count_2 > 110)
                        {
                            T_RW_spin[0] = 0.0;
                            T_RW_spin[1] = 0.0;
                            T_RW_spin[2] = 0.0;
                            T_RW_spin[3] = 0.0;
                            spin_up_avg_count = 0;
                            spin_up_avg_count_2 = 0;
                        }
                    }
                    else
                    {
                        T_RW_spin[0] = 0.0;
                        T_RW_spin[1] = 0.0;
                        T_RW_spin[2] = 0.0;
                        T_RW_spin[3] = 0.0;
                    }
                }
            }
        }
        else
        {
            T_RW_spin[0] = 0.0;
            T_RW_spin[1] = 0.0;
            T_RW_spin[2] = 0.0;
            T_RW_spin[3] = 0.0;
        }
        T_RW[0] = T_RW[0] + T_RW_spin[0];
        T_RW[1] = T_RW[1] + T_RW_spin[1];
        T_RW[2] = T_RW[2] + T_RW_spin[2];
        T_RW[3] = T_RW[3] + T_RW_spin[3];
    }
}

static void rWheel_Auto_Reconfiguration(void)
{
    if (CB_Wheel_Auto_Reconfiguration == Enable)
    {
        if (TC_boolean_u.TC_Boolean_Table.TC_Wheel_AutoReConfig_Logic == Enable)
        {
            exp_whsp_ch[0] = exp_whsp_ch[0] + ((T_RW[0] * c_MaC/c_MOI_wh)*c_RADps2RPM);
            exp_whsp_ch[1] = exp_whsp_ch[1] + ((T_RW[1] * c_MaC/c_MOI_wh)*c_RADps2RPM);
            exp_whsp_ch[2] = exp_whsp_ch[2] + ((T_RW[2] * c_MaC/c_MOI_wh)*c_RADps2RPM);
            exp_whsp_ch[3] = exp_whsp_ch[3] + ((T_RW[3] * c_MaC/c_MOI_wh)*c_RADps2RPM);

            RW_ARC_Count = RW_ARC_Count + 1;

            if ((Major_Cycle_Count % 800) == 0)
            {
                for (i_lict=0; i_lict<=3; i_lict++)
                {
                    pres_exp_whsp_ch[i_lict] = exp_whsp_ch[i_lict];
                    ch_obs_whsp[i_lict] = (RWSpeed_RPM[i_lict]) - (prev_obs_whsp_ch[i_lict]);
                    prev_obs_whsp_ch[i_lict] = RWSpeed_RPM[i_lict];
                    diff_obs_exp_ch[i_lict] = ch_obs_whsp[i_lict] - pres_exp_whsp_ch[i_lict];
                }

                if (fabs(diff_obs_exp_ch[0]) >= TC_ARC_RPM_Thres)
                {
                    count_arc_w0 = count_arc_w0 + 1;
                    if (count_arc_w0 > TC_ARC_Time_Cycle)
                    {
                        wheel_index[0] = 0;
                    }
                }
                else
                {
                    count_arc_w0 = 0;
                }

                if (fabs(diff_obs_exp_ch[1]) >= TC_ARC_RPM_Thres)
                {
                    count_arc_w1 = count_arc_w1 + 1;
                    if (count_arc_w1 > TC_ARC_Time_Cycle)
                    {
                        wheel_index[1] = 0;
                    }
                }
                else
                {
                    count_arc_w1 = 0;
                }

                if (fabs(diff_obs_exp_ch[2]) >= TC_ARC_RPM_Thres)
                {
                    count_arc_w2 = count_arc_w2 + 1;
                    if (count_arc_w2 > TC_ARC_Time_Cycle)
                    {
                        wheel_index[2] = 0;
                    }
                }
                else
                {
                    count_arc_w2 = 0;
                }

                if (fabs(diff_obs_exp_ch[3]) >= TC_ARC_RPM_Thres)
                {
                    count_arc_w3 = count_arc_w3 + 1;

                    if (count_arc_w3 > TC_ARC_Time_Cycle)
                    {
                        wheel_index[3] = 0;
                    }
                }
                else
                {
                    count_arc_w3 = 0;
                }


                exp_whsp_ch[0] = 0.0;
                exp_whsp_ch[1] = 0.0;
                exp_whsp_ch[2] = 0.0;
                exp_whsp_ch[3] = 0.0;

            }

            if (TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW1_enable == 1)
			{
				wheel_index[0] = 1;
			}
			else
			{
				wheel_index[0] = 0;
			}

			if (TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW2_enable == 1)
			{
				wheel_index[1] = 2;
			}
			else
			{
				wheel_index[1] = 0;
			}

			if (TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW3_enable == 1)
			{
				wheel_index[2] = 4;
			}
			else
			{
				wheel_index[2] = 0;
			}

			if (TC_boolean_u.TC_Boolean_Table.TC_wheel_index_ground_RW4_enable == 1)
			{
				wheel_index[3] = 8;
			}
			else
			{
				wheel_index[3] = 0;
			}

            wheel_index_ARCsum = wheel_index[0] + wheel_index[1] + wheel_index[2] + wheel_index[3];

            ST_special.ST_SP_Buffer.TM_wheel_index_ARCsum = (char)wheel_index_ARCsum;
            ST_normal.ST_NM_Buffer.TM_wheel_index_ARCsum  = (char)wheel_index_ARCsum;

            if (wheel_index_ARCsum == 15)
            {
                Wheel_Config = 0;
                for (i_lict=0; i_lict<3; i_lict++)
                {
                    for (j_lict=0; j_lict<4; j_lict++)
                    {
                        wh2B_mat[i_lict][j_lict] = c_wh2B_mat_4RW[i_lict][j_lict];
                    }
                }
                for (i_lict=0; i_lict<4; i_lict++)
                {
                    for (j_lict=0; j_lict<3; j_lict++)
                    {
                        B2wh_mat[i_lict][j_lict] = c_B2wh_mat_4RW[i_lict][j_lict];
                    }
                }
            }


            if (wheel_index_ARCsum == 14)
            {
                Wheel_Config = 1;
                for (i_lict=0; i_lict<3; i_lict++)
                {
                    for (j_lict=0; j_lict<4; j_lict++)
                    {
                        wh2B_mat[i_lict][j_lict] = c_wh2B_mat_0234[i_lict][j_lict];
                    }
                }
                for (i_lict=0; i_lict<4; i_lict++)
                {
                    for (j_lict=0; j_lict<3; j_lict++)
                    {
                        B2wh_mat[i_lict][j_lict] = c_B2wh_mat_0234[i_lict][j_lict];
                    }
                }
            }

            if (wheel_index_ARCsum == 13)
            {
                Wheel_Config = 1;
                for (i_lict=0; i_lict<3; i_lict++)
                {
                    for (j_lict=0; j_lict<4; j_lict++)
                    {
                        wh2B_mat[i_lict][j_lict] = c_wh2B_mat_1034[i_lict][j_lict];
                    }
                }
                for (i_lict=0; i_lict<4; i_lict++)
                {
                    for (j_lict=0; j_lict<3; j_lict++)
                    {
                        B2wh_mat[i_lict][j_lict] = c_B2wh_mat_1034[i_lict][j_lict];
                    }
                }
            }

            if (wheel_index_ARCsum == 11)
            {
                Wheel_Config = 1;
                for (i_lict=0; i_lict<3; i_lict++)
                {
                    for (j_lict=0; j_lict<4; j_lict++)
                    {
                        wh2B_mat[i_lict][j_lict] = c_wh2B_mat_1204[i_lict][j_lict];
                    }
                }
                for (i_lict=0; i_lict<4; i_lict++)
                {
                    for (j_lict=0; j_lict<3; j_lict++)
                    {
                        B2wh_mat[i_lict][j_lict] = c_B2wh_mat_1204[i_lict][j_lict];
                    }
                }
            }

            if (wheel_index_ARCsum == 7)
            {
                Wheel_Config = 1;
                for (i_lict=0; i_lict<3; i_lict++)
                {
                    for (j_lict=0; j_lict<4; j_lict++)
                    {
                        wh2B_mat[i_lict][j_lict] = c_wh2B_mat_1230[i_lict][j_lict];
                    }
                }
                for (i_lict=0; i_lict<4; i_lict++)
                {
                    for (j_lict=0; j_lict<3; j_lict++)
                    {
                        B2wh_mat[i_lict][j_lict] = c_B2wh_mat_1230[i_lict][j_lict];
                    }
                }
            }
        }
    }
}
