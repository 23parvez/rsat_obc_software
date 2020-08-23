#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "adcs_VarDeclarations.h"

#include "Telecommand.h"
#include "HAL_Global.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "TC_List.h"
#include "Global.h"


void rQ_Propagation(double q_prop[4], double w_prop[3])
{
    //CB_Q_propagation = Enable;
	if(CB_Q_propagation == Enable)
	{
		///Computation Del_Y_theta, Del_R_theta, Del_P_theta
		Del_R_theta = (c_MiC * w_prop[0]);
		Del_P_theta = (c_MiC * w_prop[1]);
		Del_Y_theta = (c_MiC * w_prop[2]);

		///Computation Del_Q
		Del_Q[0] = Del_R_theta / 2.0;
		Del_Q[1] = Del_P_theta / 2.0;
		Del_Q[2] = Del_Y_theta / 2.0;
		Del_Q[3] = 1.0 - ((1.0 / 3.0) * (Del_Q[0] * Del_Q[0] + Del_Q[1] * Del_Q[1] + Del_Q[2] * Del_Q[2]));

		Qprop_prev[0] = q_prop[0];
		Qprop_prev[1] = q_prop[1];
		Qprop_prev[2] = q_prop[2];
		Qprop_prev[3] = q_prop[3];

		///Propagated Q_Prop computation
		rQs_Multiplication(Qprop_prev,Del_Q);
		q_prop_out[0] = out_Quat_mult[0];
		q_prop_out[1] = out_Quat_mult[1];
		q_prop_out[2] = out_Quat_mult[2];
		q_prop_out[3] = out_Quat_mult[3];

		rQs_Normalization(q_prop_out);
        q_prop_out[0] = out_Quat_norm[0];
        q_prop_out[1] = out_Quat_norm[1];
        q_prop_out[2] = out_Quat_norm[2];
        q_prop_out[3] = out_Quat_norm[3];
	}
	else
    {
        q_prop_out[3] = 1.0;
    }
}


void rQuestDataProcessing(void)
{
    if (CB_QuestDataProcessing == Enable)
    {
        ///Quest Data Processing for MAGAD
        sc_qst++;
        if(sc_qst >= 8)
        {
            sc_qst = 0;
            dc_qst++;
            if(dc_qst >= 9)
            {
                wc_qst++;
                if(wc_qst >= ADCS_TC_data_command_Table.TC_wp_QDP)
                {
                    wc_qst = 0;
                    dc_qst = 0;
                }

            }
            else
            {
                ///Shift and Assign
                mat_mag_DataCounter++;
                for(i_QDP = 0; i_QDP <= 22; i_QDP++)
                {
                    NRB_mag[0][i_QDP] = NRB_mag[0][(i_QDP + 1)];
                    NRB_mag[1][i_QDP] = NRB_mag[1][(i_QDP + 1)];
                    NRB_mag[2][i_QDP] = NRB_mag[2][(i_QDP + 1)];

                    NMB_mag[0][i_QDP] = NMB_mag[0][(i_QDP + 1)];
                    NMB_mag[1][i_QDP] = NMB_mag[1][(i_QDP + 1)];
                    NMB_mag[2][i_QDP] = NMB_mag[2][(i_QDP + 1)];
                }
                NRB_mag[0][23] = B_REFn[0];
                NRB_mag[1][23] = B_REFn[1];
                NRB_mag[2][23] = B_REFn[2];

                NMB_mag[0][23] = B_BODYn[0];
                NMB_mag[1][23] = B_BODYn[1];
                NMB_mag[2][23] = B_BODYn[2];

                if(mat_mag_DataCounter >= 24)
                {
                    f_DataSort_MAG = True;
                    Quest_update_available = 1;
                    mat_mag_DataCounter = 16;
                }
                else
                {
                    f_DataSort_MAG = False;
                    Quest_update_available = 0;
                }
            }
        }

        ///Quest Data Processing for SUNMAGAD
        if(f_Sunlit_Presence == True)
        {
            //f_Momentum_Dumping = 1;
            TC_SunMagAD = Enable;
            w_q_update_satisfy = 1;

            ///Shift and Assign
            mat_sm_DataCounter++;
            for(i_QDP = 0; i_QDP <= 6; i_QDP++)
            {
                NRB_sunmag[0][i_QDP] = NRB_sunmag[0][(i_QDP + 1)];
                NRB_sunmag[1][i_QDP] = NRB_sunmag[1][(i_QDP + 1)];
                NRB_sunmag[2][i_QDP] = NRB_sunmag[2][(i_QDP + 1)];

                NMB_sunmag[0][i_QDP] = NMB_sunmag[0][(i_QDP + 1)];
                NMB_sunmag[1][i_QDP] = NMB_sunmag[1][(i_QDP + 1)];
                NMB_sunmag[2][i_QDP] = NMB_sunmag[2][(i_QDP + 1)];

                NRS[0][i_QDP] = NRS[0][(i_QDP + 1)];
                NRS[1][i_QDP] = NRS[1][(i_QDP + 1)];
                NRS[2][i_QDP] = NRS[2][(i_QDP + 1)];

                NMS[0][i_QDP] = NMS[0][(i_QDP + 1)];
                NMS[1][i_QDP] = NMS[1][(i_QDP + 1)];
                NMS[2][i_QDP] = NMS[2][(i_QDP + 1)];
            }
            NRB_sunmag[0][7] = B_REFn[0];
            NRB_sunmag[1][7] = B_REFn[1];
            NRB_sunmag[2][7] = B_REFn[2];

            NMB_sunmag[0][7] = B_BODYn[0];
            NMB_sunmag[1][7] = B_BODYn[1];
            NMB_sunmag[2][7] = B_BODYn[2];

            NRS[0][7] = S_REFn[0];
            NRS[1][7] = S_REFn[1];
            NRS[2][7] = S_REFn[2];

            NMS[0][7] = S_BODYn[0];
            NMS[1][7] = S_BODYn[1];
            NMS[2][7] = S_BODYn[2];

            if(mat_sm_DataCounter >= 8)
            {
                f_DataSort_SUNMAG = True;
                Quest_update_available = 1;
                mat_sm_DataCounter = 0;
            }
            else
            {
                f_DataSort_SUNMAG = False;
                Quest_update_available = 0;
            }
        }
        else
        {
            TC_SunMagAD = Disable;
            mat_sm_DataCounter = 0;
        }
    }
}

void rDAD_quest(void)
{
    if(CB_DAD_quest == Enable) ///Control Byte
    {
    	if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 1 && TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable == 1)
		{
			if((abs_f(w_k[0]*c_R2D) <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThRoll) && (w_k[1]*c_R2D <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThPitch) && (w_k[1]*c_R2D >= ADCS_TC_data_command_Table.TC_wAD_BODYminThPitch) && (abs_f(w_k[2]*c_R2D) <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThYaw))
			{
				wAD_updatecount++;
			}
			else
			{
				wAD_updatecount = 0;
			}
		}
		else
		{
			if((abs_f(w_BODYdeg[0]) <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThRoll) && (w_BODYdeg[1] <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThPitch) && (w_BODYdeg[1] >= ADCS_TC_data_command_Table.TC_wAD_BODYminThPitch) && (abs_f(w_BODYdeg[2]) <= ADCS_TC_data_command_Table.TC_wAD_BODYmaxThYaw))
			{
				wAD_updatecount++;
			}
			else
			{
				wAD_updatecount = 0;
			}
		}

        if((magRoll_angle >= ADCS_TC_data_command_Table.TC_magMin_angle) &&
        		(magPitch_angle >= ADCS_TC_data_command_Table.TC_magMin_angle) && (magYaw_angle >= ADCS_TC_data_command_Table.TC_magMin_angle) &&
        		(magRoll_angle <= ADCS_TC_data_command_Table.TC_magMax_angle) && (magPitch_angle <= ADCS_TC_data_command_Table.TC_magMax_angle) &&
        		(magYaw_angle <= ADCS_TC_data_command_Table.TC_magMax_angle)) ///TC_magMin_angle=45deg
        {
			if(wAD_updatecount >= ADCS_TC_data_command_Table.TC_wAD_updateTimeThresh)
			{
				w_q_update_satisfy = 1;
			}
			else
			{
				w_q_update_satisfy = 0;
			}
        }

        ///TC_SunMagAD = Disable;
        if(TC_SunMagAD == Disable)
        {
            if(f_DataSort_MAG == True)
            {
                ///Computation of transpose of NRB_mag
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 24; J_DAD++)
                    {
                        NRBt_mag[J_DAD][I_DAD] = NRB_mag[I_DAD][J_DAD];
                    }
                }

                ///Computation of NMB_mag * NRB_mag'
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 3; J_DAD++)
                    {
                        NMB_NRBt[I_DAD][J_DAD]  = 0.0;
                        for(K_DAD = 0;K_DAD < 24; K_DAD++)
                        {
                            NMB_NRBt[I_DAD][J_DAD] += (NMB_mag[I_DAD][K_DAD] * NRBt_mag[K_DAD][J_DAD]);
                        }
                    }
                }

                ///values of wkm and wks for mag
                wkm = c_wkm_mag;
                wks = c_wks_mag;

                f_DataSort_MAG = False;

            }
        }
        else
        {
            if(f_DataSort_SUNMAG == True)
            {
                ///Computation of transpose of NRB_sunmag
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 8; J_DAD++)
                    {
                        NRBt_sunmag[J_DAD][I_DAD] = NRB_sunmag[I_DAD][J_DAD];
                    }
                }

                ///Computation of NMB_sunmag * NRB_sunmag'
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 3; J_DAD++)
                    {
                        NMB_NRBt[I_DAD][J_DAD]  = 0.0;
                        for(K_DAD = 0; K_DAD < 8; K_DAD++)
                        {
                            NMB_NRBt[I_DAD][J_DAD] += (NMB_sunmag[I_DAD][K_DAD] * NRBt_sunmag[K_DAD][J_DAD]);
                        }
                    }
                }

                ///Computation of transpose of NRS
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 8; J_DAD++)
                    {
                        NRSt[J_DAD][I_DAD] = NRS[I_DAD][J_DAD];
                    }
                }

                ///Computation of NMS * NRS'
                for(I_DAD = 0; I_DAD < 3; I_DAD++)
                {
                    for(J_DAD = 0; J_DAD < 3; J_DAD++)
                    {
                        NMS_NRSt[I_DAD][J_DAD]  = 0.0;
                        for(K_DAD = 0; K_DAD < 8; K_DAD++)
                        {
                            NMS_NRSt[I_DAD][J_DAD] += (NMS[I_DAD][K_DAD] * NRSt[K_DAD][J_DAD]);
                        }
                    }
                }

                ///values of wkm and wks for sunmag
                wkm = c_wkm_sunmag;
                wks = c_wks_sunmag;
            }
        }

        ///B = (wkm * NMB * NRB') + (wks * NMS * NRS') computation
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                B_DAD[I_DAD][J_DAD] = (wkm * NMB_NRBt[I_DAD][J_DAD]) + (wks * NMS_NRSt[I_DAD][J_DAD]);
            }
        }

        ///Computation of transpose of B (B')
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                Bt_DAD[J_DAD][I_DAD] = B_DAD[I_DAD][J_DAD];
            }
        }


        ///Computation of S = B + B'
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                S_DAD[I_DAD][J_DAD] = B_DAD[I_DAD][J_DAD] + Bt_DAD[I_DAD][J_DAD];
            }
        }

        ///Computation of Z
        Z_DAD[0] = B_DAD[1][2] - B_DAD[2][1];
        Z_DAD[1] = B_DAD[2][0] - B_DAD[0][2];
        Z_DAD[2] = B_DAD[0][1] - B_DAD[1][0];

        ///Computation of sigma = 0.5 * trace(S)
        trace_S_DAD = S_DAD[0][0] + S_DAD[1][1] + S_DAD[2][2];
        sigma_DAD = (0.5 * trace_S_DAD);

        ///Computation of k_kf = trace(adj(S))
        rMat_adjoint3(S_DAD);

        ///Assigning global variable value to adj(S)
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                adj_S_DAD[I_DAD][J_DAD] = mat_adj[I_DAD][J_DAD];
            }
        }

        k_DAD = adj_S_DAD[0][0] + adj_S_DAD[1][1] + adj_S_DAD[2][2];

        ///Computation of Delta = det(S)
        delta_DAD = ((S_DAD[0][0] * ((S_DAD[1][1] * S_DAD[2][2]) - (S_DAD[2][1] * S_DAD[1][2])))
                    -(S_DAD[0][1] * ((S_DAD[1][0] * S_DAD[2][2]) - (S_DAD[2][0] * S_DAD[1][2])))
                    +(S_DAD[0][2] * ((S_DAD[1][0] * S_DAD[2][1]) - (S_DAD[2][0] * S_DAD[1][1]))));

        ///Computation of Z'Z
        Zt_Z_DAD = ((Z_DAD[0] * Z_DAD[0]) + (Z_DAD[1] * Z_DAD[1]) + (Z_DAD[2] * Z_DAD[2]));

        ///Computation of Z'S Z, Nu1, Nv1, Nu2, Nv2
        Zt_S_Z_DAD = (Z_DAD[0]*((S_DAD[0][0] * Z_DAD[0]) + (S_DAD[0][1] * Z_DAD[1]) + (S_DAD[0][2] * Z_DAD[2])))
                    +(Z_DAD[1]*((S_DAD[1][0] * Z_DAD[0]) + (S_DAD[1][1] * Z_DAD[1]) + (S_DAD[1][2] * Z_DAD[2])))
                    +(Z_DAD[2]*((S_DAD[2][0] * Z_DAD[0]) + (S_DAD[2][1] * Z_DAD[1]) + (S_DAD[2][2] * Z_DAD[2])));

        ///Computation of S^2
        rMatMul3x3(S_DAD, S_DAD);

            ///Assigning global variable value to S^2
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                S_sqr_DAD[I_DAD][J_DAD] = Matout33[I_DAD][J_DAD];
            }
        }

        ///Computation of Z'S S Z
        Zt_Ssqr_Z_DAD = (Z_DAD[0]*((S_sqr_DAD[0][0] * Z_DAD[0]) + (S_sqr_DAD[0][1] * Z_DAD[1]) + (S_sqr_DAD[0][2] * Z_DAD[2])))
                        +(Z_DAD[1]*((S_sqr_DAD[1][0] * Z_DAD[0]) + (S_sqr_DAD[1][1] * Z_DAD[1]) + (S_sqr_DAD[1][2] * Z_DAD[2])))
                        +(Z_DAD[2]*((S_sqr_DAD[2][0] * Z_DAD[0]) + (S_sqr_DAD[2][1] * Z_DAD[1]) + (S_sqr_DAD[2][2] * Z_DAD[2])));

        ///Computation of a,b,c,d
        /**
        a = sigma^2 - k_kf
        b = sigma^2 + Z'Z
        c = delta + Z'S Z
        d = Z'S S Z
        */

        a_DAD = (sigma_DAD * sigma_DAD) - k_DAD;
        b_DAD = (sigma_DAD * sigma_DAD) + Zt_Z_DAD;
        c_DAD = delta_DAD + Zt_S_Z_DAD;
        d_DAD = Zt_Ssqr_Z_DAD;

        ///Newton Ralphson Method to find the root
        root_DAD = 1.0;
        func_prev_DAD = 10.0;
        for(I_DAD = 0; I_DAD < 100; I_DAD++)
        {
            func_DAD = ((root_DAD * root_DAD) - a_DAD) * ((root_DAD * root_DAD) - b_DAD) - (c_DAD * root_DAD) + (c_DAD * sigma_DAD) - d_DAD;
            Dfunc_DAD = 2.0 * root_DAD * (2.0 * (root_DAD * root_DAD) - a_DAD - b_DAD) - c_DAD;

            if((func_DAD < func_prev_DAD) && (func_DAD > 0.0))
            {
                ///-------------Divide by zero check--------------
                if(abs_f(Dfunc_DAD) <= c_dividebyzerovalue)
                {
                    Dfunc_DAD = c_dividebyzerovalue;
                }
                ///------------------------------------------------
                root_DAD = root_DAD - (func_DAD / Dfunc_DAD);
            }
            func_prev_DAD = func_DAD;
        }

        ///Computation of alpha, beta and gamma
        /**
        alpha = root^2 - sigma^2 + k_kf
        beta = root - sigma
        gamma = (root + sigma) alpha - delta
        */

        alpha_DAD = ((root_DAD * root_DAD) - (sigma_DAD * sigma_DAD) + k_DAD);
        beta_DAD = (root_DAD - sigma_DAD);
        gamma_DAD = (((root_DAD + sigma_DAD) * alpha_DAD) - delta_DAD);

        ///Computation of alpha * I
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                if(I_DAD == J_DAD)
                {
                    alpha_I_DAD[I_DAD][J_DAD] = alpha_DAD;
                }
                else
                {
                    alpha_I_DAD[I_DAD][J_DAD] = 0.0;
                }
            }
        }

        ///Computation of beta*S
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                beta_S_DAD[I_DAD][J_DAD] = (beta_DAD * S_DAD[I_DAD][J_DAD]);
            }
        }

        ///Computation of sumX_DAD = (alpha*I + beta*S + S^2)
        for(I_DAD = 0; I_DAD < 3; I_DAD++)
        {
            for(J_DAD = 0; J_DAD < 3; J_DAD++)
            {
                sumX_DAD[I_DAD][J_DAD] = (alpha_I_DAD[I_DAD][J_DAD] + beta_S_DAD[I_DAD][J_DAD] + S_sqr_DAD[I_DAD][J_DAD]);
            }
        }

        ///Computation of X = (alpha*I + beta*S + S^2) Z_DAD
        X_DAD[0] = ((sumX_DAD[0][0] * Z_DAD[0]) + (sumX_DAD[0][1] * Z_DAD[1]) + (sumX_DAD[0][2] * Z_DAD[2]));
        X_DAD[1] = ((sumX_DAD[1][0] * Z_DAD[0]) + (sumX_DAD[1][1] * Z_DAD[1]) + (sumX_DAD[1][2] * Z_DAD[2]));
        X_DAD[2] = ((sumX_DAD[2][0] * Z_DAD[0]) + (sumX_DAD[2][1] * Z_DAD[1]) + (sumX_DAD[2][2] * Z_DAD[2]));

        ///Computation of Q_opt
        Q_quest_DAD_den = sqrt((gamma_DAD * gamma_DAD) + (X_DAD[0] * X_DAD[0]) + (X_DAD[1] * X_DAD[1]) + (X_DAD[2] * X_DAD[2]));

        if(abs_f(Q_quest_DAD_den) <= c_dividebyzerovalue)
        {
            Q_quest_DAD_den = c_dividebyzerovalue;
        }

        Q_quest_DAD[0] = (1.0 / Q_quest_DAD_den) * X_DAD[0];
        Q_quest_DAD[1] = (1.0 / Q_quest_DAD_den) * X_DAD[1];
        Q_quest_DAD[2] = (1.0 / Q_quest_DAD_den) * X_DAD[2];
        Q_quest_DAD[3] = (1.0 / Q_quest_DAD_den) * gamma_DAD;

        TM.Buffer.TM_Q_Sunmagad[0] = (1.0 / Q_quest_DAD_den) * X_DAD[0];
        TM.Buffer.TM_Q_Sunmagad[1] = (1.0 / Q_quest_DAD_den) * X_DAD[1];
        TM.Buffer.TM_Q_Sunmagad[2] = (1.0 / Q_quest_DAD_den) * X_DAD[2];
        TM.Buffer.TM_Q_Sunmagad[3] = (1.0 / Q_quest_DAD_den) * X_DAD[3];

        ST_normal.ST_NM_Buffer.TM_Q_Sunmagad[0] = (1.0 / Q_quest_DAD_den) * X_DAD[0];
        ST_normal.ST_NM_Buffer.TM_Q_Sunmagad[1] = (1.0 / Q_quest_DAD_den) * X_DAD[1];
        ST_normal.ST_NM_Buffer.TM_Q_Sunmagad[2] = (1.0 / Q_quest_DAD_den) * X_DAD[2];
        ST_normal.ST_NM_Buffer.TM_Q_Sunmagad[3] = (1.0 / Q_quest_DAD_den) * X_DAD[3];

        ST_special.ST_SP_Buffer.TM_Q_Sunmagad[0] = (1.0 / Q_quest_DAD_den) * X_DAD[0];
        ST_special.ST_SP_Buffer.TM_Q_Sunmagad[1] = (1.0 / Q_quest_DAD_den) * X_DAD[1];
        ST_special.ST_SP_Buffer.TM_Q_Sunmagad[2] = (1.0 / Q_quest_DAD_den) * X_DAD[2];
        ST_special.ST_SP_Buffer.TM_Q_Sunmagad[3] = (1.0 / Q_quest_DAD_den) * X_DAD[3];

        ///Normalization of Q_opt (Q_quest)
        rQs_Normalization(Q_quest_DAD);
        Q_quest_DAD[0] = out_Quat_norm[0];
        Q_quest_DAD[1] = out_Quat_norm[1];
        Q_quest_DAD[2] = out_Quat_norm[2];
        Q_quest_DAD[3] = out_Quat_norm[3];

        rQs_Multiplication(Q_REF, Q_quest_DAD);
        Qquest_update[0] = out_Quat_mult[0];
        Qquest_update[1] = out_Quat_mult[1];
        Qquest_update[2] = out_Quat_mult[2];
        Qquest_update[3] = out_Quat_mult[3];

        rQs_Normalization(Qquest_update);
        Qquest_update[0] = out_Quat_norm[0];
        Qquest_update[1] = out_Quat_norm[1];
        Qquest_update[2] = out_Quat_norm[2];
        Qquest_update[3] = out_Quat_norm[3];

        TM.Buffer.TM_Q_Magad[0] = (int)(Q_quest_DAD[0]/4.65661287E-7);
		TM.Buffer.TM_Q_Magad[1] = (int)(Q_quest_DAD[1]/4.65661287E-7);
		TM.Buffer.TM_Q_Magad[2] = (int)(Q_quest_DAD[2]/4.65661287E-7);
		TM.Buffer.TM_Q_Magad[3] = (int)(Q_quest_DAD[3]/4.65661287E-7);

        ///w_q_update_satisfy = 1;
        if ((Quest_update_available == 1) && (OBC_Quest_update == 1) && (TC_boolean_u.TC_Boolean_Table.TC_QuestUpdate_Enable == Enable) && (w_q_update_satisfy == 1) && (TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable == Disable))
        {
            Qbody[0] = Qquest_update[0];
            Qbody[1] = Qquest_update[1];
            Qbody[2] = Qquest_update[2];
            Qbody[3] = Qquest_update[3];
            //Quest_update_available = 0;
            w_q_update_satisfy = 0;

            OBC_Quest_update = 0;
        }
    }
}

void rErrorComputation(void)
{
    if (CB_ErrorComputation == Enable)
    {
        if (TC_boolean_u.TC_Boolean_Table.TC_EKFControl_Enable == Enable)
        {
        	if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 1)
			{
        		rQs_Multiplication(Q_REF_conj,qk_plus);
				Qerror[0] = out_Quat_mult[0];
				Qerror[1] = out_Quat_mult[1];
				Qerror[2] = out_Quat_mult[2];
				Qerror[3] = out_Quat_mult[3];
			}
        	else if (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == 1)
			{
				rQ_Propagation(qk_plus, w_BODY);
				qk_minus[0] = q_prop_out[0];
				qk_minus[1] = q_prop_out[1];
				qk_minus[2] = q_prop_out[2];
				qk_minus[3] = q_prop_out[3];

				rQs_Normalization(qk_minus);
				qk_minus[0] = out_Quat_norm[0];
				qk_minus[1] = out_Quat_norm[1];
				qk_minus[2] = out_Quat_norm[2];
				qk_minus[3] = out_Quat_norm[3];

				qk_plus[0] = qk_minus[0];
				qk_plus[1] = qk_minus[1];
				qk_plus[2] = qk_minus[2];
				qk_plus[3] = qk_minus[3];

				rQs_Multiplication(Q_REF_conj,qk_minus);
				Qerror[0] = out_Quat_mult[0];
				Qerror[1] = out_Quat_mult[1];
				Qerror[2] = out_Quat_mult[2];
				Qerror[3] = out_Quat_mult[3];

				TM.Buffer.TM_Q_EKF[0] = (int)(qk_minus[0]/4.65661287E-7);
				TM.Buffer.TM_Q_EKF[1] = (int)(qk_minus[1]/4.65661287E-7);
				TM.Buffer.TM_Q_EKF[2] = (int)(qk_minus[2]/4.65661287E-7);
				TM.Buffer.TM_Q_EKF[3] = (int)(qk_minus[3]/4.65661287E-7);

				ST_normal.ST_NM_Buffer.TM_Q_EKF[0] = (int)(qk_minus[0]/4.65661287E-7);
				ST_normal.ST_NM_Buffer.TM_Q_EKF[1] = (int)(qk_minus[1]/4.65661287E-7);
				ST_normal.ST_NM_Buffer.TM_Q_EKF[2] = (int)(qk_minus[2]/4.65661287E-7);
				ST_normal.ST_NM_Buffer.TM_Q_EKF[3] = (int)(qk_minus[3]/4.65661287E-7);

				ST_special.ST_SP_Buffer.TM_Q_EKF[0] = (int)(qk_minus[0]/4.65661287E-7);
				ST_special.ST_SP_Buffer.TM_Q_EKF[1] = (int)(qk_minus[1]/4.65661287E-7);
				ST_special.ST_SP_Buffer.TM_Q_EKF[2] = (int)(qk_minus[2]/4.65661287E-7);
				ST_special.ST_SP_Buffer.TM_Q_EKF[3] = (int)(qk_minus[3]/4.65661287E-7);

			}
        	else
			{
				//
			}
        }
        else
        {

        	rQ_Propagation(Qbody,w_BODY);
			Qbody[0] = q_prop_out[0];
			Qbody[1] = q_prop_out[1];
			Qbody[2] = q_prop_out[2];
			Qbody[3] = q_prop_out[3];

			rQs_Normalization(Qbody);
			Qbody[0] = out_Quat_norm[0];
			Qbody[1] = out_Quat_norm[1];
			Qbody[2] = out_Quat_norm[2];
			Qbody[3] = out_Quat_norm[3];

			TM.Buffer.TM_Q_BODY[0] = (int)(Qbody[0]/4.65661287E-7);
			TM.Buffer.TM_Q_BODY[1] = (int)(Qbody[1]/4.65661287E-7);
			TM.Buffer.TM_Q_BODY[2] = (int)(Qbody[2]/4.65661287E-7);
			TM.Buffer.TM_Q_BODY[3] = (int)(Qbody[3]/4.65661287E-7);

			ST_normal.ST_NM_Buffer.TM_Q_BODY[0] = (int)(Qbody[0]/4.65661287E-7);
			ST_normal.ST_NM_Buffer.TM_Q_BODY[1] = (int)(Qbody[1]/4.65661287E-7);
			ST_normal.ST_NM_Buffer.TM_Q_BODY[2] = (int)(Qbody[2]/4.65661287E-7);
			ST_normal.ST_NM_Buffer.TM_Q_BODY[3] = (int)(Qbody[3]/4.65661287E-7);

            rQs_Multiplication(Q_REF_conj,Qbody);
			Qerror[0] = out_Quat_mult[0];
			Qerror[1] = out_Quat_mult[1];
			Qerror[2] = out_Quat_mult[2];
			Qerror[3] = out_Quat_mult[3];
        }

        rQs_Normalization(Qerror);
        Qerror[0] = out_Quat_norm[0];
        Qerror[1] = out_Quat_norm[1];
        Qerror[2] = out_Quat_norm[2];
        Qerror[3] = out_Quat_norm[3];

        TM.Buffer.TM_Q_Error[0] = (int)(Qerror[0]/4.65661287E-7);
		TM.Buffer.TM_Q_Error[1] = (int)(Qerror[1]/4.65661287E-7);
		TM.Buffer.TM_Q_Error[2] = (int)(Qerror[2]/4.65661287E-7);

		ST_normal.ST_NM_Buffer.TM_Q_Error[0] = (int)(Qerror[0]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_Q_Error[1] = (int)(Qerror[1]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_Q_Error[2] = (int)(Qerror[2]/4.65661287E-7);

		ST_special.ST_SP_Buffer.TM_Q_Error[0] = (int)(Qerror[0]/4.65661287E-7);
		ST_special.ST_SP_Buffer.TM_Q_Error[1] = (int)(Qerror[1]/4.65661287E-7);
		ST_special.ST_SP_Buffer.TM_Q_Error[2] = (int)(Qerror[2]/4.65661287E-7);

    }
}

void rExtendedKalmanFilter1_p1(void)
{
    if (CB_ExtendedKalmanFilter == Enable)
    {
        if (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == Enable)
        {
            qk_DCM[0][0] = (qk_minus[3] * qk_minus[3]) + (qk_minus[0] * qk_minus[0]) - (qk_minus[1] * qk_minus[1]) - (qk_minus[2] * qk_minus[2]);
            qk_DCM[0][1] = 2.0 * ((qk_minus[0] * qk_minus[1]) + (qk_minus[3] * qk_minus[2]));
            qk_DCM[0][2] = 2.0 * ((qk_minus[0] * qk_minus[2]) - (qk_minus[3] * qk_minus[1]));
            qk_DCM[1][0] = 2.0 * ((qk_minus[0] * qk_minus[1]) - (qk_minus[3] * qk_minus[2]));
            qk_DCM[1][1] = (qk_minus[3] * qk_minus[3]) - (qk_minus[0] * qk_minus[0]) + (qk_minus[1] * qk_minus[1]) - (qk_minus[2] * qk_minus[2]);
            qk_DCM[1][2] = 2.0 * ((qk_minus[1] * qk_minus[2]) + (qk_minus[3] * qk_minus[0]));
            qk_DCM[2][0] = 2.0 * ((qk_minus[0] * qk_minus[2]) + (qk_minus[3] * qk_minus[1]));
            qk_DCM[2][1] = 2.0 * ((qk_minus[1] * qk_minus[2]) - (qk_minus[3] * qk_minus[0]));
            qk_DCM[2][2] = (qk_minus[3] * qk_minus[3]) - (qk_minus[0] * qk_minus[0]) - (qk_minus[1] * qk_minus[1]) + (qk_minus[2] * qk_minus[2]);

            Rk[0][0] = 1.0;
            Rk[0][1] = 0.0;
            Rk[0][2] = 0.0;
            Rk[1][0] = 0.0;
            Rk[1][1] = 1.0;
            Rk[1][2] = 0.0;
            Rk[2][0] = 0.0;
            Rk[2][1] = 0.0;
            Rk[2][2] = 1.0;

            if(f_Sunlit_Presence == 1)
            {
                Yk[0] = S_BODY[0];
                Yk[1] = S_BODY[1];
                Yk[2] = S_BODY[2];

                hk_xk_minus[0] = S_ECI[0];
                hk_xk_minus[1] = S_ECI[1];
                hk_xk_minus[2] = S_ECI[2];

                Rk[0][0] = sun_noise * sun_noise;
                Rk[1][1] = sun_noise * sun_noise;
                Rk[2][2] = sun_noise * sun_noise;

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Hk[i_kf][j_kf] = 0.0;

                    }
                }
            }
            else
            {
                Yk[0] = B_BODYtesla[0];
                Yk[1] = B_BODYtesla[1];
                Yk[2] = B_BODYtesla[2];

                hk_xk_minus[0] = B_ECItesla[0];
                hk_xk_minus[1] = B_ECItesla[1];
                hk_xk_minus[2] = B_ECItesla[2];

                Rk[0][0] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;
                Rk[1][1] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;
                Rk[2][2] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Hk[i_kf][j_kf] = 0.0;

                    }
                }

                Hk[0][6] = 1.0;
                Hk[1][7] = 1.0;
                Hk[2][8] = 1.0;
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                hk[i_kf] = 0.0;
                for (k_kf = 0; k_kf < 3; k_kf++)
                {
                    hk[i_kf] = hk[i_kf] + qk_DCM[i_kf][k_kf] * hk_xk_minus[k_kf];
                }
            }

            ref_cross[0][0] =  0.0;
            ref_cross[0][1] =  -hk[2];
            ref_cross[0][2] =  hk[1];
            ref_cross[1][0] =  hk[2];
            ref_cross[1][1] =  0.0;
            ref_cross[1][2] =  -hk[0];
            ref_cross[2][0] =  -hk[1];
            ref_cross[2][1] =  hk[0];
            ref_cross[2][2] =  0.0;

            Hk[0][1] = ref_cross[0][1];
            Hk[0][2] = ref_cross[0][2];
            Hk[1][0] = ref_cross[1][0];
            Hk[1][2] = ref_cross[1][2];
            Hk[2][0] = ref_cross[2][0];
            Hk[2][1] = ref_cross[2][1];

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    Hk_temp[i_kf][j_kf] = Hk[j_kf][i_kf];

                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {

                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    pk_HkT[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        pk_HkT[i_kf][j_kf] =  pk_HkT[i_kf][j_kf] + pk[i_kf][k_kf] * Hk_temp[k_kf][j_kf];

                    }
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    pk_HkT_Hk[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        pk_HkT_Hk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Hk[i_kf][k_kf] *pk_HkT[k_kf][j_kf] ;
                    }

                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    pk_HkT_Hk_Rk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Rk[i_kf][j_kf];

                }
            }

            rMatInv(pk_HkT_Hk_Rk);
            for(i_kf=0; i_kf<3; i_kf++)
            {
                for(j_kf=0; j_kf<3; j_kf++)
                {
                    K_temp[i_kf][j_kf] = Invmatout33[i_kf][j_kf];
                }
            }


            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    Kk_matrix[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3; k_kf++)
                    {
                        Kk_matrix[i_kf][j_kf] = Kk_matrix[i_kf][j_kf]  + pk_HkT[i_kf][k_kf] * K_temp[k_kf][j_kf];
                    }
                }
            }

			for (i_kf = 0; i_kf < 3; i_kf++)
			{
				Yk_minus_hk[i_kf] = Yk[i_kf] - hk[i_kf];
			}

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                Yk_minus_hk_Kk[i_kf] = 0.0;
                for (k_kf = 0; k_kf < 3 ; k_kf++)
                {
                    Yk_minus_hk_Kk[i_kf] = Yk_minus_hk_Kk[i_kf] + Kk_matrix[i_kf][k_kf] * Yk_minus_hk[k_kf];
                }
            }

            Xk[0] = 0.0;
            Xk[1] = 0.0;
            Xk[2] = 0.0;

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                Xk_plus[i_kf] = Xk[i_kf] + Yk_minus_hk_Kk[i_kf];
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                Xk[i_kf] =  Xk_plus[i_kf];
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                Xk_plus_temp[i_kf] = Xk_plus[i_kf];
            }

            TM.Buffer.TM_Error_EKF[0] = (int)(Xk[0]/4.19095159E-7);
            TM.Buffer.TM_Error_EKF[1] = (int)(Xk[1]/4.19095159E-7);
            TM.Buffer.TM_Error_EKF[2] = (int)(Xk[2]/4.19095159E-7);

            ST_special.ST_SP_Buffer.TM_Error_EKF[0] = (int)(Xk[0]/4.19095159E-7);
            ST_special.ST_SP_Buffer.TM_Error_EKF[1] = (int)(Xk[1]/4.19095159E-7);
            ST_special.ST_SP_Buffer.TM_Error_EKF[2] = (int)(Xk[2]/4.19095159E-7);

            TM.Buffer.TM_w_EKF_Drift[0] = (int)((3600.0 * Xk[3])/c_TM_Resol_w);
            TM.Buffer.TM_w_EKF_Drift[1] = (int)((3600.0 * Xk[4])/c_TM_Resol_w);
            TM.Buffer.TM_w_EKF_Drift[2] = (int)((3600.0 * Xk[5])/c_TM_Resol_w);

            ST_normal.ST_NM_Buffer.TM_w_EKF_Drift[0] = (int)((3600.0 * Xk[3])/c_TM_Resol_w);
            ST_normal.ST_NM_Buffer.TM_w_EKF_Drift[1] = (int)((3600.0 * Xk[4])/c_TM_Resol_w);
            ST_normal.ST_NM_Buffer.TM_w_EKF_Drift[2] = (int)((3600.0 * Xk[5])/c_TM_Resol_w);

            ST_special.ST_SP_Buffer.TM_w_EKF_Drift[0] = (int)((3600.0 * Xk[3])/c_TM_Resol_w);
            ST_special.ST_SP_Buffer.TM_w_EKF_Drift[1] = (int)((3600.0 * Xk[4])/c_TM_Resol_w);
            ST_special.ST_SP_Buffer.TM_w_EKF_Drift[2] = (int)((3600.0 * Xk[5])/c_TM_Resol_w);

            TM.Buffer.TM_B_EKF_Bias[0] = (int)(Xk[6]/1.0E-12);
            TM.Buffer.TM_B_EKF_Bias[1] = (int)(Xk[7]/1.0E-12);
            TM.Buffer.TM_B_EKF_Bias[2] = (int)(Xk[8]/1.0E-12);

            ST_special.ST_SP_Buffer.TM_B_EKF_Bias[0] = (int)(Xk[6]/1.0E-12);
            ST_special.ST_SP_Buffer.TM_B_EKF_Bias[1] = (int)(Xk[7]/1.0E-12);
            ST_special.ST_SP_Buffer.TM_B_EKF_Bias[2] = (int)(Xk[8]/1.0E-12);

            q_by_two[0][0] =  0.5 * qk_minus[3];
            q_by_two[0][1] = -0.5 * qk_minus[2];
            q_by_two[0][2] =  0.5 * qk_minus[1];
            q_by_two[1][0] =  0.5 * qk_minus[2];
            q_by_two[1][1] =  0.5 * qk_minus[3];
            q_by_two[1][2] = -0.5 * qk_minus[0];
            q_by_two[2][0] = -0.5 * qk_minus[1];
            q_by_two[2][1] =  0.5 * qk_minus[0];
            q_by_two[2][2] =  0.5 * qk_minus[3];
            q_by_two[3][0] = -0.5 * qk_minus[0];
            q_by_two[3][1] = -0.5 * qk_minus[1];
            q_by_two[3][2] = -0.5 * qk_minus[2];

            for (i_kf = 0; i_kf < 4; i_kf++)
            {
                qk_plus_temp[i_kf] = 0.0;
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    qk_plus_temp[i_kf]  = qk_plus_temp[i_kf] + q_by_two[i_kf][j_kf] * Xk_plus_temp[j_kf];
                }
            }


            qk_plus[0]  = qk_minus[0] + qk_plus_temp[0];
            qk_plus[1]  = qk_minus[1] + qk_plus_temp[1];
            qk_plus[2]  = qk_minus[2] + qk_plus_temp[2];
            qk_plus[3]  = qk_minus[3] + qk_plus_temp[3];

            rQs_Normalization(qk_plus);
            qk_plus[0] = out_Quat_norm[0];
            qk_plus[1] = out_Quat_norm[1];
            qk_plus[2] = out_Quat_norm[2];
            qk_plus[3] = out_Quat_norm[3];

        //*******************(Pk_plus_matrix_implementation)********************************//

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    Kk_Hk[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3; k_kf++)
                    {
                        Kk_Hk[i_kf][j_kf] = Kk_Hk[i_kf][j_kf] + Kk_matrix[i_kf][k_kf] * Hk[k_kf][j_kf];
                    }

                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    Kk_Hk_with_minus_one[i_kf][j_kf] = c_I_nine_cross_nine[i_kf][j_kf] - Kk_Hk[i_kf][j_kf];
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    pk_plus[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        pk_plus[i_kf][j_kf] =  pk_plus[i_kf][j_kf] + Kk_Hk_with_minus_one[i_kf][k_kf] * pk[k_kf][j_kf] ;
                    }
                }
            }
        }
    }
}
void rExtendedKalmanFilter1_p2(void)
{
    if (CB_ExtendedKalmanFilter == Enable)
    {
        if (TC_boolean_u.TC_Boolean_Table.TC_EKF1_Enable == Enable)
        {
            mod_of_Wk1 = sqrt((w_BODY[0] * w_BODY[0]) + (w_BODY[1] * w_BODY[1]) + (w_BODY[2] * w_BODY[2]));

            sine_of_Wk1_Delta = sin(mod_of_Wk1 * c_MaC);

            sine_of_Wk1_Delta_by_Wk1 = (sine_of_Wk1_Delta / mod_of_Wk1);


            Wk1_one[0][0] =  Wk1_one[1][1] =  Wk1_one[2][2] = 0.0;
            Wk1_one[0][1] = -1.0 * w_BODY[2];
            Wk1_one[0][2] =  w_BODY[1];
            Wk1_one[1][0] =  w_BODY[2];
            Wk1_one[2][0] = -1.0 * w_BODY[1];
            Wk1_one[2][1] =  w_BODY[0];
            Wk1_one[1][2] = -1.0 * w_BODY[0];

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_11_temp1[i_kf][j_kf] =  Wk1_one[i_kf][j_kf] * sine_of_Wk1_Delta_by_Wk1;
                }
            }


            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_11_temp2[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3; k_kf++)
                    {
                        phi_11_temp2[i_kf][j_kf] = phi_11_temp2[i_kf][j_kf] + Wk1_one[i_kf][k_kf] * Wk1_one[k_kf][j_kf];
                    }
                }
            }

            cos_of_Wk1_Delta = cos(mod_of_Wk1 * c_MaC);

            mod_of_Wk1_square = (mod_of_Wk1 * mod_of_Wk1);

            cos_of_Wk1_Delta_by_Wk1_square = ((1.0 - cos_of_Wk1_Delta) / mod_of_Wk1_square);

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_11_temp3[i_kf][j_kf] = phi_11_temp2[i_kf][j_kf] * cos_of_Wk1_Delta_by_Wk1_square;
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_11_temp4[i_kf][j_kf] = phi_11_temp1[i_kf][j_kf] - phi_11_temp3[i_kf][j_kf];
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_11[i_kf][j_kf] = c_I_three_cross_three[i_kf][j_kf] - phi_11_temp4[i_kf][j_kf];
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_12_temp1[i_kf][j_kf] = c_I_three_cross_three[i_kf][j_kf] * c_MaC;
                }
            }

            Result_of_sine_Delta = (( mod_of_Wk1 * c_MaC) - sine_of_Wk1_Delta);

            mod_of_Wk1_cube = (mod_of_Wk1 * mod_of_Wk1 * mod_of_Wk1);

            Result_of_sine_Delta_cube = Result_of_sine_Delta / mod_of_Wk1_cube;

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_12_temp2[i_kf][j_kf] = phi_11_temp2[i_kf][j_kf] * Result_of_sine_Delta_cube;
                }
            }


            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_12_temp3[i_kf][j_kf] = Wk1_one[i_kf][j_kf] * cos_of_Wk1_Delta_by_Wk1_square;
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_12[i_kf][j_kf] = phi_12_temp3[i_kf][j_kf] - phi_12_temp1[i_kf][j_kf] - phi_12_temp2[i_kf][j_kf];
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    if(i_kf==j_kf)
                    {
                        phi_k[i_kf][j_kf] = 1.0;
                    }
                    else
                    {
                        phi_k[i_kf][j_kf] = 0.0;
                    }
                }
            }

            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 0; j_kf < 3; j_kf++)
                {
                    phi_k[i_kf][j_kf] = phi_11[i_kf][j_kf];
                }
            }


            for (i_kf = 0; i_kf < 3; i_kf++)
            {
                for (j_kf = 3; j_kf < 6; j_kf++)
                {
                    phi_k[i_kf][j_kf] = phi_12[i_kf][j_kf-3];
                }
            }


            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    phi_k_T[i_kf][j_kf] = phi_k[j_kf][i_kf];
                }
            }

            Qk_temp1 = (((sigma_v * sigma_v) * c_MaC) + ( (1.0/3.0) * ((sigma_u*sigma_u) * (c_MaC*c_MaC*c_MaC))));
            Qk_temp2 = (1.0/2.0)*((sigma_u * sigma_u) * (c_MaC*c_MaC));
            Qk_temp3 = ((sigma_u * sigma_u) * c_MaC);
            Qk_temp4 = ((sigma_m * sigma_m) * c_MaC);


            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    Qk[i_kf][j_kf] = 0.0;

                }
            }

            Qk[0][0] = Qk[1][1] = Qk[2][2] = Qk_temp1;
            Qk[0][3] = Qk[1][4] = Qk[2][5] = Qk[3][0] = Qk[4][1] = Qk[5][2] = Qk_temp2;
            Qk[3][3] = Qk[4][4] = Qk[5][5] = Qk_temp3;
            Qk[6][6] = Qk[7][7] = Qk[8][8] = Qk_temp4;


            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    phi_k_pk_plus[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        phi_k_pk_plus[i_kf][j_kf] = phi_k_pk_plus[i_kf][j_kf] + phi_k[i_kf][k_kf] * pk_plus[k_kf][j_kf];
                    }
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    phi_k_pk_plus_phi_k_T[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        phi_k_pk_plus_phi_k_T[i_kf][j_kf] = phi_k_pk_plus_phi_k_T[i_kf][j_kf] + phi_k_pk_plus[i_kf][k_kf] * phi_k_T[k_kf][j_kf];
                    }
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    rk_Qk[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        rk_Qk[i_kf][j_kf] =  rk_Qk[i_kf][j_kf] + c_rk[i_kf][k_kf] * Qk[k_kf][j_kf];
                    }
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    rk_Qk_rk_T[i_kf][j_kf] = 0.0;
                    for (k_kf = 0; k_kf < 9; k_kf++)
                    {
                        rk_Qk_rk_T[i_kf][j_kf] = rk_Qk_rk_T[i_kf][j_kf] +rk_Qk[i_kf][k_kf] * c_rk_T[k_kf][j_kf];
                    }
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    pk_plus_one[i_kf][j_kf] =  phi_k_pk_plus_phi_k_T[i_kf][j_kf] + rk_Qk_rk_T[i_kf][j_kf];
                }
            }

            for (i_kf = 0; i_kf < 9; i_kf++)
            {
                for (j_kf = 0; j_kf < 9; j_kf++)
                {
                    pk[i_kf][j_kf] = pk_plus_one[i_kf][j_kf];
                }
            }
        }
    }
}

void rExtendedKalmanFilter2_p1(void)
{
    if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == Enable)
    {
        qk_DCM[0][0] = (qk_minus[3] * qk_minus[3]) + (qk_minus[0] * qk_minus[0]) - (qk_minus[1] * qk_minus[1]) - (qk_minus[2] * qk_minus[2]);
        qk_DCM[0][1] = 2.0 * ((qk_minus[0] * qk_minus[1]) + (qk_minus[3] * qk_minus[2]));
        qk_DCM[0][2] = 2.0 * ((qk_minus[0] * qk_minus[2]) - (qk_minus[3] * qk_minus[1]));
        qk_DCM[1][0] = 2.0 * ((qk_minus[0] * qk_minus[1]) - (qk_minus[3] * qk_minus[2]));
        qk_DCM[1][1] = (qk_minus[3] * qk_minus[3]) - (qk_minus[0] * qk_minus[0]) + (qk_minus[1] * qk_minus[1]) - (qk_minus[2] * qk_minus[2]);
        qk_DCM[1][2] = 2.0 * ((qk_minus[1] * qk_minus[2]) + (qk_minus[3] * qk_minus[0]));
        qk_DCM[2][0] = 2.0 * ((qk_minus[0] * qk_minus[2]) + (qk_minus[3] * qk_minus[1]));
        qk_DCM[2][1] = 2.0 * ((qk_minus[1] * qk_minus[2]) - (qk_minus[3] * qk_minus[0]));
        qk_DCM[2][2] = (qk_minus[3] * qk_minus[3]) - (qk_minus[0] * qk_minus[0]) - (qk_minus[1] * qk_minus[1]) + (qk_minus[2] * qk_minus[2]);

        Rk[0][0] = 1.0;
        Rk[0][1] = 0.0;
        Rk[0][2] = 0.0;
        Rk[1][0] = 0.0;
        Rk[1][1] = 1.0;
        Rk[1][2] = 0.0;
        Rk[2][0] = 0.0;
        Rk[2][1] = 0.0;
        Rk[2][2] = 1.0;

        if (f_Sunlit_Presence == 1)
            ecl_ekf = 0;
        else
            ecl_ekf = 2;

        for (kfmeas=ecl_ekf;kfmeas<3;kfmeas++)
        {
            if (kfmeas == 0)
            {
                Yk[0] = S_BODYn[0];
                Yk[1] = S_BODYn[1];
                Yk[2] = S_BODYn[2];

                hk_xk_minus[0] = S_ECI[0];
                hk_xk_minus[1] = S_ECI[1];
                hk_xk_minus[2] = S_ECI[2];

                Rk[0][0] = sun_noise * sun_noise;
                Rk[1][1] = sun_noise * sun_noise;
                Rk[2][2] = sun_noise * sun_noise;

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Hk[i_kf][j_kf] = 0.0;

                    }
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    hk[i_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3; k_kf++)
                    {
                        hk[i_kf] = hk[i_kf] + qk_DCM[i_kf][k_kf] * hk_xk_minus[k_kf];
                    }
                }

                ref_cross[0][0] =  0.0;
                ref_cross[0][1] =  -hk[2];
                ref_cross[0][2] =  hk[1];
                ref_cross[1][0] =  hk[2];
                ref_cross[1][1] =  0.0;
                ref_cross[1][2] =  -hk[0];
                ref_cross[2][0] =  -hk[1];
                ref_cross[2][1] =  hk[0];
                ref_cross[2][2] =  0.0;

                Hk[0][1] = ref_cross[0][1];
                Hk[0][2] = ref_cross[0][2];
                Hk[1][0] = ref_cross[1][0];
                Hk[1][2] = ref_cross[1][2];
                Hk[2][0] = ref_cross[2][0];
                Hk[2][1] = ref_cross[2][1];

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        Hk_temp[i_kf][j_kf] = Hk[j_kf][i_kf];

                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {

                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_HkT[i_kf][j_kf] =  pk_HkT[i_kf][j_kf] + pk[i_kf][k_kf] * Hk_temp[k_kf][j_kf];

                        }
                    }
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT_Hk[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_HkT_Hk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Hk[i_kf][k_kf] *pk_HkT[k_kf][j_kf] ;
                        }

                    }
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT_Hk_Rk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Rk[i_kf][j_kf];

                    }
                }

                rMatInv(pk_HkT_Hk_Rk);
                for(i_kf=0; i_kf<3; i_kf++)
                {
                    for(j_kf=0; j_kf<3; j_kf++)
                    {
                        K_temp[i_kf][j_kf] = Invmatout33[i_kf][j_kf];
                    }
                }


                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        Kk_matrix[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 3; k_kf++)
                        {
                            Kk_matrix[i_kf][j_kf] = Kk_matrix[i_kf][j_kf]  + pk_HkT[i_kf][k_kf] * K_temp[k_kf][j_kf];
                        }
                    }
                }


                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    Yk_minus_hk[i_kf] = Yk[i_kf] - hk[i_kf];
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Yk_minus_hk_Kk[i_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3 ; k_kf++)
                    {
                        Yk_minus_hk_Kk[i_kf] = Yk_minus_hk_Kk[i_kf] + Kk_matrix[i_kf][k_kf] * Yk_minus_hk[k_kf];
                    }
                }

                Xk[0] = 0.0;
                Xk[1] = 0.0;
                Xk[2] = 0.0;

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Xk_plus[i_kf] = Xk[i_kf] + Yk_minus_hk_Kk[i_kf];
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Xk[i_kf] =  Xk_plus[i_kf];
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    Xk_plus_temp[i_kf] = Xk_plus[i_kf];
                }

                w_k[0] = Xk[3];
                w_k[1] = Xk[4];
                w_k[2] = Xk[5];

                q_by_two[0][0] =  0.5 * qk_minus[3];
                q_by_two[0][1] = -0.5 * qk_minus[2];
                q_by_two[0][2] =  0.5 * qk_minus[1];
                q_by_two[1][0] =  0.5 * qk_minus[2];
                q_by_two[1][1] =  0.5 * qk_minus[3];
                q_by_two[1][2] = -0.5 * qk_minus[0];
                q_by_two[2][0] = -0.5 * qk_minus[1];
                q_by_two[2][1] =  0.5 * qk_minus[0];
                q_by_two[2][2] =  0.5 * qk_minus[3];
                q_by_two[3][0] = -0.5 * qk_minus[0];
                q_by_two[3][1] = -0.5 * qk_minus[1];
                q_by_two[3][2] = -0.5 * qk_minus[2];

                for (i_kf = 0; i_kf < 4; i_kf++)
                {
                    qk_plus_temp[i_kf] = 0.0;
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        qk_plus_temp[i_kf]  = qk_plus_temp[i_kf] + q_by_two[i_kf][j_kf] * Xk_plus_temp[j_kf];
                    }
                }


                qk_plus[0]  = qk_minus[0] + qk_plus_temp[0];
                qk_plus[1]  = qk_minus[1] + qk_plus_temp[1];
                qk_plus[2]  = qk_minus[2] + qk_plus_temp[2];
                qk_plus[3]  = qk_minus[3] + qk_plus_temp[3];

                rQs_Normalization(qk_plus);
                qk_plus[0] = out_Quat_norm[0];
                qk_plus[1] = out_Quat_norm[1];
                qk_plus[2] = out_Quat_norm[2];
                qk_plus[3] = out_Quat_norm[3];

                qk_minus[0] = qk_plus[0];
                qk_minus[1] = qk_plus[1];
                qk_minus[2] = qk_plus[2];
                qk_minus[3] = qk_plus[3];


                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Kk_Hk[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 3; k_kf++)
                        {
                            Kk_Hk[i_kf][j_kf] = Kk_Hk[i_kf][j_kf] + Kk_matrix[i_kf][k_kf] * Hk[k_kf][j_kf];
                        }

                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Kk_Hk_with_minus_one[i_kf][j_kf] = c_I_nine_cross_nine[i_kf][j_kf] - Kk_Hk[i_kf][j_kf];
                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        pk_plus[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_plus[i_kf][j_kf] =  pk_plus[i_kf][j_kf] + Kk_Hk_with_minus_one[i_kf][k_kf] * pk[k_kf][j_kf] ;
                        }
                    }
                }
            }

            else if (kfmeas == 1)
            {
                Yk[0] = B_BODY_LUT[0] * 1.0e-9;
                Yk[1] = B_BODY_LUT[1] * 1.0e-9;
                Yk[2] = B_BODY_LUT[2] * 1.0e-9;

                hk_xk_minus[0] = B_ECItesla[0];
                hk_xk_minus[1] = B_ECItesla[1];
                hk_xk_minus[2] = B_ECItesla[2];

                Rk[0][0] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;
                Rk[1][1] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;
                Rk[2][2] = mag_noise * 1.0e-9 * mag_noise * 1.0e-9;

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Hk[i_kf][j_kf] = 0.0;

                    }
                }

                Hk[0][6] = 1.0;
                Hk[1][7] = 1.0;
                Hk[2][8] = 1.0;

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    hk[i_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3; k_kf++)
                    {
                        hk[i_kf] = hk[i_kf] + qk_DCM[i_kf][k_kf] * hk_xk_minus[k_kf];
                    }
                }

                ref_cross[0][0] =  0.0;
                ref_cross[0][1] =  -hk[2];
                ref_cross[0][2] =  hk[1];
                ref_cross[1][0] =  hk[2];
                ref_cross[1][1] =  0.0;
                ref_cross[1][2] =  -hk[0];
                ref_cross[2][0] =  -hk[1];
                ref_cross[2][1] =  hk[0];
                ref_cross[2][2] =  0.0;

                Hk[0][1] = ref_cross[0][1];
                Hk[0][2] = ref_cross[0][2];
                Hk[1][0] = ref_cross[1][0];
                Hk[1][2] = ref_cross[1][2];
                Hk[2][0] = ref_cross[2][0];
                Hk[2][1] = ref_cross[2][1];

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        Hk_temp[i_kf][j_kf] = Hk[j_kf][i_kf];

                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {

                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_HkT[i_kf][j_kf] =  pk_HkT[i_kf][j_kf] + pk[i_kf][k_kf] * Hk_temp[k_kf][j_kf];

                        }
                    }
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT_Hk[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_HkT_Hk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Hk[i_kf][k_kf] *pk_HkT[k_kf][j_kf] ;
                        }

                    }
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        pk_HkT_Hk_Rk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Rk[i_kf][j_kf];

                    }
                }

                rMatInv(pk_HkT_Hk_Rk);
                for(i_kf=0; i_kf<3; i_kf++)
                {
                    for(j_kf=0; j_kf<3; j_kf++)
                    {
                        K_temp[i_kf][j_kf] = Invmatout33[i_kf][j_kf];
                    }
                }


                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        Kk_matrix[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 3; k_kf++)
                        {
                            Kk_matrix[i_kf][j_kf] = Kk_matrix[i_kf][j_kf]  + pk_HkT[i_kf][k_kf] * K_temp[k_kf][j_kf];
                        }
                    }
                }


//                if (TC_EKF_MagBias_Compensation_Enable_or_Disable == 0)
//				{
					Yk_minus_hk[0] = Yk[0] - (hk[0]+Xk_plus[6]);
					Yk_minus_hk[1] = Yk[1] - (hk[1]+Xk_plus[7]);
					Yk_minus_hk[2] = Yk[2] - (hk[2]+Xk_plus[8]);
//				}
//				else
//				{
//					for (i_kf = 0; i_kf < 3; i_kf++)
//					{
//						Yk_minus_hk[i_kf] = Yk[i_kf] - hk[i_kf];
//					}
//				}

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Yk_minus_hk_Kk[i_kf] = 0.0;
                    for (k_kf = 0; k_kf < 3 ; k_kf++)
                    {
                        Yk_minus_hk_Kk[i_kf] = Yk_minus_hk_Kk[i_kf] + Kk_matrix[i_kf][k_kf] * Yk_minus_hk[k_kf];
                    }
                }

                Xk[0] = 0.0;
                Xk[1] = 0.0;
                Xk[2] = 0.0;

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Xk_plus[i_kf] = Xk[i_kf] + Yk_minus_hk_Kk[i_kf];
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    Xk[i_kf] =  Xk_plus[i_kf];
                }

                for (i_kf = 0; i_kf < 3; i_kf++)
                {
                    Xk_plus_temp[i_kf] = Xk_plus[i_kf];
                }

                w_k[0] = Xk[3];
                w_k[1] = Xk[4];
                w_k[2] = Xk[5];

                q_by_two[0][0] =  0.5 * qk_minus[3];
                q_by_two[0][1] = -0.5 * qk_minus[2];
                q_by_two[0][2] =  0.5 * qk_minus[1];
                q_by_two[1][0] =  0.5 * qk_minus[2];
                q_by_two[1][1] =  0.5 * qk_minus[3];
                q_by_two[1][2] = -0.5 * qk_minus[0];
                q_by_two[2][0] = -0.5 * qk_minus[1];
                q_by_two[2][1] =  0.5 * qk_minus[0];
                q_by_two[2][2] =  0.5 * qk_minus[3];
                q_by_two[3][0] = -0.5 * qk_minus[0];
                q_by_two[3][1] = -0.5 * qk_minus[1];
                q_by_two[3][2] = -0.5 * qk_minus[2];

                for (i_kf = 0; i_kf < 4; i_kf++)
                {
                    qk_plus_temp[i_kf] = 0.0;
                    for (j_kf = 0; j_kf < 3; j_kf++)
                    {
                        qk_plus_temp[i_kf]  = qk_plus_temp[i_kf] + q_by_two[i_kf][j_kf] * Xk_plus_temp[j_kf];
                    }
                }


                qk_plus[0]  = qk_minus[0] + qk_plus_temp[0];
                qk_plus[1]  = qk_minus[1] + qk_plus_temp[1];
                qk_plus[2]  = qk_minus[2] + qk_plus_temp[2];
                qk_plus[3]  = qk_minus[3] + qk_plus_temp[3];

                rQs_Normalization(qk_plus);
                qk_plus[0] = out_Quat_norm[0];
                qk_plus[1] = out_Quat_norm[1];
                qk_plus[2] = out_Quat_norm[2];
                qk_plus[3] = out_Quat_norm[3];

                qk_minus[0] = qk_plus[0];
                qk_minus[1] = qk_plus[1];
                qk_minus[2] = qk_plus[2];
                qk_minus[3] = qk_plus[3];


                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Kk_Hk[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 3; k_kf++)
                        {
                            Kk_Hk[i_kf][j_kf] = Kk_Hk[i_kf][j_kf] + Kk_matrix[i_kf][k_kf] * Hk[k_kf][j_kf];
                        }

                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        Kk_Hk_with_minus_one[i_kf][j_kf] = c_I_nine_cross_nine[i_kf][j_kf] - Kk_Hk[i_kf][j_kf];
                    }
                }

                for (i_kf = 0; i_kf < 9; i_kf++)
                {
                    for (j_kf = 0; j_kf < 9; j_kf++)
                    {
                        pk_plus[i_kf][j_kf] = 0.0;
                        for (k_kf = 0; k_kf < 9; k_kf++)
                        {
                            pk_plus[i_kf][j_kf] =  pk_plus[i_kf][j_kf] + Kk_Hk_with_minus_one[i_kf][k_kf] * pk[k_kf][j_kf] ;
                        }
                    }
                }
            }
            else
            {
                if (Quest_update_available == 1 && w_q_update_satisfy == 1 && OBC_Quest_update == 1)
                {

                    Rk[0][0] = 0.000097*0.000097;
                    Rk[1][1] = 0.000097*0.000097;
                    Rk[2][2] = 0.000097*0.000097;

                    for (i_kf = 0; i_kf < 3; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 9; j_kf++)
                        {
                            Hk[i_kf][j_kf] = 0.0;

                        }
                    }

                    Hk[0][0] = 1.0;
                    Hk[1][1] = 1.0;
                    Hk[2][2] = 1.0;

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            Hk_temp[i_kf][j_kf] = Hk[j_kf][i_kf];

                        }
                    }

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {

                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            pk_HkT[i_kf][j_kf] = 0.0;
                            for (k_kf = 0; k_kf < 9; k_kf++)
                            {
                                pk_HkT[i_kf][j_kf] =  pk_HkT[i_kf][j_kf] + pk[i_kf][k_kf] * Hk_temp[k_kf][j_kf];

                            }
                        }
                    }

                    for (i_kf = 0; i_kf < 3; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            pk_HkT_Hk[i_kf][j_kf] = 0.0;
                            for (k_kf = 0; k_kf < 9; k_kf++)
                            {
                                pk_HkT_Hk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Hk[i_kf][k_kf] *pk_HkT[k_kf][j_kf] ;
                            }

                        }
                    }

                    for (i_kf = 0; i_kf < 3; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            pk_HkT_Hk_Rk[i_kf][j_kf] = pk_HkT_Hk[i_kf][j_kf] + Rk[i_kf][j_kf];

                        }
                    }

                    rMatInv(pk_HkT_Hk_Rk);
                    for(i_kf=0; i_kf<3; i_kf++)
                    {
                        for(j_kf=0; j_kf<3; j_kf++)
                        {
                            K_temp[i_kf][j_kf] = Invmatout33[i_kf][j_kf];
                        }
                    }


                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            Kk_matrix[i_kf][j_kf] = 0.0;
                            for (k_kf = 0; k_kf < 3; k_kf++)
                            {
                                Kk_matrix[i_kf][j_kf] = Kk_matrix[i_kf][j_kf]  + pk_HkT[i_kf][k_kf] * K_temp[k_kf][j_kf];
                            }
                        }
                    }

                    qk_plus_conj[0] = -qk_plus[0];
                    qk_plus_conj[1] = -qk_plus[1];
                    qk_plus_conj[2] = -qk_plus[2];
                    qk_plus_conj[3] = qk_plus[3];

                    rQs_Multiplication(qk_plus_conj,Qquest_update);
                    rQs_Normalization(out_Quat_mult);
                    Yk_minus_hk[0] = out_Quat_norm[0];
                    Yk_minus_hk[1] = out_Quat_norm[1];
                    Yk_minus_hk[2] = out_Quat_norm[2];

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        Yk_minus_hk_Kk[i_kf] = 0.0;
                        for (k_kf = 0; k_kf < 3 ; k_kf++)
                        {
                            Yk_minus_hk_Kk[i_kf] = Yk_minus_hk_Kk[i_kf] + Kk_matrix[i_kf][k_kf] * Yk_minus_hk[k_kf];
                        }
                    }

                    Xk[0] = 0.0;
                    Xk[1] = 0.0;
                    Xk[2] = 0.0;

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        Xk_plus[i_kf] = Xk[i_kf] + Yk_minus_hk_Kk[i_kf];
                    }

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        Xk[i_kf] =  Xk_plus[i_kf];
                    }

                    for (i_kf = 0; i_kf < 3; i_kf++)
                    {
                        Xk_plus_temp[i_kf] = Xk_plus[i_kf];
                    }

                    w_k[0] = Xk[3];
                    w_k[1] = Xk[4];
                    w_k[2] = Xk[5];

                    q_by_two[0][0] =  0.5 * qk_minus[3];
                    q_by_two[0][1] = -0.5 * qk_minus[2];
                    q_by_two[0][2] =  0.5 * qk_minus[1];
                    q_by_two[1][0] =  0.5 * qk_minus[2];
                    q_by_two[1][1] =  0.5 * qk_minus[3];
                    q_by_two[1][2] = -0.5 * qk_minus[0];
                    q_by_two[2][0] = -0.5 * qk_minus[1];
                    q_by_two[2][1] =  0.5 * qk_minus[0];
                    q_by_two[2][2] =  0.5 * qk_minus[3];
                    q_by_two[3][0] = -0.5 * qk_minus[0];
                    q_by_two[3][1] = -0.5 * qk_minus[1];
                    q_by_two[3][2] = -0.5 * qk_minus[2];

                    for (i_kf = 0; i_kf < 4; i_kf++)
                    {
                        qk_plus_temp[i_kf] = 0.0;
                        for (j_kf = 0; j_kf < 3; j_kf++)
                        {
                            qk_plus_temp[i_kf]  = qk_plus_temp[i_kf] + q_by_two[i_kf][j_kf] * Xk_plus_temp[j_kf];
                        }
                    }


                    qk_plus[0]  = qk_minus[0] + qk_plus_temp[0];
                    qk_plus[1]  = qk_minus[1] + qk_plus_temp[1];
                    qk_plus[2]  = qk_minus[2] + qk_plus_temp[2];
                    qk_plus[3]  = qk_minus[3] + qk_plus_temp[3];

                    rQs_Normalization(qk_plus);
                    qk_plus[0] = out_Quat_norm[0];
                    qk_plus[1] = out_Quat_norm[1];
                    qk_plus[2] = out_Quat_norm[2];
                    qk_plus[3] = out_Quat_norm[3];

                    qk_minus[0] = qk_plus[0];
                    qk_minus[1] = qk_plus[1];
                    qk_minus[2] = qk_plus[2];
                    qk_minus[3] = qk_plus[3];


                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 9; j_kf++)
                        {
                            Kk_Hk[i_kf][j_kf] = 0.0;
                            for (k_kf = 0; k_kf < 3; k_kf++)
                            {
                                Kk_Hk[i_kf][j_kf] = Kk_Hk[i_kf][j_kf] + Kk_matrix[i_kf][k_kf] * Hk[k_kf][j_kf];
                            }

                        }
                    }

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 9; j_kf++)
                        {
                            Kk_Hk_with_minus_one[i_kf][j_kf] = c_I_nine_cross_nine[i_kf][j_kf] - Kk_Hk[i_kf][j_kf];
                        }
                    }

                    for (i_kf = 0; i_kf < 9; i_kf++)
                    {
                        for (j_kf = 0; j_kf < 9; j_kf++)
                        {
                            pk_plus[i_kf][j_kf] = 0.0;
                            for (k_kf = 0; k_kf < 9; k_kf++)
                            {
                                pk_plus[i_kf][j_kf] =  pk_plus[i_kf][j_kf] + Kk_Hk_with_minus_one[i_kf][k_kf] * pk[k_kf][j_kf] ;
                            }
                        }
                    }
                    Quest_update_available = 0;
                    f_EKF2_prop_en = 1;
                }
            }
        }
    }
    else
    {
        Quest_update_available = 0;
    }
}

void rExtendedKalmanFilter2_Prop(void)
{
    if (TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == 1)
    {
        rMatMul34x1(wh2B_mat, T_RW);
        T_RW_NETk[0] = -Matout341[0];
        T_RW_NETk[1] = -Matout341[1];
        T_RW_NETk[2] = -Matout341[2];

        tor_counterk ++;
        if(tor_counterk <= 8)
        {
            if(tor_counterk <= Ton[0])
            {
                Mk[0] = c_DPMMAX * (double)DPM_Polarity[0];
            }
            else
            {
                Mk[0] = 0.0;
            }

            if(tor_counterk <= Ton[1])
            {
                Mk[1] = c_DPMMAX * (double)DPM_Polarity[1];
            }
            else
            {
                Mk[1] = 0.0;
            }

            if(tor_counterk <= Ton[2])
            {
                Mk[2] = c_DPMMAX * (double)DPM_Polarity[2];
            }
            else
            {
                Mk[2] = 0.0;
            }
            //Torquer_Actuate = 0;
        }

        rCross_Product(Mk,B_BODYtesla);
        T_MTk[0] = Cross_Product[0];
        T_MTk[1] = Cross_Product[1];
        T_MTk[2] = Cross_Product[2];

//        T_NETk[0] = T_RW_NETk[0] + T_MTk[0] + T_DIST[0];
//        T_NETk[1] = T_RW_NETk[1] + T_MTk[1] + T_DIST[1];
//        T_NETk[2] = T_RW_NETk[2] + T_MTk[2] + T_DIST[2];

        rEKFDynamics();

        Xk_plus[3] = w_k[0];
        Xk_plus[4] = w_k[1];
        Xk_plus[5] = w_k[2];

        Xk[3] = w_k[0];
        Xk[4] = w_k[1];
        Xk[5] = w_k[2];

        qk_minus[0] = qk_plus[0];
        qk_minus[1] = qk_plus[1];
        qk_minus[2] = qk_plus[2];
        qk_minus[3] = qk_plus[3];
    }

}

void rExtendedKalmanFilter2_p2(void)
{
    if ((TC_boolean_u.TC_Boolean_Table.TC_EKF2_Enable == Enable)&& (f_EKF2_prop_en == 1))
    {
        f_EKF2_prop_en = 0;

        F_k1[0][0] =  F_k1[1][1] =  F_k1[2][2] = 0.0;
        F_k1[0][1] = w_k[2];
        F_k1[0][2] = -w_k[1];
        F_k1[1][0] = -w_k[2];
        F_k1[1][2] = w_k[0];
        F_k1[2][0] = w_k[1];
        F_k1[2][1] = -w_k[0];

        mod_of_w_k = sqrt((w_k[0] * w_k[0]) + (w_k[1] * w_k[1]) + (w_k[2] * w_k[2]));

        jw_cross[0][0] =  jw_cross[1][1] =  jw_cross[2][2] = 0.0;
        jw_cross[0][1] = -1.0 * (I_MAT[2][2] * w_k[2] + HB[2]);
        jw_cross[0][2] =  I_MAT[1][1] * w_k[1] + HB[1];
        jw_cross[1][0] =  I_MAT[2][2] * w_k[2] + HB[2];
        jw_cross[1][2] = -1.0 * (I_MAT[0][0] * w_k[0] + HB[0]);
        jw_cross[2][0] = -1.0 * (I_MAT[1][1] * w_k[1] + HB[1]);
        jw_cross[2][1] =  I_MAT[0][0] * w_k[0] + HB[0];


        rMatMul3x3(F_k1, I_MAT);
        for (i_kf = 0; i_kf < 3; i_kf++)
        {
            for (j_kf = 0; j_kf < 3; j_kf++)
            {
                F_k4_temp1[i_kf][j_kf] =  Matout33[i_kf][j_kf];
            }
        }

        for (i_kf = 0; i_kf < 3; i_kf++)
        {
            for (j_kf = 0; j_kf < 3; j_kf++)
            {
                F_k4_temp2[i_kf][j_kf] =  F_k4_temp1[i_kf][j_kf] - jw_cross[i_kf][j_kf];
            }
        }

        rMatMul3x3(I_MAT_Inv, F_k4_temp2);
        for (i_kf = 0; i_kf < 3; i_kf++)
        {
            for (j_kf = 0; j_kf < 3; j_kf++)
            {
                F_k4[i_kf][j_kf] = -Matout33[i_kf][j_kf];
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                F_k[i_kf][j_kf] = 0.0;

            }
        }

        for (i_kf = 0; i_kf < 3; i_kf++)
        {
            for (j_kf = 0; j_kf < 3; j_kf++)
            {
                F_k[i_kf][j_kf] =  F_k1[i_kf][j_kf] * c_MaC;
            }
        }

        F_k[0][3] = F_k[1][4] = F_k[2][5] = 0.5 * c_MaC;

        for (i_kf = 0; i_kf < 3; i_kf++)
        {
            for (j_kf = 0; j_kf < 3; j_kf++)
            {
                F_k[i_kf+3][j_kf+4] =  F_k4[i_kf][j_kf] * c_MaC;
            }
        }

        F_k[6][6] = F_k[7][7] = F_k[8][8] = 1.7583e-04  * c_MaC;

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                phi_k[i_kf][j_kf] = c_I_nine_cross_nine[i_kf][j_kf] + F_k[i_kf][j_kf];

            }
        }


        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                phi_k_T[i_kf][j_kf] = phi_k[j_kf][i_kf];
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            Xk_plus[i_kf] = 0.0;
            for (k_kf = 0; k_kf < 9 ; k_kf++)
            {
                Xk_plus[i_kf] = Xk_plus[i_kf] + phi_k[i_kf][k_kf] * Xk[k_kf];
            }
        }

        if ((f_Sunlit_Presence == 1)&& (Sunlit_presence_timer > 4650) && f_station_tracking_enabled == 0) //count after eclipse
        {
            b_k[0] = Xk_plus[6]*1.0e9;
            b_k[1] = Xk_plus[7]*1.0e9;
            b_k[2] = Xk_plus[8]*1.0e9;

            lpfx_magbk = pbk * b_k[0] + qqbk * lpfx_magbk;
            lpfy_magbk = pbk * b_k[1] + qqbk * lpfy_magbk;
            lpfz_magbk = pbk * b_k[2] + qqbk * lpfz_magbk;

            lpfb_k_prev[0] = lpfb_k[0];
            lpfb_k_prev[1] = lpfb_k[1];
            lpfb_k_prev[2] = lpfb_k[2];

            lpfb_k[0] = lpfx_magbk*1.0e-9;
            lpfb_k[1] = lpfy_magbk*1.0e-9;
            lpfb_k[2] = lpfz_magbk*1.0e-9;

        }

        Qk_temp1 = (((sigma_v * sigma_v) * c_MaC) + ( (1.0/3.0) * ((sigma_u*sigma_u) * (c_MaC*c_MaC*c_MaC))));
        Qk_temp2 = (1.0/2.0)*((sigma_u * sigma_u) * (c_MaC*c_MaC));
        Qk_temp3 = ((sigma_u * sigma_u) * c_MaC);
        Qk_temp4 = ((sigma_m * sigma_m) * c_MaC);


        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                Qk[i_kf][j_kf] = 0.0;

            }
        }

        Qk[0][0] = Qk[1][1] = Qk[2][2] = Qk_temp1;
        Qk[0][3] = Qk[1][4] = Qk[2][5] = Qk[3][0] = Qk[4][1] = Qk[5][2] = Qk_temp2;
        Qk[3][3] = Qk[4][4] = Qk[5][5] = Qk_temp3;
        Qk[6][6] = Qk[7][7] = Qk[8][8] = Qk_temp4;


        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                phi_k_pk_plus[i_kf][j_kf] = 0.0;
                for (k_kf = 0; k_kf < 9; k_kf++)
                {
                    phi_k_pk_plus[i_kf][j_kf] = phi_k_pk_plus[i_kf][j_kf] + phi_k[i_kf][k_kf] * pk_plus[k_kf][j_kf];
                }
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                phi_k_pk_plus_phi_k_T[i_kf][j_kf] = 0.0;
                for (k_kf = 0; k_kf < 9; k_kf++)
                {
                    phi_k_pk_plus_phi_k_T[i_kf][j_kf] = phi_k_pk_plus_phi_k_T[i_kf][j_kf] + phi_k_pk_plus[i_kf][k_kf] * phi_k_T[k_kf][j_kf];
                }
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                rk_Qk[i_kf][j_kf] = 0.0;
                for (k_kf = 0; k_kf < 9; k_kf++)
                {
                    rk_Qk[i_kf][j_kf] =  rk_Qk[i_kf][j_kf] + c_rk[i_kf][k_kf] * Qk[k_kf][j_kf];
                }
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                rk_Qk_rk_T[i_kf][j_kf] = 0.0;
                for (k_kf = 0; k_kf < 9; k_kf++)
                {
                    rk_Qk_rk_T[i_kf][j_kf] = rk_Qk_rk_T[i_kf][j_kf] +rk_Qk[i_kf][k_kf] * c_rk_T[k_kf][j_kf];
                }
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                pk_plus_one[i_kf][j_kf] =  phi_k_pk_plus_phi_k_T[i_kf][j_kf] + rk_Qk_rk_T[i_kf][j_kf];
            }
        }

        for (i_kf = 0; i_kf < 9; i_kf++)
        {
            for (j_kf = 0; j_kf < 9; j_kf++)
            {
                pk[i_kf][j_kf] = pk_plus_one[i_kf][j_kf];
            }
        }
    }
}

void rEKFDynamics(void)
{
    rEKF_dy_int(qk_plus, w_k, RWSpeed);
    kfk1[0] = kfq_dot[0]*c_MiC;
    kfk1[1] = kfq_dot[1]*c_MiC;
    kfk1[2] = kfq_dot[2]*c_MiC;
    kfk1[3] = kfq_dot[3]*c_MiC;

    kfg1[0] = kfw_dot[0]*c_MiC;
    kfg1[1] = kfw_dot[1]*c_MiC;
    kfg1[2] = kfw_dot[2]*c_MiC;

    kfv1[0] = kfv_dot[0]*c_MiC;
    kfv1[1] = kfv_dot[1]*c_MiC;
    kfv1[2] = kfv_dot[2]*c_MiC;
    kfv1[3] = kfv_dot[3]*c_MiC;

    kfq01[0] = qk_plus[0] + 0.5*kfk1[0];
    kfq01[1] = qk_plus[1] + 0.5*kfk1[1];
    kfq01[2] = qk_plus[2] + 0.5*kfk1[2];
    kfq01[3] = qk_plus[3] + 0.5*kfk1[3];

    kfw01[0] = w_k[0] + 0.5*kfg1[0];
    kfw01[1] = w_k[1] + 0.5*kfg1[1];
    kfw01[2] = w_k[2] + 0.5*kfg1[2];

    kfv01[0] = RWSpeed[0] + 0.5*kfv1[0];
    kfv01[1] = RWSpeed[1] + 0.5*kfv1[1];
    kfv01[2] = RWSpeed[2] + 0.5*kfv1[2];
    kfv01[3] = RWSpeed[3] + 0.5*kfv1[3];

    rEKF_dy_int(kfq01, kfw01, kfv01);
    kfk2[0] = kfq_dot[0]*c_MiC;
    kfk2[1] = kfq_dot[1]*c_MiC;
    kfk2[2] = kfq_dot[2]*c_MiC;
    kfk2[3] = kfq_dot[3]*c_MiC;

    kfg2[0] = kfw_dot[0]*c_MiC;
    kfg2[1] = kfw_dot[1]*c_MiC;
    kfg2[2] = kfw_dot[2]*c_MiC;

    kfv2[0] = kfv_dot[0]*c_MiC;
    kfv2[1] = kfv_dot[1]*c_MiC;
    kfv2[2] = kfv_dot[2]*c_MiC;
    kfv2[3] = kfv_dot[3]*c_MiC;

    kfq02[0] = qk_plus[0] + 0.5*kfk2[0];
    kfq02[1] = qk_plus[1] + 0.5*kfk2[1];
    kfq02[2] = qk_plus[2] + 0.5*kfk2[2];
    kfq02[3] = qk_plus[3] + 0.5*kfk2[3];

    kfw02[0] = w_k[0] + 0.5*kfg2[0];
    kfw02[1] = w_k[1] + 0.5*kfg2[1];
    kfw02[2] = w_k[2] + 0.5*kfg2[2];

    kfv02[0] = RWSpeed[0] + 0.5*kfv2[0];
    kfv02[1] = RWSpeed[1] + 0.5*kfv2[1];
    kfv02[2] = RWSpeed[2] + 0.5*kfv2[2];
    kfv02[3] = RWSpeed[3] + 0.5*kfv2[3];

    rEKF_dy_int(kfq02,kfw02, kfv02);
    kfk3[0] = kfq_dot[0]*c_MiC;
    kfk3[1] = kfq_dot[1]*c_MiC;
    kfk3[2] = kfq_dot[2]*c_MiC;
    kfk3[3] = kfq_dot[3]*c_MiC;

    kfg3[0] = kfw_dot[0]*c_MiC;
    kfg3[1] = kfw_dot[1]*c_MiC;
    kfg3[2] = kfw_dot[2]*c_MiC;

    kfv3[0] = kfv_dot[0]*c_MiC;
    kfv3[1] = kfv_dot[1]*c_MiC;
    kfv3[2] = kfv_dot[2]*c_MiC;
    kfv3[3] = kfv_dot[3]*c_MiC;

    kfq03[0] = qk_plus[0] + kfk3[0];
    kfq03[1] = qk_plus[1] + kfk3[1];
    kfq03[2] = qk_plus[2] + kfk3[2];
    kfq03[3] = qk_plus[3] + kfk3[3];

    kfw03[0] = w_k[0] + kfg3[0];
    kfw03[1] = w_k[1] + kfg3[1];
    kfw03[2] = w_k[2] + kfg3[2];

    kfv03[0] = RWSpeed[0] + kfv3[0];
    kfv03[1] = RWSpeed[1] + kfv3[1];
    kfv03[2] = RWSpeed[2] + kfv3[2];
    kfv03[3] = RWSpeed[3] + kfv3[3];

    rEKF_dy_int(kfq03,kfw03, kfv03);
    kfk4[0] = kfq_dot[0]*c_MiC;
    kfk4[1] = kfq_dot[1]*c_MiC;
    kfk4[2] = kfq_dot[2]*c_MiC;
    kfk4[3] = kfq_dot[3]*c_MiC;

    kfg4[0] = kfw_dot[0]*c_MiC;
    kfg4[1] = kfw_dot[1]*c_MiC;
    kfg4[2] = kfw_dot[2]*c_MiC;

    qk_plus[0] = qk_plus[0] + (1.0/6.0)*(kfk1[0]+2.0*kfk2[0]+2.0*kfk3[0]+kfk4[0]);
    qk_plus[1] = qk_plus[1] + (1.0/6.0)*(kfk1[1]+2.0*kfk2[1]+2.0*kfk3[1]+kfk4[1]);
    qk_plus[2] = qk_plus[2] + (1.0/6.0)*(kfk1[2]+2.0*kfk2[2]+2.0*kfk3[2]+kfk4[2]);
    qk_plus[3] = qk_plus[3] + (1.0/6.0)*(kfk1[3]+2.0*kfk2[3]+2.0*kfk3[3]+kfk4[3]);

    rQs_Normalization(qk_plus);
    qk_plus[0] = out_Quat_norm[0];
    qk_plus[1] = out_Quat_norm[1];
    qk_plus[2] = out_Quat_norm[2];
    qk_plus[3] = out_Quat_norm[3];

    w_k[0] = w_k[0] + (1.0/6.0)*(kfg1[0]+2.0*kfg2[0]+2.0*kfg3[0]+kfg4[0]);
    w_k[1] = w_k[1] + (1.0/6.0)*(kfg1[1]+2.0*kfg2[1]+2.0*kfg3[1]+kfg4[1]);
    w_k[2] = w_k[2] + (1.0/6.0)*(kfg1[2]+2.0*kfg2[2]+2.0*kfg3[2]+kfg4[2]);
}


void rEKF_dy_int(double kfq_dy[4], double kfw_dy[3], double kfv_dy[4])
{

    kfom[0][0] = kfom[1][1] = kfom[2][2] = kfom[3][3] = 0.0;
    kfom[0][1] = kfw_dy[2];
    kfom[0][1] = -kfw_dy[1];
    kfom[0][1] = kfw_dy[0];
    kfom[1][0] = -kfw_dy[2];
    kfom[1][2] = kfw_dy[0];
    kfom[1][3] = kfw_dy[1];
    kfom[2][0] = kfw_dy[1];
    kfom[2][1] = -kfw_dy[0];
    kfom[2][3] = kfw_dy[2];
    kfom[3][0] = -kfw_dy[0];
    kfom[3][1] = -kfw_dy[1];
    kfom[3][2] = -kfw_dy[2];

    rMatMul44x1(kfom, kfq_dy);
    kfq_dot[0] = 0.5 * Matout441[0];
    kfq_dot[1] = 0.5 * Matout441[1];
    kfq_dot[2] = 0.5 * Matout441[2];
    kfq_dot[3] = 0.5 * Matout441[3];

    rMatMul44x1(MOI_wh_mat_Inv,T_RW);
    kfv_dot[0] = Matout441[0];
    kfv_dot[1] = Matout441[1];
    kfv_dot[2] = Matout441[2];
    kfv_dot[3] = Matout441[3];

    rMatMul44x1(MOI_wh_mat,kfv_dot);
    kfT_RW_NET_temp[0] = Matout441[0];
    kfT_RW_NET_temp[1] = Matout441[1];
    kfT_RW_NET_temp[2] = Matout441[2];
    kfT_RW_NET_temp[3] = Matout441[3];

    rMatMul34x1(wh2B_mat, kfT_RW_NET_temp);
    kfT_RW_NET[0] = -Matout341[0];
    kfT_RW_NET[1] = -Matout341[1];
    kfT_RW_NET[2] = -Matout341[2];

    ///rMatMul44x1(MOI_wh,RWSpeed);
    kfH_wh_ind[0] = c_MOI_wh * kfv_dy[0];
    kfH_wh_ind[1] = c_MOI_wh * kfv_dy[1];
    kfH_wh_ind[2] = c_MOI_wh * kfv_dy[2];
    kfH_wh_ind[3] = c_MOI_wh * kfv_dy[3];

    //H_wh_ind = -MOI_wh*RWSpeed; //Angular momentum generated by the individual wheel (Nms)

    rMatMul34x1(wh2B_mat, kfH_wh_ind);
    kfH_wh2body[0] = Matout341[0];
    kfH_wh2body[1] = Matout341[1];
    kfH_wh2body[2] = Matout341[2];

    //H_wh2body = c_wh2B_mat_4RW*H_wh_ind;
    rMatMul3x1(I_MAT, kfw_dy);
    kfI_MAT_w0[0] = Matout31[0];
    kfI_MAT_w0[1] = Matout31[1];
    kfI_MAT_w0[2] = Matout31[2];

    kfI_MAT_w0_HW2B[0] = kfI_MAT_w0[0] + kfH_wh2body[0];
    kfI_MAT_w0_HW2B[1] = kfI_MAT_w0[1] + kfH_wh2body[1];
    kfI_MAT_w0_HW2B[2] = kfI_MAT_w0[2] + kfH_wh2body[2];

    kfT_NET[0] = kfT_RW_NET[0] + T_MTk[0];// + T_DIST[0];
    kfT_NET[1] = kfT_RW_NET[1] + T_MTk[1];// + T_DIST[1];
    kfT_NET[2] = kfT_RW_NET[2] + T_MTk[2];// + T_DIST[2];

    rCross_Product(kfw_dy,kfI_MAT_w0_HW2B);
    kfw0crossI_MAT_w0_HW2B[0] = Cross_Product[0];
    kfw0crossI_MAT_w0_HW2B[1] = Cross_Product[1];
    kfw0crossI_MAT_w0_HW2B[2] = Cross_Product[2];

    kfw_dot_temp[0] = kfT_NET[0] - kfw0crossI_MAT_w0_HW2B[0];
    kfw_dot_temp[1] = kfT_NET[1] - kfw0crossI_MAT_w0_HW2B[1];
    kfw_dot_temp[2] = kfT_NET[2] - kfw0crossI_MAT_w0_HW2B[2];

    rMatMul3x1(I_MAT_Inv, kfw_dot_temp);
    kfw_dot[0] = Matout31[0];
    kfw_dot[1] = Matout31[1];
    kfw_dot[2] = Matout31[2];

    //w_dot = inv(I_mat)*(T_NET-(cross(w0,(I_mat*w0+H_wh2body))));
}



