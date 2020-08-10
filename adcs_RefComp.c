#include <math.h>

#include "adcs_VarDeclarations.h"

#include "Telecommand.h"
#include "HAL_Global.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "TC_List.h"

void rGH_generation(void)
{
    if (CB_MagFieldComp == Enable)
    {
        m_gh = 0;
        n_gh = 0;

        //DeltaT_MFC = DeltaT_Updated; For testing
        if (DeltaT_MFC == DeltaT_Updated)
        {
            for (i_gh =1;i_gh <=14;i_gh++)
            {
                for(j_gh=1;j_gh<=15;j_gh++)
                {
                    g[i_gh][j_gh]= c_gval[m_gh] + c_gsval[m_gh] * Delta_T;

                    if (j_gh  > i_gh +1)
                    {
                        g[i_gh][j_gh] = 0.0;

                        continue;
                    }

                    m_gh++;
                }

            }

            for (i_gh=1;i_gh<=14;i_gh++)
            {
                for(j_gh=1;j_gh<=15;j_gh++)
                {
                    h[i_gh][j_gh]= c_hval[n_gh] + c_hsval[n_gh] * Delta_T;

                    if (j_gh > i_gh+1)
                    {
                        h[i_gh][j_gh] = 0.0;

                        continue;
                    }

                    n_gh++;
                }

            }
        }

        ///eDeltaT_MFC = eDeltaT_NO_Update;
        return;
    }

}

void rMagFieldComp1(void)
{
    if (CB_MagFieldComp == Enable)
    {
        //Initialization of parameters
        for(I_MFC=1; I_MFC < c_Nmax+1; I_MFC++)
        {
            cl_magad[I_MFC]= 0.0;
            sl_magad[I_MFC]= 0.0;
        }

        for(I_MFC=1; I_MFC < c_Kmax+1; I_MFC++)
        {
            p_magad[I_MFC]= 0.0;
            q_magad[I_MFC]= 0.0;
        }

        p_magad[1]= 1.0;
        q_magad[1]= 0.0;
        BNorth= 0.0;
        BEast= 0.0;
        BDown= 0.0;
        M_MAGAD= 1;
        N_MAGAD= 0;

        //check polar region cutoff
        //if(abs_fun(declination) < (80.0 * c_Deg2Rad))
        {
            //compute codeclination,st,ct,cl,sl
            co_dec= c_Pibytwo - latitude;
            st_magad=sin(co_dec);
            ct_magad=cos(co_dec);
            cl_magad[1]=cos(longitude);
            sl_magad[1]=sin(longitude);
            ///Alti = 514.5628;
            B_Rho = c_radiusearthkm*(1.0-(1.0/298.257223563));
            Rho = sqrt(c_radiusearthkm*c_radiusearthkm*st_magad*st_magad + B_Rho*B_Rho*ct_magad*ct_magad);

            if(abs_f(Rho) <= c_dividebyzerovalue)
            {
                Rho = c_dividebyzerovalue;
            }

            Alti_Mod = sqrt(Alti*Alti + 2.0*Alti*Rho + (pow(c_radiusearthkm,4.0) * st_magad*st_magad + pow(B_Rho,4.0) * ct_magad*ct_magad)/ (Rho*Rho));

            if(abs_f(Alti_Mod) <= c_dividebyzerovalue)
            {
                Alti_Mod = c_dividebyzerovalue;
            }

            cd_magad = (Alti + Rho)/Alti_Mod;
            s_magad = (c_radiusearthkm*c_radiusearthkm - B_Rho*B_Rho) / Rho*st_magad*ct_magad/Alti_Mod;

            old_cos = ct_magad;
            ct_magad = ct_magad*cd_magad - st_magad*s_magad;
            st_magad = st_magad*cd_magad + old_cos*s_magad;

            p_magad[3]=st_magad;
            q_magad[3]=ct_magad;


            A_R = pow((c_a / Alti_Mod),2.0);///c_Kmax+1

            for(K_MAGAD=2;K_MAGAD<52;K_MAGAD++)
            {
                //increment n
                if(N_MAGAD<M_MAGAD)
                {
                    M_MAGAD= 0;
                    N_MAGAD=N_MAGAD+1;
                    fN_MAGAD=N_MAGAD;
                    gN_MAGAD=N_MAGAD-1;

                    A_R = (A_R * c_a) / Alti_Mod;

                }

                if(M_MAGAD==N_MAGAD&&K_MAGAD!=3)
                {
                    //Compute Gauss function p(k) and its partial derivative q(k)
                    if(M_MAGAD != 0)
                    {
                        one_magad=sqrt(1.0 - (0.5/(float)M_MAGAD));
                    }

                    J_MAGAD=K_MAGAD-N_MAGAD-1;
                    p_magad[K_MAGAD]=one_magad*st_magad*p_magad[J_MAGAD];
                    q_magad[K_MAGAD]=one_magad*(st_magad*q_magad[J_MAGAD]+ct_magad*p_magad[J_MAGAD]);
                    cl_magad[M_MAGAD]=cl_magad[M_MAGAD-1]*cl_magad[1]-sl_magad[M_MAGAD-1]*sl_magad[1];
                    sl_magad[M_MAGAD]=sl_magad[M_MAGAD-1]*cl_magad[1]+cl_magad[M_MAGAD-1]*sl_magad[1];
                }

                if(M_MAGAD!=N_MAGAD)
                {
                    gmm = M_MAGAD*M_MAGAD;
                    A1_magad=sqrt((fN_MAGAD *fN_MAGAD)-gmm);

                    if(abs_f(A1_magad) <= c_dividebyzerovalue)
                    {
                        A1_magad = c_dividebyzerovalue;
                    }

                    A2_magad=sqrt((gN_MAGAD*gN_MAGAD)-gmm)/A1_magad;
                    A3_magad=(2.0 * (float)fN_MAGAD - 1.0)/A1_magad;
                    I_MAGAD=K_MAGAD-N_MAGAD;
                    J_MAGAD=I_MAGAD-N_MAGAD+1;
                    p_magad[K_MAGAD]=(A3_magad*ct_magad*p_magad[I_MAGAD] )-(A2_magad * p_magad[J_MAGAD]);
                    q_magad[K_MAGAD]=(A3_magad*(ct_magad*q_magad[I_MAGAD]-st_magad*p_magad[I_MAGAD]))-(A2_magad*q_magad[J_MAGAD]);
                }

                //Compute temp & intermediate variables
                //temp_magad = powertable(N_MAGAD + 2);
                temp_magad = A_R;
                one_magad=g[N_MAGAD][M_MAGAD+1]*temp_magad;
                two_magad=h[N_MAGAD][M_MAGAD+1]*temp_magad;

                //Computation of BNorth, BDown
                if(M_MAGAD!=0)
                {
                    BNorth=BNorth + (one_magad * cl_magad[M_MAGAD] + two_magad * sl_magad[M_MAGAD]) * q_magad[K_MAGAD];
                    BDown=BDown - ((float)N_MAGAD+1.0) * (one_magad* cl_magad[M_MAGAD] + two_magad * sl_magad[M_MAGAD]) * p_magad[K_MAGAD];
                }
                if(M_MAGAD==0)
                {
                    BNorth=BNorth+one_magad*q_magad[K_MAGAD];
                    BDown=BDown-((float)N_MAGAD+1.0)*one_magad*p_magad[K_MAGAD];
                }

                //Computation of BEast
                if(M_MAGAD!=0 && st_magad!=0.0)
                {
                    BEast=BEast+(1.0/st_magad)*(one_magad*sl_magad[M_MAGAD]-two_magad*cl_magad[M_MAGAD]) * (float)M_MAGAD *p_magad[K_MAGAD]; //as if (st_magad != 0) we are calling this routine. No need of divide by zero check
                }

                if(M_MAGAD!=0 && st_magad==0.0)
                {
                    BEast=BEast+ct_magad*(one_magad*sl_magad[M_MAGAD]-two_magad* cl_magad[M_MAGAD])* q_magad[K_MAGAD];
                }
                M_MAGAD++;
            }

            BNorth_old = BNorth;
            BNorth = (BNorth * cd_magad) + (BDown * s_magad);
            BDown = (BDown * cd_magad) - (BNorth_old * s_magad);

            B_NED[0] = BNorth;
            B_NED[1] = BEast;
            B_NED[2] = BDown;

        }
        return;
    }
}


void rMagFieldComp2(void)
{
    if (CB_MagFieldComp == Enable)
    {
        for(K_MAGAD=52;K_MAGAD<c_Kmax+1;K_MAGAD++)
        {
            //increment n
            if(N_MAGAD<M_MAGAD)
            {
                M_MAGAD= 0;
                N_MAGAD=N_MAGAD+1;
                fN_MAGAD=N_MAGAD;
                gN_MAGAD=N_MAGAD-1;

                A_R = (A_R * c_a) / Alti_Mod;

            }

            if(M_MAGAD==N_MAGAD&&K_MAGAD!=3)
            {
                //Compute Gauss function p(k) and its partial derivative q(k)

                if(M_MAGAD != 0)
                {
                    one_magad=sqrt(1.0 - (0.5/(float)M_MAGAD));
                }

                J_MAGAD=K_MAGAD-N_MAGAD-1;
                p_magad[K_MAGAD]=one_magad*st_magad*p_magad[J_MAGAD];
                q_magad[K_MAGAD]=one_magad*(st_magad*q_magad[J_MAGAD]+ct_magad*p_magad[J_MAGAD]);
                cl_magad[M_MAGAD]=cl_magad[M_MAGAD-1]*cl_magad[1]-sl_magad[M_MAGAD-1]*sl_magad[1];
                sl_magad[M_MAGAD]=sl_magad[M_MAGAD-1]*cl_magad[1]+cl_magad[M_MAGAD-1]*sl_magad[1];
            }

            if(M_MAGAD!=N_MAGAD)
            {
                gmm = M_MAGAD*M_MAGAD;
                A1_magad=sqrt((fN_MAGAD *fN_MAGAD)-gmm);

                if(abs_f(A1_magad) <= c_dividebyzerovalue)
                {
                    A1_magad = c_dividebyzerovalue;
                }

                A2_magad=sqrt((gN_MAGAD*gN_MAGAD)-gmm)/A1_magad;
                A3_magad=(2.0 * (float)fN_MAGAD - 1.0)/A1_magad;
                I_MAGAD=K_MAGAD-N_MAGAD;
                J_MAGAD=I_MAGAD-N_MAGAD+1;
                p_magad[K_MAGAD]=(A3_magad*ct_magad*p_magad[I_MAGAD] )-(A2_magad * p_magad[J_MAGAD]);
                q_magad[K_MAGAD]=(A3_magad*(ct_magad*q_magad[I_MAGAD]-st_magad*p_magad[I_MAGAD]))-(A2_magad*q_magad[J_MAGAD]);
            }

            //Compute temp & intermediate variables
            //temp_magad = powertable(N_MAGAD + 2);
            temp_magad = A_R;
            one_magad=g[N_MAGAD][M_MAGAD+1]*temp_magad;
            two_magad=h[N_MAGAD][M_MAGAD+1]*temp_magad;

            //Computation of BNorth, BDown
            if(M_MAGAD!=0)
            {
                BNorth=BNorth + (one_magad * cl_magad[M_MAGAD] + two_magad * sl_magad[M_MAGAD]) * q_magad[K_MAGAD];
                BDown=BDown - ((float)N_MAGAD+1.0) * (one_magad* cl_magad[M_MAGAD] + two_magad * sl_magad[M_MAGAD]) * p_magad[K_MAGAD];
            }
            if(M_MAGAD==0)
            {
                BNorth=BNorth+one_magad*q_magad[K_MAGAD];
                BDown=BDown-((float)N_MAGAD+1.0)*one_magad*p_magad[K_MAGAD];
            }

            //Computation of BEast
            if(M_MAGAD!=0 && st_magad!=0.0)
            {
                BEast=BEast+(1.0/st_magad)*(one_magad*sl_magad[M_MAGAD]-two_magad*cl_magad[M_MAGAD]) * (float)M_MAGAD *(float)p_magad[K_MAGAD]; //as if (st_magad != 0) we are calling this routine. No need of divide by zero check
            }

            if(M_MAGAD!=0 && st_magad==0.0)
            {
                BEast=BEast+ct_magad*(one_magad*sl_magad[M_MAGAD]-two_magad* cl_magad[M_MAGAD])* q_magad[K_MAGAD];
            }
            M_MAGAD++;
        }

        BNorth_old = BNorth;
        BNorth = (BNorth * cd_magad) + (BDown * s_magad);
        BDown = (BDown * cd_magad) - (BNorth_old * s_magad);

        B_NED[0] = BNorth;
        B_NED[1] = BEast;
        B_NED[2] = BDown;

        rMatMul3x1(NEDtoECEF,B_NED);
        B_ECEF[0] = Matout31[0];
        B_ECEF[1] = Matout31[1];
        B_ECEF[2] = Matout31[2];

        rMatMul3x1(ECEFtoECI,B_ECEF);
        B_ECI[0] = Matout31[0];
        B_ECI[1] = Matout31[1];
        B_ECI[2] = Matout31[2];

        B_ECItesla[0] = B_ECI[0]*1.0E-9;
        B_ECItesla[1] = B_ECI[1]*1.0E-9;
        B_ECItesla[2] = B_ECI[2]*1.0E-9;

        rVectorNorm(B_ECI);
        B_ECIn[0] = Norm_out[0];
        B_ECIn[1] = Norm_out[1];
        B_ECIn[2] = Norm_out[2];

        return;
    }
}

void rSun_Ephemeris(void)
{
    if (CB_Sun_model == Enable)
    {
        L_Msun = c_L_Msun1*c_D2R + c_L_Msun2*c_D2R * tut; //mean longitude of Sun
        Msun = c_Msun1*c_D2R + c_Msun2*c_D2R * tut; //mean anomaly of Sun
        L_Ecliptic = L_Msun + c_L_Ecliptic1*c_D2R * sin(Msun) + c_L_Ecliptic2*c_D2R * sin(2.0 * Msun); //ecliptic longitude of Sun
        Sun_Dis = c_Sun_Dis1*c_D2R - c_Sun_Dis2*c_D2R * cos(Msun) - c_Sun_Dis3*c_D2R * cos(2.0 * Msun); //distance to Sun in AUs
        Epsilon = c_Epsilon1*c_D2R - c_Epsilon2*c_D2R * tut; //obliquity of the eclipse

        ///Sun vector in ECI frame in AUs
        S_ECI[0] = cos(L_Ecliptic);
        S_ECI[1] = cos(Epsilon) * sin(L_Ecliptic);
        S_ECI[2] = sin(Epsilon) * sin(L_Ecliptic);

        rVectorNorm(S_ECI);
        S_ECIn[0] = Norm_out[0];
        S_ECIn[1] = Norm_out[1];
        S_ECIn[2] = Norm_out[2];

        return;
    }
}

void rReferenceQuatComputation(void)
{
    if (CB_ReferenceQuatComputation == Enable)
    {
        ///---------------------------------------SVO to ECI Transformation--------------------------------------------------
        Y_SVO2ECI[0] = -1.0*S_ECIn[0];
        Y_SVO2ECI[1] = -1.0*S_ECIn[1];
        Y_SVO2ECI[2] = -1.0*S_ECIn[2];

        Z_SVO2ECI[0] = -1.0*Pos_ECIn[0];
        Z_SVO2ECI[1] = -1.0*Pos_ECIn[1];
        Z_SVO2ECI[2] = -1.0*Pos_ECIn[2];

        rCross_Product(Y_SVO2ECI, Z_SVO2ECI);

        X_SVO2ECI[0] = Cross_Product[0];
        X_SVO2ECI[1] = Cross_Product[1];
        X_SVO2ECI[2] = Cross_Product[2];

        rVectorNorm(X_SVO2ECI);
        X_SVO2ECI[0] = Norm_out[0];
        X_SVO2ECI[1] = Norm_out[1];
        X_SVO2ECI[2] = Norm_out[2];

        rCross_Product(X_SVO2ECI, Y_SVO2ECI);

        Z_SVO2ECI[0] = Cross_Product[0];
        Z_SVO2ECI[1] = Cross_Product[1];
        Z_SVO2ECI[2] = Cross_Product[2];

        rVectorNorm(Z_SVO2ECI);
        Z_SVO2ECI[0] = Norm_out[0];
        Z_SVO2ECI[1] = Norm_out[1];
        Z_SVO2ECI[2] = Norm_out[2];

        for(i_rfc=0; i_rfc<3;i_rfc++)
        {
            R_SVO2ECI[0][i_rfc] = X_SVO2ECI[i_rfc];
            R_SVO2ECI[1][i_rfc] = Y_SVO2ECI[i_rfc];
            R_SVO2ECI[2][i_rfc] = Z_SVO2ECI[i_rfc];
        }

        rRM_to_Quat(R_SVO2ECI);
        Q_SVO2ECI[0] = DCM2Q_out[0];
        Q_SVO2ECI[1] = DCM2Q_out[1];
        Q_SVO2ECI[2] = DCM2Q_out[2];
        Q_SVO2ECI[3] = DCM2Q_out[3];

        rQs_Normalization(Q_SVO2ECI);
        Q_SVO2ECI[0] = out_Quat_norm[0];
        Q_SVO2ECI[1] = out_Quat_norm[1];
        Q_SVO2ECI[2] = out_Quat_norm[2];
        Q_SVO2ECI[3] = out_Quat_norm[3];

        ///---------------------------------------SPO to ECI Transformation--------------------------------------------------

        STATION_ECEF[0] = c_radiusearthkm * (cos(ADCS_TC_data_command_Table.TC_ECEF_stationLongitude) * cos(ADCS_TC_data_command_Table.TC_ECEF_stationlatitude));
        STATION_ECEF[1] = c_radiusearthkm * (sin(ADCS_TC_data_command_Table.TC_ECEF_stationLongitude) * cos(ADCS_TC_data_command_Table.TC_ECEF_stationlatitude));
        STATION_ECEF[2] = c_radiusearthkm * sin(ADCS_TC_data_command_Table.TC_ECEF_stationlatitude);

        rMatMul3x1(ECEFtoECI,STATION_ECEF);
        STATION_ECI[0] = Matout31[0];
        STATION_ECI[1] = Matout31[1];
        STATION_ECI[2] = Matout31[2];

        STATION_vector[0] = (STATION_ECI[0] - Pos_ECI[0]);
        STATION_vector[1] = (STATION_ECI[1] - Pos_ECI[1]);
        STATION_vector[2] = (STATION_ECI[2] - Pos_ECI[2]);

        rVectorNorm(STATION_ECI);
        STATION_ECIn[0] = Norm_out[0];
        STATION_ECIn[1] = Norm_out[1];
        STATION_ECIn[2] = Norm_out[2];

        rdotrst = Pos_ECIn[0] * STATION_ECIn[0] + Pos_ECIn[1] * STATION_ECIn[1] + Pos_ECIn[2] * STATION_ECIn[2];
        SAT_ANGLE_STAT = acos(rdotrst) *c_R2D;

        rVectorNorm(STATION_vector);
        STATION_vector[0] = Norm_out[0];
        STATION_vector[1] = Norm_out[1];
        STATION_vector[2] = Norm_out[2];

        Z_SPO2ECI[0] = STATION_vector[0];
        Z_SPO2ECI[1] = STATION_vector[1];
        Z_SPO2ECI[2] = STATION_vector[2];

        Y_SPO2ECI[0] = -S_ECIn[0];
        Y_SPO2ECI[1] = -S_ECIn[1];
        Y_SPO2ECI[2] = -S_ECIn[2];

        rCross_Product(Y_SPO2ECI, Z_SPO2ECI);

        X_SPO2ECI[0] = Cross_Product[0];
        X_SPO2ECI[1] = Cross_Product[1];
        X_SPO2ECI[2] = Cross_Product[2];

        rVectorNorm(X_SPO2ECI);
        X_SPO2ECI[0] = Norm_out[0];
        X_SPO2ECI[1] = Norm_out[1];
        X_SPO2ECI[2] = Norm_out[2];

        rCross_Product(Z_SPO2ECI, X_SPO2ECI);

        Y_SPO2ECI[0] = Cross_Product[0];
        Y_SPO2ECI[1] = Cross_Product[1];
        Y_SPO2ECI[2] = Cross_Product[2];

        rVectorNorm(Y_SPO2ECI);
        Y_SPO2ECI[0] = Norm_out[0];
        Y_SPO2ECI[1] = Norm_out[1];
        Y_SPO2ECI[2] = Norm_out[2];

        for(i_rfc=0; i_rfc<3;i_rfc++)
        {
            R_StP2ECI[0][i_rfc] = X_SPO2ECI[i_rfc];
            R_StP2ECI[1][i_rfc] = Y_SPO2ECI[i_rfc];
            R_StP2ECI[2][i_rfc] = Z_SPO2ECI[i_rfc];
        }

        rRM_to_Quat(R_StP2ECI);
        Q_StP2ECI[0] = DCM2Q_out[0];
        Q_StP2ECI[1] = DCM2Q_out[1];
        Q_StP2ECI[2] = DCM2Q_out[2];
        Q_StP2ECI[3] = DCM2Q_out[3];

        rQs_Normalization(Q_StP2ECI);
        Q_StP2ECI[0] = out_Quat_norm[0];
        Q_StP2ECI[1] = out_Quat_norm[1];
        Q_StP2ECI[2] = out_Quat_norm[2];
        Q_StP2ECI[3] = out_Quat_norm[3];


        ///---------------------------------------EPO to ECI Transformation--------------------------------------------------
        Z_EPO2ECI[0] = -Pos_ECIn[0];
        Z_EPO2ECI[1] = -Pos_ECIn[1];
        Z_EPO2ECI[2] = -Pos_ECIn[2];

        rCross_Product(Pos_ECI, Vel_ECI);

        Y_EPO2ECI[0] = Cross_Product[0];
        Y_EPO2ECI[1] = Cross_Product[1];
        Y_EPO2ECI[2] = Cross_Product[2];

        rVectorNorm(Y_EPO2ECI);
        Y_EPO2ECI[0] = Norm_out[0];
        Y_EPO2ECI[1] = Norm_out[1];
        Y_EPO2ECI[2] = Norm_out[2];

        rCross_Product(Y_EPO2ECI, Z_EPO2ECI);

        X_EPO2ECI[0] = Cross_Product[0];
        X_EPO2ECI[1] = Cross_Product[1];
        X_EPO2ECI[2] = Cross_Product[2];

        rVectorNorm(X_EPO2ECI);
        X_EPO2ECI[0] = Norm_out[0];
        X_EPO2ECI[1] = Norm_out[1];
        X_EPO2ECI[2] = Norm_out[2];

        for(i_rfc=0; i_rfc<3;i_rfc++)
        {
            R_EPO2ECI[0][i_rfc] = X_EPO2ECI[i_rfc];
            R_EPO2ECI[1][i_rfc] = Y_EPO2ECI[i_rfc];
            R_EPO2ECI[2][i_rfc] = Z_EPO2ECI[i_rfc];
        }

        rRM_to_Quat(R_EPO2ECI);
        Q_EPO2ECI[0] = DCM2Q_out[0];
        Q_EPO2ECI[1] = DCM2Q_out[1];
        Q_EPO2ECI[2] = DCM2Q_out[2];
        Q_EPO2ECI[3] = DCM2Q_out[3];

        rQs_Normalization(Q_EPO2ECI);
        Q_EPO2ECI[0] = out_Quat_norm[0];
        Q_EPO2ECI[1] = out_Quat_norm[1];
        Q_EPO2ECI[2] = out_Quat_norm[2];
        Q_EPO2ECI[3] = out_Quat_norm[3];

        ///---------------------------------------SFAO to ECI Transformation--------------------------------------------------
        Y_SFAO2ECI[0] = -S_ECIn[0];
        Y_SFAO2ECI[1] = -S_ECIn[1];
        Y_SFAO2ECI[2] = -S_ECIn[2];

        Z_SFAO2ECI[0] = -cos(RAAN);
        Z_SFAO2ECI[1] = -sin(RAAN);
        Z_SFAO2ECI[2] = 0.0;

        rVectorNorm(Z_SFAO2ECI);
        Z_SFAO2ECI[0] = Norm_out[0];
        Z_SFAO2ECI[1] = Norm_out[1];
        Z_SFAO2ECI[2] = Norm_out[2];

        rCross_Product(Y_SFAO2ECI, Z_SFAO2ECI);

        X_SFAO2ECI[0] = Cross_Product[0];
        X_SFAO2ECI[1] = Cross_Product[1];
        X_SFAO2ECI[2] = Cross_Product[2];

        rVectorNorm(X_SFAO2ECI);
        X_SFAO2ECI[0] = Norm_out[0];
        X_SFAO2ECI[1] = Norm_out[1];
        X_SFAO2ECI[2] = Norm_out[2];

        rCross_Product(X_SFAO2ECI, Y_SFAO2ECI);

        Z_SFAO2ECI[0] = Cross_Product[0];
        Z_SFAO2ECI[1] = Cross_Product[1];
        Z_SFAO2ECI[2] = Cross_Product[2];

        rVectorNorm(Z_SFAO2ECI);
        Z_SFAO2ECI[0] = Norm_out[0];
        Z_SFAO2ECI[1] = Norm_out[1];
        Z_SFAO2ECI[2] = Norm_out[2];

        for(i_rfc=0; i_rfc<3;i_rfc++)
        {
            R_SFAO2ECI[0][i_rfc] = X_SFAO2ECI[i_rfc];
            R_SFAO2ECI[1][i_rfc] = Y_SFAO2ECI[i_rfc];
            R_SFAO2ECI[2][i_rfc] = Z_SFAO2ECI[i_rfc];
        }

        rRM_to_Quat(R_SFAO2ECI);
        Q_SFAO2ECI[0] = DCM2Q_out[0];
        Q_SFAO2ECI[1] = DCM2Q_out[1];
        Q_SFAO2ECI[2] = DCM2Q_out[2];
        Q_SFAO2ECI[3] = DCM2Q_out[3];

        rQs_Normalization(Q_SFAO2ECI);
        Q_SFAO2ECI[0] = out_Quat_norm[0];
        Q_SFAO2ECI[1] = out_Quat_norm[1];
        Q_SFAO2ECI[2] = out_Quat_norm[2];
        Q_SFAO2ECI[3] = out_Quat_norm[3];

        ///---------------------------------------SFDO to ECI Transformation--------------------------------------------------
        Y_SFDO2ECI[0] = -S_ECIn[0];
        Y_SFDO2ECI[1] = -S_ECIn[1];
        Y_SFDO2ECI[2] = -S_ECIn[2];

        Z_SFDO2ECI[0] = cos(RAAN);
        Z_SFDO2ECI[1] = sin(RAAN);
        Z_SFDO2ECI[2] = 0.0;

        rVectorNorm(Z_SFAO2ECI);
        Z_SFDO2ECI[0] = Norm_out[0];
        Z_SFDO2ECI[1] = Norm_out[1];
        Z_SFDO2ECI[2] = Norm_out[2];

        rCross_Product(Y_SFDO2ECI, Z_SFDO2ECI);

        X_SFDO2ECI[0] = Cross_Product[0];
        X_SFDO2ECI[1] = Cross_Product[1];
        X_SFDO2ECI[2] = Cross_Product[2];

        rVectorNorm(Z_SFAO2ECI);
        X_SFDO2ECI[0] = Norm_out[0];
        X_SFDO2ECI[1] = Norm_out[1];
        X_SFDO2ECI[2] = Norm_out[2];

        rCross_Product(X_SFAO2ECI, Y_SFDO2ECI);

        Z_SFDO2ECI[0] = Cross_Product[0];
        Z_SFDO2ECI[1] = Cross_Product[1];
        Z_SFDO2ECI[2] = Cross_Product[2];

        rVectorNorm(Z_SFAO2ECI);
        Z_SFDO2ECI[0] = Norm_out[0];
        Z_SFDO2ECI[1] = Norm_out[1];
        Z_SFDO2ECI[2] = Norm_out[2];

        for(i_rfc=0; i_rfc<3;i_rfc++)
        {
            R_SFDO2ECI[0][i_rfc] = X_SFDO2ECI[i_rfc];
            R_SFDO2ECI[1][i_rfc] = Y_SFDO2ECI[i_rfc];
            R_SFDO2ECI[2][i_rfc] = Z_SFDO2ECI[i_rfc];
        }

        rRM_to_Quat(R_SFDO2ECI);
        Q_SFDO2ECI[0] = DCM2Q_out[0];
        Q_SFDO2ECI[1] = DCM2Q_out[1];
        Q_SFDO2ECI[2] = DCM2Q_out[2];
        Q_SFDO2ECI[3] = DCM2Q_out[3];

        rQs_Normalization(Q_SFDO2ECI);
        Q_SFDO2ECI[0] = out_Quat_norm[0];
        Q_SFDO2ECI[1] = out_Quat_norm[1];
        Q_SFDO2ECI[2] = out_Quat_norm[2];
        Q_SFDO2ECI[3] = out_Quat_norm[3];
    }

}

void rRefVectorGeneration(void)
{
    if (CB_RefVectorGeneration == Enable)
    {
        if(TC_boolean_u.TC_Boolean_Table.TC_Sun_Varying_Mode == 1)
        {
            for(i_rfc=0;i_rfc<3;i_rfc++)
            {
                for(j_rfc=0;j_rfc<3;j_rfc++)
                {
                    REF_FRAME_DCM[i_rfc][j_rfc] = R_SVO2ECI[i_rfc][j_rfc];
                }
            }

            Q_REF[0] = Q_SVO2ECI[0];
            Q_REF[1] = Q_SVO2ECI[1];
            Q_REF[2] = Q_SVO2ECI[2];
            Q_REF[3] = Q_SVO2ECI[3];
        }

        else if(TC_boolean_u.TC_Boolean_Table.TC_Orbit_Reference_Mode == 1)
        {
            for(i_rfc=0;i_rfc<3;i_rfc++)
            {
                for(j_rfc=0;j_rfc<3;j_rfc++)
                {
                    REF_FRAME_DCM[i_rfc][j_rfc] = R_EPO2ECI[i_rfc][j_rfc];
                }
            }

            Q_REF[0] = Q_EPO2ECI[0];
            Q_REF[1] = Q_EPO2ECI[1];
            Q_REF[2] = Q_EPO2ECI[2];
            Q_REF[3] = Q_EPO2ECI[3];
        }

        else if(TC_boolean_u.TC_Boolean_Table.TC_Asd_Node_Mode == 1)
        {
            for(i_rfc=0;i_rfc<3;i_rfc++)
            {
                for(j_rfc=0;j_rfc<3;j_rfc++)
                {
                    REF_FRAME_DCM[i_rfc][j_rfc] = R_SFAO2ECI[i_rfc][j_rfc];
                }
            }

            Q_REF[0] = Q_SFAO2ECI[0];
            Q_REF[1] = Q_SFAO2ECI[1];
            Q_REF[2] = Q_SFAO2ECI[2];
            Q_REF[3] = Q_SFAO2ECI[3];
        }

        else if(TC_boolean_u.TC_Boolean_Table.TC_Des_Node_Mode == 1)
        {
            for(i_rfc=0;i_rfc<3;i_rfc++)
            {
                for(j_rfc=0;j_rfc<3;j_rfc++)
                {
                    REF_FRAME_DCM[i_rfc][j_rfc] = R_SFDO2ECI[i_rfc][j_rfc];
                }
            }

            Q_REF[0] = Q_SFDO2ECI[0];
            Q_REF[1] = Q_SFDO2ECI[1];
            Q_REF[2] = Q_SFDO2ECI[2];
            Q_REF[3] = Q_SFDO2ECI[3];
        }
        else
        {
            //
        }

        if ((TC_boolean_u.TC_Boolean_Table.TC_Station_Tracking_Mode == 1) && (SAT_ANGLE_STAT < 22.29))
        {
            for(i_rfc=0;i_rfc<3;i_rfc++)
            {
                for(j_rfc=0;j_rfc<3;j_rfc++)
                {
                    REF_FRAME_DCM[i_rfc][j_rfc] = R_StP2ECI[i_rfc][j_rfc];
                }
            }

            Q_REF[0] = Q_StP2ECI[0];
            Q_REF[1] = Q_StP2ECI[1];
            Q_REF[2] = Q_StP2ECI[2];
            Q_REF[3] = Q_StP2ECI[3];

            f_Momentum_Dumping = 0;

            if (f_Sunlit_Presence == 1)
                TC_enQuest_update = 1;
            else
                TC_enQuest_update = 0;

        }
        else
        {

            f_Momentum_Dumping = 1;
            TC_enQuest_update = 1;
        }

        /*Q_REF[0] = 0.547167587642792;
        Q_REF[1] = -0.495955257085227;//test
        Q_REF[2] = 0.530026596382111;
        Q_REF[3] = 0.416782702532494;*/

        Q_REF_conj[0] = -Q_REF[0];
        Q_REF_conj[1] = -Q_REF[1];
        Q_REF_conj[2] = -Q_REF[2];
        Q_REF_conj[3] = Q_REF[3];

        rMatMul3x1(REF_FRAME_DCM,B_ECI);
        B_REF[0] = Matout31[0];
        B_REF[1] = Matout31[1];
        B_REF[2] = Matout31[2];

        rVectorNorm(B_REF);
        B_REFn[0] = Norm_out[0];
        B_REFn[1] = Norm_out[1];
        B_REFn[2] = Norm_out[2];

        rMatMul3x1(REF_FRAME_DCM,S_ECI);
        S_REF[0] = Matout31[0];
        S_REF[1] = Matout31[1];
        S_REF[2] = Matout31[2];

        rVectorNorm(S_REF);
        S_REFn[0] = Norm_out[0];
        S_REFn[1] = Norm_out[1];
        S_REFn[2] = Norm_out[2];

        TM.Buffer.TM_B_DOT[0] = (int)(B_REF[0] / c_TM_Resol_B);
		TM.Buffer.TM_B_DOT[1] = (int)(B_REF[1] / c_TM_Resol_B);
		TM.Buffer.TM_B_DOT[2] = (int)(B_REF[2] / c_TM_Resol_B);

		TM.Buffer.TM_S_BODY_Red[0] = (int)(S_REF[0]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Red[1] = (int)(S_REF[1]/4.65661287E-7);
		TM.Buffer.TM_S_BODY_Red[2] = (int)(S_REF[2]/4.65661287E-7);

		ST_normal.ST_NM_Buffer.TM_S_BODY_Red[0] = (int)(S_REF[0]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_S_BODY_Red[1] = (int)(S_REF[1]/4.65661287E-7);
		ST_normal.ST_NM_Buffer.TM_S_BODY_Red[2] = (int)(S_REF[2]/4.65661287E-7);

        TM.Buffer.TM_Q_Ref[0] = (int)(Q_REF[0]/4.65661287E-7);
        TM.Buffer.TM_Q_Ref[1] = (int)(Q_REF[1]/4.65661287E-7);
        TM.Buffer.TM_Q_Ref[2] = (int)(Q_REF[2]/4.65661287E-7);
        TM.Buffer.TM_Q_Ref[3] = (int)(Q_REF[3]/4.65661287E-7);

        ST_normal.ST_NM_Buffer.TM_Q_Ref[0] = (int)(Q_REF[0]/4.65661287E-7);
        ST_normal.ST_NM_Buffer.TM_Q_Ref[1] = (int)(Q_REF[1]/4.65661287E-7);
        ST_normal.ST_NM_Buffer.TM_Q_Ref[2] = (int)(Q_REF[2]/4.65661287E-7);
        ST_normal.ST_NM_Buffer.TM_Q_Ref[3] = (int)(Q_REF[3]/4.65661287E-7);

        ST_special.ST_SP_Buffer.TM_Q_Ref[0] = (int)(Q_REF[0]/4.65661287E-7);
        ST_special.ST_SP_Buffer.TM_Q_Ref[1] = (int)(Q_REF[1]/4.65661287E-7);
        ST_special.ST_SP_Buffer.TM_Q_Ref[2] = (int)(Q_REF[2]/4.65661287E-7);
        ST_special.ST_SP_Buffer.TM_Q_Ref[3] = (int)(Q_REF[3]/4.65661287E-7);

    }
}

void rRefRate_Computation(void)
{
    if (CB_RefRate_Computation == Enable)
    {
        Q_REF_prev[0] = Q_REF_pres[0];
        Q_REF_prev[1] = Q_REF_pres[1];
        Q_REF_prev[2] = Q_REF_pres[2];
        Q_REF_prev[3] = Q_REF_pres[3];

        Q_REF_pres[0] = Q_REF[0];
        Q_REF_pres[1] = Q_REF[1];
        Q_REF_pres[2] = Q_REF[2];
        Q_REF_pres[3] = Q_REF[3];

        ///Computation of conjugate of Reference Quaternion in previous time cycle
        Q_REF_prev_conj[0] = ((-1.0) * Q_REF_prev[0]);
        Q_REF_prev_conj[1] = ((-1.0) * Q_REF_prev[1]);
        Q_REF_prev_conj[2] = ((-1.0) * Q_REF_prev[2]);
        Q_REF_prev_conj[3] = Q_REF_prev[3];

        ///Computation of Delta_Q between reference quaternion in two consecutive time step.
        rQs_Multiplication(Q_REF_prev_conj, Q_REF_pres);
        Q_REF_diff[0] = out_Quat_mult[0];
        Q_REF_diff[1] = out_Quat_mult[1];
        Q_REF_diff[2] = out_Quat_mult[2];
        Q_REF_diff[3] = out_Quat_mult[3];

        ///Computation of Q-Angle
        QRD_vect_norm = sqrt((Q_REF_diff[0] * Q_REF_diff[0]) + (Q_REF_diff[1] * Q_REF_diff[1]) + (Q_REF_diff[2] * Q_REF_diff[2]));
        Q_angle = (2.0 * atan2(QRD_vect_norm, Q_REF_diff[3]));

        if(Q_angle <= 0.01745329252) ///If angle is less than or equal to 1.0s deg/sec
        {
            if(QRD_vect_norm > 0.0)
            {
                ///Computation of Q-Axis
                Q_axis[0] = (Q_REF_diff[0] / QRD_vect_norm);
                Q_axis[1] = (Q_REF_diff[1] / QRD_vect_norm);
                Q_axis[2] = (Q_REF_diff[2] / QRD_vect_norm);
            }
            else
            {
                Q_axis[0] = 1.0;
                Q_axis[1] = 0.0;
                Q_axis[2] = 0.0;
            }

            ///Computation of reference rates
            w_REF[0] = (Q_axis[0] * (Q_angle / c_MaC));
            w_REF[1] = (Q_axis[1] * (Q_angle / c_MaC));
            w_REF[2] = (Q_axis[2] * (Q_angle / c_MaC));

            TM.Buffer.TM_w_Ref[0] = (int)(w_REF[0]/c_TM_Resol_w);
			TM.Buffer.TM_w_Ref[1] = (int)(w_REF[1]/c_TM_Resol_w);
			TM.Buffer.TM_w_Ref[2] = (int)(w_REF[2]/c_TM_Resol_w);
        }
        else
        {
            w_REF[0] = 0.0;
            w_REF[1] = 0.0;
            w_REF[2] = 0.0;

            TM.Buffer.TM_w_Ref[0] = (int)(w_REF[0]/4.656612874E-9);
			TM.Buffer.TM_w_Ref[1] = (int)(w_REF[1]/4.656612874E-9);
			TM.Buffer.TM_w_Ref[2] = (int)(w_REF[2]/4.656612874E-9);
        }
    }
}

void rSl_Ecl_OnBrd_detection(void)
{
    if (CB_Sl_Ecl_OnBrd_detection == Enable)
    {
        rsun[0]= S_ECI[0]*c_au;
        rsun[1]= S_ECI[1]*c_au;
        rsun[2]= S_ECI[2]*c_au;

        magrsun = sqrt((rsun[0] * rsun[0]) + (rsun[1] * rsun[1]) + (rsun[2] * rsun[2]));

        theta1 = acos((c_radiusearthkm) / radialdistance);
        theta2 = acos((c_radiusearthkm) / (magrsun));

        psi_sl_ecl = acos(((Pos_ECI[0] * rsun[0]) + (Pos_ECI[1] * rsun[1]) + (Pos_ECI[2] * rsun[2])) / (radialdistance * magrsun));///Angle between s/c position vector and Sun position vector (radians)

        if(psi_sl_ecl >= (theta1 + theta2))///If psi is >= thetal+theta2, the s/c is in eclipse, otherwise it's in sunlight
        {
            f_Sunlit_Presence = False;
            f_Momentum_Dumping = 0;
            CB_Torquer_Polarity_Check = 0;
            //wAD_updatecount = 0;
        }
        else
        {
            f_Sunlit_Presence = True;
            f_Momentum_Dumping = 1;
            CB_Torquer_Polarity_Check = 1;
        }

        TC_elapsed_orbitTimer = TC_elapsed_orbitTimer + c_MaC;

        if (TC_elapsed_orbitTimer > Orbit_Period)
        {
            TC_elapsed_orbitTimer = TC_elapsed_orbitTimer - Orbit_Period;
        }

        //entrytime2eclipse = TC_eclipse_entrytime - 900.0;
        /*if (TC_Sunlit_detctn_timer == Enable)
        {
            TC_elapsed_orbitTimer = TC_elapsed_orbitTimer + majorcyclecount;
            TC_elapsed_orbitTimer = TC_elapsed_orbitTimer % Orbit_Period;
            if (TC_elapsed_orbitTimer < TC_eclipse_entrytime)
            {
                f_Sunlit_Presence = True;
            }
            else if (elapsed_ruuning_timer > TC_eclipse_exittime)
            {
                f_Sunlit_Presence = True;
            }
            else
            {
                f_Sunlit_Presence = False;
            }
        }
		else
		{
			///Sunlit and eclipse check
			if((SC1 > c_Sunlit_Thrsld) || (SC2 > c_Sunlit_Thrsld) || (SC3 > c_Sunlit_Thrsld) || (SC4 > c_Sunlit_Thrsld) || (SC5 > c_Sunlit_Thrsld) || (SC6 > c_Sunlit_Thrsld))
			{
				//f_Sunlit_Presence_SunS = True;
				Sunlit_presence_timer++;
			}

			else
			{
				//f_Sunlit_Presence_SunS = False;
				Sunlit_presence_timer = 0;
			}
		}*/
    }
    return;
}
