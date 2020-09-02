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

double rsign_f(double const sign_var)
{
	return (((sign_var) < 0.0)? (-1.0) : (1.0));
}

void rxRot(double const th)
{
    Rx[0][0] = 1.0;
    Rx[0][1] = 0.0;
    Rx[0][2] = 0.0;
    Rx[1][0] = 0.0;
    Rx[1][1] = cos(th);
    Rx[1][2] = -sin(th);
    Rx[2][0] = 0.0;
    Rx[2][1] = sin(th);
    Rx[2][2] = cos(th);
}

void ryRot(double const th)
{
    Ry[0][0] = cos(th);
    Ry[0][1] = 0.0;
    Ry[0][2] = sin(th);
    Ry[1][0] = 0.0;
    Ry[1][1] = 1.0;
    Ry[1][2] = 0.0;
    Ry[2][0] = -sin(th);
    Ry[2][1] = 0.0;
    Ry[2][2] = cos(th);
}

void rzRot(double const th)
{
    Rz[0][0] = cos(th);
    Rz[0][1] = -1.0*sin(th);
    Rz[0][2] = 0.0;
    Rz[1][0] = sin(th);
    Rz[1][1] = cos(th);
    Rz[1][2] = 0.0;
    Rz[2][0] = 0.0;
    Rz[2][1] = 0.0;
    Rz[2][2] = 1.0;
}

void rMatInv(double mat2inv[3][3])
{
    determinant = 0.0;
    for(i_comr = 0; i_comr < 3; i_comr++)
    {
        determinant = determinant + (mat2inv[0][i_comr] * ((mat2inv[1][(i_comr+1)%3] * mat2inv[2][(i_comr+2)%3]) - (mat2inv[1][(i_comr+2)%3] * mat2inv[2][(i_comr+1)%3])));
    }

    for(i_comr = 0; i_comr < 3; i_comr++)
    {
        for(j_comr = 0; j_comr < 3; j_comr++)
        {
            Invmatout33[i_comr][j_comr] = ((mat2inv[(j_comr+1)%3][(i_comr+1)%3] * mat2inv[(j_comr+2)%3][(i_comr+2)%3]) - (mat2inv[(j_comr+1)%3][(i_comr+2)%3] * mat2inv[(j_comr+2)%3][(i_comr+1)%3]))/ determinant;
        }
    }
}

void rCross_Product(double const u[3], double const v[3])
{
    Cross_Product[0] = ((u[1] * v[2]) - (v[1] * u[2]));
    Cross_Product[1] = -((u[0] * v[2]) - (v[0] * u[2]));
    Cross_Product[2] = ((u[0] * v[1]) - (v[0] * u[1]));
}

void rMatMul3x1(double mat33[3][3], double const vec31[3])
{
    for(i_comr=0; i_comr<3; i_comr++)
    {
        Matout31[i_comr] = 0.0;
        for(j_comr=0; j_comr<3; j_comr++)
        {
            Matout31[i_comr] += mat33[i_comr][j_comr]*vec31[j_comr];
        }
    }
}

void rMatMul3x3(double mat331[3][3], double mat332[3][3])
{
    for(i_comr=0; i_comr<3; i_comr++)
    {
        for(j_comr=0; j_comr<3; j_comr++)
        {
            Matout33[i_comr][j_comr] = 0.0;
            for(k_comr=0; k_comr<3; k_comr++)
            {
                Matout33[i_comr][j_comr] += mat331[i_comr][k_comr]*mat332[k_comr][j_comr];
            }
        }
    }
}

///---------------------Function to conver Rotational Matrix to Quaternion------------------
void rRM_to_Quat(double DCM2Q_in[3][3])
{

    double d[3], tr, sqtrp1, sqdip1, DCM2Q_out_den;
    tr = DCM2Q_in[0][0] + DCM2Q_in[1][1] + DCM2Q_in[2][2];

    if(tr > 0.0)
    {
        sqtrp1 = sqrt(tr + 1.0);

        DCM2Q_out_den = 2.0*sqtrp1;

        if(fabs(DCM2Q_out_den) <= c_dividebyzerovalue)
        {
            DCM2Q_out_den = c_dividebyzerovalue;
        }

        DCM2Q_out[0] = (DCM2Q_in[1][2] - DCM2Q_in[2][1])/(DCM2Q_out_den);
        DCM2Q_out[1] = (DCM2Q_in[2][0] - DCM2Q_in[0][2])/(DCM2Q_out_den);
        DCM2Q_out[2] = (DCM2Q_in[0][1] - DCM2Q_in[1][0])/(DCM2Q_out_den);
        DCM2Q_out[3] = 0.5*sqtrp1;
    }
    else
    {
        d[0] = DCM2Q_in[0][0];
        d[1] = DCM2Q_in[1][1];
        d[2] = DCM2Q_in[2][2];

        if ((d[1] > d[0]) && (d[1] > d[2]))
        {
            //max value at DCM2Q_in(2,2,i_comr)
            sqdip1 = sqrt(d[1] - d[0] - d[2] + 1.0);

            DCM2Q_out[1] = 0.5*sqdip1;

            if (sqdip1 != 0.0)
            {
                sqdip1 = 0.5/sqdip1;
            }

            DCM2Q_out[3] = (DCM2Q_in[2][0] - DCM2Q_in[0][2])*sqdip1;
            DCM2Q_out[0] = (DCM2Q_in[0][1] + DCM2Q_in[1][0])*sqdip1;
            DCM2Q_out[2] = (DCM2Q_in[1][2] + DCM2Q_in[2][1])*sqdip1;
        }
        else if(d[2]>d[0])
        {
            //max value at DCM2Q_in(3,3,i_comr)
            sqdip1 = sqrt(d[2] - d[0] - d[1] + 1.0);

            DCM2Q_out[2] = 0.5*sqdip1;

            if (sqdip1 != 0.0)
            {
                sqdip1 = 0.5/sqdip1;
            }

            DCM2Q_out[3] = (DCM2Q_in[0][1] - DCM2Q_in[1][0])*sqdip1;
            DCM2Q_out[0] = (DCM2Q_in[2][0] + DCM2Q_in[0][2])*sqdip1;
            DCM2Q_out[1] = (DCM2Q_in[1][2] + DCM2Q_in[2][1])*sqdip1;
        }

        else
        {
            //max value at DCM2Q_in(1,1,i_comr)
            sqdip1 = sqrt(d[0] - d[1] - d[2] + 1.0);

            DCM2Q_out[0] = 0.5*sqdip1;

            if (sqdip1 != 0.0)
            {
                sqdip1 = 0.5/sqdip1;
            }

            DCM2Q_out[3] = (DCM2Q_in[1][2] - DCM2Q_in[2][1])*sqdip1;
            DCM2Q_out[1] = (DCM2Q_in[0][1] + DCM2Q_in[1][0])*sqdip1;
            DCM2Q_out[2] = (DCM2Q_in[2][0] + DCM2Q_in[0][2])*sqdip1;
        }

    }

	//DCM2Q_out[3] = (0.5 * sqrt(DCM2Q_in[0][0] + DCM2Q_in[1][1] + DCM2Q_in[2][2] + 1.0)); ///q3 computation.

	//DCM2Q_out[0] = ((DCM2Q_in[1][2] - DCM2Q_in[2][1]) / (4.0 * DCM2Q_out[3])); ///q0 computation.
	//DCM2Q_out[1] = ((DCM2Q_in[2][0] - DCM2Q_in[0][2]) / (4.0 * DCM2Q_out[3])); ///q1 computation.
	//DCM2Q_out[2] = ((DCM2Q_in[0][1] - DCM2Q_in[1][0]) / (4.0 * DCM2Q_out[3])); ///q2 computation.
}

void rQs_Normalization(double const in_Quat_norm[4])
{
	double in_Quat_norm_Mag, in_Quat_norm_in[4];
	if(in_Quat_norm[3] < 0.0)
	{
        in_Quat_norm_in[0] = -1.0 * in_Quat_norm[0];
        in_Quat_norm_in[1] = -1.0 * in_Quat_norm[1];
        in_Quat_norm_in[2] = -1.0 * in_Quat_norm[2];
        in_Quat_norm_in[3] = -1.0 * in_Quat_norm[3];
	}
	else
	{
		in_Quat_norm_in[0] = in_Quat_norm[0];
		in_Quat_norm_in[1] = in_Quat_norm[1];
		in_Quat_norm_in[2] = in_Quat_norm[2];
		in_Quat_norm_in[3] = in_Quat_norm[3];
	}

	in_Quat_norm_Mag = sqrt((in_Quat_norm_in[0] * in_Quat_norm_in[0]) + (in_Quat_norm_in[1] * in_Quat_norm_in[1]) + (in_Quat_norm_in[2] * in_Quat_norm_in[2]) +(in_Quat_norm_in[3] * in_Quat_norm_in[3]));

    if(fabs(in_Quat_norm_Mag) <= c_dividebyzerovalue)
    {
        in_Quat_norm_Mag = c_dividebyzerovalue;
    }

    out_Quat_norm[0] = in_Quat_norm_in[0] / in_Quat_norm_Mag;
    out_Quat_norm[1] = in_Quat_norm_in[1] / in_Quat_norm_Mag;
    out_Quat_norm[2] = in_Quat_norm_in[2] / in_Quat_norm_Mag;
    out_Quat_norm[3] = in_Quat_norm_in[3] / in_Quat_norm_Mag;
}

///---------------------------Function to Multiply two Quaternions-----------------------------------------------
void rQs_Multiplication(double const in_Quat1[4],double const in_Quat2[4])
{
	out_Quat_mult[0] = (in_Quat1[3]*in_Quat2[0]) + (in_Quat2[3]*in_Quat1[0]) + (in_Quat1[1]*in_Quat2[2])-(in_Quat1[2]*in_Quat2[1]);
	out_Quat_mult[1] = (in_Quat1[3]*in_Quat2[1]) + (in_Quat2[3]*in_Quat1[1]) + (in_Quat1[2]*in_Quat2[0])-(in_Quat1[0]*in_Quat2[2]);
	out_Quat_mult[2] = (in_Quat1[3]*in_Quat2[2]) + (in_Quat2[3]*in_Quat1[2]) + (in_Quat1[0]*in_Quat2[1])-(in_Quat1[1]*in_Quat2[0]);
	out_Quat_mult[3] = (in_Quat1[3]*in_Quat2[3]) - (in_Quat1[0]*in_Quat2[0]) - (in_Quat1[1]*in_Quat2[1])-(in_Quat1[2]*in_Quat2[2]);
}


void rVectorNorm(double const vecnorm_in[3])
{
    vecnorm_mag = sqrt((vecnorm_in[0]*vecnorm_in[0]) + (vecnorm_in[1]*vecnorm_in[1]) + (vecnorm_in[2]*vecnorm_in[2]));

    if(fabs(vecnorm_mag) <= c_dividebyzerovalue)
    {
        vecnorm_mag = c_dividebyzerovalue;
    }

    Norm_out[0] = vecnorm_in[0]/vecnorm_mag;
    Norm_out[1] = vecnorm_in[1]/vecnorm_mag;
    Norm_out[2] = vecnorm_in[2]/vecnorm_mag;
}

///---------------------------Function to find the adjoint of a matrix[3][3]------------------------------
void rMat_adjoint3(double in_mat_adj[3][3])
{
	///First Row
	mat_adj[0][0] = ((in_mat_adj[1][1]) * (in_mat_adj[2][2])) - ((in_mat_adj[2][1]) * (in_mat_adj[1][2]));
	mat_adj[0][1] = ((in_mat_adj[2][0]) * (in_mat_adj[1][2])) - ((in_mat_adj[1][0]) * (in_mat_adj[2][2]));
	mat_adj[0][2] = ((in_mat_adj[1][0]) * (in_mat_adj[2][1])) - ((in_mat_adj[2][0]) * (in_mat_adj[1][1]));

	///Second Row
	mat_adj[1][0] = ((in_mat_adj[2][1]) * (in_mat_adj[0][2])) - ((in_mat_adj[0][1]) * (in_mat_adj[2][2]));
	mat_adj[1][1] = ((in_mat_adj[0][0]) * (in_mat_adj[2][2])) - ((in_mat_adj[2][0]) * (in_mat_adj[0][2]));
	mat_adj[1][2] = ((in_mat_adj[2][0]) * (in_mat_adj[0][1])) - ((in_mat_adj[0][0]) * (in_mat_adj[2][1]));

	///Third Row
	mat_adj[2][0] = ((in_mat_adj[0][1]) * (in_mat_adj[1][2])) - ((in_mat_adj[1][1]) * (in_mat_adj[0][2]));
	mat_adj[2][1] = ((in_mat_adj[1][0]) * (in_mat_adj[0][2])) - ((in_mat_adj[0][0]) * (in_mat_adj[1][2]));
	mat_adj[2][2] = ((in_mat_adj[0][0]) * (in_mat_adj[1][1])) - ((in_mat_adj[1][0]) * (in_mat_adj[0][1]));
}

void rMatMul34x1(double mat34[3][4], double const vec41[4])
{
    for(i_comr=0; i_comr<3; i_comr++)
    {
        Matout341[i_comr] = 0.0;
        for(j_comr=0; j_comr<4; j_comr++)
        {
            Matout341[i_comr] += mat34[i_comr][j_comr]*vec41[j_comr];
        }
    }
}

void rMatMul43x1(double mat43[4][3], double const vec31[3])
{
    for(i_comr=0; i_comr<4; i_comr++)
    {
        Matout431[i_comr] = 0.0;
        for(j_comr=0; j_comr<3; j_comr++)
        {
            Matout431[i_comr] += + mat43[i_comr][j_comr]*vec31[j_comr];
        }
    }
}

void rMatMul44x1(double mat44[4][4], double const vec41[4])
{
    for (i_comr = 0; i_comr < 4; i_comr++)
    {
        Matout441[i_comr] = 0.0;
        for (j_comr = 0; j_comr < 4; j_comr++)
        {
          Matout441[i_comr] += mat44[i_comr][j_comr] * vec41[j_comr];
        }
    }
}


