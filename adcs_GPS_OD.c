#include <stdio.h>
#include <math.h>
#include "HAL_Address.h"
#include "adcs_VarDeclarations.h"
#include "Global.h"
#include "Telecommand.h"
#include "HAL_Global.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "TC_List.h"
#include "HAL_GPS.h"

void rGPSTLEProcessing(void)
{
	GPS_1_DATA();                            //GPS

	rGPS_TM_Extract();						//Extract GPS data for telemetry
	ST_TM_gps_data();

	if(TC_boolean_u.TC_Boolean_Table.TC_GPS_TLE_Select== True)
	{
		rGPSDataProcessing();
	}
	else if(TC_boolean_u.TC_Boolean_Table.TC_GPS_TLE_Select == False)
	{
		rTLEDataProcessing();
	}
	else
	{
		//
	}
}

void rOrbit_Initialization(void)
{
    if (CB_OrbitModel == Enable)
    {
        ///----------------------Initialization------------------------------------------------------------
        //DeltaT computation using TLE data added on 4-3-2016
        Day_Of_Year_DeltaT = epochdays_sel + (Tsince/1440.0);
        if((int)epochyr_sel % 4 == 0)///epochyr is not got by GPS, how to implement this logic?
        {
            Delta_T = (((2000.0 + (float)epochyr_sel) + ((float)Day_Of_Year_DeltaT / 366.0)) - 2015.0);/// replace (2000.0 + epochyr_sel) with year_sel
            ///DeltaT_MFC = DeltaT_Updated;
        }
        else
        {
            Delta_T = (((2000.0 + (float)epochyr_sel) + ((float)Day_Of_Year_DeltaT / 365.0)) - 2015.0);
            ///DeltaT_MFC = DeltaT_Updated;
        }

        //Orbit frequency computation added on 4-3-16

        if(abs_f(no_sel) <= c_dividebyzerovalue)
        {
            no_sel = c_dividebyzerovalue;
        }

        Orbit_Period_Comp = c_Day_To_Seconds / no_sel;

        if(abs_f(Orbit_Period_Comp) <= c_dividebyzerovalue)
        {
            Orbit_Period_Comp = c_dividebyzerovalue;
        }

        wo = (2.0 * c_Pi) / Orbit_Period_Comp;


        //Block4: Find standard orbital elements
        inclo = inclination_sel;
        nodeo = nodeo_sel;
        argpo = argpo_sel;
        mo    = mo_sel;
        ecco = ecc_sel;
        no   = no_sel / c_xpdotp;
        ibexp = ibexp_sel;
        bstar= bstar_sel;

        //Block5: initialization the variables with respective values
        con41   = 0.0; cc1    = 0.0; cc4      = 0.0;
        cc5     = 0.0; d2     = 0.0; d3       = 0.0;
        d4      = 0.0; delmo = 0.0; eta      = 0.0;
        argpdot = 0.0; omgcof = 0.0; sinmao   = 0.0;
        aycof   = 0.0; t2cof  = 0.0; t3cof    = 0.0;
        t4cof   = 0.0; t5cof = 0.0;  x1mth2   = 0.0;
        x7thm1 = 0.0; mdot   = 0.0; nodedot = 0.0;
        xlcof   = 0.0; xmcof  = 0.0; nodecf  = 0.0;

        //Block6: Calculate auxiliary epoch quantities
        eccsq = ecco * ecco;
        omeosq = 1.0 - eccsq;
        rteosq = sqrt(omeosq);
        cosio = cos(inclo);
        cosio2 = cosio * cosio;

        //Block7: Compute un-kozai the mean motion
        if (abs_f (no) <= c_dividebyzerovalue)
        {
            no = c_dividebyzerovalue;
        }


        ak_temp = (c_xke / no);
        ak    = pow(ak_temp, c_x2o3);
        rtom = (rteosq * omeosq);

        if(abs_f(rtom) <= c_dividebyzerovalue)
        {
            rtom = c_dividebyzerovalue;
        }


        d1    = 0.75 * c_j2 * (3.0 * cosio2 - 1.0) / rtom;

        if(abs_f(ak) <= c_dividebyzerovalue)
        {
            ak = c_dividebyzerovalue;
        }

        del   = d1 / (ak * ak);
        adel  = ak * (1.0 - del * del - del *(1.0 / 3.0 + 134.0 * del * del / 81.0));

        if(abs_f(adel) <= c_dividebyzerovalue)
        {
            adel = c_dividebyzerovalue;
        }

        del   = d1/(adel * adel);
        no    = no / (1.0 + del);

        if (abs_f (no) <= c_dividebyzerovalue)
        {
            no = c_dividebyzerovalue;
        }

        ao_temp = c_xke / no;
        ao    = pow(ao_temp,c_x2o3);
        sinio = sin(inclo);
        po    = ao * omeosq;
        con42 = 1.0 - 5.0 * cosio2;
        con41 = -con42-cosio2-cosio2;
        posq = po * po;
        rp    = ao * (1.0 - ecco);

        if ((omeosq >= 0.0 ) || ( no >= 0.0)) //Block8:if condition check is then go to next computation,else return
        {
            //Block9: Compute Perigee
            sfour = c_ss;
            qzms24 = c_qzms2t;
            perigee = (rp - 1.0) * c_radiusearthkm;


            if (perigee < 156.0)	//Block10: For perigees below 156 km, s and qoms2t are altered
            {
                //Block11: Then sfour and qzms24 are altered
                sfour = perigee - 78.0;
                if(perigee < 98.0)
                {
                    sfour = 20.0;
                }
                qzms24_temp = (120.0 - sfour) / c_radiusearthkm ;
                qzms24 = pow(qzms24_temp, 4.0);
                sfour  = sfour / c_radiusearthkm + 1.0;
            }

            //Block12: Compute intermediate variables.
            if(abs_f(posq) <= c_dividebyzerovalue)
            {
                posq = c_dividebyzerovalue;
            }

            pinvsq = 1.0 / posq;
            ao_sfour = (ao - sfour);

            if (abs_f(ao_sfour) <= c_dividebyzerovalue)
            {
                ao_sfour = c_dividebyzerovalue;
            }

            tsi  = 1.0 / ao_sfour;
            eta = ao * ecco * tsi;
            etasq = eta * eta;
            eeta = ecco * eta;
            psisq = abs_f(1.0 - etasq);
            coef = qzms24 * pow(tsi,4.0);
            pow_psisq = pow(psisq,3.5);

            if(abs_f(pow_psisq) <= c_dividebyzerovalue)
            {
                pow_psisq = c_dividebyzerovalue;
            }

            coef1 = coef / pow_psisq; //Corrected on 13-5-2015
            psetasq =  psisq * con41 * (8.0 + 3.0 * etasq * (8.0 + etasq));

            if(abs_f(psetasq) <= c_dividebyzerovalue)
            {
                psetasq = c_dividebyzerovalue;
            }

            cc2   = coef1 * no * (ao * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) + 0.375 * c_j2 * tsi / psetasq);   // need to put divide by zero exception check condition for psisq
            cc1   = bstar * cc2;
            cc3 = 0.0;

            if(ecco > 0.0001)
            {
                cc3 = -2.0 * coef * tsi * c_j3oj2 * no * sinio / ecco;           // (not needed divided by zero check as this happens only if ecco > 0.0001)
            }

            x1mth2 = 1.0 - cosio2;
            intermediate = (ao * psisq);

            if (abs_f(intermediate) <= c_dividebyzerovalue)
            {
                intermediate = c_dividebyzerovalue;
            }

            cc4    = 2.0* no * coef1 * ao * omeosq *(eta * (2.0 + 0.5 * etasq) + ecco * (0.5 + 2.0 * etasq) - c_j2 * tsi / intermediate * (-3.0 * con41 * (1.0 - 2.0 * eeta + etasq * (1.5 - 0.5 * eeta)) + 0.75 * x1mth2 *(2.0 * etasq - eeta * (1.0 + etasq)) * cos(2.0 * argpo)));    // need to put divide by zero exception check condition
            cc5 = 2.0 * coef1 * ao * omeosq * (1.0 + 2.75 *(etasq + eeta) + eeta * etasq);
            cosio4 = cosio2 * cosio2;
            temp1_or_init = 1.5 * c_j2 * pinvsq * no;
            temp2 = 0.5 * temp1_or_init * c_j2 * pinvsq;
            temp3 = -0.46875 * c_j4 * pinvsq * pinvsq * no;
            mdot   = no + 0.5 * temp1_or_init * rteosq * con41 + 0.0625 * temp2 * rteosq * (13.0 - 78.0 * cosio2 + 137.0 * cosio4);
            argpdot = -0.5 * temp1_or_init * con42 + 0.0625 * temp2 * (7.0 - 114.0 * cosio2 + 395.0 * cosio4) + temp3 * (3.0 - 36.0 * cosio2 + 49.0 * cosio4);
            xhdot1 = -temp1_or_init * cosio;
            nodedot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * cosio2) + 2.0 * temp3 * (3.0 - 7.0 * cosio2)) * cosio;
            omgcof = bstar * cc3 * cos(argpo);
            xmcof = 0.0;

            if(ecco > 0.0001) //Corrected on 13-5-2015
            {
                if(abs_f(eeta) <= c_dividebyzerovalue)
                {
                    eeta = c_dividebyzerovalue;
                }
            }

            eeta = c_dividebyzerovalue;
            xmcof = -c_x2o3 * coef * bstar / eeta;
            nodecf = 3.5 * omeosq * xhdot1 * cc1;
            t2cof = 1.5 * cc1;

            //Block13: sgp4fix for divide by zero with xinco = 180 deg
            if (abs_f (cosio + 1.0) > (0.0000000000015))//Corrected on 13-5-2015
            {
                xlcof = -0.25 * c_j3oj2 * sinio *(3.0 + 5.0 * cosio) / (1.0 + cosio);          // as + condition is present, divide by zero exception will not arise
            }
            else
            {
                xlcof = -0.25 * c_j3oj2 * sinio *(3.0 + 5.0 * cosio) / c_temp4;
            }


            aycof = -0.5 * c_j3oj2 * sinio;
            delmo_temp = (1.0 +eta * cos(mo));
            delmo = pow(delmo_temp, 3.0);
            sinmao = sin(mo);
            x7thm1 = 7.0 * cosio2 - 1.0;

            //Block14: Set variables if not deep space
            cc1sq = cc1 * cc1;
            d2 = 4.0 * ao * tsi * cc1sq;
            temp_or_init = d2 * tsi * cc1 / 3.0;
            d3 = (17.0 * ao + sfour) * temp_or_init;
            d4 = 0.5 * temp_or_init * ao * tsi *(221.0 * ao + 31.0 * sfour) *cc1;
            t3cof = d2 + 2.0 * cc1sq;
            t4cof = 0.25 * (3.0 *d3 + cc1 *(12.0 * d2 + 10.0 * cc1sq));
            t5cof = 0.2 * (3.0 * d4 + 12.0 * cc1 * d3 +6.0 * d2 * d2 + 15.0 * cc1sq * (2.0 * d2 + cc1sq));
        }
    }
}


void rOrbit_Propagation(void)
{
	if(CB_OrbitModel == Enable)// && OrbitModel_Start ==Set)
	{
//Block1: Update for secular gravity and atmospheric
		xmdf    = mo + mdot * Tsince;
		argpdf = argpo + argpdot * Tsince;
		nodedf = nodeo + nodedot * Tsince;
		argpm   = argpdf;
		mm = xmdf;
		t2 = Tsince * Tsince;
		nodem   = nodedf + nodecf * t2;
		tempa   = 1.0 - cc1 * Tsince;
		tempe   = bstar * cc4 * Tsince;
		templ   = t2cof * t2;

		delomg = omgcof * Tsince;

		del_temp = (1.0 + eta * cos(xmdf));
		delm   = xmcof * (pow(del_temp,3.0) - delmo);

		temp_om   = delomg + delm;
		mm     = xmdf + temp_om;
		argpm = argpdf - temp_om;
		t3 = t2 * Tsince;
		t4 = t3 * Tsince;
		tempa  = tempa - d2 * t2 - d3 * t3 - d4 * t4;
		etempe = tempe + bstar * cc5 * (sin(mm) - sinmao);
		templ = templ + t3cof * t3 + t4 * (t4cof + Tsince * t5cof);
		nm    = no;
		em    = ecco;
		inclm = inclo;

		if(abs_f(nm) <= c_dividebyzerovalue)
        {
            nm = c_dividebyzerovalue;
        }


		am_temp = (c_xke / nm);
		am = (pow(am_temp, c_x2o3)) * tempa * tempa;
		nm_temp = pow(am,1.5);

		if(abs_f(nm_temp) <= c_dividebyzerovalue)
        {
            nm_temp = c_dividebyzerovalue;
        }


		nm = c_xke / nm_temp;
		em = em - etempe;

		//Block2: change test condition for eccentricity
		if (em < 0.000001)
        {
            em = (1.0 * 0.000001);
        }


		mm = mm + no * templ;
		xlm = mm + argpm + nodem;
		emsq   = em * em;
		temp_om   = 1.0 - emsq;
		nodem = fmod(nodem, c_Twopi);
		argpm = fmod(argpm, c_Twopi);
		xlm    = fmod(xlm, c_Twopi);
		mm_temp = xlm - argpm - nodem;
		mm = fmod(mm_temp,c_Twopi);

		//Block3: compute extra mean e_vec[0]quantities
		sinim = sin(inclm);
		cosim = cos(inclm);

		//Block4: add lunar-solar periodic
		ep     = em;
		xincp = inclm;
		argpp = argpm;
		nodep = nodem;
		mp     = mm;
		sinip = sinim;
		cosip = cosim;

		//block5: long period periodic
		axnl = ep * cos(argpp);
		am_den = (am * (1.0 - ep * ep));

		if(abs_f(am_den) <= c_dividebyzerovalue)
        {
            am_den = c_dividebyzerovalue;
        }

		temp_om = 1.0 / am_den;
		aynl = ep* sin(argpp) + temp_om * aycof;
		xl   = mp + argpp + nodep + temp_om * xlcof * axnl;

		//Block6: solve Kepler's equation
		u_temp = xl - nodep;
		uuu = fmod(u_temp, c_Twopi);
		eol = uuu;
		var = 9999.9;
		ktr = 1;

		//Block7: Kepler's iteration

        while ((abs_f(var) >= (0.000000000001))&&(ktr <= 10))
        {
            //if(abs_f(var) >= (0.000000000001))
            {
                sinkp = sin(eol);
                coskp = cos(eol);
                temp_prop = 1.0 - (coskp * axnl) - (sinkp * aynl);
                var = (uuu - aynl * coskp + axnl * sinkp - eol);
                if(abs_f(temp_prop) <= c_dividebyzerovalue)
                {
                    temp_prop = c_dividebyzerovalue;
                }


                var1 = var / temp_prop;

                if(abs_f(var1) >= 0.95)
                {
                    if (var1 > 0.0)
                    {
                        var1 = 0.95;
                    }

                    else
                    {
                        var1 = -0.95;
                    }
                }
                eol  = eol + var1;
                ktr = ktr + 1;
            }
        }

		//Block8: short period preliminary quantities
		ecose = axnl*coskp + aynl*sinkp;
		esine = axnl*sinkp - aynl*coskp;
		el2   = axnl*axnl + aynl*aynl;
		pl = am * (1.0 - el2);

		if (pl < 0.0)
		{
			Pos_ECI[0] = 0.0;
			Pos_ECI[1] = 0.0;
			Pos_ECI[2] = 0.0;
			Vel_ECI[0] = 0.0;
			Vel_ECI[1] = 0.0;
			Vel_ECI[2] = 0.0;
		}

		else
		{
			rl = am * (1.0 - ecose);

			if(abs_f(r1) <= c_dividebyzerovalue)
            {
                r1= c_dividebyzerovalue;
            }


			rdotl  = sqrt(am) * esine/rl;
			rvdotl = sqrt(pl) / rl;
			betal  = sqrt(1.0 - el2);
			temp_temp   = esine / (1.0 + betal);

			if(abs_f(r1) <= c_dividebyzerovalue)
            {
                r1 = c_dividebyzerovalue;
            }


			sinu   = am / rl * (sinkp - aynl - axnl * temp_temp);
			cosu   = am / rl * (coskp - axnl + aynl * temp_temp);
			suu     = atan2(sinu, cosu);
			sin2u  = (cosu + cosu) * sinu;
			cos2u  = 1.0 - 2.0 * sinu * sinu;

			if(abs_f(pl) <= c_dividebyzerovalue)
            {
                pl = c_dividebyzerovalue;
            }


			temp_om   = 1.0 / pl;
			temp1  = 0.5 * c_j2 * temp_om;
			temp2  = temp1 * temp_om;

			//Block9:update for short period periodic
			mrt   = rl * (1.0 - 1.5 * temp2 * betal * con41) + 0.5 * temp1 * x1mth2 * cos2u;
			suu    = suu - 0.25 * temp2 * x7thm1 * sin2u;
			xnode = nodep + 1.5 * temp2 * cosip * sin2u;
			xinc  = xincp + 1.5 * temp2 * cosip * sinip * cos2u;
			mvt   = rdotl - nm * temp1 * x1mth2 * sin2u / c_xke;
			rvdot = rvdotl + nm * temp1 * (x1mth2 * cos2u + 1.5 * con41) / c_xke;

			//Block10: Orientation vectors
			sinsu = sin(suu);
			cossu = cos(suu);
			snod  = sin(xnode);
			cnod  = cos(xnode);
			sini  = sin(xinc);
			cosi  = cos(xinc);

			xmx   = -snod * cosi;
			xmy   =  cnod * cosi;
			ux    =  xmx * sinsu + cnod * cossu;
			uy    =  xmy * sinsu + snod * cossu;
			uz    =  sini * sinsu;
			vx    =  xmx * cossu - cnod * sinsu;
			vy    =  xmy * cossu - snod * sinsu;
			vz    =  sini * cossu;

			//Block11: position and velocity [ECI frame]
			Pos_ECI[0] = (mrt * ux)* c_radiusearthkm;
			Pos_ECI[1] = (mrt * uy)* c_radiusearthkm;
			Pos_ECI[2] = (mrt * uz)* c_radiusearthkm;
			Vel_ECI[0] = (mvt * ux + rvdot * vx) * c_vkmpersec;
			Vel_ECI[1] = (mvt * uy + rvdot * vy) * c_vkmpersec;
			Vel_ECI[2] = (mvt * uz + rvdot * vz) * c_vkmpersec;

			TM.Buffer.TM_Pos_ECI[0] = (int)(Pos_ECI[0]/9.31322575E-6);
			TM.Buffer.TM_Pos_ECI[1] = (int)(Pos_ECI[1]/9.31322575E-6);
			TM.Buffer.TM_Pos_ECI[2] = (int)(Pos_ECI[2]/9.31322575E-6);

			ST_normal.ST_NM_Buffer.TM_Pos_ECI[0] = (int)(Pos_ECI[0]/9.31322575E-6);
			ST_normal.ST_NM_Buffer.TM_Pos_ECI[1] = (int)(Pos_ECI[1]/9.31322575E-6);
			ST_normal.ST_NM_Buffer.TM_Pos_ECI[2] = (int)(Pos_ECI[2]/9.31322575E-6);

			ST_special.ST_SP_Buffer.TM_Pos_ECI[0] = (int)(Pos_ECI[0]/9.31322575E-6);
			ST_special.ST_SP_Buffer.TM_Pos_ECI[1] = (int)(Pos_ECI[1]/9.31322575E-6);
			ST_special.ST_SP_Buffer.TM_Pos_ECI[2] = (int)(Pos_ECI[2]/9.31322575E-6);

			TM.Buffer.TM_Vel_ECI[0] = (int)(Vel_ECI[0]/9.31322575E-6);
			TM.Buffer.TM_Vel_ECI[1] = (int)(Vel_ECI[1]/9.31322575E-6);
			TM.Buffer.TM_Vel_ECI[2] = (int)(Vel_ECI[2]/9.31322575E-6);

			ST_normal.ST_NM_Buffer.TM_Vel_ECI[0] = (int)(Vel_ECI[0]/9.31322575E-6);
			ST_normal.ST_NM_Buffer.TM_Vel_ECI[1] = (int)(Vel_ECI[1]/9.31322575E-6);
			ST_normal.ST_NM_Buffer.TM_Vel_ECI[2] = (int)(Vel_ECI[2]/9.31322575E-6);

			ST_special.ST_SP_Buffer.TM_Vel_ECI[0] = (int)(Vel_ECI[0]/9.31322575E-6);
			ST_special.ST_SP_Buffer.TM_Vel_ECI[1] = (int)(Vel_ECI[1]/9.31322575E-6);
			ST_special.ST_SP_Buffer.TM_Vel_ECI[2] = (int)(Vel_ECI[2]/9.31322575E-6);

		}

        rMatMul3x1(ECItoECEF, Pos_ECI);
        Pos_ECEF[0] = Matout31[0];
        Pos_ECEF[1] = Matout31[1];
        Pos_ECEF[2] = Matout31[2];

        wcreci[0] = -c_OMEGAE*Pos_ECEF[2];
        wcreci[1] = c_OMEGAE*Pos_ECEF[1];
        wcreci[2] = 0.0;

        rMatMul3x1(ECItoECEF, Vel_ECI);
        Vel_ECEF_temp[0] = Matout31[0];
        Vel_ECEF_temp[1] = Matout31[1];
        Vel_ECEF_temp[2] = Matout31[2];

        Vel_ECEF[0] =  Vel_ECEF_temp[0] - wcreci[0];
        Vel_ECEF[1] =  Vel_ECEF_temp[1] - wcreci[1];
        Vel_ECEF[2] =  Vel_ECEF_temp[2] - wcreci[2];

        rVectorNorm(Pos_ECI);
        Pos_ECIn[0] = Norm_out[0];
        Pos_ECIn[1] = Norm_out[1];
        Pos_ECIn[2] = Norm_out[2];

        rVectorNorm(Vel_ECI);
        Vel_ECIn[0] = Norm_out[0];
        Vel_ECIn[1] = Norm_out[1];
        Vel_ECIn[2] = Norm_out[2];

        rVectorNorm(Pos_ECEF);
        Pos_ECEFn[0] = Norm_out[0];
        Pos_ECEFn[1] = Norm_out[1];
        Pos_ECEFn[2] = Norm_out[2];

        rVectorNorm(Vel_ECEF);
        Vel_ECEFn[0] = Norm_out[0];
        Vel_ECEFn[1] = Norm_out[1];
        Vel_ECEFn[2] = Norm_out[2];

        Tsince = Tsince + c_tsince_min;
        TM.Buffer.TM_Tsince = (int)(Tsince/0.01);
        ST_special.ST_SP_Buffer.TM_Tsince = (int)(Tsince/0.01);
        ST_normal.ST_NM_Buffer.TM_Tsince = (int)(Tsince/0.01);

	}
	return;
}


void rOrbitalElements_computation(double Pos_ECI_in[3], double Vel_ECI_in[3], double Pos_ECEF_in[3])
{
	if(CB_OrbitModel == Set)// && CB_OrbitModel == Enable)
	{
		//Block1: Compute the radial distance using the position vector
		radialdistance = sqrt(Pos_ECI_in[0] * Pos_ECI_in[0] + Pos_ECI_in[1] * Pos_ECI_in[1] + Pos_ECI_in[2] * Pos_ECI_in[2]);

		radialdistance_ecef = sqrt(Pos_ECEF_in[0] * Pos_ECEF_in[0] + Pos_ECEF_in[1] * Pos_ECEF_in[1] + Pos_ECEF_in[2] * Pos_ECEF_in[2]);

		r_delta = sqrt(Pos_ECEF_in[0] * Pos_ECEF_in[0] + Pos_ECEF_in[1] * Pos_ECEF_in[1]);

		velmag = sqrt(Vel_ECI_in[0] * Vel_ECI_in[0] + Vel_ECI_in[1] * Vel_ECI_in[1] + Vel_ECI_in[2] * Vel_ECI_in[2]);

		//Block4: compute the square of the magnitude of the velocity vector
		velmagsq = velmag * velmag;

		/// Compute the Eccentricity
		rdtv = (Pos_ECI_in[0] * Vel_ECI_in[0]) + (Pos_ECI_in[1] * Vel_ECI_in[1]) + (Pos_ECI_in[2] * Vel_ECI_in[2]);

		if(abs_f(radialdistance) <= c_dividebyzerovalue)
        {
            radialdistance = c_dividebyzerovalue;
        }

		ecc_temp1 = velmagsq - (c_mu/radialdistance);

		ecc_temp2 = ecc_temp1 * Pos_ECI_in[0];

		ecc_temp3 = ecc_temp1 * Pos_ECI_in[1];

		ecc_temp4 = ecc_temp1 * Pos_ECI_in[2];

		ecc_temp5 = rdtv * Vel_ECI_in[0];

		ecc_temp6 = rdtv * Vel_ECI_in[1];

		ecc_temp7 = rdtv * Vel_ECI_in[2];

		e_vec[0] = (ecc_temp2 - ecc_temp5) * c_invmu;

		e_vec[1] = (ecc_temp3 - ecc_temp6) * c_invmu;

		e_vec[2] = (ecc_temp4 - ecc_temp7) * c_invmu;

		ecc = sqrt(e_vec[0]*e_vec[0] + e_vec[1]*e_vec[1] + e_vec[2]*e_vec[2]);

		if(abs_f(ecc) <= c_dividebyzerovalue)
        {
            ecc = c_dividebyzerovalue;
        }
		///Compute the semimajor axis
		semi_den = (c_twomu-radialdistance*velmagsq);

		if(abs_f(semi_den) <= c_dividebyzerovalue)
        {
            semi_den = c_dividebyzerovalue;
        }

		semimajoraxis= (radialdistance*c_mu / semi_den);

		if((1.0-abs_f(ecc)) <= c_dividebyzerovalue)
        {
            semimajoraxis = c_dividebyzerovalue;
        }

		Alti = radialdistance - c_radiusearthkm; ///Correction required Alti = semimajoraxis - c_radiusearthkm;

		///Compute the X,Y,Z component of the angular momentum
		rCross_Product(Pos_ECI_in, Vel_ECI_in);
		angmomentumvec[0] = Cross_Product[0];
		angmomentumvec[1] = Cross_Product[1];
		angmomentumvec[2] = Cross_Product[2];

		/// compute square of the magnitude of angular momentum vector
		angmomentumvecmag = sqrt(angmomentumvec[0] * angmomentumvec[0] + angmomentumvec[1] * angmomentumvec[1] + angmomentumvec[2] * angmomentumvec[2]);

		delta_hmag = sqrt(angmomentumvec[1] * angmomentumvec[1] + angmomentumvec[0] * angmomentumvec[0]);

        /// compute the inverse of the square of angular momentum vector magnitude
		if(abs_f(angmomentumvecmag) <= c_dividebyzerovalue)
        {
            angmomentumvecmag = c_dividebyzerovalue;
        }
		invangmomentumvecmag = 1.0 / angmomentumvecmag;

		///compute Inclination
		inclination_temp = angmomentumvec[2] * invangmomentumvecmag;
		inclination = acos(inclination_temp);


		/// Right Ascension of ascending node
		sinlongacnode = angmomentumvec[0] * invangmomentumvecmag;

		if(delta_hmag <= c_dividebyzerovalue)
		{
		    delta_hmag = c_dividebyzerovalue;
		}
		coslongacnode = -1.0*angmomentumvec[1] / delta_hmag;

        if(abs_f(coslongacnode) >= 1.0)
        {
            coslongacnode = sign_f(coslongacnode);
        }

		RAAN = acos(coslongacnode);

		if(angmomentumvec[1] <= 0.0)
        {
            RAAN = c_Twopi - RAAN;
        }

        ///compute True anomaly
		e_vecdtr = (Pos_ECI_in[0] * e_vec[0]) + (Pos_ECI_in[1] * e_vec[1]) + (Pos_ECI_in[2] * e_vec[2]);

		ecc_r = ecc * radialdistance;

		tempta = e_vecdtr/ecc_r;
		if(abs_f(tempta) >= 1.0)
        {
            tempta = sign_f(tempta);
        }

		trueanomoly = acos(tempta);

		if(rdtv <= 0.0)
        {
            trueanomoly = c_Twopi - trueanomoly;
        }

		///Compute the Argument of perigee
        Ndte_vec = -angmomentumvec[1]*e_vec[0] + angmomentumvec[0]*e_vec[1];

        N_ecc = delta_hmag * ecc;

        if(abs_f(N_ecc) <= c_dividebyzerovalue)
        {
            N_ecc = c_dividebyzerovalue;
        }

        temparg = Ndte_vec/N_ecc;
        if(abs_f(temparg) >= 1.0)
        {
            temparg = sign_f(temparg);
        }

        argofperigee = acos(temparg);

        if(e_vec[2] <= 0.0)
        {
            argofperigee = c_Twopi - argofperigee;
        }

        ///Eccentric anomaly
        omecc = 1.0 + ecc*cos(trueanomoly);

        if(abs_f(omecc) <= c_dividebyzerovalue)
        {
            omecc = c_dividebyzerovalue;
        }

        sine = sqrt(1.0-ecc*ecc)*sin(trueanomoly)/omecc;

		cose = ecc + cos(trueanomoly)/omecc;
        eccanomaly = atan2(sine,cose);

        ///Mean anomaly computation. Is this required here?
        mo = eccanomaly - ecc*sin(eccanomaly);
        mo = fmod(mo,c_Twopi);
        if(mo <= 0.0)
        {
            mo = mo + c_Twopi;
        }

		///Compute the longitude of the spacecraft using position in ECF frame

		if(abs_f(r_delta) <= c_dividebyzerovalue)
        {
            r_delta = c_dividebyzerovalue;
        }

		longitude_tan_num = Pos_ECEF_in[1]/r_delta;
		longitude_tan_den = Pos_ECEF_in[0]/r_delta;

		longitude = atan2(longitude_tan_num,longitude_tan_den);

		///Compute the latitude of the satellite

		if(abs_f(radialdistance_ecef) <= c_dividebyzerovalue)
		{
			radialdistance_ecef = c_dividebyzerovalue;
		}

		latitude_temp = (Pos_ECEF_in[2]/radialdistance_ecef);
		latitude = asin(latitude_temp);

		xa_gcgd = 6356752.314245/(sqrt(tan(latitude)*tan(latitude) + 0.99330562));

		if (xa_gcgd<c_radiusearthkm)
			mua_gcgd = atan2(sqrt(c_radiusearthkm*c_radiusearthkm*1000000.0 - xa_gcgd*xa_gcgd),0.996647189335*xa_gcgd);
		else
			mua_gcgd = 0.0;

		if (latitude < 0)
		{
			mua_gcgd = -mua_gcgd;
		}

		ra_gcgd = xa_gcgd/cos(latitude);

		l_gcgd = radialdistance_ecef*1.0 - ra_gcgd;

		dlambda_gcgd = mua_gcgd - latitude;

		h_gcgd = l_gcgd*cos(dlambda_gcgd);

		den_gcgd = 1 - (0.00669437999)*sin(mua_gcgd)*sin(mua_gcgd);
		rhoa_gcgd = (6335439.32722)/sqrt(den_gcgd*den_gcgd*den_gcgd);

		dmu_gcgd = atan2(l_gcgd*sin(dlambda_gcgd),rhoa_gcgd + h_gcgd);

		gd_gcgd = mua_gcgd - dmu_gcgd;

		latitude = gd_gcgd;
        Orbit_Period = c_Twopi*sqrt(pow(semimajoraxis,3.0)/c_mu);

        if(abs_f(Orbit_Period) <= c_dividebyzerovalue)
        {
            Orbit_Period = c_dividebyzerovalue;
        }

		no = 24.0*3600.0/Orbit_Period;

		TM.Buffer.TM_Latitude = (int)(latitude/4.19095159E-8);
		ST_special.ST_SP_Buffer.TM_Latitude = (int)(latitude/4.19095159E-8);
		TM.Buffer.TM_Longitude = (int)(longitude/8.38190317E-8);
		ST_special.ST_SP_Buffer.TM_Longitude = (int)(longitude/8.38190317E-8);
		TM.Buffer.TM_Orb_Period = (int)(Orbit_Period/1.0);
		TM.Buffer.TM_Altitude = (int)(Alti/4.658941181E-7);
		ST_special.ST_SP_Buffer.TM_Altitude = (int)(Alti/4.658941181E-7);
	}
	return;
}

void rTLEDataProcessing(void)
{
    if (CB_OrbitModel == Enable)
    {
    	if (TLE_Data_Available == 1)
    	{
			for(i_jday = 1; i_jday <= 12; i_jday++)
			{
				lmonth[i_jday] = 0;
			}

			year_TLE_tc = Epochyear_TLE_tc + 2000;

			for (i_jday = 1; i_jday <=12; i_jday++)
			{
				lmonth[i_jday] = 31;
				if (i_jday == 2)
				{
					lmonth[i_jday] = 28;
				}

				if (i_jday == 4 || i_jday == 6 || i_jday == 9 || i_jday == 11)
				{
					lmonth[i_jday] = 30;
				}

			}

			dayofyr = (int)(epochdays_TLE_tc);

			/*if (fmod(year_TLE_tc-1900,4.0) == 0.0)
			{
				lmonth[2]= 29;
			}*/

			if (((year_TLE_tc-1900) % 4) == 0)
			{
				lmonth[2]= 29;
			}


			i_jday= 1;
			inttemp= 0;

			while (( dayofyr > (inttemp + lmonth[i_jday] )) && ( i_jday < 12 ))
			{
				inttemp= inttemp + lmonth[i_jday];
				i_jday= i_jday + 1;
			}

			mon_TLE_tc= i_jday;
			//mon_TLE_tc= dayofyr - inttemp;
			temp_jday= (epochdays_TLE_tc - (double)dayofyr )*24.0;
			hr_TLE_tc  = (int)( temp_jday );
			temp_jday= (temp_jday - (double)hr_TLE_tc) * 60.0;
			minute_TLE_tc = (int)(temp_jday);
			sec_TLE_tc = (temp_jday - (double)minute_TLE_tc) * 60.0;


			//Tsince = Tsince_TLE_tc;
			Delta_TLE = Minor_Cycle_Count - OBT_at_TLE_epoch;
			Tsince = (double)Delta_TLE * c_MiC/c_min_per_day;
			epochdays_sel = epochdays_TLE_tc;
			inclination_sel = inclination_TLE_tc*c_D2R;
			nodeo_sel = nodeo_TLE_tc*c_D2R;
			mo_sel = mo_TLE_tc*c_D2R;
			argpo_sel = argpo_TLE_tc*c_D2R;
			epochyr_sel = Epochyear_TLE_tc;
			ecc_sel = ecc_TLE_tc;
			no_sel = no_TLE_tc;
			ibexp_sel = ibexp_TLE_tc;
			bstar_sel = bstar_TLE_tc * pow(10.0, ibexp_TLE_tc);
			year_sel = year_TLE_tc;
			mon_sel = mon_TLE_tc;
			days_sel = day_TLE_tc;
			hr_sel = hr_TLE_tc;
			minute_sel = minute_TLE_tc;
			sec_sel = sec_TLE_tc;
			TLE_Data_Available = 0;
		}
    }
}

void rJulian_Day(int year, int mon, int days, int hr, int minute, double sec)
{
    if (CB_OrbitModel == Enable)
    {
        Julian_day = 367.0 * (float)year - floor( (7.0 * ((float)year + floor( ((float)mon + 9.0) / 12.0) ) ) * 0.25 )+ floor( 275.0 * (float)mon / 9.0 )+ (float)days + 1721013.5;      // as denominator consists of constant, divide by zero exception will not arise

        jd_time = ((sec/60.0 + (float)minute) / 60.0 + (float)hr ) / 24.0 + Tsince/c_min_per_day;
        tut = ( (Julian_day + jd_time) - 2451545.0 ) / 36525.0;

        return;
    }
}

void rNEDtoECEF(void)
{
    if (CB_OrbitModel == Enable)
    {
        NEDtoECEF[0][0] = -cos(longitude) * sin(latitude);
        NEDtoECEF[0][1] = -sin(longitude);
        NEDtoECEF[0][2] = -cos(longitude) * cos(latitude);
        NEDtoECEF[1][0] = -sin(longitude) * sin(latitude);
        NEDtoECEF[1][1] = cos(longitude);
        NEDtoECEF[1][2] = -sin(longitude) * cos(latitude);
        NEDtoECEF[2][0] = cos(latitude);
        NEDtoECEF[2][1] = 0.0;
        NEDtoECEF[2][2] = -sin(latitude);
    }
}

void rECEFtoECItoECEF(void)
{
    if (CB_OrbitModel == Enable)
    {
        /// compute julian day of UT1 JDUT1
        UT1 = jd_time * c_min_per_day;

        UTC = UT1*60.0 - ADCS_TC_data_command_Table.TC_delUT1_ECEF2ECI;

        /// compute julian day of UTC JDUTC

        TAI = UTC + ADCS_TC_data_command_Table.TC_delAT_ECEF2ECI;

        TDT = TAI + 32.184;

        /// compute julian day of TDT JDTDT
        JDTDT = Julian_day + TDT/c_Day_To_Seconds;

        TTDT = (JDTDT - 2451545.0) / 36525.0;

        M_quad = (357.5277233 + (35999.05034 * TTDT));

        if (abs_f(M_quad)>= 360.0)
        {
            M_quad = (fmod((M_quad),360.0));
        }


        sine1 =  sin(M_quad*c_D2R);
        sine2 = sin(2.0*M_quad*c_D2R);

        TDB = TDT + (0.00165800000 * sine1 + 0.000013850000 * sine2);

        /// compute julian day of TDB JDTDB
        JDTDB = Julian_day + TDB/c_Day_To_Seconds;

        TTDB = (JDTDB - 2451545.0) / 36525.0;

        TTDB2=TTDB*TTDB;
        TTDB3=TTDB2*TTDB;

        /// Precession

        zeta= (0.011180860*TTDB) + (1.464E-6*TTDB2)+ ( 8.7E-8*TTDB3);
        z = (0.011180860*TTDB) + (5.308E-6 * TTDB2)+ (8.9E-8*TTDB3);
        theta = (0.009717173*TTDB) -( 2.068E-6 *TTDB2) - ( 2.02E-7*TTDB3);


        ryRot(-theta);
        rzRot(zeta);

        rMatMul3x3(Ry, Rz);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                pre_temp[i_god][j_god] = Matout33[i_god][j_god];
            }
        }

        rzRot(z);
        rMatMul3x3(Rz, pre_temp);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                precession[i_god][j_god] = Matout33[i_god][j_god];
            }
        }

        // iau 1980 nutation
		eps=0.40909280 - 0.000226966*TTDB - 2.86E-9*TTDB2 + 8.8E-9*TTDB3;
		rxRot(-eps);
		rzRot(ADCS_TC_data_command_Table.TC_nut_dpsi);
		rMatMul3x3(Rz, Rx);
		for(i_god=0; i_god<3; i_god++)
		{
			for(j_god=0; j_god<3; j_god++)
			{
				nut_temp[i_god][j_god] = Matout33[i_god][j_god];
			}
		}

		xin_temp = eps+ADCS_TC_data_command_Table.TC_nut_deps;
		rxRot(xin_temp);
		rMatMul3x3(Rx, nut_temp);
		for(i_god=0; i_god<3; i_god++)
		{
			for(j_god=0; j_god<3; j_god++)
			{
				nutation[i_god][j_god] = Matout33[i_god][j_god];
			}
		}

        // astronomical arguments
        /*rast_args(TTDB);

        // iau 1980 nutation
        eps=0.40909280 - 0.000226966*TTDB - 2.86E-9*TTDB2 + 8.8E-9*TTDB3;
        rnut_iau1980(TTDB,f);
        rxRot(-eps);
        rzRot(dpsi);
        rMatMul3x3(Rz, Rx);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                nut_temp[i_god][j_god] = Matout33[i_god][j_god];
            }
        }

        xin_temp = eps+deps;
        rxRot(xin_temp);
        rMatMul3x3(Rx, nut_temp);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                nutation[i_god][j_god] = Matout33[i_god][j_god];
            }
        }*/

        tut1 = ((Julian_day) - 2451545.0)/36525.0;
        gmst0 = 100.4606184+36000.77005361*tut1+0.000038793*tut1*tut1-2.6E-8*tut1*tut1*tut1;
        gmst_=gmst0+(0.2506844773374)*UT1;
        gmst_=fmod(gmst_,360.0);
        gmst_ = gmst_ * c_D2R;
        gast=gmst_+dpsi*cos(eps);
        gast+=(0.00264*sin(f[4])+0.000063*sin(2.0*f[4]))*c_AS2R;
        //gast = 2.267539;
        rzRot(gast);

        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                sidereal[i_god][j_god] = Rz[i_god][j_god];
            }
        }

        ryRot(ADCS_TC_data_command_Table.TC_xp_ECEF2ECI*c_AS2R);
        rxRot(ADCS_TC_data_command_Table.TC_yp_ECEF2ECI*c_AS2R);
        rMatMul3x3(Ry, Rx);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                polarmotion[i_god][j_god] = Matout33[i_god][j_god];
            }
        }

//        rMatMul3x3(nutation, sidereal);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                nut_sid_temp[i_god][j_god] = sidereal[i_god][j_god];
            }
        }

        rMatMul3x3(precession, nut_sid_temp);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                ECEFtoECI[i_god][j_god] = Matout33[i_god][j_god];
            }
        }

        rMatInv(ECEFtoECI);
        for(i_god=0; i_god<3; i_god++)
        {
            for(j_god=0; j_god<3; j_god++)
            {
                ECItoECEF[i_god][j_god] = Invmatout33[i_god][j_god];
            }
        }
    }

}

/*void rast_args(double TTDBin)
{
    for(tt[0]=TTDBin,i_god=1 ; i_god<4 ; i_god++)
    {
        tt[i_god]=tt[i_god-1]*TTDBin;
    }
    for (i_god=0;i_god<5;i_god++)
    {
        f[i_god]=fc[i_god][0]*3600.0;
        for (j_god=0; j_god<4; j_god++)
        {
            f[i_god]= f[i_god]+fc[i_god][j_god+1]*tt[j_god];
        }
        f[i_god]=fmod(f[i_god]*c_AS2R,2.0*c_Pi);
    }
}

// iau 1980 nutation ---------------------------------------------------------
void rnut_iau1980(double TTDBin, const double *fin)
{
    dpsi=deps=0.0;

    for (i_god=0;i_god<106;i_god++)
    {
        ang=0.0;
        for (j_god=0;j_god<5;j_god++)
        {
            ang+=nut[i_god][j_god]*fin[j_god];
        }
        dpsi+=(nut[i_god][6]+nut[j_god][7]*TTDBin)*sin(ang);
        deps+=(nut[i_god][8]+nut[i_god][9]*TTDBin)*cos(ang);
    }
    dpsi*=1.0E-4*c_AS2R; // 0.1 mas -> rad
    deps*=1.0E-4*c_AS2R;
}*/

void rGPSDataProcessing(void)
{
	unsigned int tempdata;
    if (CB_OrbitModel == Enable)
    {
        if(f_GPS_Valid_Data ==1 && (Major_Cycle_Count % TC_GPS_pulse_duration == 0))
        {
        	Pos_ECEF_GPS[0] =(*((int*)(GPS_TM_Buffer_Addr_USC+160))) * 0.00001;
			Pos_ECEF_GPS[1] = (*((int*)(GPS_TM_Buffer_Addr_USC+164))) * 0.00001;
			Pos_ECEF_GPS[2] = (*((int*)(GPS_TM_Buffer_Addr_USC+168))) * 0.00001;
			Vel_ECEF_GPS[0] = (*((int*)(GPS_TM_Buffer_Addr_USC+172))) * 0.00001;
			Vel_ECEF_GPS[1] = (*((int*)(GPS_TM_Buffer_Addr_USC+176))) * 0.00001;
			Vel_ECEF_GPS[2] = (*((int*)(GPS_TM_Buffer_Addr_USC+180))) * 0.00001;
			year_GPS = *((unsigned short*)(GPS_TM_Buffer_Addr_USC+152));
			UTC_mon_GPS = *(GPS_TM_Buffer_Addr_USC+151);
			UTC_day_GPS = *(GPS_TM_Buffer_Addr_USC+150);
			UTC_hr_GPS = *(GPS_TM_Buffer_Addr_USC+154);
			UTC_min_GPS = *(GPS_TM_Buffer_Addr_USC+155);
			UTC_sec_GPS = (*((unsigned short*)(GPS_TM_Buffer_Addr_USC+156))) * 0.001;
			GPS_PPS_OBT = *(GPS_TM_Buffer_Addr_USC+207);

			GPS_OBT_Latch_enable          =  	(unsigned short)(GPS_Latch_enable & 0x0000FFFF);
			GPS_OBT_Read_1  =  GPS_OBT_Count_1 & 0x0000FFFF;
			TM.Buffer.TM_GPS_OBT_Read_1   = GPS_OBT_Read_1;
			tempdata = GPS_OBT_Count_2 & 0x0000FFFF;
			 GPS_OBT_Read_2 = ((tempdata << 16) & 0xFFFF0000);
			TM.Buffer.TM_GPS_OBT_Read_2   = GPS_OBT_Read_2;
			GPS_READ_OBT = (unsigned int)((GPS_OBT_Read_2 | GPS_OBT_Read_1) & 0xFFFFFFFF);
			gps_pulse_mic_cnt = GPS_READ_OBT - GPS_PPS_OBT;

            UTC_sec_GPS = UTC_sec_GPS + gps_pulse_mic_cnt * 0.001;

            rJulian_Day(year_GPS,UTC_mon_GPS,UTC_day_GPS,UTC_hr_GPS,UTC_min_GPS,UTC_sec_GPS);
            rECEFtoECItoECEF();
            Tsince = 0.0;
            for(i_jday = 1; i_jday <= 12; i_jday++)
            {
                lmonth[i_jday] = 0;
            }

            for (i_jday = 1; i_jday <=12; i_jday++)
            {
                lmonth[i_jday] = 31;
                if (i_jday == 2)
                {
                    lmonth[i_jday] = 28;
                }

                if (i_jday == 4 || i_jday == 6 || i_jday == 9 || i_jday == 11)
                {
                    lmonth[i_jday] = 30;
                }

            }

            /*if (fmod(year_GPS-1900,4.0) == 0.0)
            {
                lmonth[2]= 29;
            }*/

            if (((year_GPS-1900) % 4) == 0)
            {
                lmonth[2]= 29;
            }

            tempdays = 0;
            for(i_god=1; i_god<UTC_mon_GPS; i_god++)
            {
                tempdays = tempdays + lmonth[i_god];
            }

            Numofdays = tempdays + UTC_day_GPS;

            epochdays_GPS = (float)Numofdays + (float)UTC_hr_GPS/24.0 + (float)UTC_min_GPS/(60.0*24.0) + UTC_sec_GPS/(60.0*60.0*24.0);

            Epochyear_GPS = year_GPS - 2000;

            rMatMul3x1(polarmotion, Pos_ECEF_GPS);
            rpef[0] = Matout31[0];
            rpef[1] = Matout31[1];
            rpef[2] = Matout31[2];

            rMatMul3x1(polarmotion, Vel_ECEF_GPS);
            vpef[0] = Matout31[0];
            vpef[1] = Matout31[1];
            vpef[2] = Matout31[2];

            rMatMul3x1(ECEFtoECI, rpef);
            Pos_ECI_GPS[0] = Matout31[0];
            Pos_ECI_GPS[1] = Matout31[1];
            Pos_ECI_GPS[2] = Matout31[2];

            wcrecef[0] = -c_OMEGAE*rpef[2];
            wcrecef[1] = c_OMEGAE*rpef[1];
            wcrecef[2] = 0.0;

            Vel_ECI_temp[0] = wcrecef[0] + vpef[0];
            Vel_ECI_temp[1] = wcrecef[1] + vpef[1];
            Vel_ECI_temp[2] = wcrecef[2] + vpef[2];

            rMatMul3x1(ECEFtoECI, Vel_ECI_temp);
            Vel_ECI_GPS[0] = Matout31[0];
            Vel_ECI_GPS[1] = Matout31[1];
            Vel_ECI_GPS[2] = Matout31[2];

            rOrbitalElements_computation(Pos_ECI_GPS, Vel_ECI_GPS, Pos_ECEF_GPS);
            ecc_GPS = ecc;
            semimajoraxis_GPS = semimajoraxis;
            Alti_GPS = Alti;
            inclination_GPS = inclination;
            nodeo_GPS = RAAN;
            trueanomoly_GPS = trueanomoly;
            argpo_GPS = argofperigee;
            eccanomaly_GPS = eccanomaly;
            mo_GPS = mo;
            longitude_GPS = longitude;
            latitude_GPS = latitude;
            no_GPS = no;

            ///rGPS_data_validity();
            GPS_Elements_Available = 1;
            f_GPS_Valid_Data = 0; //RESET BY OBC DISCUSSED 11 SEP
            GPS_pulse_rcvd = 0;
            gps_pulse_mic_cnt = 0;
            GPSDataReady_NA_count = 0;

            Tsince_GPS = 0.0;
			Tsince = Tsince_GPS;
			epochdays_sel = epochdays_GPS;
			inclination_sel = inclination_GPS;
			nodeo_sel = nodeo_GPS;
			//trueanomoly_sel = trueanomoly_GPS;
			mo_sel = mo_GPS;
			argpo_sel = argpo_GPS;
			epochyr_sel = Epochyear_GPS;
			ecc_sel = ecc_GPS;
			no_sel = no_GPS;
			ibexp_sel = ibexp_TLE_tc;
			bstar_sel = bstar_TLE_tc * pow(10.0, ibexp_TLE_tc);
			year_sel = year_GPS;
			mon_sel = UTC_mon_GPS;
			days_sel = UTC_day_GPS;
			hr_sel = UTC_hr_GPS;
			minute_sel = UTC_min_GPS;
			sec_sel = UTC_sec_GPS;
        }
        else
        {
            GPSDataReady_NA_count = GPSDataReady_NA_count + 1;
            if (GPSDataReady_NA_count >= ADCS_TC_data_command_Table.TC_Time_GPS2TLE)
            {
                //TLE_Select = 1;
                //GPS_Select = 0;
                //Delta_TLE = Present_OBT - OBT_at_TLE_epoch;
                //Tsince = (float)Delta_TLE * c_MaC/c_min_per_day;
            }
        }
    }
}
