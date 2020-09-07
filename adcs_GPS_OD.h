#ifndef ADCS_GPS_OD_H_INCLUDED
#define ADCS_GPS_OD_H_INCLUDED

//New variables

extern unsigned int GPS_PPS_OBT;
extern unsigned int GPS_READ_OBT;

extern int GPSDataReady_NA_count;
extern int TC_GPS2TLE_Switch;
extern int Present_OBT;
extern unsigned long int OBT_at_TLE_epoch;
extern double Delta_TLE;
extern int TC_GPSvalidity_Threshold;

//GPS
/*extern unsigned char* GPS_TM_Buffer_Addr_USC;
extern unsigned long int GPS_Data_Read_Status;
extern unsigned long int GPS_RCVD_DATA[106];*/

extern unsigned char GPS_obc_checkum;
extern unsigned int f_GPS_Valid_Data;

//Julian Day
extern double jd_time, tut;
extern double pps_deltaT, Julian_day;

//ast_args
extern double tt[4], f[5];

//main
extern int i_god, j_god;
extern int ktr, TLE_Select;

//rOrbitalElements_generation_GPS
extern unsigned int GPS_Select, GPSDataReady, TC_GPS_pulse_duration;
extern int i_jday, lmonth[13], year_GPS, UTC_mon_GPS, tempdays, Numofdays, UTC_day_GPS, UTC_hr_GPS, UTC_min_GPS;
extern int pps_delta_utc;
extern double epochdays_GPS, UTC_sec_GPS, PosX_ECEF_GPS, PosY_ECEF_GPS, PosZ_ECEF_GPS, Pos_ECEF_GPS[3];
extern double VelX_ECEF_GPS, VelY_ECEF_GPS, VelZ_ECEF_GPS, Vel_ECEF_GPS[3], Vel_ECI_temp[3], Vel_ECI_GPS[3], Pos_ECI_GPS[3];
extern double rpef[3], vpef[3], wcrecef[3], e_vec[3], angmomentumvec[3];
extern int Epochyear_GPS;


extern double radialdistance, radialdistance_ecef, velmag, r_delta, velmagsq, rdtv, ecc_temp1, ecc_temp2;
extern double ecc_temp3, ecc_temp4, ecc_temp5, ecc_temp6, ecc_temp7, ecc_GPS, semi_den, semimajoraxis_GPS, Alti_GPS;
extern double angmomentumvecmag, delta_hmag, invangmomentumvecmag;
extern double inclination_GPS, sinlongacnode, coslongacnode, nodeo_GPS, e_vecdtr, tempta, trueanomoly_GPS, Ndte_vec, N_ecc;
extern double temparg, argpo_GPS, omecc, sine, cose, eccanomaly_GPS, mo_GPS, Orbit_Period, no_GPS, longitude_GPS, latitude_GPS;

//Orbital Elements generation TLE
extern double sec_TLE_tc, no_TLE_tc, ecc_TLE_tc, trueanomoly_TLE_tc, epochdays_TLE_tc;
extern int minute_TLE_tc, hr_TLE_tc, day_TLE_tc, mon_TLE_tc, year_TLE_tc, Epochyear_TLE_tc, TC_TLE_Elements_select;
extern int epochyr, dayofyr, inttemp;
extern double epochdays, temp_jday, ibexp_TLE_tc, bstar_TLE_tc, inclination_TLE_tc, nodeo_TLE_tc, argpo_TLE_tc;
extern double mo_TLE_tc, ibexp_tc, bstar_tc, inclo_tc, nodeo_tc, argpo_tc, mo_tc;

//Orbit Initialization and Propagation
extern double Tsince_GPS, epochdays_sel, inclination_sel, nodeo_sel, trueanomoly_sel, mo_sel, argpo_sel;
extern double ecc_sel, no_sel, ibexp_sel, bstar_sel, sec_sel;
extern int year_sel, mon_sel, days_sel, hr_sel, minute_sel, epochyr_sel;
extern int GPS_Elements_Available, TLE_Data_Available;
extern double rtom, ao_sfour, pow_psisq, psetasq, intermediate, am_den, temp_temp, temp1, wcreci[3];
extern double Tsince, Tsince_TLE;
extern double Tsince_TLE_tc, Day_Of_Year_DeltaT, Delta_T, Orbit_Period_Comp, wo, inclo, nodeo, argpo, mo, ecco, no, ibexp, bstar;
extern int CB_OrbitModel, OrbitModel_Start;
extern char chksum_tle;

extern double con41, cc1, cc4, cc5, d2, d3, d4, delmo, eta, argpdot, omgcof, sinmao, aycof, t2cof, t3cof;

extern double t4cof, t5cof, x7thm1, mdot, nodedot, xmcof, nodecf, cc3, var;

extern double uuu, temp_prop, var1;


extern double eccsq, omeosq, rteosq, cosio, cosio2;

extern double ak, d1,  del, adel, ao, sinio, po, con42, posq, rp;

extern double sfour, qzms24;

extern double pinvsq, tsi, etasq, eeta, psisq,  coef, cc2, coef1,qzms24_temp;

extern double cosio4, temp1_or_init, temp2, temp3, temp_or_init, temp_om;

extern double xhdot1, xlcof;

extern double cc1sq, ak_temp, ao_temp, delmo_temp;

extern double x1mth2, perigee;

extern double xmdf, argpdf, nodedf, mm, t2, tempa, tempe, delomg, delm, t3, t4, nm, em, inclm, ecose, am, argpm, nodem, xlm, sinim, cosim;
extern double ep, xl, sinip, cosip, xincp, argpp, nodep, mp, axnl, aynl, esine;
extern double betal, sinu, cosu, suu, sin2u, cos2u, mrt, xnode, xinc, mvt, rvdot, emsq, sinsu, cossu, snod, cnod, sini, cosi;
extern double xmx, xmy, ux, uy, uz, vx, vy, vz,  xsatx_eci, xsaty_eci, xsatz_eci, vsatx_eci, vsaty_eci, vsatz_eci;
extern double templ_op, am_temp, nm_temp, mm_temp, u_temp, el2, pl, rl, rdotl, rvdotl;
extern double sinkp, coskp, eol, etempe;
extern double del_temp;
extern double Pos_ECI[3], Vel_ECI[3], Pos_ECEF[3], Vel_ECEF_temp[3], Vel_ECEF[3];
extern double Pos_ECIn[3], Vel_ECIn[3], Pos_ECEFn[3], Vel_ECEFn[3];

//Orbital elements computation
extern double semimajoraxis, ecc, Alti, inclination_temp, inclination, RAAN, trueanomoly, argofperigee, eccanomaly, ecc_r;
extern double longitude, latitude_temp, latitude ,longitude_tan_num, longitude_tan_den;
extern double xa_gcgd, mua_gcgd,ra_gcgd,l_gcgd,dlambda_gcgd,h_gcgd,den_gcgd,rhoa_gcgd,dmu_gcgd,gd_gcgd;

//Ecef to ECI to ecef
extern double UT1, UTC_EE, TC_delUT1, TAI, JDTDT, M_quad, sine1, sine2, TDB, TDT, TTDB, JDTDB, TTDB2, TTDB3, zeta, z_prsn, theta_prsn;
extern double pre_temp[3][3], precession[3][3], eps, dpsi, nut_temp[3][3], xin_temp, nutation[3][3], tut1, gmst0, gmst_, gast;
extern double sidereal[3][3], TC_xp, TC_yp, polarmotion[3][3], nut_sid_temp[3][3], ECEFtoECI[3][3], ECItoECEF[3][3],ang;
extern double TC_delAT, TTDT, deps;

//NED to ECEF
extern double NEDtoECEF[3][3];


//Function declarations

extern void rnut_iau1980(const double TTDBin, const double *fin);
extern void rast_args(const double TTDBin);
extern void rSidereal(void);
extern void rOrbitalElements_generation_GPS(void);
extern void rOrbit_Initialization(void);
extern void rJulian_Day(const int year, const int mon, const int days, const int hr, const int minute, const double sec);
extern void rECEFtoECItoECEF(void);
extern void rOrbit_Propagation(void);
extern void rOrbitalElements_computation(const double Pos_ECI_in[3], const double Vel_ECI_in[3], const double Pos_ECEF_in[3]);
extern void rNEDtoECEF(void);
/*static void rGPSDataProcessing(void);
static void rTLEDataProcessing(void);*/
extern void rGPSTLEProcessing(void);

#endif // ADCS_GPS_OD_H_INCLUDED




