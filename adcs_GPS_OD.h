#ifndef ADCS_GPS_OD_H_INCLUDED
#define ADCS_GPS_OD_H_INCLUDED

//GPS
extern unsigned char* GPS_TM_Buffer_Addr_USC;
extern unsigned long int GPS_Data_Read_Status;
extern unsigned long int GPS_Buffer_Data[300];
extern unsigned long int GPS_RCVD_DATA[60];
extern unsigned char GPS_obc_checkum;
extern unsigned int f_GPS_Valid_Data;

//Function Declarations
void rGPS_TM_Extract(void);

unsigned char GPS_obc_checkum;
unsigned int f_GPS_Valid_Data;

int GPSDataReady_NA_count;
int Present_OBT;
int OBT_at_TLE_uplink;
int Delta_TLE;

///Julian Day
double jd_time, tut;
double pps_deltaT, Julian_day;

///ast_args
double tt[4], f[5];

///main
int i_god, j_god;
int minorcyclecount, majorcyclecount;
int k, i, ktr, TLE_Select;

///rOrbitalElements_generation_GPS
int GPS_Select, GPSDataReady, TC_GPS_pulse_duration;
int i_jday, lmonth[13], year_GPS, UTC_mon_GPS, tempdays, Numofdays, UTC_day_GPS, UTC_hr_GPS, UTC_min_GPS;

double epochdays_GPS, UTC_sec_GPS, PosX_ECEF_GPS, PosY_ECEF_GPS, PosZ_ECEF_GPS, Pos_ECEF_GPS[3];
double VelX_ECEF_GPS, VelY_ECEF_GPS, VelZ_ECEF_GPS, Vel_ECEF_GPS[3], Vel_ECI_temp[3], Vel_ECI_GPS[3], Pos_ECI_GPS[3];
double rpef[3], vpef[3], wcrecef[3], e_vec[3], angmomentumvec[3];
int Epochyear_GPS;


double radialdistance, radialdistance_ecef, velmag, r_delta, velmagsq, rdtv, ecc_temp1, ecc_temp2;
double ecc_temp3, ecc_temp4, ecc_temp5, ecc_temp6, ecc_temp7, ecc_GPS, semi_den, semimajoraxis_GPS, Alti_GPS;
double angmomentumvecmag, delta_hmag, invangmomentumvecmag;
double inclination_GPS, sinlongacnode, coslongacnode, nodeo_GPS, e_vecdtr, tempta, trueanomoly_GPS, Ndte_vec, N_ecc;
double temparg, argpo_GPS, omecc, sine, cose, eccanomaly_GPS, mo_GPS, Orbit_Period, no_GPS, longitude_GPS, latitude_GPS;


int epochyr, dayofyr, inttemp;


///Orbit Initialization and Propagation
double Tsince_GPS, epochdays_sel, inclination_sel, nodeo_sel, trueanomoly_sel, mo_sel, argpo_sel;
double ecc_sel, no_sel, ibexp_sel, bstar_sel, sec_sel;
int year_sel, mon_sel, days_sel, hr_sel, minute_sel, epochyr_sel;
int GPS_Elements_Available, TLE_Data_Available;
double rtom, ao_sfour, pow_psisq, psetasq, intermediate, am_den, temp_temp, temp1, wcreci[3];
double Tsince, Tsince_TLE;
double Tsince_TLE_tc, Day_Of_Year_DeltaT, Delta_T, Orbit_Period_Comp, wo, inclo, nodeo, argpo, mo, ecco, no, ibexp, bstar;
int CB_OrbitModel, OrbitModel_Start;

double con41, cc1, cc4, cc5, d2, d3, d4, delmo, eta, argpdot, omgcof, sinmao, aycof, t2cof, t3cof;

double t4cof, t5cof, x7thm1, mdot, nodedot, xmcof, nodecf, cc3, var;

double uuu, temp_prop, var1;


double eccsq, omeosq, rteosq, cosio, cosio2;

double ak, d1,  del, adel, ao, sinio, po, con42, posq, rp;

double sfour, qzms24;

double pinvsq, tsi, etasq, eeta, psisq,  coef, cc2, coef1,qzms24_temp;

double cosio4, temp1_or_init, temp2, temp3, temp_or_init, temp_om;

double xhdot1, xlcof;

double cc1sq, ak_temp, ao_temp, delmo_temp;

double x1mth2, perigee;

double xmdf, argpdf, nodedf, mm, t2, tempa, tempe, delomg, delm, t3, t4, nm, em, inclm, ecose, am, argpm, nodem, xlm, sinim, cosim;
double ep, xl, sinip, cosip, xincp, argpp, nodep, mp, axnl, aynl, esine, r1;
double betal, sinu, cosu, suu, sin2u, cos2u, mrt, xnode, xinc, mvt, rvdot, emsq, sinsu, cossu, snod, cnod, sini, cosi;
double xmx, xmy, ux, uy, uz, vx, vy, vz,  xsatx_eci, xsaty_eci, xsatz_eci, vsatx_eci, vsaty_eci, vsatz_eci;
double templ, am_temp, nm_temp, mm_temp, u_temp, el2, pl, rl, rdotl, rvdotl;
double sinkp, coskp, eol, etempe;
double del_temp;
double Pos_ECI[3], Vel_ECI[3], Pos_ECEF[3], Vel_ECEF_temp[3], Vel_ECEF[3];
double Pos_ECIn[3], Vel_ECIn[3], Pos_ECEFn[3], Vel_ECEFn[3];

///Orbital elements computation
double semimajoraxis, ecc, Alti, inclination_temp, inclination, RAAN, trueanomoly, argofperigee, eccanomaly, ecc_r;
double longitude, latitude_temp, latitude, longitude_tan_num, longitude_tan_den;


///Ecef to ECI to ecef
double UT1, UTC, TAI, JDTDT, M_quad, sine1, sine2, TDB, TDT, TTDB, JDTDB, TTDB2, TTDB3, zeta, z, theta;
double pre_temp[3][3], precession[3][3], eps, dpsi, nut_temp[3][3], xin_temp, nutation[3][3], tut1, gmst0, gmst_, gast;
double sidereal[3][3], polarmotion[3][3], nut_sid_temp[3][3], ECEFtoECI[3][3], ECItoECEF[3][3],ang;
double TTDT, deps;

///NED to ECEF
double NEDtoECEF[3][3];

///Function declarations
void rnut_iau1980(double TTDBin, const double *fin);
void rast_args(double TTDBin);
void rSidereal(void);
void rxRot(double th);
void ryRot(double th);
void rzRot(double th);
void rOrbitalElements_TLE(void);
void rOrbitalElements_generation_GPS(void);
void rOrbit_Initialization(void);
void rJulian_Day(int year, int mon, int days, int hr, int minute, double sec);
void rECEFtoECItoECEF(void);
void rMatInv(double mat2inv[3][3]);
void rOrbit_Propagation(void);
void rOrbitalElements_computation(double Pos_ECI_in[3], double Vel_ECI_in[3], double Pos_ECEF_in[3]);
void rGPS_data_validity(void);
void rNEDtoECEF(void);
void rGPSDataProcessing(void);

#endif // ADCS_GPS_OD_H_INCLUDED




