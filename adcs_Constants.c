
#include "adcs_Constants.h"

#include "adcs_VarDeclarations.h"


	//IMU
	const unsigned int c_FFFF = 0xFFFF;
	const double c_Resol_B = 1.525902E-04; // Resolution for the Magnetometer
	const double c_Resol_DelThta = 5.8590438E-09; // Resolution for Delta Theta
	//Telemetry IMU
	const double c_TM_Resol_DelThta  = 5.8590438E-09;
	const double c_TM_Resol_w = 2.09547579E-7;
	const double c_TM_Resol_B = 10.0;
	const double c_TM_RW_Resol = 3.25962901E-06;

    const double c_MaC = 0.128;
    const double c_MiC = 0.032;

///Orbit Model
    const double c_twomu = 797200.87500;
    const double c_tsince_min = 0.002133333;
    const double c_OMEGAE = 0.000072921159;
    const double c_vkmpersec = 7.905366296149016;
    const double c_radiusearthkm = 6378.137;
    const double c_polarradiuskm = 6356.7523142;
	const double c_ecc_ellip = 0.00669437999;
    const double c_auer= 149597870.0 /6378.1363;
    const double c_au= 149597870.7;
    const double c_j2     =  0.001082629989050;
    const double c_Twopi = 6.283185307179586;
    const double c_x2o3 = 0.666666666666667;
    const double c_xke    = 0.074366853168714;
    const double c_dividebyzerovalue = 0.0000000001;
    const double c_mu     = 398600.4418;
    const double c_invmu = 0.000002508777951886354381858088547658;
    const double c_Day_To_Seconds = 86400.0;
    const double c_Pi = 3.141592653589793;
    const double c_xpdotp   =  229.18311805232930;
    const double c_ss = 1.012229276354522;
    const double c_qzms2t = 0.000000001880276800610897;
    const double c_j3oj2  = -0.002338890558742;
    const double c_j4     = -0.00000161098761;
    const double c_temp4 = 0.0000000000015;
    const double c_min_per_day = 1440.0;
    const double c_D2R = 3.141592653589793/180.0;
    const double c_R2D = 180.0/3.141592653589793;
    const double c_AS2R = 0.0174532925199433/3600.0;
    const double c_Mmax = 1.5;
    const double c_halfhour = 14063.0;
    const int c_oneminute = 469;
    const int c_onesecond = 8;

    const int c_I_four_cross_four[4][4] = {{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1},
        };

const double c_rk[9][9] = { {-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                          {0.0,	    -1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                          {0.0,	    0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                          {0.0,	    0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                          {0.0,	    0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0},
                          {0.0,	    0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0},
                          {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0},
                          {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0},
                          {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0},
                        };

const double c_rk_T[9][9] = { {-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                              {0.0,	    -1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                              {0.0,	    0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                              {0.0,	    0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0},
                              {0.0,	    0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0},
                              {0.0,	    0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0},
                              {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0},
                              {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0},
                              {0.0,	    0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0},
                            };

	//coefficients for iau 1980 nutation
	/*const double fc[5][5]={
        { 134.96340251, 1717915923.2178,  31.8792,  0.051635, -0.00024470},
        { 357.52910918,  129596581.0481,  -0.5532,  0.000136, -0.00001149},
        {  93.27209062, 1739527262.8478, -12.7512, -0.001037,  0.00000417},
        { 297.85019547, 1602961601.2090,  -6.3706,  0.006593, -0.00003169},
        { 125.04455501,   -6962890.2665,   7.4722,  0.007702,  -0.00005939}
    };

    const double nut[106][10]={  {0,	0,	0,	0,	1,	6798.4,	-171996,	-174.2,	92025,	8.9,},
        {0,	0,	2,	-2,	2,	182.6,	-13187,	-1.6,	5736,	-3.1,},
        {0,	0,	2,	0,	2,	13.7,	-2274,	-0.2,	977,	-0.5,},
        {0,	0,	0,	0,	2,	3399.2,	2062,	0.2,	-895,	0.5,},
        {0,	1,	0,	0,	0,	365.2,	1426,	-3.4,	54,	-0.1,},
        {1,	0,	0,	0,	0,	27.6,	712,	0.1,	-7,	0,},
        {0,	1,	2,	-2,	2,	121.7,	-517,	1.2,	224,	-0.6,},
        {0,	0,	2,	0,	1,	13.6,	-386,	-0.4,	200,	0,},
        {1,	0,	2,	0,	2,	9.1,	-301,	0,	129,	-0.1,},
        {0,	-1,	2,	-2,	2,	365.3,	217,	-0.5,	-95,	0.3,},
        {1,	0,	0,	-2,	0,	31.8,	-158,	0,	-1,	0,},
        {0,	0,	2,	-2,	1,	177.8,	129,	0.1,	-70,	0,},
        {-1,	0,	2,	0,	2,	27.1,	123,	0,	-53,	0,},
        {1,	0,	0,	0,	1,	27.7,	63,	0.1,	-33,	0,},
        {0,	0,	0,	2,	0,	14.8,	63,	0,	-2,	0,},
        {-1,	0,	2,	2,	2,	9.6,	-59,	0,	26,	0,},
        {-1,	0,	0,	0,	1,	27.4,	-58,	-0.1,	32,	0,},
        {1,	0,	2,	0,	1,	9.1,	-51,	0,	27,	0,},
        {2,	0,	0,	-2,	0,	205.9,	48,	0,	1,	0,},
        {-2,	0,	2,	0,	1,	1305.5,	46,	0,	-24,	0,},
        {0,	0,	2,	2,	2,	7.1,	-38,	0,	16,	0,},
        {2,	0,	2,	0,	2,	6.9,	-31,	0,	13,	0,},
        {2,	0,	0,	0,	0,	13.8,	29,	0,	-1,	0,},
        {1,	0,	2,	-2,	2,	23.9,	29,	0,	-12,	0,},
        {0,	0,	2,	0,	0,	13.6,	26,	0,	-1,	0,},
        {0,	0,	2,	-2,	0,	173.3,	-22,	0,	0,	0,},
        {-1,	0,	2,	0,	1,	27,	21,	0,	-10,	0,},
        {0,	2,	0,	0,	0,	182.6,	17,	-0.1,	0,	0,},
        {0,	2,	2,	-2,	2,	91.3,	-16,	0.1,	7,	0,},
        {-1,	0,	0,	2,	1,	32,	16,	0,	-8,	0,},
        {0,	1,	0,	0,	1,	386,	-15,	0,	9,	0,},
        {1,	0,	0,	-2,	1,	31.7,	-13,	0,	7,	0,},
        {0,	-1,	0,	0,	1,	346.6,	-12,	0,	6,	0,},
        {2,	0,	-2,	0,	0,	1095.2,	11,	0,	0,	0,},
        {-1,	0,	2,	2,	1,	9.5,	-10,	0,	5,	0,},
        {1,	0,	2,	2,	2,	5.6,	-8,	0,	3,	0,},
        {0,	-1,	2,	0,	2,	14.2,	-7,	0,	3,	0,},
        {0,	0,	2,	2,	1,	7.1,	-7,	0,	3,	0,},
        {1,	1,	0,	-2,	0,	34.8,	-7,	0,	0,	0,},
        {0,	1,	2,	0,	2,	13.2,	7,	0,	-3,	0,},
        {-2,	0,	0,	2,	1,	199.8,	-6,	0,	3,	0,},
        {0,	0,	0,	2,	1,	14.8,	-6,	0,	3,	0,},
        {2,	0,	2,	-2,	2,	12.8,	6,	0,	-3,	0,},
        {1,	0,	0,	2,	0,	9.6,	6,	0,	0,	0,},
        {1,	0,	2,	-2,	1,	23.9,	6,	0,	-3,	0,},
        {0,	0,	0,	-2,	1,	14.7,	-5,	0,	3,	0,},
        {0,	-1,	2,	-2,	1,	346.6,	-5,	0,	3,	0,},
        {2,	0,	2,	0,	1,	6.9,	-5,	0,	3,	0,},
        {1,	-1,	0,	0,	0,	29.8,	5,	0,	0,	0,},
        {1,	0,	0,	-1,	0,	411.8,	-4,	0,	0,	0,},
        {0,	0,	0,	1,	0,	29.5,	-4,	0,	0,	0,},
        {0,	1,	0,	-2,	0,	15.4,	-4,	0,	0,	0,},
        {1,	0,	-2,	0,	0,	26.9,	4,	0,	0,	0,},
        {2,	0,	0,	-2,	1,	212.3,	4,	0,	-2,	0,},
        {0,	1,	2,	-2,	1,	119.6,	4,	0,	-2,	0,},
        {1,	1,	0,	0,	0,	25.6,	-3,	0,	0,	0,},
        {1,	-1,	0,	-1,	0,	3232.9,	-3,	0,	0,	0,},
        {-1,	-1,	2,	2,	2,	9.8,	-3,	0,	1,	0,},
        {0,	-1,	2,	2,	2,	7.2,	-3,	0,	1,	0,},
        {1,	-1,	2,	0,	2,	9.4,	-3,	0,	1,	0,},
        {3,	0,	2,	0,	2,	5.5,	-3,	0,	1,	0,},
        {-2,	0,	2,	0,	2,	1615.7,	-3,	0,	1,	0,},
        {1,	0,	2,	0,	0,	9.1,	3,	0,	0,	0,},
        {-1,	0,	2,	4,	2,	5.8,	-2,	0,	1,	0,},
        {1,	0,	0,	0,	2,	27.8,	-2,	0,	1,	0,},
        {-1,	0,	2,	-2,	1,	32.6,	-2,	0,	1,	0,},
        {0,	-2,	2,	-2,	1,	6786.3,	-2,	0,	1,	0,},
        {-2,	0,	0,	0,	1,	13.7,	-2,	0,	1,	0,},
        {2,	0,	0,	0,	1,	13.8,	2,	0,	-1,	0,},
        {3,	0,	0,	0,	0,	9.2,	2,	0,	0,	0,},
        {1,	1,	2,	0,	2,	8.9,	2,	0,	-1,	0,},
        {0,	0,	2,	1,	2,	9.3,	2,	0,	-1,	0,},
        {1,	0,	0,	2,	1,	9.6,	-1,	0,	0,	0,},
        {1,	0,	2,	2,	1,	5.6,	-1,	0,	1,	0,},
        {1,	1,	0,	-2,	1,	34.7,	-1,	0,	0,	0,},
        {0,	1,	0,	2,	0,	14.2,	-1,	0,	0,	0,},
        {0,	1,	2,	-2,	0,	117.5,	-1,	0,	0,	0,},
        {0,	1,	-2,	2,	0,	329.8,	-1,	0,	0,	0,},
        {1,	0,	-2,	2,	0,	32.8,	-1,	0,	0,	0,},
        {1,	0,	-2,	-2,	0,	9.5,	-1,	0,	0,	0,},
        {1,	0,	2,	-2,	0,	32.8,	-1,	0,	0,	0,},
        {1,	0,	0,	-4,	0,	10.1,	-1,	0,	0,	0,},
        {2,	0,	0,	-4,	0,	15.9,	-1,	0,	0,	0,},
        {0,	0,	2,	4,	2,	4.8,	-1,	0,	0,	0,},
        {0,	0,	2,	-1,	2,	25.4,	-1,	0,	0,	0,},
        {-2,	0,	2,	4,	2,	7.3,	-1,	0,	1,	0,},
        {2,	0,	2,	2,	2,	4.7,	-1,	0,	0,	0,},
        {0,	-1,	2,	0,	1,	14.2,	-1,	0,	0,	0,},
        {0,	0,	-2,	0,	1,	13.6,	-1,	0,	0,	0,},
        {0,	0,	4,	-2,	2,	12.7,	1,	0,	0,	0,},
        {0,	1,	0,	0,	2,	409.2,	1,	0,	0,	0,},
        {1,	1,	2,	-2,	2,	22.5,	1,	0,	-1,	0,},
        {3,	0,	2,	-2,	2,	8.7,	1,	0,	0,	0,},
        {2,	0,	2,	2,	2,	14.6,	1,	0,	-1,	0,},
        {1,	0,	0,	0,	2,	27.3,	1,	0,	-1,	0,},
        {0,	0,	-2,	2,	1,	169,	1,	0,	0,	0,},
        {0,	1,	2,	0,	1,	13.1,	1,	0,	0,	0,},
        {1,	0,	4,	0,	2,	9.1,	1,	0,	0,	0,},
        {2,	1,	0,	-2,	0,	131.7,	1,	0,	0,	0,},
        {2,	0,	0,	2,	0,	7.1,	1,	0,	0,	0,},
        {2,	0,	2,	-2,	1,	12.8,	1,	0,	-1,	0,},
        {2,	0,	-2,	0,	1,	943.2,	1,	0,	0,	0,},
        {1,	-1,	0,	-2,	0,	29.3,	1,	0,	0,	0,},
        {1,	0,	0,	1,	1,	388.3,	1,	0,	0,	0,},
        {1,	-1,	0,	2,	1,	35,	1,	0,	0,	0,},
        {0,	1,	0,	1,	0,	27.3,	1,	0,	0,	0,}
        };*/

    ///IGRF
    const int c_Nmax = 13;
    const int c_Kmax = 105;
    const double c_Pibytwo = 1.570796326794897;
    const double c_a = 6371.2;
    const double c_sdmax=1.0;

     const double c_gval[104]  = {
    		 -29404.8,
    		 -1450.9,
    		 -2499.6,
    		 2982.0,
    		 1677.0,
    		 1363.2,
    		 -2381.2,
    		 1236.2,
    		 525.7,
    		 903.0,
    		 809.5,
    		 86.3,
    		 -309.4,
    		 48.0,
    		 -234.3,
    		 363.2,
    		 187.8,
    		 -140.7,
    		 -151.2,
    		 13.5,
    		 66.0,
    		 65.5,
    		 72.9,
    		 -121.5,
    		 -36.2,
    		 13.5,
    		 -64.7,
    		 80.6,
    		 -76.7,
    		 -8.2,
    		 56.5,
    		 15.8,
    		 6.4,
    		 -7.2,
    		 9.8,
    		 23.7,
    		 9.7,
    		 -17.6,
    		 -0.5,
    		 -21.1,
    		 15.3,
    		 13.7,
    		 -16.5,
    		 -0.3,
    		 5.0,
    		 8.4,
    		 2.9,
    		 -1.5,
    		 -1.1,
    		 -13.2,
    		 1.1,
    		 8.8,
    		 -9.3,
    		 -11.9,
    		 -1.9,
    		 -6.2,
    		 -0.1,
    		 1.7,
    		 -0.9,
    		 0.7,
    		 -0.9,
    		 1.9,
    		 1.4,
    		 -2.4,
    		 -3.8,
    		 3.0,
    		 -1.4,
    		 -2.5,
    		 2.3,
    		 -0.9,
    		 0.3,
    		 -0.7,
    		 -0.1,
    		 1.4,
    		 -0.6,
    		 0.2,
    		 3.1,
    		 -2.0,
    		 -0.1,
    		 0.5,
    		 1.3,
    		 -1.2,
    		 0.7,
    		 0.3,
    		 0.5,
    		 -0.3,
    		 -0.5,
    		 0.1,
    		 -1.1,
    		 -0.3,
    		 0.1,
    		 -0.9,
    		 0.5,
    		 0.7,
    		 -0.3,
    		 0.8,
    		 0.0,
    		 0.8,
    		 0.0,
    		 0.4,
    		 0.1,
    		 0.5,
    		 -0.5,
    		 -0.4

    		 };
const double c_gsval[104] = {
		5.7,
		7.4,
		-11.0,
		-7.0,
		-2.1,
		2.2,
		-5.9,
		3.1,
		-12,
		-1.2,
		-1.6,
		-5.9,
		5.2,
		-5.1,
		-0.3,
		0.5,
		-0.6,
		0.2,
		1.3,
		0.9,
		-0.5,
		-0.3,
		0.4,
		1.3,
		-1.4,
		0.0,
		0.9,
		-0.1,
		-0.2,
		0.0,
		0.7,
		0.1,
		-0.5,
		-0.8,
		0.8,
		0.0,
		0.1,
		-0.1,
		0.4,
		-0.1,
		0.4,
		0.3,
		-0.1,
		0.4,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0

		};
const double c_hval[104] = {
		0.0,
		4652.5,
		0.0,
		-2991.6,
		-734.6,
		0.0,
		-82.1,
		241.9,
		-543.4,
		0.0,
		281.9,
		-158.4,
		199.7,
		-349.7,
		0.0,
		47.7,
		208.3,
		-121.2,
		32.3,
		98.9,
		0.0,
		-19.1,
		25.1,
		52.8,
		-64.5,
		8.9,
		68.1,
		0.0,
		-51.5,
		-16.9,
		2.2,
		23.5,
		-2.2,
		-27.2,
		-1.8,
		0.0,
		8.4,
		-15.3,
		12.8,
		-11.7,
		14.9,
		3.6,
		-6.9,
		2.8,
		0.0,
		-23.4,
		11.0,
		9.8,
		-5.1,
		-6.3,
		7.8,
		0.4,
		-1.4,
		9.6,
		0.0,
		3.4,
		-0.2,
		3.6,
		4.8,
		-8.6,
		-0.1,
		-4.3,
		-3.4,
		-0.1,
		-8.8,
		0.0,
		0.0,
		2.5,
		-0.6,
		-0.4,
		0.6,
		-0.2,
		-1.7,
		-1.6,
		-3.0,
		-2.0,
		-2.6,
		0.0,
		-1.2,
		0.5,
		1.4,
		-1.8,
		0.1,
		0.8,
		-0.2,
		0.6,
		0.2,
		-0.9,
		0.0,
		0.5,
		0.0,
		-0.9,
		0.6,
		1.4,
		-0.4,
		-1.3,
		-0.1,
		0.3,
		-0.1,
		0.5,
		0.5,
		-0.4,
		-0.4,
		-0.6,


		};
const double c_hsval[104] = {
		0.0,
		-25.9,
		0.0,
		-30.2,
		-22.4,
		0.0,
		6.0,
		-1.1,
		0.5,
		0.0,
		-0.1,
		6.5,
		3.6,
		-5,
		0.0,
		0.0,
		2.5,
		-0.6,
		3.0,
		0.3,
		0.0,
		0.0,
		-1.6,
		-1.3,
		0.8,
		0.0,
		1.0,
		0.0,
		0.6,
		0.6,
		-0.8,
		-0.2,
		-1.1,
		0.1,
		0.3,
		0.0,
		-0.2,
		0.6,
		-0.2,
		0.5,
		-0.3,
		-0.4,
		0.5,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0

		};

///Sun Model
const double c_L_Msun1 = 280.4606184;
const double c_L_Msun2 = 36000.77005361;
const double c_Msun1 = 357.5277233;
const double c_Msun2 = 35999.05034;
const double c_L_Ecliptic1 = 1.914666471;
const double c_L_Ecliptic2 = 0.019994643;
const double c_Sun_Dis1 = 1.000140612;
const double c_Sun_Dis2 = 0.016708617;
const double c_Sun_Dis3 = 0.000139589;
const double c_Epsilon1 = 23.439291;
const double c_Epsilon2 = 0.0130042;
const double c_KDset1 = 3.1E-4;
const double c_KDset2 = 2E-3;
const double c_KDset3 = 2E-2;


///16 Imax constants of for all sensors (Main and Redundant cells) ( NOTE: constant = Overall_Imax / Individual_Imax)
const double c_Imax_RPD_Red = 1.096912051131744;
const double c_Imax_RND_Red = 1.132238986590674;
const double c_Imax_RPND_Red = 1.125363752764521;
const double c_Imax_RNND_Red = 1.089843309660692;
const double c_Imax_PP_Red = 1.0;
const double c_Imax_PN_Red = 1.076554757530204;
const double c_Imax_YP_Red = 1.015119697606048;
const double c_Imax_YN_Red = 1.144345150026632;

const double c_Imax_RPD_Main = 1.073296595956431;
const double c_Imax_RND_Main = 1.172387413685503;
const double c_Imax_RPND_Main = 1.143227326266195;
const double c_Imax_RNND_Main = 1.113413888039156;
const double c_Imax_PP_Main = 1.0;
const double c_Imax_PN_Main = 1.127119300147093;
const double c_Imax_YP_Main = 1.060224293620740;
const double c_Imax_YN_Main = 1.075774928880186;

///---------------------------------------------------------------------------------------------------------------------------
const double c_SSThrsld = 0.5; ///Sunsensor threshold value
const double c_Sunlit_Thrsld = 0.5; ///Threshold for sunlit detection
const double c_AngDev_SMtransit_thrsld = 0.52359877559; /// (30 degrees) Threshold for sun presence in negative pitch side
const double c_AngDev_SAMtransit_thrsld = 0.52359877559; /// (30 degrees) Threshold for sun presence in negative pitch side

///SunSensor's Misalignment Correction Matrices
const double c_misaln_cor125[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor126[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor325[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor326[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor345[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor346[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor145[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};
const double c_misaln_cor146[3][3] = {{1, -0.005235963831420, 0.005235963831420},{0.005235963831420, 1, -0.005235963831420},{-0.005235963831420, 0.005235963831420, 1}};

///Sun Sensor to Body frame conversion for all the 8 quadrants
const double c_ss2b1256[3][3] = {{0.707106781186547,-0.683012701892219,-0.183012701892219},{0.707106781186547,0.683012701892219,0.183012701892219},{0.0,-0.258819045102521,0.965925826289068}}; ///sun sensor to body frame
const double c_ss2b2356[3][3] = {{-0.683012701892219,-0.707106781186548,-0.183012701892219},{0.683012701892219,-0.707106781186548,0.183012701892219},{-0.258819045102521,0.0,0.965925826289068}};
const double c_ss2b3456[3][3] = {{-0.707106781186547,0.683012701892219,-0.183012701892219},{-0.707106781186547,-0.683012701892219,0.183012701892219},{0.0,0.258819045102521,0.965925826289068}};
const double c_ss2b4156[3][3] = {{0.683012701892219,0.707106781186548,-0.183012701892219},{-0.683012701892219,0.707106781186548,0.183012701892219},{0.258819045102521,0.0,0.965925826289068}};

    ///---------------------------------------------------------------------------------------------------------------------------
/// Quest
const double c_wks_mag = 0.0;
const double c_wkm_mag = 0.041666666666667;
const double c_wks_sunmag = 0.0625;
const double c_wkm_sunmag = 0.0625;

/// Linear Controller
const double c_DPMMAX = 1.5;
const double c_RPM2RADpS = 0.10471975511965;
const double c_RADps2RPM = 9.54929658551372;
const double TC_T_RW_MAX = 0.001;
const double TC_Hmin = 0.001;

const double c_MOI_wh = 2.8E-5;

const double c_wh2B_mat_4RW[3][4] = {{0.5774,-0.5774,-0.5774,0.5774},
{0.5774,0.5774,-0.5774,-0.5774},
{0.5774,0.5774,0.5774,0.5774}};

const double c_wh2B_mat_1230[3][4] = {{0.5774,-0.5774,-0.5774,0.0},
{0.5774,0.5774,-0.5774,0.0},
{0.5774,0.5774,0.5774,0.0}};

const double c_wh2B_mat_1204[3][4] = {{0.5774,-0.5774,0.0,0.5774},
{0.5774,0.5774,0.0,-0.5774},
{0.5774,0.5774,0.0,0.5774}};

const double c_wh2B_mat_1034[3][4] = {{0.5774,0.0,-0.5774,0.5774},
{0.5774,0.0,-0.5774,-0.5774},
{0.5774,0.0,0.5774,0.5774}};

const double c_wh2B_mat_0234[3][4] = {{0.0,-0.5774,-0.5774,0.5774},
{0.0,0.5774,-0.5774,-0.5774},
{0.0,0.5774,0.5774,0.5774}};

const double c_B2wh_mat_4RW[4][3] = {{0.4330,    0.4330,   0.4330},
   {-0.4330,    0.4330,    0.4330},
   {-0.4330,   -0.4330,    0.4330},
    {0.4330,   -0.4330,    0.4330}};


const double c_B2wh_mat_1230[4][3] = {{0.8660,   -0.0000,    0.8660},
   {-0.8660,    0.8660,   -0.0000},
    {0.0000,   -0.8660,    0.8660},
         {0.0000,         0.0000,         0.0000}};

const double c_B2wh_mat_1204[4][3] = {{0.8660,    0.8660,   -0.0000},
   {-0.8660,   -0.0000,    0.8660},
         {0.0000,         0.0000,         0.0000},
   {-0.0000,   -0.8660,    0.8660}};

const double c_B2wh_mat_1034[4][3] = {{0.0000,    0.8660,    0.8660},
   {-0.0000,    0.0000,    0.0000},
   {-0.8660,   -0.0000,    0.8660},
    {0.8660,   -0.8660,    0.0000}};


const double c_B2wh_mat_0234[4][3] = {{-0.0000,   -0.0000,   -0.0000},
    {0.0000,    0.8660,    0.8660},
   {-0.8660,   -0.8660,   -0.0000},
    {0.8660,    0.0000,    0.8660}};


// Mag Bias LUT

const int c_DPM_Pol_LookUpTable[27][3] = {
	{   0,	0,	0},
	{   0,	0,	1},
	{   0,	0,	-1},
	{   0,	1,	0},
	{   0,	-1,	0},
	{   1,	0,	0},
	{   -1,	0,	0},
	{   0,	1,	1},
	{   0,	1,	-1},
	{   0,	-1,	-1},
	{   0,	-1,	1},
	{   1,	0,	1},
	{   -1,	0,	-1},
	{   -1,	0,	1},
	{   1,	0,	-1},
	{   1,	1,	0},
	{   -1,	-1,	0},
	{   -1,	1,	0},
	{   1,	-1,	0},
	{   1,	1,	1},
	{   -1,	-1,	-1},
	{   -1,	1,	1},
	{   1,	-1, 1},
	{   -1,	1,	-1},
	{   1,	1,	-1},
	{   -1,	-1,	1},
	{   1,	-1,	-1}


	};

const double c_MagBias_act_LUT[27][3] = {
	{   0.0,	0.0,	0.0       },
	{   -4100.0,	600.0,	-1800.0 },
	{   4700.0,	-1100.0,	2100.0  },
	{   -700.0,	-10600.0,	9500.0  },
	{   600.0,	13900.0,	-10600.0},
	{   600.0,	5000.0,	-3300.0     },
	{   -700.0,	-5000.0,	3300.0  },
	{   -4800.0,	-10000.0,	7700.0},
	{   3500.0,	-15000.0,	13000.0 },
	{   4800.0,	10000.0,	-7700.0 },
	{   -3500.0,	15000.0,	-13000.0},
	{   -3500.0,	5600.0,	-5100.0  },
	{   3500.0,	-5600.0,	5100.0      },
	{   -4800.0,	-4400.0,	1500.0  },
	{   4800.0,	4400.0,	-1500.0     },
	{   -100.0,	-5600.0,	6200.0  },
	{   100.0,	5600.0,	-6200.0     },
	{   -1400.0,	-15600.0,	12800.0},
	{   1400.0,	15600.0,	-12800.0},
	{   -3900.0,	-5700.0,	4700.0},
	{  3900.0,	5700.0,	-4700.0     },
	{   -6700.0,	-20000.0,	12700.0},
	{   -3500.0,	20000.0,	-13500.0},
	{   3500.0,	-20000.0,	13500.0     },
	{   4600.0,	-6700.0,	8300.0  },
	{   -4600.0,	6700.0,	-8300.0 },
	{   6700.0,	20000.0,	-12700.0}};

///Kalman Filter
const double c_I_three_cross_three[3][3] = {{1,0,0},
		{0,1,0},
		{0,0,1},
};

const double c_I_nine_cross_nine[9][9] = {{1,0,0,0,0,0,0,0,0},
		{0,1,0,0,0,0,0,0,0},
		{0,0,1,0,0,0,0,0,0},
		{0,0,0,1,0,0,0,0,0},
		{0,0,0,0,1,0,0,0,0},
		{0,0,0,0,0,1,0,0,0},
		{0,0,0,0,0,0,1,0,0},
		{0,0,0,0,0,0,0,1,0},
		{0,0,0,0,0,0,0,0,1},

};

double c_I_MAT[3][3] = {{0.336469195, -7.395986e-4, 0.007594767},
		                         {-7.395986e-4, 0.382923449, -0.00499996207},
		                         {0.007594767, -0.00499996207, 0.343231127}};

double c_I_MAT_Inv[3][3] = {{2.973534558742573,   0.004885047730652,  -0.065725032819457},
							 {0.004885047730652,   2.611992873592453,   0.037941676821678},
							 {-0.065725032819457,   0.037941676821678,   2.915495695282064}};

double c_MOI_wh_mat_Inv[4][4] = {{35714.2857, 0.0, 0.0, 0.0},
							{0.0, 35714.2857, 0.0, 0.0},
							{0.0, 0.0, 35714.2857, 0.0},
							{0.0, 0.0, 0.0, 35714.2857}};

double c_MOI_wh_mat[4][4] = {{2.8E-5, 0.0, 0.0, 0.0},
							{0.0, 2.8E-5, 0.0, 0.0},
							{0.0, 0.0, 2.8E-5, 0.0},
							{0.0, 0.0, 0.0, 2.8E-5}};
