#ifndef TM_GLOBAL_BUFFER
#define TM_GLOBAL_BUFFER

#define TM_ADC_MAX_LIMIT 39
#define RW_TM_MAX         4
#define PL_TM_BUF_MAX     8

//----------------------IMU Data(TM)---------------------
#pragma pack(1)
struct TM_IMU_DATA{
	signed int TM_Theta_IMU[3];//Theta Before Bias Correction
	signed short TM_B_IMU[3];//Magnetic Field from IMU
	unsigned short TM_IMU_Diag_REG;//Diagnostics REG Data of IMU
	unsigned short TM_IMU_Temp; //Temperature of IMU
	unsigned short TM_IMU_Diag_STS;//Diagnostics STS Data of IMU
}TM_IMU1,TM_IMU2;

//----------------------SA STATUS---------------------
#pragma pack(1)
struct SWHW_STATUS_ST{
  unsigned char SA_status_1;
  unsigned char SA_status_2;
  unsigned char dummy[2];
}SWHW_STATUS;
//----------------------GPS TM Data----------------------------
#pragma pack(1)
struct TM_GPS_STRUCTURE{
unsigned char TM_No_Of_Sat;
unsigned char TM_UTC_Day;
unsigned char TM_UTC_Month;
unsigned short TM_UTC_Year;
unsigned char TM_UTC_hour;
unsigned char TM_UTC_Min;
unsigned short TM_UTC_Sec;
unsigned short TM_GPSWeek;
unsigned int TM_GPS_Xpos;
unsigned int TM_GPS_Ypos;
unsigned int TM_GPS_Zpos;
unsigned int TM_GPS_XVel;
unsigned int TM_GPS_YVel;
unsigned int TM_GPS_ZVel;
unsigned char TM_No_Of_Sat_InFix;
unsigned char TM_Pos_Validity;
unsigned char TM_BIST_Info;
unsigned char TM_CheckSum;
};

#pragma pack(1)
struct TM_MTR_STRUCTURE
{
	// MTR Statuses
	unsigned char Roll_MTR_Status:2;
	unsigned char Pitch_MTR_Status:2;
	unsigned char Yaw_MTR_Status:2;
	unsigned char filler:2;
	// Polarities of Torquers
	unsigned char Roll_Polarity:2;
	unsigned char Pitch_Polarity:2;
	unsigned char Yaw_Polarity:2;
};

#pragma pack(1)
struct TM_BUFFER_STUCTURE{
	unsigned int FrameSynch;
	unsigned char Sat_ID;
	unsigned char Main_Frame: 4;
	unsigned char Sub_Frame: 4;
	unsigned int OBT; // On Board Timer

	// IMU
	struct TM_IMU_DATA   TM_IMU1;
	struct TM_IMU_DATA   TM_IMU2;
	struct TM_IMU_DATA   TM_IMU_SELECTED;
	struct TM_IMU_DATA   TM_IMU_UNSELECTED;
	signed short       	 TM_B_BODY[3];
	signed int           TM_w_BODY[3];
	signed int           TM_Thta_BODY[3];
	signed int           TM_w_IMU[3];

	unsigned char TM_TC_Buffer[63];


	// Remote Address and Data
	unsigned int TM_Remote_Addr_SF0;
	unsigned int TM_Remote_Addr_SF1;
	unsigned int TM_Remote_Addr_SF2;
	unsigned int TM_Remote_Addr_SF3;
	unsigned int TM_Remote_Addr_SF4;
	unsigned int TM_Remote_Addr_SF5;
	unsigned int TM_Remote_Addr_SF6;
	unsigned int TM_Remote_Addr_SF7;
	unsigned int TM_Remote_Addr_SF8;
	unsigned int TM_Remote_Addr_SF9;
	unsigned int TM_Remote_Addr_SF10;
	unsigned int TM_Remote_Addr_SF11;
	unsigned int TM_Remote_Addr_SF12;
	unsigned int TM_Remote_Addr_SF13;
	unsigned int TM_Remote_Addr_SF14;
	unsigned int TM_Remote_Addr_SF15;

	unsigned int TM_Remote_Data_SF0[16];
	unsigned int TM_Remote_Data_SF1[16];
	unsigned int TM_Remote_Data_SF2[16];
	unsigned int TM_Remote_Data_SF3[16];
	unsigned int TM_Remote_Data_SF4[16];
	unsigned int TM_Remote_Data_SF5[16];
	unsigned int TM_Remote_Data_SF6[16];
	unsigned int TM_Remote_Data_SF7[16];
	unsigned int TM_Remote_Data_SF8[16];
	unsigned int TM_Remote_Data_SF9[16];
	unsigned int TM_Remote_Data_SF10[16];
	unsigned int TM_Remote_Data_SF11[16];
	unsigned int TM_Remote_Data_SF12[16];
	unsigned int TM_Remote_Data_SF13[16];
	unsigned int TM_Remote_Data_SF14[16];
	unsigned int TM_Remote_Data_SF15[16];

	unsigned int TM_Last_seen_TC[2];

	unsigned char TM_ADC_Data[TM_ADC_MAX_LIMIT]; // ADC Data

	unsigned int Last_seen_TC[2];

	unsigned short TM_Input_Latch[4];
	unsigned short TM_Output_Latch[5];

    // Payload
    unsigned short TM_PAYLOAD_ACK[4];
    // TC

    // payloadSS
    unsigned short Debug_Data[4];
    unsigned short TM_PL_Command;

    // GPS
    struct TM_GPS_STRUCTURE TM_GPS; //GPS TM Structure
    unsigned short TM_GPS1_Status;
    unsigned short TM_GPS2_Status;


    // MTR
    struct TM_MTR_STRUCTURE TM_MTR; //MTR TM Structure

    unsigned int TM_Q_BODY[4];
    unsigned int TM_RW_Speed[4];
    unsigned int TM_RW_DeltaSpeed[4];

    // EKF
    unsigned int TM_Q_EKF[4];
    unsigned int TM_B_EKF_Bias[3];
    unsigned int TM_Error_EKF[3];
    unsigned int TM_w_EKF_Drift[3];

    // Orbital Elements
    unsigned int TM_Pos_ECI[3];
    unsigned int TM_Vel_ECI[3];
    unsigned int TM_Pos_ECEF[3];
	unsigned int TM_Vel_ECEF[3];
    unsigned int TM_Latitude;
    unsigned int TM_Longitude;
    unsigned short TM_Orb_Period;
    unsigned short TM_Tsince;
    unsigned short TM_Orb_Elapsd_Time;
    unsigned int TM_Altitude;


    // Detumbling
    int TM_B_DOT[3];
    unsigned int TM_DPM[3];

    // Quest (AD)
    unsigned int TM_Q_Sunmagad[4];
    unsigned int TM_Q_Magad[4];

    // SS Data Processing
    unsigned int TM_S_BODY_Main[3];
    unsigned int TM_S_BODY_Red[3];

    // References
    unsigned int TM_w_Ref[3];
    unsigned int TM_Q_Ref[4];

    int TM_Q_Error[3];

    unsigned short TM_S_ECI[3];
    unsigned short TM_S_Ref[3];

    unsigned short TM_B_NED[3];
    unsigned short TM_B_ECI[3];
    unsigned short TM_B_Reg[3];

    unsigned short TM_Station_Angle;

    unsigned short TM_RW_DFTSpeed[4];
    unsigned short TM_RW_DitherSpeed[4];

    unsigned short TM_SunSens_Roll_Error;
    unsigned short TM_SunSens_Yaw_Error;

    unsigned char xyz[246];	 // testing (to be removed)
    unsigned short TC_cntr1;
    unsigned short TC_cntr2;
    unsigned short gps_pulse_mic_counter;

    unsigned short TM_manual;

    unsigned char TM_NSP_addr_table[RW_TM_MAX];

   unsigned short Payload_TM[PL_TM_BUF_MAX];
}Buffer;

#endif // TM_GLOBAL_BUFFER
