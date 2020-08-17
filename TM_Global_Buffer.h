#ifndef TM_GLOBAL_BUFFER
#define TM_GLOBAL_BUFFER

#define TM_ADC_MAX_LIMIT 33
#define TM_ADC_THERMISTOR_MAX_LIMIT 14
#define RW_TM_MAX         4
#define PL_TM_BUF_MAX     10
#define TC_HISTORY_MAX 4000

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

struct TM_SP_GPS_STRUCTURE{
	unsigned char TM_UTC_Day;
	unsigned char TM_UTC_hour;
	unsigned char TM_UTC_Min;
	unsigned short TM_UTC_Sec;
	unsigned char TM_UTC_month;
	unsigned short TM_UTC_Year;
	unsigned char TM_No_Of_Sat;
	unsigned char TM_Pos_Validity;
	unsigned int TM_GPS_Xpos;
	unsigned int TM_GPS_Ypos;
	unsigned int TM_GPS_Zpos;
	unsigned int TM_GPS_XVel;
	unsigned int TM_GPS_YVel;
	unsigned int TM_GPS_ZVel;
	unsigned char TM_BIST_Info;
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
	unsigned char filler_1:2;
};

struct ST_NRM_MTR_STRUCTURE
{
	unsigned char filler:2;
	unsigned char Roll_Polarity:2;
	unsigned char Pitch_Polarity:2;
	unsigned char Yaw_Polarity:2;
};

#pragma pack(1)
struct TM_BUFFER_STUCTURE{
	unsigned int FrameSynch;
	unsigned char Sat_ID;
	unsigned char Real_Storage_TM: 1;
	unsigned char Norm_Spec_TC_hist_TM: 2;
	//unsigned char TC_history_TM:1;
	unsigned char filler:1;
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
	unsigned short       TM_inter_HAL_IMU_Status_Data_2;
	unsigned short       TM_inter_HAL_IMU_Status_Data;

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

	unsigned int TM_Remote_Data_SF0[256];


	unsigned int TM_Last_seen_TC[2];

	unsigned char TM_ADC_Data[TM_ADC_MAX_LIMIT]; // ADC Data
	unsigned short TM_ADC_Thermistor_Data[TM_ADC_THERMISTOR_MAX_LIMIT]; // ADC Data
	unsigned int Thermister_1,Thermister_2,Thermister_3,Thermister_4,Thermister_5,Thermister_6,Thermister_7,Thermister_8,Thermister_9,Thermister_10,Thermister_11,Thermister_12,Thermister_13,Thermister_14,Thermister_15,Thermister_16;
	unsigned short TM_ADC_Thermistor_15;
	unsigned short TM_SA_thermistor1;
	unsigned short TM_SA_thermistor2;
	unsigned short TM_Bus_current;
	unsigned short TM_Bus_voltage;
	unsigned short TM_Bus_voltage2;
	unsigned short TM_SA_current;
	unsigned short TM_AGC;
	//unsigned char TM_AGC;
	unsigned short SA1_Shunt_sw;
	unsigned short SA2_Shunt_sw;
	unsigned short SA3_Shunt_sw;

	unsigned int Last_seen_TC[2];
	unsigned long int TM_TC_cmd_executed_count;
	unsigned long long int TM_TC_exe;
	unsigned int TM_ATTC_count;
	unsigned long long int TM_Next_exe_TC;

	//remote_address
	unsigned int TM_Remote_Addr;

	unsigned short TM_Input_Latch[4];
	unsigned short TM_Output_Latch[5];

    // Payload
    unsigned short TM_PAYLOAD_ACK[4];
    unsigned short TM_pl_data[PL_TM_BUF_MAX];
    // TC
    unsigned char TM_NMI_fail_count;

    // payloadSS
    unsigned short Debug_Data[4];
    unsigned short TM_PL_Command;

    //payload acknowledgement count
    unsigned short TM_pl_ack_count;

    // GPS
    struct TM_GPS_STRUCTURE TM_GPS;        // GPS TM Structure
    unsigned short TM_GPS1_Status;
    unsigned short TM_GPS2_Status;
    unsigned short TM_GPS_OBT_Latch_enable;
    unsigned short TM_GPS_OBT_Read_1;
    unsigned short TM_GPS_OBT_Read_2;


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
    unsigned short TM_SunSens_Pitch_Error;

    unsigned char xyz[246];	 // testing (to be removed)
    unsigned short TC_rcvd_cntr2;
    unsigned short TC_cmd_executed_cntr2;
    unsigned short TC_pending_cntr2;
    unsigned short TC_ATT_cntr2;

    unsigned short gps_pulse_mic_counter;

    unsigned short TM_auto_manual;

    unsigned char TM_NSP_addr_table[RW_TM_MAX];

   //unsigned short Payload_TM[PL_TM_BUF_MAX];

   //battery temp

   short TM_battery_temp_1;
   short TM_battery_temp_2;
   unsigned char TM_FDI_NMI_Count;

   //ANTENNA_DATA
   unsigned short TM_Antenna_ACK;

//   EPS_card_MUX_switch_status
   unsigned short TM_MUX_1;
   unsigned short TM_MUX_2;
   unsigned short TM_MUX_3;
   unsigned short TM_MUX_4;
   unsigned short TM_MUX_5;
   unsigned short TM_MUX_6;
   unsigned short TM_MUX_7;
   unsigned short TM_MUX_8;

// heaters_auto_manual_status
   unsigned short TM_heaters_auto_manual;                            // heaters_auto_manual_status
   unsigned short TM_heaters_manual_control;                         // heaters manually controlled

}Buffer;

union ST_output_status
{
	unsigned int data;
	struct
	{
		unsigned filler         :6;
		unsigned IMU2_ON_OFF	:1;
		unsigned IMU2_RESET		:1;
		unsigned IMU1_ON_OFF	:1;
		unsigned IMU1_RESET		:1;
		unsigned GPS2_ON_OFF	:1;
		unsigned GPS2_RESET		:1;
		unsigned GPS1_ON_OFF	:1;
		unsigned GPS1_RESET		:1;
		unsigned MTR_ONOFF		:1;
		unsigned RF_TX_ON_OFF	:1;
		unsigned X_TX_ON_OFF	:1;
		unsigned heater6		:1;
		unsigned heater5        :1;
		unsigned heater4        :1;
		unsigned heater3        :1;
		unsigned heater2        :1;
		unsigned heater1        :1;
		unsigned RW4_ON_OFF     :1;
		unsigned RW3_ON_OFF     :1;
		unsigned RW2_ON_OFF     :1;
		unsigned RW1_ON_OFF     :1;
		unsigned Payload2_ON_OFF:1;
		unsigned Payload1_ON_OFF:1;
		unsigned SA3_ON_OFF     :1;
		unsigned SA2_ON_OFF     :1;
		unsigned SA1_ON_OFF     :1;
	};
}ST_output_latch;


#pragma pack(1)
 struct ST_NORMAL
 {
	unsigned int FrameSynch;
	unsigned char Sat_ID;
	unsigned char Real_Storage_TM: 1;
	unsigned char Norm_Spec_TC_hist_TM: 2;
	//unsigned char TC_history_TM:1;
	unsigned char filler:1;
	unsigned char Sub_Frame: 4;
	unsigned int OBT;                    // On Board Timer
	unsigned short frame_count;
	signed short  TM_B_BODY[3];
	unsigned short TM_IMU_1_Temp;
	unsigned short TM_IMU_2_Temp;
	unsigned short TM_IMU_1_Diag_STS;
	unsigned short TM_IMU_2_Diag_STS;
	unsigned int TM_Q_BODY[4];
	unsigned int TM_w_BODY[3];
	unsigned long int TM_RW_Speed[4];         //change it to short
	struct ST_NRM_MTR_STRUCTURE TM_MTR;
	unsigned int TM_Last_seen_TC[2];
	unsigned int TM_Q_EKF[4];
	unsigned int TM_w_EKF_Drift[3];
	unsigned int TM_Pos_ECI[3];
	unsigned int TM_Vel_ECI[3];
	unsigned short TM_Orb_Elapsd_Time;
	unsigned short TM_SunSens_Roll_Error;
	unsigned short TM_SunSens_Pitch_Error;
	unsigned short TM_SunSens_Yaw_Error;
	unsigned int TM_Q_Sunmagad[4];
	unsigned char TM_No_Of_Sat;
    unsigned int TM_S_BODY_Main[3];     //change to short
    unsigned int TM_S_BODY_Red[3];      //change to short
    short TM_battery_temp_1;
    short TM_battery_temp_2;
	unsigned char TM_ADC_Data[TM_ADC_MAX_LIMIT]; // ADC Data
	unsigned short TM_Thermistor_2;
	unsigned short TM_Thermistor_3;
	unsigned short TM_Thermistor_4;
	unsigned short TM_Thermistor_5;
	unsigned int TM_Q_Ref[4];
	int TM_Q_Error[3];
	unsigned short TM_Output_Latch[5];
	unsigned short TC_rcvd_cntr2;
	union ST_output_status ST_output_latch;
	unsigned short TM_Bus_voltage;
	unsigned short TM_Bus_current;
	unsigned short TM_SA_current;
	unsigned short TM_AGC;
	unsigned short ang_momtm[0];
	struct TM_SP_GPS_STRUCTURE TM_GPS;
	unsigned short TM_Tsince;
	char TM_wheel_index_ARCsum;
	unsigned char sunlit_eclipse_status;
	unsigned int TM_B_DOT[3];

 }ST_NM_Buffer;

#pragma pack(1)
 struct ST_SPECIAL_table
  {
 	unsigned int FrameSynch;
 	unsigned char Sat_ID;
 	unsigned char Real_Storage_TM: 1;
 	unsigned char Norm_Spec_TC_hist_TM: 2;
	//unsigned char TC_history_TM:1;
	unsigned char filler:1;
 	unsigned char Sub_Frame: 4;
 	unsigned int OBT;                    // On Board Timer
 	unsigned short frame_count;
 	unsigned int TM_w_BODY[3];
 	signed short  TM_B_BODY[3];
 	unsigned int TM_S_BODY_Main[3];
 	unsigned int TM_Q_Sunmagad[3];
 	unsigned int TM_Error_EKF[3];
 	unsigned int TM_Q_Ref[4];
 	unsigned int TM_Q_EKF[4];
 	unsigned int TM_B_EKF_Bias[3];
 	unsigned int TM_w_EKF_Drift[3];
 	unsigned int TM_RW_Speed[4];
	unsigned short TM_IMU_1_Temp;
	unsigned short TM_IMU_2_Temp;

 	unsigned int TM_Pos_ECI[3];
 	unsigned int TM_Vel_ECI[3];
    unsigned int TM_Latitude;
    unsigned int TM_Longitude;
    unsigned short TM_Orb_Elapsd_Time;        //not assigned
    unsigned int TM_Altitude;
    struct TM_SP_GPS_STRUCTURE TM_GPS;        // GPS TM Structure
    unsigned short TM_GPS1_Status;
    unsigned short TM_GPS2_Status;
    unsigned short TM_Output_Latch[1];
    unsigned short TM_Tsince;
    int TM_Q_Error[3];
    struct ST_NRM_MTR_STRUCTURE TM_MTR;
    int cntrl_torque[3];
    char TM_wheel_index_ARCsum;

    unsigned char ST_TM_GPS_RCVD_DATA[424];


  }ST_SP_Buffer;



union TCH_header
{
	 // unsigned char data[12];
	  unsigned int data[3];
	  struct
	  {
		unsigned int FrameSynch;
		unsigned char Sat_ID;
		unsigned char Real_Storage_TM: 1;
		unsigned char Norm_Spec_TC_hist_TM: 2;
		//unsigned char TC_history_TM:1;
		unsigned char filler:1;
		unsigned char Sub_Frame: 4;
		unsigned int OBT; // On Board Timer
		unsigned short frame_count;
	  };
}ST_TC_header_Buffer;




#endif // TM_GLOBAL_BUFFER
