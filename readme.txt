The OBC software folder named Flight_code_V4 was uploaded in to git repository on 17 JULY 2020

Status of software while uploading in to Git : 3 July 2020 for TCH storage TM.

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
++++++++++++++++++++++++++++ Changes log ++++++++++++++++++++++++++++
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++++++++++++++++
	+ Changes made on 18 JULY 2020 +
	++++++++++++++++++++++++++++++++

1. Remote_patch.c and Remote_patch.h added to the project location
2. entry hook and exit hook for all mode processing functions.
3. Done necessary declarations in Global.h
4. Added Remote_patch.c in Make file
5. TCH warnings (incompatable pointer to integer assignment) has been
   cleared  in Global.c, Func = rPORinit()
6. Implicit declaration of function ST_TM_gps_data() in GPS_OD.c is 
   been resolved
7. HAL_GPS_Read function incompatible pointer assignment from 
   struct GPS_No to unsigned long int GPS_Buffer_Addr is resolved.


Func execute command
1.GPS config command names are modified
2.Roll_Torquer_ON and Roll_Torquer_OFF names are changed to TC_MTR_ON 
  and TC_MTR_OFF respectively.

3. Reaction wheel TC_Ping_RW1, TC_Ping_RW2, TC_Ping_RW3, and TC_Ping_RW4
   changed to TC_init_RW1 TC_init_RW2 TC_init_RW3, and TC_init_RW4 respectively.
4. Definition of struct HAL_RW_Data_Structure which was in HAL_RW.h is now
   defined in HAL_Global.h
5. rRW_init_cmd is called in Telecommand.c instead of Function rRW_Ping_TC1,
   rRW_Ping_TC2, rRW_Ping_TC3, and rRW_Ping_TC4 which are now eliminated.

6. Func exe EEPROM_CHECK_COMMAND  is replaced by S_band_tx_on


+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++++++++++++++++
	+ Changes made on 23 JULY 2020 +
	++++++++++++++++++++++++++++++++
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
1. Storage telemetry command names are modified



+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++++++++++++++++
	+ Changes made on 24 JULY 2020 +
	++++++++++++++++++++++++++++++++
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

1. Removed Remote_block_select (unused) Data telecommand from data command list.
2. Added TC_Mag_Misalignment_IMU1, TC_Mag_Misalignment_IMU2, 
	TC_Mag_Scale_Factor_IMU1, TC_Mag_Scale_Factor_IMU2,
	TC_SS_misalnCM1256, TC_SS_misalnCM2356,
	TC_SS_misalnCM3456, TC_SS_misalnCM4156,
	TC_SS_Imax_ALPHA offsets in rADCS_Data_TC function.
3. Removed TC_Tle from the adcs data command list as it was decided to use
   remote data command for TLE.
4. Offset addresses in adcs data command structure and case statement in 
   rADCS_Data_TC function are revised.
5. Payload data commands data type is changed from float to integer.


 