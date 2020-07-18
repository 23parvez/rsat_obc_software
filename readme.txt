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

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++