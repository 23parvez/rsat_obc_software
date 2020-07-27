#include "HAL_MTR.h"
#include "HAL_Address.h"
#include "HAL_Global.h"
#include "Global.h"
#include "Telemetry.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "TM_Global_Buffer.h"
#include "adcs_VarDeclarations.h"


void rHAL_MTR_ON(void)
{
	unsigned short tempdata;
	Out_Latch_2.MTR_ON_OFF = 1;
	tempdata =  Out_Latch_2.data;
	IO_LATCH_REGISTER_2;
	IO_LATCH_REGISTER_2 = tempdata;
}
void rHAL_MTR_OFF(void)
{
	unsigned short tempdata;
	Out_Latch_2.MTR_ON_OFF = 0;
	tempdata =  Out_Latch_2.data;
	IO_LATCH_REGISTER_2;
	IO_LATCH_REGISTER_2 = tempdata;
}

//void rHAL_MTR(unsigned long int MTR_Axis,int MTR_Polarity)
//{
//	if(MTR_Axis == Pitch)
//	  {
//		  if(MTR_Polarity == Positive)
//		  {
//			     if((MTR_Current_Data & 0x0000000F) == 0x00000009)
//			     {
//			    	 MTR_Current_Data = MTR_Current_Data & 0x0000FFF0;
//			    	 MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Pitch
//			    	 //delay
//			    	 for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//			     }
//			     MTR_Current_Data = MTR_Current_Data  | 0x00000006;
//			     MTR_LATCH_REGISTER = MTR_Current_Data;//Positive Direction
//		  }
//		  else if(MTR_Polarity == Negative)
//		  {
//			     if((MTR_Current_Data & 0x0000000F) == 0x00000006)
//			  	 {
//			    	 MTR_Current_Data = MTR_Current_Data  & 0x0000FFF0;
//			    	 MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Pitch
//			    	 //delay
//			    	 for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//			  	 }
//			     MTR_Current_Data = MTR_Current_Data  | 0x00000009;
//			     MTR_LATCH_REGISTER = MTR_Current_Data;//Negative Direction
//		  }
//		  else if(MTR_Polarity == No_Current)
//		  {
//			     MTR_Current_Data = MTR_Current_Data  & 0x0000FFF0;
//			     MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Pitch
//		  }
//		  else
//		  {
//			  //Do Nothing & Exit
//		  }
//	  }
//	  else if(MTR_Axis == Yaw)
//	  {
//		  if(MTR_Polarity == Positive)
//		  {
//		  		if((MTR_Current_Data & 0x000000F0) == 0x00000090)
//		  		{
//		  		    MTR_Current_Data = MTR_Current_Data  & 0x0000FF0F;
//		 		    MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Yaw
//		 		     //delay
//		 		    for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//		  		}
//		 		MTR_Current_Data = MTR_Current_Data  | 0x00000060;
//		 		MTR_LATCH_REGISTER = MTR_Current_Data;//Positive Direction
//		  }
//		  else if(MTR_Polarity == Negative)
//		  {
//		 		if((MTR_Current_Data & 0x000000F0) == 0x00000060)
//		 		 {
//		 		     MTR_Current_Data = MTR_Current_Data  & 0x0000FF0F;
//		 		     MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Yaw
//		 		     //delay
//		 		     for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//		 		 }
//		 		 MTR_Current_Data = MTR_Current_Data  | 0x00000090;
//		 		 MTR_LATCH_REGISTER = MTR_Current_Data;//Negative Direction
//		  }
//		  else if(MTR_Polarity == No_Current)
//		  {
//		 		 MTR_Current_Data = MTR_Current_Data  & 0x0000FF0F;
//		 		 MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Yaw
//		  }
//		  else
//		  {
//		 		  //Do Nothing & Exit
//		  }
//	  }
//	  else if(MTR_Axis == Roll)
//	  {
//		  if(MTR_Polarity == Positive)
//		  {
//			    if((MTR_Current_Data & 0x00000F00) == 0x00000900)
//			    {
//			    	 MTR_Current_Data = MTR_Current_Data  & 0x0000F0FF;
//			    	 MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Roll
//			    	 //delay
//			    	 for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//			    }
//		 		MTR_Current_Data = MTR_Current_Data  | 0x00000600;
//		 		MTR_LATCH_REGISTER = MTR_Current_Data;//Positive Direction
//		 }
//		 else if(MTR_Polarity == Negative)
//		 {
//		 	    if((MTR_Current_Data & 0x00000F00) == 0x00000600)
//		 		{
//		 		     MTR_Current_Data = MTR_Current_Data  & 0x0000F0FF;
//		 		     MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Roll
//		 		     //delay
//		 		     for(wait_i=0;wait_i<3000;wait_i++);//2.24ms delay
//		 		}
//		 		MTR_Current_Data = MTR_Current_Data  | 0x00000900;
//		 		MTR_LATCH_REGISTER = MTR_Current_Data;//Negative Direction
//		 }
//		 else if(MTR_Polarity == No_Current)
//		 {
//		 		MTR_Current_Data = MTR_Current_Data  & 0x0000F0FF;
//		 		MTR_LATCH_REGISTER = MTR_Current_Data;//Turn OFF Roll
//		 }
//		 else
//		 {
//		 		  //Do Nothing & Exit
//		 }
//	  }
//
//	  else
//	  {
//		  //Do Nothing & Exit
//	  }
//
//}
#if 1

//int CB_Torquer_Polarity_Check=1;
void rHAL_MTR(void)
{
	short tempdata;

	MTR_Monitoring();

	if (CB_Torquer_Polarity_Check == 1)
	{
		tor_counter ++;

		/*********************************/
			if(tor_counter <= 8)
			{
							if(tor_counter <= Ton[0])
							{
								if(TC_boolean_u.TC_Boolean_Table.Roll_Torquer_Polarity_Reversal == 1)// 1 - high, 0 - low
								{
									Roll_Pol = -1 * DPM_Polarity[0];
								}
								else
								{
									Roll_Pol = DPM_Polarity[0];
								}
						//Roll_Pol=-1;
								switch(Roll_Pol)		// ROLL
								{
									case Cur_Positive : Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
														Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
														Out_Latch_1.FP_CTRL_MTR2_P2 = 1;
														Out_Latch_1.FP_CTRL_MTR2_N1 = 1;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;

														break;

									case Cur_Negative:  Out_Latch_1.FP_CTRL_MTR2_P1 = 1;
														Out_Latch_1.FP_CTRL_MTR2_N2 = 1;
														Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
														Out_Latch_1.FP_CTRL_MTR2_N1 = 0;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;

														break;

									case No_Current:    Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
														Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
														Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
														Out_Latch_1.FP_CTRL_MTR2_N1 = 0;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
														MTR_Reset_Flag = 1;

														break;

									default			 :  break;

								}
							}
							else
							{
								Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
								Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
								Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
								Out_Latch_1.FP_CTRL_MTR2_N1 = 0;
								MTR_LATCH_REGISTER = Out_Latch_1.data;
								TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
								ST_normal.ST_NM_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
								ST_special.ST_SP_Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
								MTR_Reset_Flag = 1;
							}

						/*********************************/
							if(tor_counter <= Ton[1])
							{
								if(TC_boolean_u.TC_Boolean_Table.Pitch_Torquer_Polarity_Reversal == 1)      // 1 - high, 0 - low
								{
									Pitch_Pol = -1 * DPM_Polarity[1];
								}
								else
								{
									Pitch_Pol = DPM_Polarity[1];
								}
									//Pitch_Pol=1;
								switch(Pitch_Pol)		// PITCH
								{
									case Cur_Positive : Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
														Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
														Out_Latch_1.FP_CTRL_MTR3_P2 = 1;
														Out_Latch_1.FP_CTRL_MTR3_N1 = 1;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														break;

									case Cur_Negative : Out_Latch_1.FP_CTRL_MTR3_P1 = 1;
														Out_Latch_1.FP_CTRL_MTR3_N2 = 1;
														Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
														Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														break;

									case No_Current   : Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
														Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
														Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
														Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
														MTR_LATCH_REGISTER = Out_Latch_1.data;
														MTR_Reset_Flag = 1;
														TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_normal.ST_NM_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														ST_special.ST_SP_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
														break;

									default			 :   break;

								}
							}
							else
							{
								Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
								Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
								Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
								Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
								MTR_LATCH_REGISTER = Out_Latch_1.data;
								TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
								ST_normal.ST_NM_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
								ST_special.ST_SP_Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
								MTR_Reset_Flag = 1;
							}

							/*********************************/
							if(tor_counter <= Ton[2])
							{
								if(TC_boolean_u.TC_Boolean_Table.Yaw_Torquer_Polarity_Reversal == 1)// 1 - high, 0 - low
								{
									Yaw_Pol = -1 * DPM_Polarity[2];
								}
								else
								{
									Yaw_Pol = DPM_Polarity[2];
								}
								//Yaw_Pol=-1;
								switch(Yaw_Pol)		//YAW
								{
								case Cur_Positive : Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
													Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
													Out_Latch_1.FP_CTRL_MTR1_P2 = 1;
													Out_Latch_1.FP_CTRL_MTR1_N1 = 1;
													MTR_LATCH_REGISTER = Out_Latch_1.data;
													TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_normal.ST_NM_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_special.ST_SP_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													break;

								case Cur_Negative:  Out_Latch_1.FP_CTRL_MTR1_P1 = 1;
													Out_Latch_1.FP_CTRL_MTR1_N2 = 1;
													Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
													Out_Latch_1.FP_CTRL_MTR1_N1 = 0;
													MTR_LATCH_REGISTER = Out_Latch_1.data;
													TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_normal.ST_NM_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_special.ST_SP_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													break;

								 case No_Current  : Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
													Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
													Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
													Out_Latch_1.FP_CTRL_MTR1_N1 = 0;
													MTR_LATCH_REGISTER = Out_Latch_1.data;
													TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_normal.ST_NM_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													ST_special.ST_SP_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
													MTR_Reset_Flag = 1;

														break;

									default			 :  break;

								}
							}
							else
							{
								Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
								Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
								Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
								Out_Latch_1.FP_CTRL_MTR1_N1 = 0;
								MTR_LATCH_REGISTER = Out_Latch_1.data;
								TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
								ST_normal.ST_NM_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
								ST_special.ST_SP_Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
								MTR_Reset_Flag = 1;
							}

			}
	  }
}


void MTR_Monitoring()
{
	input_latch_1.data = IO_IN_LATCH_REGISTER_1;
}



void rHAL_MTR_TC(unsigned long int MTR_Axis,int MTR_Polarity)
{
	unsigned short tempdata;
	if(MTR_Axis == Roll)
	{
		if(MTR_Polarity == Positive)
				{

						Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
						Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
						Out_Latch_1.FP_CTRL_MTR2_P2 = 1;
						Out_Latch_1.FP_CTRL_MTR2_N1 = 1;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;

				}
				else if(MTR_Polarity == Negative)
				{
						Out_Latch_1.FP_CTRL_MTR2_P1 = 1;
						Out_Latch_1.FP_CTRL_MTR2_N2 = 1;
						Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
						Out_Latch_1.FP_CTRL_MTR2_N1 = 0;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;


				}
				else if(MTR_Polarity == No_Current)
				{
					Out_Latch_1.FP_CTRL_MTR2_P1 = 0;
					Out_Latch_1.FP_CTRL_MTR2_N2 = 0;
					Out_Latch_1.FP_CTRL_MTR2_P2 = 0;
					Out_Latch_1.FP_CTRL_MTR2_N1 = 0;
					tempdata = Out_Latch_1.data;
					MTR_LATCH_REGISTER;
					MTR_LATCH_REGISTER = tempdata;
					TM.Buffer.TM_MTR.Roll_Polarity = input_latch_1.FP_MTR2_mon1_mon2;
				}
				else{}
	}
			else if(MTR_Axis == Pitch)
			{
				if(MTR_Polarity == Positive)
				{

						Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
						Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
						Out_Latch_1.FP_CTRL_MTR3_P2 = 1;
						Out_Latch_1.FP_CTRL_MTR3_N1 = 1;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;


				}
				else if(MTR_Polarity == Negative)
				{
						Out_Latch_1.FP_CTRL_MTR3_P1 = 1;
						Out_Latch_1.FP_CTRL_MTR3_N2 = 1;
						Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
						Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;

				 }
				else if(MTR_Polarity == No_Current)
				{
					Out_Latch_1.FP_CTRL_MTR3_P1 = 0;
					Out_Latch_1.FP_CTRL_MTR3_N2 = 0;
					Out_Latch_1.FP_CTRL_MTR3_P2 = 0;
					Out_Latch_1.FP_CTRL_MTR3_N1 = 0;
					tempdata = Out_Latch_1.data;
					MTR_LATCH_REGISTER;
					MTR_LATCH_REGISTER = tempdata;
					TM.Buffer.TM_MTR.Pitch_Polarity = input_latch_1.FP_MTR3_mon1_mon2;
				}
				else{}
			}
			else if(MTR_Axis == Yaw)
			{
				if(MTR_Polarity == Positive)
				{
						Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
						Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
						Out_Latch_1.FP_CTRL_MTR1_P2 = 1;
						Out_Latch_1.FP_CTRL_MTR1_N1 = 1;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
				}
				else if(MTR_Polarity == Negative)
				{
						Out_Latch_1.FP_CTRL_MTR1_P1 = 1;
						Out_Latch_1.FP_CTRL_MTR1_N2 = 1;
						Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
						Out_Latch_1.FP_CTRL_MTR1_N1 = 0;
						tempdata = Out_Latch_1.data;
						MTR_LATCH_REGISTER;
						MTR_LATCH_REGISTER = tempdata;
						TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;

				}
				else if(MTR_Polarity == No_Current)
				{
					Out_Latch_1.FP_CTRL_MTR1_P1 = 0;
					Out_Latch_1.FP_CTRL_MTR1_N2 = 0;
					Out_Latch_1.FP_CTRL_MTR1_P2 = 0;
					Out_Latch_1.FP_CTRL_MTR1_N1 = 0;
					tempdata = Out_Latch_1.data;
					MTR_LATCH_REGISTER;
					MTR_LATCH_REGISTER = tempdata;
					TM.Buffer.TM_MTR.Yaw_Polarity = input_latch_1.FP_MTR1_mon1_mon2;
				}
				else{}
			}
	else{}
}


#endif
