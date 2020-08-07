/*
 * remote_patch.h
 *
 *  Created on: 18-Jul-2020
 *      Author: Ajeeth
 */

#ifndef REMOTE_PATCH_H_
#define REMOTE_PATCH_H_

#include <math.h>
#include "Global.h"
#include "HAL_Global.h"
#include "HAL_IMU.h"
#include "HAL_Address.h"
#include "TM_Global_Buffer.h"
#include "Telemetry.h"
#include "adcs_VarDeclarations.h"
#include "Telecommand.h"
#include "TC_List.h"
#include "adcs_Constants.h"
#include "adcs_SensorDataProcs.h"

#define nop() 	asm(" nop")
#define nop_S1() 	asm(" nop")
#define nop_S2() 	asm(" nop")
#define nop_S3() 	asm(" nop")
#define nop_S4() 	asm(" nop")
#define nop_S5() 	asm(" nop")
#define nop_S6() 	asm(" nop")
#define nop_L1() 	asm(" nop")
#define nop_L2() 	asm(" nop")


/** Declaration of remote hook routines **/
void rSus_mode_remote_entry_hook ();

void rSus_mode_remote_exit_hook ();

void rDBDOT_mode_remote_entry_hook ();

void rDBDOT_mode_remote_entry_hook ();

void rDBDOT_mode_remote_exit_hook ();

void rDGYRO_mode_remote_entry_hook ();

void rDGYRO_mode_remote_exit_hook ();

void rSACQ_mode_remote_entry_hook ();

void rSACQ_mode_remote_exit_hook ();

void r3AXIS_mode_remote_entry_hook ();

void r3AXIS_mode_remote_exit_hook ();

void rSAFE_mode_remote_entry_hook ();

void rSAFE_mode_remote_exit_hook ();


/*************************************************/
/************* Patch area declaration ************/

void rRemote_patch_area_S1 ();

void rRemote_patch_area_S2 ();

void rRemote_patch_area_S3 ();

void rRemote_patch_area_S4 ();

void rRemote_patch_area_S5 ();

void rRemote_patch_area_S6 ();

void rRemote_patch_area_L1 ();

void rRemote_patch_area_L2 ();


#endif /* REMOTE_PATCH_H_ */
