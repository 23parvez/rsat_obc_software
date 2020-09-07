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

/** Declaration of remote hook routines **/
void rSus_mode_remote_entry_hook (void);

void rSus_mode_remote_exit_hook (void);

void rDBDOT_mode_remote_entry_hook (void);

void rDBDOT_mode_remote_entry_hook (void);

void rDBDOT_mode_remote_exit_hook (void);

void rDGYRO_mode_remote_entry_hook (void);

void rDGYRO_mode_remote_exit_hook (void);

void rSACQ_mode_remote_entry_hook (void);

void rSACQ_mode_remote_exit_hook (void);

void r3AXIS_mode_remote_entry_hook (void);

void r3AXIS_mode_remote_exit_hook (void);

void rSAFE_mode_remote_entry_hook (void);

void rSAFE_mode_remote_exit_hook (void);


/*************************************************/
/************* Patch area declaration ************/

void rRemote_patch_area_S1 (void);

void rRemote_patch_area_S2 (void);

void rRemote_patch_area_S3 (void);

void rRemote_patch_area_S4 (void);

void rRemote_patch_area_S5 (void);

void rRemote_patch_area_S6 (void);

void rRemote_patch_area_L1 (void);

void rRemote_patch_area_L2 (void);


#endif /* REMOTE_PATCH_H_ */
