/*
 * remote_patch.c
 *
 *  Created on: 18-Jul-2020
 *      Author: Ajeeth
 */
#include "remote_patch.h"
void rSus_mode_remote_entry_hook ()
{
	nop();
}

void rSus_mode_remote_exit_hook ()
{
	nop();
}

void rDBDOT_mode_remote_entry_hook ()
{
	nop();
}

void rDBDOT_mode_remote_exit_hook ()
{
	nop();
}

void rDGYRO_mode_remote_entry_hook ()
{
	nop();
}

void rDGYRO_mode_remote_exit_hook ()
{
	nop();
}

void rSACQ_mode_remote_entry_hook ()
{
	nop();
}

void rSACQ_mode_remote_exit_hook ()
{
	nop();
}

void r3AXIS_mode_remote_entry_hook ()
{
	nop();
}

void r3AXIS_mode_remote_exit_hook ()
{

	nop();
}

void rSAFE_mode_remote_entry_hook ()
{
	nop();
}

void rSAFE_mode_remote_exit_hook()
{
	nop();
}

/******************************************************************/
/*********** Routines to create patch area **********/
/****************************************************************
 *@function name  rRemote_patch_area_1, rRemote_patch_area_2
 *@				  rRemote_patch_area_3, rRemote_patch_area_4
 *@				  rRemote_patch_area_5, rRemote_patch_area_6
 *@return type    NONE
 *@Description    Creating Space to patch new code.
 *@
 ******************************************************************/

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S1 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
}	

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S2 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();

}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S3 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S4 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S5 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 200 (instructions) * 4 (bytes) = 800 Bytes
void rRemote_patch_area_S6 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();

}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 1000 (instructions) * 4 (bytes) = 4000 Bytes
void rRemote_patch_area_L1 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();


}

#pragma GCC optimize ("O0")

// Function having 1000 nops creating space of 1000 (instructions) * 4 (bytes) = 4000 Bytes
void rRemote_patch_area_L2 ()
{
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
	nop ();	nop (); nop (); nop (); nop (); nop ();	nop (); nop (); nop (); nop ();
}
