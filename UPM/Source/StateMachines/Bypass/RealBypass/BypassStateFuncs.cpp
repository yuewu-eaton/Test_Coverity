// ******************************************************************************************************
// *            BypassStateFuncs.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporation, ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BypassStateFuncs.c
// *
// *    DESCRIPTION: Functions used by the bypass state machine.
// *
// *    HISTORY: See SVN history for author and revision history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "F28335Port.h"
#include "BypassState.h"
#include "NB_Funcs.h"
#include "IOexpansion.h"
#include "MCUState.h"
#include "InvSync.h"
#include "UPM_NB_Ids.h"
#include "DQPhaseLockLoop.h"
#include "Meters.h"
#include "ACMeter.h"
#include "DebuggerBlocks.h"
#include "Alarms_AC.h"
#include "PICState.h"
#include "BackfeedState.h"

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************

// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ****************************************************************************
// *
// *  Function:    Task100ms
// *
// *  Purpose :    Periodic bypass task function
// *
// *  Parms Passed: none          
// *  Returns     : Nothing
// *
// *  Description: By counting the number of transitions this function
// *               determines what frequency BYP_READY is generating (state)
// *               checks slow bypass alarms
// *
// ****************************************************************************
void BypassStateMachine::Task100ms(void)
{
    Backfeed->RunStateMachine(BypassPLL.IsSourcePresent() );
    
    stThreePhase BypassVoltage2;
    stThreePhase BypassVoltage;
    {
    	CriticalSection enter;
    	// Read RMS voltages atomically, since they are updated by the zero-crossing
    	// SWI's.
    	BypassVoltage2 = BypassVoltage2RMS.FilteredRMS;
    	BypassVoltage = BypassVoltageRMS.FilteredRMS;
    }
    
    if( BackfeedContactorInstalled )    //disable backfeed when it is not installed for HV system.
    {
        Backfeed->CheckBackfeedFail( MCUStateMachine.GetMetersReadyFlag() &&
                                     BypassPLL.IsSourcePresent() &&
                                     !NB_GetNodebit(UPM_NB_BYPASS_AC_UNDER_VOLTAGE),
                                     &BypassVoltage,
                                     &BypassVoltage2 );
    }
    
    PIC->Task100ms();
    
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        NB_DebounceAndQue( UPM_NB_BYPASS_PHASE_ROTATION, BypassPLL.IsPhaseRotationError() );
    }
}

// ******************************************************************************************************
// *            End of BypassState.cpp
// ******************************************************************************************************
