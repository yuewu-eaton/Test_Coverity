// ******************************************************************************************************
// *            BypassState.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporation, ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BypassState.c
// *
// *    DESCRIPTION: Bypass State machine.
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
#include "DebuggerBlocks.h"
#include "Alarms_AC.h"
#include "PICState.h"
#include "BackfeedState.h"
#include "ACPowerMeter.h"
#include "ParallelCan.h"
// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************
namespace {
PICStateMachine PICMachine;
BackfeedStateMachine BackfeedRelay;
}
// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ********************************************************************************************************
// *
// * Function: BypassInit(void);
// *
// * Purpose:  See description.
// *
// * Description:   Executes bypass state.
// *
// ********************************************************************************************************
BypassStateMachine::BypassStateMachine(void)
{ 
    BypassStatus.word = 0;
    
    PIC = &PICMachine;
    Backfeed = &BackfeedRelay;
    
    RequestedBypassState = BYPASS_INIT_STATE;
    OldRequestedBypassState = BYPASS_INIT_STATE;
}

// ********************************************************************************************************
// *
// * Function: RunStateMachine(void);
// *
// * Purpose:  See description.
// *
// * Description:   Executes bypass state.
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void BypassStateMachine::RunStateMachine(void)	//2.165k Odd
{
    switch( RequestedBypassState )
    {
        case BYPASS_OFF_STATE:
            PIC->RequestState( PIC_IDLE_STATE );
            break;
        case BYPASS_READY_STATE:
            PIC->RequestState( PIC_READY_STATE );
            break;
        case BYPASS_PRIMED_STATE:
            PIC->RequestState( PIC_READY_ARMED_STATE );
            break;
        case BYPASS_FIRE_STATE:
            PIC->RequestState( PIC_FIRE_STATE );
            break;
        default:
            break;
    }
    
    // Run the PIC state machine
    PIC->RunStateMachine( MCUStateMachine.GetMetersReadyFlag()
    		           && (!(( BypassPLL.IsPhaseLocked() || ATBEnabled ) && !BypassDQLoss.GetState())) );
    
    // Don't force the relays closed in STSW short case.  Rely on normal methods.
    Backfeed->ForceClosed( ( PIC->GetState() == PIC_FIRE_STATE ) &&
                           !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT ) );
    
    // Run bypass not available alarm logic
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // check fast bypass alarms
        CheckBypassVoltageAlarms();
        
        CheckBypassNotAvailable();
    }
    
    if ( OldRequestedBypassState != RequestedBypassState )
    {
        NB_LogStateChange(UPM_NB_BYPASS_STATE_CHANGED, RequestedBypassState);
    }
    
    OldRequestedBypassState = RequestedBypassState;
}

// ***********************************************************************
// *
// *    FUNCTION: GetMonitoredBypassState 
// *
// *    DESCRIPTION: Return the monitored bypass state.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bypass_states_t BypassStateMachine::GetMonitoredBypassState( void )
{
    bypass_states_t ret = BYPASS_INIT_STATE;
    
    if ( !Backfeed->IsInitialized() || !PIC->IsInitialized() )
    {
        ret = BYPASS_INIT_STATE;
    }
    else if ( PIC->GetMonitoredState() == PIC_FIRE_STATE )
    {
        ret = BYPASS_FIRE_STATE;
    }
    else if ( Backfeed->GetState() == BACKFEED_CLOSED_STATE )
    {
        switch( PIC->GetMonitoredState() )
        {
            default:
            case PIC_IDLE_STATE:
                ret = BYPASS_OFF_STATE;
                break;
            case PIC_READY_STATE:
                ret = BYPASS_READY_STATE;
                break;
            case PIC_READY_ARMED_STATE:
                ret = BYPASS_PRIMED_STATE;
                break;
        }
        ret = ret;
    }
    else
    {
        ret = BYPASS_OFF_STATE;
    }
    
    return ret;
}

// ***********************************************************************
// *
// *    FUNCTION: CheckBypassNotAvailable 
// *
// *    DESCRIPTION: Checks and sets the bypass not available nodebit.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
inline void BypassStateMachine::CheckBypassNotAvailable( void )
{
    bool allowBypass = false;
    
    bypass_states_t state = GetBypassState();
    
    if ( ( Backfeed->GetState() == BACKFEED_OPENED_STATE ) ||
         Backfeed->IsFailedOpen() )
    {
    	qualifyBackfeed.ClearTimer();
    }
    else if ( qualifyBackfeed.CheckTimeout( BYPASS_10_SEC ) )
    {
    	// Allow the failed open detection to work on reset alarms.
    	allowBypass = true;
    	//fix LFP-187: If Remote building bypass active, when re-power on ups did not goto bypass
    	if ( qualifyBackfeed.CheckTimeout( BYPASS_12_SEC ) )
    	{
    		if ((ParallelCan.parallelSynchFlags & PARALLEL_SYNCH_FLAG_IDENTIFY_MASTER) == 0U)
    		{
    			ParallelCan.parallelSynchFlags &= ~PARALLEL_SYNCH_FLAG_BYPASS_NOT_AVAILABLE;
    		}
    	}
    }
    
    // set/clear bypass available flag
    if ( ( BypassPLL.IsPhaseLocked() || ATBEnabled )          &&
         !BypassDQLoss.GetState()                             &&
         !NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION )       &&
         !NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )     &&
         !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )      &&
         !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY ) &&
         !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )       &&
         !NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT )           &&
         !NB_GetNodebit( UPM_NB_BACKFEED_CONTACTOR_FAILURE )  &&
         Backfeed->GetState() == BACKFEED_CLOSED_STATE )  // Keming/20120712,add UPM_NB_BACKFEED_CONTACTOR_FAILURE
    {
        if (state == BYPASS_FIRE_STATE )
        {
            NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, false );
        }
        else
        {//EOSD-18:ATB time bigger than 4ms
            if ( (allowBypass &&
            	 ( MCUStateMachine.GetState() == SHUTDOWN_STATE  ||
            	   MCUStateMachine.GetState() == STANDBY_STATE ))   ||
                 (( ( state == BYPASS_READY_STATE ) ||
                   ( state == BYPASS_PRIMED_STATE ) )  &&
                 ( InverterSyncedToBypass() || ATBEnabled ))  )
            {
                NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, false );
            }
            else
            {
                NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, true, 0x04  );
            }
        }    
    }
    else
    {
        NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, true , !( BypassPLL.IsPhaseLocked() || ATBEnabled )
        		                                              + (BypassDQLoss.GetState() << 1) );
    }
}

// ***********************************************************************
// *
// * Function: GetBypassState()
// *
// * Purpose: returns current bypass state
// *
// * Parms passed: none
// *
// * Returns: Current bypass state machine state
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bypass_states_t BypassStateMachine::GetBypassState(void)
{
    bypass_states_t ret = BYPASS_INIT_STATE;
    
    if ( !Backfeed->IsInitialized() || !PIC->IsInitialized() )
    {
        ret = BYPASS_INIT_STATE;
    }
    else if ( PIC->GetState() == PIC_FIRE_STATE )
    {
        ret = BYPASS_FIRE_STATE;
    }
    else if ( Backfeed->GetState() == BACKFEED_CLOSED_STATE )
    {
        switch( PIC->GetState() )
        {
            default:
            case PIC_IDLE_STATE:
                ret = BYPASS_OFF_STATE;
                break;
            case PIC_READY_STATE:
                ret = BYPASS_READY_STATE;
                break;
            case PIC_READY_ARMED_STATE:
                ret = BYPASS_PRIMED_STATE;
                break;
        }
        ret = ret;
    }
    else
    {
        ret = BYPASS_OFF_STATE;
    }
    
    return ret;
}

// ***********************************************************************
// *
// * Function: IsBypassInitialized()
// *
// * Purpose: Returns if the bypass is fully initalized
// *
// ***********************************************************************
bool BypassStateMachine::IsBypassInitialized( void )
{
    return PIC->IsInitialized() && Backfeed->IsInitialized();
}

// ***********************************************************************
// *
// * Function: GetBypassStatus()
// *
// * Purpose: Returns current bypass status
// *
// ***********************************************************************
uBypassStatus BypassStateMachine::GetBypassStatus( void )
{
	uBypassStatus ret;
	ret.bit.status = uint16_t((Backfeed->GetStatus() << 11) | PIC->GetStatus());
	ret.bit.MonitoredBypassState = GetMonitoredBypassState();
	ret.bit.BypassState = GetBypassState();
	ret.words[3] = 0;
	return ret;
}


// ***********************************************************************
// *
// *    FUNCTION: ResetSticky() 
// *
// *    DESCRIPTION: Reset sticky type alarms and bits in state machine.
// *
// ***********************************************************************
void BypassStateMachine::ResetSticky( void )
{
    Backfeed->ResetSticky();
}

const stAC_Power& BypassStateMachine::GetActivePower(void)
{
	return BypassPower.ActivePower;
}

// ******************************************************************************************************
// *            End of BypassState.cpp
// ******************************************************************************************************
