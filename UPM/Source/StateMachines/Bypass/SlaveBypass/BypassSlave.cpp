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
#ifdef 0
#include "DSP28x_Project.h"
//#include "BypassSlave.h"
#include "ParallelCan.h"
#include "InvSync.h"
#include "Alarms_AC.h"
#include "MCUState.h"
#include "NB_Funcs.h"
#include <cstring>
using std::memcpy;

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************

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
BypassSlave::BypassSlave(void)
{ 
    BypSlaveStatus.words[0] = 0;
    BypSlaveStatus.words[1] = 0;
    BypSlaveStatus.words[2] = 0;
    MasterBypassPower.phA = 0.0f;
    MasterBypassPower.phB = 0.0f;
    MasterBypassPower.phC = 0.0f;
    
    
    // Assume the init status until further notice by the master node
    BypSlaveStatus.bit.MonitoredBypassState = BYPASS_INIT_STATE;
    BypSlaveStatus.bit.BypassState = BYPASS_INIT_STATE;
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
void BypassSlave::RunStateMachine(void)
{
    // Run bypass not available alarm logic
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // check fast bypass alarms
        CheckBypassVoltageAlarms();
        
        CheckBypassNotAvailable();
    }
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
bypass_states_t BypassSlave::GetMonitoredBypassState( void )
{
    return bypass_states_t(BypSlaveStatus.bit.MonitoredBypassState);
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
inline void BypassSlave::CheckBypassNotAvailable( void )
{
    bypass_states_t state = GetBypassState();
    
    
    // set/clear bypass available flag
    if ( ( BypassPLL.IsPhaseLocked() || ATBEnabled )          &&
         !BypassDQLoss.GetState()                             &&
         !NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION )       &&
         !NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )     &&
         !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )      &&
         !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY ) &&
         !NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT )           &&
         IsBypassInitialized() )
    {
        if ( BypSlaveStatus.bit.BypassState == BYPASS_FIRE_STATE )
        {
            NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, false );
        }
        else
        {
            if ( ( MCUStateMachine.GetState() == SHUTDOWN_STATE  ||
                   MCUStateMachine.GetState() == STANDBY_STATE )   ||
                   ( state == BYPASS_READY_STATE                   ||
                     state == BYPASS_PRIMED_STATE )              &&
                   ( InverterSyncedToBypass() || ATBEnabled ) )
            {
                NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, false );
            }
            else
            {
                NB_DebounceAndQue( UPM_NB_BYPASS_NOT_AVAILABLE, true, 0x04 );
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
bypass_states_t BypassSlave::GetBypassState(void)
{
    return bypass_states_t(BypSlaveStatus.bit.BypassState);
}

// ***********************************************************************
// *
// * Function: IsBypassInitialized()
// *
// * Purpose: Returns if the bypass is fully initalized
// *
// ***********************************************************************
bool BypassSlave::IsBypassInitialized( void )
{
    return BypSlaveStatus.bit.BypassState != BYPASS_INIT_STATE;
}

// ***********************************************************************
// *
// * Function: GetBypassStatus()
// *
// * Purpose: Returns current bypass status
// *
// ***********************************************************************
uBypassStatus BypassSlave::GetBypassStatus( void )
{
    return BypSlaveStatus;
}


// ***********************************************************************
// *
// *    FUNCTION: ResetSticky() 
// *
// *    DESCRIPTION: Reset sticky type alarms and bits in state machine.
// *
// ***********************************************************************
void BypassSlave::ResetSticky( void )
{
}

void BypassSlave::SetStatus_CAN(const uint16_t status[])
{
    memcpy(BypSlaveStatus.words, status, sizeof(BypSlaveStatus.words));
}

void BypassSlave::SetBypassPower_CAN(const int16_t power[])
{
    MasterBypassPower.phA = float(power[0]) * 100;
    MasterBypassPower.phB = float(power[1]) * 100;
    MasterBypassPower.phC = float(power[2]) * 100;
    MasterBypassPower.UpdateSum();
}
        
void BypassSlave::RequestBypassState( bypass_states_t state )
{
#if 0
    // Historical note: At one time, the design was to allow slave nodes to request
    // a bypass transfer over CAN through this method.  IE, bypass state sync was going to
    // be maintained in the bypass state machine.  For 9E, bypass state sync is being
    // done through the MCU state machine instead.  This code remains for reference
    // purposes.
    if (this->LocallyRequestedState != state)
    {
        // Only transmit the request if the locally requested state differs from
        // the last requested state that was transmitted.
        ParallelCan.TransmitBypassCommand(state);
        LocallyRequestedState = state;
    }
#endif
}

void BypassSlave::Task100ms(void)
{   
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {   
        NB_DebounceAndQue( UPM_NB_BYPASS_PHASE_ROTATION, BypassPLL.IsPhaseRotationError() );
    }
}

const stAC_Power& BypassSlave::GetActivePower(void)
{
    return MasterBypassPower;
}
#endif
// ******************************************************************************************************
// *            End of BypassState.cpp
// ******************************************************************************************************
