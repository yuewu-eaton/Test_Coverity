// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: BackfeedState.c
// *
// *    DESCRIPTION: Backfeed State machine.
// *
// *    HISTORY: See SVN history for author and revision history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"
#include <cmath>
#include "BackfeedState.h"
#include "F28335Port.h"
#include "NB_Funcs.h"
#include "FilteredBit.h"
#include "eeprom_map.h"

using namespace std;

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************

// Backfeed open and close delays, in 100's of ms
const uint32_t BackfeedOpenDelay = 10 * 10;
const uint32_t BackfeedOpenDelay_STSW_Short = 10 * 3;
const uint32_t BackfeedCloseDelay = 10 * 10;

// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ********************************************************************************************************
// *
// * Function: BackfeedStateMachine(void);
// *
// * Purpose:  See description.
// *
// * Description:   Initializes backfeed state machine.
// *
// ********************************************************************************************************
BackfeedStateMachine::BackfeedStateMachine( void )
{
    // Drive the relay open, if we are on bypass the PIC will keep it closed.
    Status.word = 0;
    CloseDelay_ms = 50;
    OpenDelay_ms = 80;
    DSPOutRegister.GpoB.bit.BackFeedRelay = 0;
    
    Status.bit.State_Unknown = true;
    
    MachineState = BACKFEED_UNKNOWN_STATE;
    MachineStateNext = BACKFEED_UNKNOWN_STATE;
}

// ********************************************************************************************************
// *
// * Function: RunStateMachine( void );
// *
// * Purpose:  See description.
// *
// * Description:   Executes PIC state machine.
// *
// ********************************************************************************************************

void BackfeedStateMachine::RunStateMachine( bool voltage_present )
{
    switch ( MachineState )
    {
        case BACKFEED_UNKNOWN_STATE:
            Status.bit.State_Unknown = false;
            
            // fall through into BACKFEED_OPENED_STATE
            //lint -fallthrough
        case BACKFEED_OPENED_STATE:
        default:
            OpenedState( voltage_present );
            break;

        case BACKFEED_CLOSED_STATE:
            ClosedState( voltage_present );
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: TransferState(pic_states_t new_state);
// *
// * Purpose:
// *
// * Description: Handles the tranfers between states
// *
// ********************************************************************************************************

void BackfeedStateMachine::TransferState( backfeed_states_t new_state )
{
    Status.bit.State_Opened = false;
    Status.bit.State_Closed = false;
    Status.bit.State_Unknown = false;
    
    BackfeedRelayActionTimer.ClearTimer();
    BackfeedRelayAvailableTimer.ClearTimer();
    
    switch (new_state)
    {
        case BACKFEED_OPENED_STATE:
            Status.bit.State_Opened = true;
            DSPOutRegister.GpoB.bit.BackFeedRelay = 0;
            break;
        case BACKFEED_CLOSED_STATE:
            Status.bit.State_Closed = true;
            DSPOutRegister.GpoB.bit.BackFeedRelay = 1;
            break;
        default:
            break;
    }

    MachineState = new_state;
}

// ********************************************************************************************************
// *
// * Function: OpenedState( void );
// *
// * Purpose:  Control when to close the backfeed relay.
// *
// *   We can't determine when Failed Closed is a relay stuck or a shorted STSW,
// *    so we will disable bypass in that case and not close the contactor.
// *
// * Description:  
// *
// ********************************************************************************************************
inline void BackfeedStateMachine::OpenedState( bool voltage_present )
{
    Status.bit.State_Opened = true;

    NB_SetNodebit( UPM_NB_BACKFEED_RELAY_CLOSE, false );

    if ( Status.bit.ForceClosed        ||
         ( voltage_present                                     &&
           !NB_GetNodebit( UPM_NB_BACKFEED_CONTACTOR_FAILURE ) &&
           !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )        &&
           !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )      &&
           !Status.bit.ForceOpen ) )
    {
        if ( Status.bit.ForceClosed ||
             BackfeedRelayAvailableTimer.CheckTimeout( BackfeedCloseDelay ) )
        {
            MachineStateNext = BACKFEED_CLOSED_STATE;
        }
    }
    else
    {
        BackfeedRelayAvailableTimer.ClearTimer();
        MachineStateNext = BACKFEED_OPENED_STATE;
    }

    switch (MachineStateNext)
    {
        case BACKFEED_CLOSED_STATE:
            DSPOutRegister.GpoB.bit.BackFeedRelay = 1;
            
            if ( BackfeedRelayActionTimer.CheckTimeout((CloseDelay_ms + 200) / 100) )
            {
                // Transfer state AFTER confirming contactor changed states
                TransferState( BACKFEED_CLOSED_STATE );
            }
            break;

        case BACKFEED_OPENED_STATE:
        default:
            DSPOutRegister.GpoB.bit.BackFeedRelay = 0;
            BackfeedRelayActionTimer.ClearTimer();
            break;
    }
}


// ********************************************************************************************************
// *
// * Function: ClosedState( void );
// *
// * Purpose:  Control when to open the backfeed relay.
// *
// * Description: 
// *
// ********************************************************************************************************
inline void BackfeedStateMachine::ClosedState( bool voltage_present )
{
    Status.bit.State_Closed = true;

    NB_SetNodebit( UPM_NB_BACKFEED_RELAY_CLOSE, true );

    if ( !Status.bit.ForceClosed      &&
         ( !voltage_present || Status.bit.ForceOpen ) )
    {
        uint32_t delay;
        
        if ( NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT ) )
        {
            delay = BackfeedOpenDelay_STSW_Short;
        }
        else
        {
            delay = BackfeedOpenDelay;
        }
        
        if ( BackfeedRelayAvailableTimer.CheckTimeout( delay ) )
        {
            MachineStateNext = BACKFEED_OPENED_STATE;
        }
    }
    else
    {
        BackfeedRelayAvailableTimer.ClearTimer();
        MachineStateNext = BACKFEED_CLOSED_STATE;
    }
    
    switch ( MachineStateNext )
    {
        case BACKFEED_OPENED_STATE:
            if ( BackfeedRelayActionTimer.CheckTimeout((OpenDelay_ms + 200) / 100) )
            {
                // Transfer state AFTER confirming contactor changed states
                TransferState( BACKFEED_OPENED_STATE );
            }
            break;

        case BACKFEED_CLOSED_STATE:
        default:
            DSPOutRegister.GpoB.bit.BackFeedRelay = 1;
            BackfeedRelayActionTimer.ClearTimer();
            break;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: CheckBackfeedFail() 
// *
// *    DESCRIPTION: Check to see if the backfeed relay has been stuck.
// *
// *    ARGUMENTS: voltagePresent - Whether to run the check or not.
// *
// *    RETURNS: None
// *
// ***********************************************************************
void BackfeedStateMachine::CheckBackfeedFail( bool voltagePresent, const stThreePhase* primary, const stThreePhase* secondary )
{
    static FilteredBit BackfeedFilterL1( UPM_NB_BACKFEED_CONTACTOR_FAILURE, NOT_STICKY );
    static FilteredBit BackfeedFilterL2( UPM_NB_BACKFEED_CONTACTOR_FAILURE, NOT_STICKY );
    static FilteredBit BackfeedFilterL3( UPM_NB_BACKFEED_CONTACTOR_FAILURE, NOT_STICKY );
    
    const float percentage = BackfeedRelayFailSetting * 0.01;    // Backfeed relay failure detection set at 50% of nominal voltage
	const float FailOpenpercentage = BackfeedRlyFailOpenSetting * 0.01;
    uint16_t tmpBackfeedFail = 0;
    bool tempBackfeedOK = false;  //add for backfeed alarm deactivate.
    
    if ( voltagePresent ) // Backfeed Relay should be closed, look for the absence of voltage                                   
    {
		if(  Status.bit.State_Opened			   &&
				   !DSPOutRegister.GpoB.bit.BackFeedRelay )
		{
		   if( BackfeedFilterL1.Debounce( secondary->phA > primary->phA * FailOpenpercentage ) )
		   {
			   tmpBackfeedFail = 1;
		   }
			   
		   if( BackfeedFilterL2.Debounce( secondary->phB > primary->phB * FailOpenpercentage ) )
		   {
			   tmpBackfeedFail *= 10;
			   tmpBackfeedFail += 2;
		   }
			   
		   if( BackfeedFilterL3.Debounce( secondary->phC > primary->phC * FailOpenpercentage ) )
		   {
			   tmpBackfeedFail *= 10;
			   tmpBackfeedFail += 3;
		   }
		   
		   if( 0 == tmpBackfeedFail )
		   {
			   tempBackfeedOK = true;
		   }
		}
		else if ( Status.bit.State_Closed               &&
                  DSPOutRegister.GpoB.bit.BackFeedRelay )
        {
            if( BackfeedFilterL1.Debounce( secondary->phA < primary->phA * percentage ) )
            {
                tmpBackfeedFail = 4;
            }
            
            if( BackfeedFilterL2.Debounce( secondary->phB < primary->phB * percentage ) )
            {
                tmpBackfeedFail *= 10;
                tmpBackfeedFail += 5;
            }
            
            if( BackfeedFilterL3.Debounce( secondary->phC < primary->phC * percentage ) )
            {
                tmpBackfeedFail *= 10;
                tmpBackfeedFail += 6;
            }
            
            // add for backfeed alarm deactivate.
            if( ( fabs( secondary->phA - primary->phA ) < 20.0 ) &&
                ( fabs( secondary->phB - primary->phB ) < 20.0 ) &&
                ( fabs( secondary->phC - primary->phC ) < 20.0 ) )
            {
                tempBackfeedOK = true;
            }
            
            if ( tmpBackfeedFail )
            {
                Status.bit.Failed_Open = true;
            }
            else
            {
                Status.bit.Failed_Open = false;
            }
        }
        else
        {
            BackfeedFilterL1.SetState( false );
            BackfeedFilterL2.SetState( false );
            BackfeedFilterL3.SetState( false );
        }

        NB_DebounceAndQue_Hysterisis( UPM_NB_BACKFEED_CONTACTOR_FAILURE,
                                      tmpBackfeedFail,
                                      !tmpBackfeedFail && tempBackfeedOK,
                                      tmpBackfeedFail );
    }
    else
    {
        BackfeedFilterL1.SetState( false );
        BackfeedFilterL2.SetState( false );
        BackfeedFilterL3.SetState( false );
    }
}

// ******************************************************************************************************
// *            End of BackfeedState.cpp
// ******************************************************************************************************
