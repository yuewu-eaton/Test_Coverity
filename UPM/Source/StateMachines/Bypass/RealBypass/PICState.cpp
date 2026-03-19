// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: PICState.c
// *
// *    DESCRIPTION: PIC State machine.
// *
// *    HISTORY: See SVN history for author and revision history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "PICState.h"
#include "F28335Port.h"
#include "NB_Funcs.h"

#include "Debugger.h"

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************

// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ********************************************************************************************************
// *
// * Function: PICStateMachine( void );
// *
// * Purpose:  See description.
// *
// * Description:   Initializes PIC state machine.
// *
// ********************************************************************************************************
PICStateMachine::PICStateMachine( void )
{
    Status.word = 0;
    Status.bit.Uninitialized = true;
    MachineState = PIC_INIT_STATE;
    MachineStateNext = PIC_INIT_STATE;

    RequestedState = PIC_UNKNOWN_STATE;
    MonitoredState = PIC_UNKNOWN_STATE;
    StateMonitorDebounce = 0;
    ReadyLast = false;
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
#pragma CODE_SECTION( "ramfuncs" )
void PICStateMachine::RunStateMachine( bool voltage_fail )
{
    // Check the ready signal
    if ( ( DSPInRegister.GpiB.bit.Byp_Ready && !ReadyLast ) ||
         ( !DSPInRegister.GpiB.bit.Byp_Ready && ReadyLast ) )
    {
        ReadyCount++;
        if ( DSPInRegister.GpiB.bit.Byp_Ready )
        {
            ReadyLast = true;
        }
        else
        {
            ReadyLast = false;
        }            
    }
    
	Status.bit.VoltageFail = voltage_fail;
    
    // Execute the corresponding Machine State Module
    if ( !Status.bit.Uninitialized )
    {
        switch ( MachineState )
        {
            case PIC_UNKNOWN_STATE:
            case PIC_INIT_STATE:
                InitState();
                break;
    
            case PIC_IDLE_STATE:
                IdleState();
                break;

            case PIC_READY_STATE:
                ReadyState();
                break;

            case PIC_READY_ARMED_STATE:
                ReadyArmedState();
                break;

            case PIC_FIRE_STATE:
                FireState();
                break;

            default:
                InitState();
                break;
        }
    }
    else
    {
        // Make sure that we don't drop out of fire state when we enable IO_GOOD                                   
        DSPOutRegister.GpoC.bit.Inv_Online = 1;

        // PIC state not available
        Status.bit.State_Init = 0;
        Status.bit.State_Idle = 0;
        Status.bit.State_Ready = 0;
        Status.bit.State_ReadyArmed = 0;
        Status.bit.State_Fire = 0;
        Status.bit.State_Unknown = 1;
        Status.bit.MachineStateChanging = 0;
            
        // Wait to read PIC status, then set the state
        if ( MonitoredState != PIC_UNKNOWN_STATE &&
             Timer1.CheckTimeout( BYPASS_300_MSEC ) )
        {
            InitSignals( MonitoredState );
            Status.bit.Uninitialized = false;
            TransferState( MonitoredState );
        }
    }
}

// ********************************************************************************************************
// *
// * Function: TransferState(pic_states_t new_state);
// *
// * Purpose:
// *
// * Parms Passed   :   new_state
// * Returns        :   Nothing
// *
// * Description: Handles the tranfers between states
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void PICStateMachine::TransferState( pic_states_t new_state )
{

    Status.bit.State_Init = 0;
    Status.bit.State_Idle = 0;
    Status.bit.State_Ready = 0;
    Status.bit.State_ReadyArmed = 0;
    Status.bit.State_Fire = 0;
    Status.bit.State_Unknown = 0;
    
    switch (new_state)
    {
        case PIC_INIT_STATE:
            Status.bit.State_Init = 1;
            break;
        case PIC_IDLE_STATE:
            Status.bit.State_Idle = 1;
            break;
        case PIC_READY_STATE:
            Status.bit.State_Ready = 1;
            break;
        case PIC_READY_ARMED_STATE:
            Status.bit.State_ReadyArmed = 1;
            break;
        case PIC_FIRE_STATE:
            Status.bit.State_Fire = 1;
            break;
        default:
            break;
    }

    Timer1.ClearTimer();

    Status.bit.MachineStateChanging = false;
    MachineState = new_state;
}

// ********************************************************************************************************
// *
// * Function: InitState( void );
// *
// * Purpose:
// *
// * Description: Intializes State machine and PIC.
// *              This state is always temporary state
// *
// ********************************************************************************************************
inline void PICStateMachine::InitState( void )
{
    Status.bit.State_Init = 1;

    InitSignals( MonitoredState );

    if ( MonitoredState != PIC_INIT_STATE ||
         MonitoredState != PIC_UNKNOWN_STATE )
    {
        Status.bit.CommunicationFailed = false;
        TransferState( MonitoredState );
    }

    if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
    {
        Status.bit.CommunicationFailed = true;
    }
} // End of InitState()


// ********************************************************************************************************
// *
// * Function: IdleState( void );
// *
// * Purpose:
// *
// * Description:   Everything off.  Waiting for command.
// *
// ********************************************************************************************************
inline void PICStateMachine::IdleState( void )
{
    Status.bit.State_Idle = 1;

        // Get new state and keep it until we have completed the state transition
    if ((MachineStateNext == PIC_IDLE_STATE) && 
        !Status.bit.MachineStateChanging &&
        !Status.bit.VoltageFail          &&
        RequestedState != PIC_UNKNOWN_STATE)
    {
        MachineStateNext = RequestedState;
    }

    switch (MachineStateNext)
    {
        case PIC_READY_STATE:
        case PIC_READY_ARMED_STATE:
        case PIC_FIRE_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // PIC from Idle to Ready state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_READY_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_READY_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;
                TransferState(PIC_INIT_STATE);
            }
            break;

        case PIC_IDLE_STATE:
        default:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // Set output signals to Idle state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;

            if (MonitoredState == PIC_IDLE_STATE)
            {
                Timer1.ClearTimer();
            }
            else
            {
                if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
                {
                    Status.bit.CommunicationFailed = true;
                }
            }
            break;
    }

    if (Status.bit.CommunicationFailed)
    {
        TransferState(PIC_INIT_STATE);
    }
} // End of IdleState()


// ********************************************************************************************************
// *
// * Function: ReadyState( void );
// *
// * Purpose:
// *
// * Description: Ready for bypass. 
// *
// ********************************************************************************************************
inline void PICStateMachine::ReadyState( void )
{
    Status.bit.State_Ready = 1;

    // Get new state and keep it until we have completed the state transition
    if (Status.bit.VoltageFail)
    {
        MachineStateNext = PIC_IDLE_STATE;
    }
    else if (MachineStateNext == PIC_READY_STATE   && 
            !Status.bit.MachineStateChanging       &&
            RequestedState != PIC_UNKNOWN_STATE   )
    {
        MachineStateNext = RequestedState;
    }
    else if (Status.bit.MachineStateChanging      &&
    	RequestedState == PIC_READY_ARMED_STATE   &&
    	MachineStateNext == PIC_FIRE_STATE)
	{
		// Allow a rapid shortcut through fire to ready armed.
		StateMonitorDebounce = 0;
		MachineStateNext = PIC_READY_ARMED_STATE;
	}
	else if (Status.bit.MachineStateChanging     &&
		RequestedState == PIC_FIRE_STATE         &&
		MachineStateNext == PIC_READY_ARMED_STATE)
	{
		// Allow rapid shortcut through ready armed to fire
		StateMonitorDebounce = 0;
		MachineStateNext = PIC_FIRE_STATE;
	}
	
    switch (MachineStateNext)
    {
        case PIC_IDLE_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // PIC from Ready to Idle state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_IDLE_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_IDLE_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;
            }
            break;

        case PIC_READY_ARMED_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // PIC from Ready to Ready armed state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_READY_ARMED_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_READY_ARMED_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;
            }
            break;

        case PIC_FIRE_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // PIC from Ready to Fire state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_FIRE_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_FIRE_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true; 
            }
            break;

        case PIC_READY_STATE:
        default:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // Set output signals to Ready state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            if (MonitoredState == PIC_READY_STATE)
            {
                Timer1.ClearTimer();
            }
            else
            {
                if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
                {
                    Status.bit.CommunicationFailed = true;
                }
            }
            break;
    }

    if (Status.bit.CommunicationFailed)
    {
        TransferState(PIC_INIT_STATE);
    }
} // End of ReadyState()


// ********************************************************************************************************
// *
// * Function: ReadyArmedState( void );
// *
// * Purpose:
// *
// * Description:   
// *
// ********************************************************************************************************
inline void PICStateMachine::ReadyArmedState( void )
{
    Status.bit.State_ReadyArmed = 1;

    // Get new state and keep it until we have completed the state transition
    if (Status.bit.VoltageFail)
    {
        MachineStateNext = PIC_IDLE_STATE;
    }
    else if ((MachineStateNext == PIC_READY_ARMED_STATE) && 
            !Status.bit.MachineStateChanging             &&
            RequestedState != PIC_UNKNOWN_STATE)
    {
        MachineStateNext = RequestedState;
    }
    else if (Status.bit.MachineStateChanging         && 
        (RequestedState == PIC_FIRE_STATE)           &&
        ( (MachineStateNext == PIC_READY_ARMED_STATE)   ||
          (MachineStateNext == PIC_READY_STATE)))
    {
        //
        // Someone calling an emergency transfer back to FIRE state. 
        // May be due to OUTPUT_AC_UNDER_VOLTAGE or something similar,
        // need to resume bypass ASAP. May happen for example in the 
        // bypass test when shutting down bypass and the other unit has
        // dead bypass.
        //
        StateMonitorDebounce = 0;
        MachineStateNext = PIC_FIRE_STATE;
    }
    else if (Status.bit.MachineStateChanging      &&
    	(RequestedState == PIC_READY_ARMED_STATE) &&
    	(MachineStateNext == PIC_FIRE_STATE))
	{
	    // Requesting a rapid transfer from Fire back to ready-armed.
		// Allow the rapid shortcut.
		StateMonitorDebounce = 0;
		MachineStateNext = PIC_READY_ARMED_STATE;
	}

    switch (MachineStateNext)
    {
        case PIC_IDLE_STATE:
        case PIC_READY_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // PIC from Ready armed to Ready state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_READY_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_READY_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;
            }
            break;

        case PIC_FIRE_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // PIC from Ready armed to Fire state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_FIRE_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_FIRE_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;
            }
            break;

        case PIC_READY_ARMED_STATE:
        default:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // Set output signals to Ready armed state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;

            if (MonitoredState == PIC_READY_ARMED_STATE)
            {
                Timer1.ClearTimer();
            }
            else
            {
                if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
                {
                    Status.bit.CommunicationFailed = true;
                }
            }
            break;
    }

    if (Status.bit.CommunicationFailed)
    {
        TransferState(PIC_INIT_STATE);
    }
} // End of ReadyArmedState()


// ********************************************************************************************************
// *
// * Function: FireState( void );
// *
// * Purpose:
// *
// * Description:   On bypass.
// *
// ********************************************************************************************************
inline void PICStateMachine::FireState( void )
{
    Status.bit.State_Fire = 1;
    
    // Get new state and keep it until we have completed the state transition
    if ((MachineStateNext == PIC_FIRE_STATE) && 
        !Status.bit.MachineStateChanging  &&
        RequestedState != PIC_UNKNOWN_STATE)
    {
        MachineStateNext = RequestedState;
    }

        //
        // Someone calling an emergency transfer back to FIRE state. 
        // May be due to OUTPUT_AC_UNDER_VOLTAGE or something similar,
        // need to resume bypass ASAP. May happen for example in the 
        // bypass test when shutting down bypass and the other unit has
        // dead bypass.
        //
    if (Status.bit.MachineStateChanging           && 
        (RequestedState == PIC_FIRE_STATE)           &&
        ((MachineStateNext == PIC_READY_ARMED_STATE) ||
         (MachineStateNext == PIC_READY_STATE)))
    {
        StateMonitorDebounce = 0;
        MachineStateNext = PIC_FIRE_STATE;
    }

    switch (MachineStateNext)
    {
        case PIC_IDLE_STATE:
        case PIC_READY_STATE:
        case PIC_READY_ARMED_STATE:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // PIC from Fire to Ready armed state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            // clear for state machine, coming back
            Status.bit.MachineStateChanging = true;

            if (MonitoredState == PIC_READY_ARMED_STATE)
            {
                // Transfer state AFTER pic has changed states
                TransferState(PIC_READY_ARMED_STATE);
            }

            if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
            {
                // wait 2 seconds, then set failed flag
                Status.bit.CommunicationFailed = true;    
            }
            break;

        case PIC_FIRE_STATE:
        default:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // Set output signals to Fire state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;

            if (MonitoredState == PIC_FIRE_STATE)
            {
                Timer1.ClearTimer();
                Status.bit.MachineStateChanging = false;
            }
            else
            {
                if ( Timer1.CheckTimeout( BYPASS_2_SEC ) )
                {
                    Status.bit.CommunicationFailed = true;
                }
            }
            break;
    }

    if (Status.bit.CommunicationFailed)
    {
        TransferState(PIC_INIT_STATE);
    }
} // End of FireState()

// ****************************************************************************
// *
// *  Function:    InitSignals( void )
// *
// *  Purpose :    Initialize PIC control signals and state machine
// *
// ****************************************************************************
void PICStateMachine::InitSignals( pic_states_t state )
{
    switch ( state )
    {
        case PIC_IDLE_STATE:
            MachineStateNext = PIC_IDLE_STATE;
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // Set output signals to Idle state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            break;

        case PIC_READY_STATE:
            MachineStateNext = PIC_READY_STATE;
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // Set output signals to Ready state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            break;

        case PIC_READY_ARMED_STATE:
            MachineStateNext = PIC_READY_ARMED_STATE;
            DSPOutRegister.GpoC.bit.Inv_Online = 0;  // Set output signals to Ready armed state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            break;

        case PIC_FIRE_STATE:
            MachineStateNext = PIC_FIRE_STATE;
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // Set output signals to Fire state
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            break;

        case PIC_INIT_STATE:
        default:
            MachineStateNext = PIC_INIT_STATE;
            DSPOutRegister.GpoC.bit.Inv_Online = 1;  // Set output signals to Init state
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            break;
    }
}

// ***********************************************************************
// *
// * Function: GetState()
// *
// * Purpose: returns current bypass state
// *
// * Parms passed: none
// *
// * Returns: Current bypass state machine state
// *
// ***********************************************************************
pic_states_t PICStateMachine::GetState(void)
{
    pic_states_t state;

    if (Status.bit.MachineStateChanging)
    {
        state = MachineStateNext;
    }
    else
    {
        state = MachineState;
    }

    return state;
}

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
void PICStateMachine::Task100ms(void)
{
	// Bypass state counter limits for 100ms task
	// Idle = 10
	const uint16_t IDLE_HI           = 13;
	const uint16_t IDLE_LOW          = 7;
	// ready = 102
	const uint16_t READY_HI          = 112;
	const uint16_t READY_LOW         = 92;
	// ready armed = 143
	const uint16_t READY_ARMED_HI    = 163;
	const uint16_t READY_ARMED_LOW   = 123;
	// fire = 204
	const uint16_t FIRE_HI           = 234;
	const uint16_t FIRE_LOW          = 174;
	
    uint16_t byp_rdy_cnt;

    {
        CriticalSection enter;
        byp_rdy_cnt = ReadyCount;
        ReadyCount = 0;
    }
    
        // ********************************************************************
        // * Check byp_rdy_cnt's value to determine what state the bypass
        // * board PIC is in.
        // ********************************************************************
    if (byp_rdy_cnt < IDLE_LOW)
    {
        // 0 when in the INIT state
        MonitoredState = PIC_INIT_STATE;
        Status.bit.PICAlive = false;
        StateMonitorDebounce = 0;
    }
    else
    {   
        Status.bit.PICAlive = true;
         
        if ((byp_rdy_cnt >= IDLE_LOW) && (byp_rdy_cnt <= IDLE_HI))
        {
            // 50 * 10msec = 500ms (20msec square wave)
            MonitoredState = PIC_IDLE_STATE;
            StateMonitorDebounce = 0;
        }
        else if ((byp_rdy_cnt >= READY_LOW) && (byp_rdy_cnt <= READY_HI))
        {
            // 500 * 1msec = 500ms (2msec square wave)
            MonitoredState = PIC_READY_STATE;
            StateMonitorDebounce = 0;
        }
        else if ((byp_rdy_cnt >= READY_ARMED_LOW) && (byp_rdy_cnt <= READY_ARMED_HI))
        {
            // 715 * 700usec = 500ms (1.4msec square wave)
            MonitoredState = PIC_READY_ARMED_STATE;
            StateMonitorDebounce = 0;
        }
        else if ((byp_rdy_cnt >= FIRE_LOW) && (byp_rdy_cnt <= FIRE_HI))
        {
            // 1000 * 500usec = 500ms (1msec square wave)
            MonitoredState = PIC_FIRE_STATE;
            StateMonitorDebounce = 0;
        }
        else
        {
            // can get garbage count when state is changing
            if (StateMonitorDebounce++ >= 2)
            {
                StateMonitorDebounce = 2;
                MonitoredState = PIC_UNKNOWN_STATE;
            }
        }
    }

    if ( Status.bit.VoltageFail )
    {
        NB_DebounceAndQue( UPM_NB_STATIC_SWITCH_FAILURE, false );
    }
    else
    {       
        NB_DebounceAndQue( UPM_NB_STATIC_SWITCH_FAILURE, Status.bit.CommunicationFailed );
    }    
}

// ******************************************************************************************************
// *            End of PICState.cpp
// ******************************************************************************************************
