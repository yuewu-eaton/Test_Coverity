// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: PICState.h
// *
// *    DESCRIPTION:
// *
// *    HISTORY: See SVN history for author and revision history.
// *********************************************************************************************************
#ifndef _PIC_STATE_H
#define _PIC_STATE_H

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "StateTimer.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

/*** DEFINE THE MACHINE STATES ****/
// Used also for RequestedState and MonitoredState
enum pic_states_t
{
    PIC_UNKNOWN_STATE        =   0, // This state cannot be requested
    PIC_INIT_STATE,                 // This state cannot be requested
    PIC_IDLE_STATE,               
    PIC_READY_STATE,              
    PIC_READY_ARMED_STATE,        
    PIC_FIRE_STATE,
    PIC_STATE_MAX
};

// Time constants, based on 2*231 = 462us task. Round up!
#define BYPASS_ONE_SEC                  2165L
#define BYPASS_300_MSEC                 649L
#define BYPASS_100_MSEC                 216L
#define BYPASS_50_MSEC                  108L
#define BYPASS_5_MSEC                   11L
#define BYPASS_750_USEC                 2L
#define BYPASS_2_SEC                    (uint32_t)(2L * BYPASS_ONE_SEC)
#define BYPASS_5_SEC                    (uint32_t)(5L * BYPASS_ONE_SEC)
#define BYPASS_9_SEC                    (uint32_t)(9L * BYPASS_ONE_SEC)
#define BYPASS_10_SEC                   (uint32_t)(10L * BYPASS_ONE_SEC)
#define BYPASS_12_SEC                   (uint32_t)(12L * BYPASS_ONE_SEC)
#define BYPASS_MS_TO_TIMEOUT(ms)        (uint32_t(ms) * 128 / 50) //no use

struct st_PICBits
{
    uint16_t MachineStateChanging         : 1;
    uint16_t PICAlive                     : 1;
    uint16_t VoltageFail                  : 1;
    uint16_t CommunicationFailed          : 1;

    uint16_t State_Init                   : 1;
    uint16_t State_Idle                   : 1;
    uint16_t State_Ready                  : 1;
    uint16_t State_ReadyArmed             : 1;
    
    uint16_t State_Fire                   : 1;
    uint16_t State_Unknown                : 1;
    uint16_t Uninitialized                : 1;
    uint16_t unused11                     : 1;
    
    uint16_t unused12                     : 1;
    uint16_t unused13                     : 1;
    uint16_t unused14                     : 1;
    uint16_t unused15                     : 1;
};

union uPICStatus
{
    st_PICBits  bit;
    uint16_t    word;
};

class PICStateMachine
{
    public:
        PICStateMachine( void );
        ~PICStateMachine( void ){};

    public:
        void         RunStateMachine( bool voltage_fail );
        void         Task100ms( void );
        pic_states_t GetState( void );
        pic_states_t GetRequestedState(void)
        {
        	return RequestedState;
        }
        pic_states_t GetMonitoredState( void )
        {
            return MonitoredState;
        }
        void RequestState( pic_states_t state )
        {
            if ( ( state > PIC_INIT_STATE ) && ( state < PIC_STATE_MAX ) )
            {
                RequestedState = state;
            }       
        }
        bool IsInitialized( void )
        {
            return !Status.bit.Uninitialized;
        }
        uint16_t GetStatus( void )
        {
            return Status.word;
        }

    
    protected:
        void InitSignals( pic_states_t state );

    private:
        void TransferState( pic_states_t new_state );
        inline void InitState( void );
        inline void IdleState( void );
        inline void ReadyState( void );
        inline void ReadyArmedState( void );
        inline void FireState( void );
        
        uPICStatus          Status;
        StateTimer          Timer1;
        
        pic_states_t        MachineState;
        pic_states_t        MachineStateNext;
        pic_states_t        RequestedState;
        pic_states_t        MonitoredState;
        
        bool                ReadyLast;
        volatile uint16_t   ReadyCount;
        uint16_t            StateMonitorDebounce;
};

// ********************************************************************************************************
// *            END OF PICState.h
// ********************************************************************************************************
#endif

