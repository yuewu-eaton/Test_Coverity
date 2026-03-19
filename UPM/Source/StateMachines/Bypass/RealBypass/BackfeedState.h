// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: BackfeedState.h
// *
// *    DESCRIPTION: Controls the backfeed relay
// *
// *    HISTORY: See SVN history for author and revision history.
// *********************************************************************************************************
#ifndef _BACKFEED_STATE_H
#define _BACKFEED_STATE_H

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "Constants.h"
#include "StateTimer.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

enum backfeed_states_t
{
    BACKFEED_UNKNOWN_STATE       = 0,
    BACKFEED_OPENED_STATE,
    BACKFEED_CLOSED_STATE,
    BACKFEED_MAX_STATE
};

struct st_BackfeedBits
{
    uint16_t State_Unknown                : 1;
    uint16_t State_Opened                 : 1;
    uint16_t State_Closed                 : 1;
    uint16_t ForceClosed                  : 1;
    
    uint16_t Failed_Open                  : 1;
    uint16_t ForceOpen                    : 1;
    uint16_t unused5_15                   : 10;
};

union uBackfeedStatus
{
    st_BackfeedBits  bit;
    uint16_t         word;
};

class BackfeedStateMachine
{
    public:
        BackfeedStateMachine( void );
        ~BackfeedStateMachine( void ){};

    public:
        void RunStateMachine( bool voltage_present );
        
        // Reset the failed status of the relay.
        void ResetSticky( void )
        {
            Status.bit.Failed_Open = false;
        }
        
        // Backfeed state machine is now requesting a state
        bool IsInitialized( void )
        {
            return !Status.bit.State_Unknown;
        }
        
        // Force the backfeed relay closed, due to the PIC firing.
        void ForceClosed( bool close )
        {
            if ( close )
            {
                Status.bit.ForceClosed = 1;
            }
            else
            {
                Status.bit.ForceClosed = 0;
            }
        }

        void ForceOpen( bool open )
        {
            if ( open )
            {
                Status.bit.ForceOpen = 1;
            }
            else
            {
                Status.bit.ForceOpen = 0;
            }
        }
        
        backfeed_states_t GetState( void )
        {
            return MachineState;
        }
        
        bool IsFailedOpen ( void )
        {
            return Status.bit.Failed_Open;
        }
        
        uint16_t GetStatus( void )
        {
            return Status.word;
        }
        
   public:
        void CheckBackfeedFail( bool voltagePresent, const stThreePhase* primary, const stThreePhase* secondary );
        
    private:
        void                TransferState( backfeed_states_t new_state );
        inline void         OpenedState( bool voltage_present );
        inline void         ClosedState( bool voltage_present );
        
        uBackfeedStatus     Status;
        
        backfeed_states_t   MachineState;
        backfeed_states_t   MachineStateNext;
                
        // Don't consider the bypass contactor closed or open after a transition
        // until after this delay has expired.
        StateTimer          BackfeedRelayActionTimer;
        StateTimer          BackfeedRelayAvailableTimer;
        uint16_t            CloseDelay_ms;
        uint16_t            OpenDelay_ms;
};
extern float BackfeedRelayFailSetting;
extern float BackfeedRlyFailOpenSetting;

// ********************************************************************************************************
// *            END OF BackfeedState.h
// ********************************************************************************************************
#endif

