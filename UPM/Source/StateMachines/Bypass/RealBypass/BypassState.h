// ********************************************************************************************************
// *            BypassState.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: BypassState.h
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *    DATE: 24.2.2004
// *
// *    HISTORY: See CVS history
// *********************************************************************************************************
#ifndef _BYPASS_STATE_H
#define _BYPASS_STATE_H

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "BypassInterface.h"
#include "StateTimer.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

struct st_BypassBits
{
    uint16_t unused0          : 1;
    uint16_t unused1          : 1;
    uint16_t unused2          : 1;
    uint16_t unused3          : 1;

    uint16_t unused4          : 1;
    uint16_t unused5          : 1;
    uint16_t unused6          : 1;
    uint16_t unused7          : 1;
    
    uint16_t unused8          : 1;
    uint16_t unused9          : 1;
    uint16_t unused10         : 1;
    uint16_t unused11         : 1;
    
    uint16_t unused12         : 1;
    uint16_t unused13         : 1;
    uint16_t ForceRelayOpen   : 1;
    uint16_t unused15         : 1;
};

union uMasterBypassStatus
{
    st_BypassBits  bit;
    uint16_t       word;
};

// Forward Declaration
class PICStateMachine;
class BackfeedStateMachine;

class BypassStateMachine : public BypassInterface
{
    public:
        BypassStateMachine();
        ~BypassStateMachine()
        {
        }

    public:
        friend class Debugger;
        
        virtual void RunStateMachine( void );
        virtual void Task100ms( void );
        virtual void ResetSticky( void );
        virtual bypass_states_t GetBypassState( void );
        virtual bypass_states_t GetMonitoredBypassState( void );
        
        virtual void RequestBypassState( bypass_states_t state )
        {
            if ( ( state > BYPASS_INIT_STATE ) && ( state < BYPASS_STATE_MAX ) )
            {
                RequestedBypassState = state;
            }
        }
        virtual bool IsBypassInitialized( void );
        virtual uBypassStatus GetBypassStatus( void );
        virtual const stAC_Power& GetActivePower( void );
        
    protected:
        // Checks for bypass limits.
        inline void CheckBypassNotAvailable( void );
        StateTimer          qualifyBackfeed;
        
        // Bypass path status
        uMasterBypassStatus       BypassStatus;
        bypass_states_t     RequestedBypassState;
        bypass_states_t     OldRequestedBypassState;
        
        // Internal pointers to various components of the bypass path
        PICStateMachine* PIC;
        BackfeedStateMachine* Backfeed;
};

// ********************************************************************************************************
// *            END OF BypassState.h
// ********************************************************************************************************
#endif

