// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: BypassSlave.h
// *
// *    DESCRIPTION:
// *
// *    HISTORY: See SVN history for author and revision history.
// *********************************************************************************************************
#ifndef _BYPASS_SLAVE_H
#define _BYPASS_SLAVE_H
// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "BypassInterface.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

class BypassSlave : public BypassInterface
{
    public:
        BypassSlave();
        ~BypassSlave()
        {
        }

    public:
        virtual void RunStateMachine( void );
        virtual void Task100ms( void );
        
        virtual void ResetSticky( void );
        virtual bypass_states_t GetBypassState( void );
        virtual bypass_states_t GetMonitoredBypassState( void );
        
        virtual void RequestBypassState( bypass_states_t state );
        
        virtual bool IsBypassInitialized( void );
        virtual uBypassStatus GetBypassStatus( void );
        virtual const stAC_Power& GetActivePower( void );
        
        // Extended hooks for the CAN interface
        void SetStatus_CAN(const uint16_t status[]);
        void SetBypassPower_CAN(const int16_t power[]);
        
    protected:
        // Checks for bypass limits.
        inline void CheckBypassNotAvailable( void );
        
        // Bypass path status
        uBypassStatus     BypSlaveStatus;
        bypass_states_t   LocallyRequestedState;
        stAC_Power        MasterBypassPower;
};

// ********************************************************************************************************
// *            END OF BypassSlave.h
// ********************************************************************************************************
#endif

