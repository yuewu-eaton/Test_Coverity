// ******************************************************************************************************
// *            end of AutoCal.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: end of AutoCal.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 12/10/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _AUTOCAL_H
#define _AUTOCAL_H

#include "ParallelCan.h"

enum autocal_states_t
{
    AUTOCAL_INIT = 0,
    AUTOCAL_LOADSHARE_BYPASS_START,
    AUTOCAL_LOADSHARE_BYPASS,
    AUTOCAL_LOADSHARE_BYPASS_SAVE,
    AUTOCAL_LOADSHARE_BASE_START,
    AUTOCAL_LOADSHARE_BASE,
    AUTOCAL_LOADSHARE_BASE_SAVE,
    AUTOCAL_DONE
};

namespace
{
    const uint32_t  AutoCal20ms     = 1;
    const uint32_t  AutoCal100ms    = 5;
    const uint32_t  AutoCal200ms    = 10;
    const uint32_t  AutoCal500ms    = 25;
    const uint32_t  AutoCal1s       = 50;
    const uint32_t  AutoCal2s       = 100;
    const uint32_t  AutoCal5s       = 250;
    const uint32_t  AutoCal10s      = 500;
    const uint32_t  AutoCal30s      = 1500;
    const uint32_t  AutoCal60s      = 3000;
}

class AutoCalControl
{   
    public:
        AutoCalControl()
        {
            CalState = AUTOCAL_INIT;
            AutoCalStatus.all = 0;
            PhaseState = 0;
            Start = false;
        }
        ~AutoCalControl()
        {
        }

    private:
        AutoCalControl( const AutoCalControl& );
        const AutoCalControl& operator=( const AutoCalControl& );
    
    public:
        void RunAutoCal( void );
        void StartAutoCal( void );
        uint16_t GetAutoCalStatus( void )
        {   return AutoCalStatus.words[ AUTO_CAL_WORD ];    }
        
        uint16_t GetState( void )
        {
            return ( (uint16_t)CalState );
        }

    protected:
        bool  CheckAutoCalOK( void );
        void RunLoadShareCal( float (&command)[4] );
        void AutoCalReset( void );

    protected:
        autocal_states_t    CalState;
        bool                Start;  
        float               CalResult;
        int16_t             EE_Data;
        int16_t             PhaseState;
        uSyncStatus         AutoCalStatus;
        
        StateTimer          AutoCalTimer1;
        StateTimer          AutoCalTimer2;
        StateTimer          AutoCalMetersTimer;
        stSecondOrderIIRFP  GPControl;
        stSecondOrderIIRFP  VarControlA;
        stSecondOrderIIRFP  VarControlB;
        stSecondOrderIIRFP  VarControlC;
};

extern AutoCalControl AutoCal;
extern stSecondOrderIIRFP MIMOShareCoeffs;

#endif
// ******************************************************************************************************
// *  end of AutoCal.h
// ******************************************************************************************************
