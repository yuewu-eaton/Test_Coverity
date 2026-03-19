// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: BypassInterface.h
// *
// *    DESCRIPTION: This is an interface file for bypass operation. All UPM's in the system must implement
// *        these defined functions in order for operation to commence.
// *
// *    HISTORY: See SVN history for author and revision history.
// *********************************************************************************************************
#ifndef _BYPASS_INTERFACE_H
#define _BYPASS_INTERFACE_H

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "Eeprom_map.h"
#include "Constants.h"
#include "Meters.h"
#include "NB_Config.h"
#include "FilteredBit.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

enum bypass_states_t
{
	// 16-bit wide bitfield.
    BYPASS_INIT_STATE       = 1 << 0,
    BYPASS_OFF_STATE        = 1 << 1,
    BYPASS_READY_STATE      = 1 << 2,
    BYPASS_PRIMED_STATE     = 1 << 3,
    BYPASS_FIRE_STATE       = 1 << 4,
    BYPASS_STATE_MAX
};

struct stBypassStatusBits
{
    uint16_t status; // Bitfield accumulator only
    uint16_t MonitoredBypassState;
    uint16_t BypassState;
    uint16_t /*unused */: 12;
    uint16_t SequenceNumber: 4;
};

union uBypassStatus
{
    stBypassStatusBits bit;
    uint16_t words[sizeof(stBypassStatusBits)];
};

class BypassInterface
{
    public:
        BypassInterface(): BypassDQLoss(25, 1) {};
        virtual ~BypassInterface(){};

    public:
        static void EECalcBypassLimits( const EE_ID* ee, const uint16_t* data ); 

        virtual void RunStateMachine( void )=0;
        virtual void Task100ms( void )=0;
        virtual void ResetSticky( void )=0;
        virtual bypass_states_t GetBypassState( void )=0;
        virtual bypass_states_t GetMonitoredBypassState( void )=0;

        virtual void RequestBypassState( bypass_states_t state )=0;
        virtual bool IsBypassInitialized( void )=0;
        virtual uBypassStatus GetBypassStatus( void )=0;
        virtual const stAC_Power& GetActivePower( void )=0;
        
    public:
        static float GetBypassDQHighLimit( void )
        {
            return BypassDQVoltageHighLimit;
        }
        static float GetBypassDQLowLimit( void )
        {
            return BypassDQVoltageLowLimit;
        }
        static float GetBypassDQLowLimit_ESS( void )
		{
			return BypassDQVoltageLowLimit_ESS;
		}
        static float GetBypassRMSHighLimit( void )
        {
            return BypassRMSVoltageHighLimit;
        }
        static float GetBypassRMSLowLimit( void )
        {
            return BypassRMSVoltageLowLimit;
        }
        static float GetBypassHighFreqLimit( void )
        {
            return float( OutNomFreq + BypFreqDeviation );
        }
        static float GetBypassLowFreqLimit( void )
        {
            return float( OutNomFreq - BypFreqDeviation );
        }
        
        void CheckBypassVoltageAlarms( void );

    protected:
        FilteredBit         BypassDQLoss;

    private:
        // Limits on bypass availablity
        static int16_t      BypFreqDeviation;
        static float        BypassDQVoltageLowLimit;
        static float        BypassDQVoltageHighLimit;
        static float        BypassRMSVoltageLowLimit;
        static float        BypassRMSVoltageHighLimit;
        static float        BypassDQVoltageLowLimit_ESS;
};

inline void ee_calc_bypass_limits( EE_ID* ee, uint16_t* data )
{   BypassInterface::EECalcBypassLimits( ee, data );             }


BypassInterface& BypassState();
// ********************************************************************************************************
// *            END OF BypassInterface.h
// ********************************************************************************************************
#endif

